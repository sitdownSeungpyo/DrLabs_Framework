//! One Dynamixel servo, as the rest of the framework sees it.
//!
//! The interesting work is unit translation. A Dynamixel speaks encoder counts,
//! 0.229 RPM steps and milliamp-scale current units; the framework speaks
//! revolutions, rev/s and amperes. Doing that conversion here — rather than
//! leaving it to applications — is what lets one control loop, one safety
//! monitor and one UI drive a Dynamixel and a moteus without knowing which is
//! which.
//!
//! What a Dynamixel cannot do is also part of the contract. It has no
//! stay-within mode, and without a motor-specific torque constant it cannot
//! report torque in newton-metres, so those are reported as unsupported and
//! unmeasured rather than approximated.

use std::sync::Arc;
use std::time::Duration;

use drlabs_core::command::{Command, ControlMode, Mode, QueryProfile};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::Telemetry;
use drlabs_core::units::{Amps, Celsius, Nm, Rev, RevPerSec, Volts};
use drlabs_transport::serial::SharedPort;

use crate::driver::Driver;
use crate::profile::{
    decode_hardware_error, known_model, profile_named, Profile, OP_MODE_CURRENT,
    OP_MODE_EXTENDED_POSITION, OP_MODE_VELOCITY,
};
use crate::protocol::MAX_VALID_MOTOR_ID;

/// How long an EEPROM write needs to settle before the next command, or the
/// motor answers with a stale mode and the first goal goes to the wrong
/// register.
const EEPROM_SETTLE: Duration = Duration::from_millis(200);

/// A [`Link`] backed by one Dynamixel servo.
pub struct DynamixelLink {
    driver: Option<Arc<Driver>>,
    port_name: String,
    baud: u32,
    id: u8,

    profile: Profile,

    /// Whether the caller named a profile or is relying on the servo to
    /// identify itself.
    profile_declared: bool,

    /// What the servo reported on the last ping, and what this build knows the
    /// number to be.
    model: Option<u32>,
    model_name: Option<&'static str>,

    /// The operating mode believed to be in the motor's EEPROM. Changing it
    /// requires torque off, an EEPROM write and a settling delay, so it is
    /// tracked rather than written every cycle.
    current_mode: Option<u8>,
    torque_enabled: bool,

    /// What was last commanded, so unchanged parameters are not rewritten every
    /// period. Profile velocity and acceleration live in EEPROM-adjacent RAM
    /// and rewriting them at the control rate wastes most of the serial
    /// bandwidth the loop has.
    last_velocity_limit: Option<f64>,
    last_accel_limit: Option<f64>,
    last_watchdog: Option<f64>,
}

impl DynamixelLink {
    /// A link on a serial port the caller already owns.
    ///
    /// This is what a daisy chain needs: one port opened once, and one link per
    /// servo id on it. Closing a link does not close the port — whoever opened
    /// it decides when the chain goes away.
    ///
    /// `profile` of `None` means "not declared": the servo's own model number
    /// chooses. A caller that hands over a real profile is stating what is on
    /// the bus, and the servo is checked against it. The distinction matters
    /// because a caller with nothing to say must not be recorded as having said
    /// "X-series" — that turns a servo this build could have identified into a
    /// contradiction it refuses to drive.
    pub fn new(port: SharedPort, id: u8, profile: Option<Profile>) -> Self {
        Self {
            driver: Some(Arc::new(Driver::new(port))),
            port_name: String::new(),
            baud: 1_000_000,
            id,
            profile: profile.unwrap_or(crate::profile::X_SERIES),
            profile_declared: profile.is_some(),
            model: None,
            model_name: None,
            current_mode: None,
            torque_enabled: false,
            last_velocity_limit: None,
            last_accel_limit: None,
            last_watchdog: None,
        }
    }

    pub fn with_port_name(mut self, name: &str, baud: u32) -> Self {
        self.port_name = name.to_string();
        self.baud = baud;
        self
    }

    /// Sets the torque constant, so torque can be reported in newton-metres.
    pub fn with_torque_constant(mut self, nm_per_amp: f64) -> Self {
        self.profile.torque_constant = Some(nm_per_amp);
        self
    }

    pub fn profile(&self) -> &Profile {
        &self.profile
    }

    fn driver(&self) -> Result<&Arc<Driver>> {
        self.driver.as_ref().ok_or(Error::Closed)
    }

    /// Settles which control table this servo is actually driven through, now
    /// that it has said what it is.
    ///
    /// The model number arrives on every ping, so the profile never has to be
    /// guessed. It matters because the profile carries the unit scales: an
    /// XL330 counts current in 1 mA and an XM430 in 2.69 mA, and the wrong one
    /// is not a visible failure — the servo still moves, every current reading
    /// is simply out by a factor of nearly three, and the voltage range belongs
    /// to a different class of motor.
    ///
    /// Three cases, and the middle one is the reason this exists:
    ///
    /// * Nothing declared, model recognised: use the model's own profile. This
    ///   is the ordinary path, and it means a config file that forgets to name
    ///   a profile is still driven correctly.
    /// * Declared, and the model says otherwise: refuse. One of the two is
    ///   wrong, and carrying on means reporting numbers scaled for a motor that
    ///   is not the one on the bus.
    /// * Model not in this build's table: leave the profile as it stands, and
    ///   report the board as unconfirmed unless the caller named one — so the
    ///   choice is acknowledged rather than assumed.
    fn resolve_profile(&mut self, model: u32) -> Result<()> {
        self.model = Some(model);

        let known = match known_model(model) {
            Some(known) => known,
            None => return Ok(()),
        };
        self.model_name = Some(known.name);

        let expected = match profile_named(known.profile) {
            Some(expected) => expected,
            None => return Ok(()),
        };
        if !self.profile_declared {
            // Anything the caller set explicitly — a torque constant, an
            // encoder override — survives the swap.
            let torque_constant = self.profile.torque_constant;
            self.profile = expected;
            self.profile.torque_constant = torque_constant.or(expected.torque_constant);
            return Ok(());
        }
        if self.profile.name != expected.name {
            return Err(Error::Config(format!(
                "servo id {} reports model {model} ({}), which needs the {:?} profile, but {:?} \
                 was configured; the profiles disagree on current scale and supply range, so every \
                 reading from one of them would be wrong",
                self.id, known.name, known.profile, self.profile.name
            )));
        }
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<()> {
        if self.torque_enabled == enabled && self.current_mode.is_some() {
            return Ok(());
        }
        let address = self.profile.addr_torque_enable;
        self.driver()?
            .write_u8(self.id, address, u8::from(enabled))?;
        self.torque_enabled = enabled;
        Ok(())
    }

    /// Switches the operating mode if needed.
    ///
    /// The mode register is in EEPROM and is rejected while torque is enabled,
    /// so the switch is torque-off, write, settle. It is slow — hundreds of
    /// milliseconds — which is exactly why it is done only when the mode
    /// actually changes.
    fn ensure_mode(&mut self, mode: u8) -> Result<()> {
        if self.current_mode == Some(mode) {
            return Ok(());
        }
        self.set_torque(false)?;
        let address = self.profile.addr_operating_mode;
        self.driver()?.write_u8(self.id, address, mode)?;
        std::thread::sleep(EEPROM_SETTLE);
        self.current_mode = Some(mode);

        // A mode change invalidates the profile registers, so they are re-sent.
        self.last_velocity_limit = None;
        self.last_accel_limit = None;
        Ok(())
    }

    /// Writes the profile velocity, acceleration and bus watchdog, but only
    /// when they change or when the caller asked for a full refresh.
    fn apply_limits(&mut self, command: &Command) -> Result<()> {
        if let Some(limit) = command.velocity_limit.get() {
            let limit = limit.value();
            if command.full || self.last_velocity_limit != Some(limit) {
                let counts = (limit * 60.0 / self.profile.velocity_rpm_per_count).round();
                let address = self.profile.addr_profile_velocity;
                self.driver()?
                    .write_u32(self.id, address, counts.max(0.0) as u32)?;
                self.last_velocity_limit = Some(limit);
            }
        }

        if let Some(limit) = command.accel_limit.get() {
            let limit = limit.value();
            if command.full || self.last_accel_limit != Some(limit) {
                // rev/s² to rev/min², then into the register's own step.
                let counts = (limit * 3600.0 / self.profile.accel_rev_per_min2_per_count).round();
                let address = self.profile.addr_profile_accel;
                self.driver()?
                    .write_u32(self.id, address, counts.max(0.0) as u32)?;
                self.last_accel_limit = Some(limit);
            }
        }

        if self.profile.addr_bus_watchdog != 0 {
            if let Some(timeout) = command.watchdog.get() {
                if command.full || self.last_watchdog != Some(timeout) {
                    let counts = watchdog_counts(timeout, &self.profile);
                    let address = self.profile.addr_bus_watchdog;
                    self.driver()?.write_u8(self.id, address, counts)?;
                    self.last_watchdog = Some(timeout);
                }
            }
        }
        Ok(())
    }

    fn operating_mode_for(&self, mode: Mode) -> Result<u8> {
        Ok(match mode {
            // Extended position mode, so a multi-turn target is not silently
            // wrapped into a single revolution. The framework's position unit
            // is revolutions with no implied range, and the single-turn mode
            // would quietly break that contract past ±0.5 rev.
            Mode::Position => OP_MODE_EXTENDED_POSITION,
            Mode::Velocity | Mode::Hold => OP_MODE_VELOCITY,
            Mode::Torque => OP_MODE_CURRENT,
            other => {
                return Err(Error::UnsupportedMode {
                    driver: "dynamixel",
                    mode: other.control(),
                })
            }
        })
    }

    /// Writes whatever this command changes.
    fn apply_command(&mut self, command: &Command) -> Result<()> {
        match command.mode {
            // Torque off is the Dynamixel equivalent of releasing the bridge.
            Mode::Stop => return self.set_torque(false),
            Mode::Query => return Ok(()),
            Mode::Brake | Mode::StayWithin { .. } | Mode::SetOutput { .. } => {
                return Err(Error::UnsupportedMode {
                    driver: "dynamixel",
                    mode: command.mode.control(),
                })
            }
            _ => {}
        }

        let operating_mode = self.operating_mode_for(command.mode)?;
        self.ensure_mode(operating_mode)?;
        self.apply_limits(command)?;
        self.set_torque(true)?;

        let driver = self.driver()?.clone();
        match command.mode {
            Mode::Position => {
                if let Some(position) = command.position.get() {
                    let counts = (position.value() * self.profile.counts_per_rev).round() as i32;
                    driver.write_u32(self.id, self.profile.addr_goal_position, counts as u32)?;
                }
            }
            Mode::Velocity => {
                if let Some(velocity) = command.velocity.get() {
                    let counts = (velocity.value() * 60.0 / self.profile.velocity_rpm_per_count)
                        .round() as i32;
                    driver.write_u32(self.id, self.profile.addr_goal_velocity, counts as u32)?;
                }
            }
            Mode::Hold => {
                // There is no zero-velocity mode; commanding zero velocity in
                // velocity mode is the closest equivalent and behaves the same.
                driver.write_u32(self.id, self.profile.addr_goal_velocity, 0)?;
            }
            Mode::Torque => {
                if let Some(torque) = command.feedforward_torque.get() {
                    let constant = self.profile.torque_constant.ok_or_else(|| {
                        Error::Config(
                            "torque control needs a torque constant; set torque_constant (Nm/A)"
                                .into(),
                        )
                    })?;
                    let amps = torque.value() / constant;
                    let counts = (amps / self.profile.current_amps_per_count).round() as i16;
                    driver.write_u16(self.id, self.profile.addr_goal_current, counts as u16)?;
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Reads the present-value block in as few transactions as the profile
    /// allows.
    ///
    /// The registers from present current through present temperature are
    /// contiguous, so the wide read is one round trip rather than five — which
    /// on a serial bus is the difference between a 200 Hz loop and a 40 Hz one.
    fn read_telemetry(&self, profile: QueryProfile) -> Result<Option<Telemetry>> {
        // Present current (2), velocity (4), position (4) = 10 bytes from 126.
        let length = if profile == QueryProfile::Fast {
            10
        } else {
            // …through present temperature at 146 = 21 bytes.
            self.profile.addr_present_temperature + 1 - self.profile.addr_present_current
        };

        let driver = self.driver()?;
        let data = match driver.read(self.id, self.profile.addr_present_current, length) {
            Ok(data) => data,
            // A single missed reply is a dropped cycle, not a broken link: the
            // loop counts it and carries on, and only a run of them is fatal.
            Err(Error::Timeout(_)) => return Ok(None),
            Err(error) => return Err(error),
        };
        if data.len() < 10 {
            return Ok(None);
        }

        let mut telemetry = Telemetry::default();

        let current =
            i16::from_le_bytes([data[0], data[1]]) as f64 * self.profile.current_amps_per_count;
        let velocity = i32::from_le_bytes([data[2], data[3], data[4], data[5]]) as f64
            * self.profile.velocity_rpm_per_count
            / 60.0;
        let position = i32::from_le_bytes([data[6], data[7], data[8], data[9]]) as f64
            / self.profile.counts_per_rev;

        telemetry.q_current = Some(Amps(current));
        telemetry.velocity = Some(RevPerSec(velocity));
        telemetry.position = Some(Rev(position));
        telemetry.torque = self
            .profile
            .torque_constant
            .map(|constant| Nm(current * constant));

        // The motor has no mode register in the moteus sense; report whether
        // the bridge is driving, which is what the safety layer actually asks.
        let (mode, name) = if self.torque_enabled {
            (1, "Driving")
        } else {
            (0, "Stopped")
        };
        telemetry.mode = Some(mode);
        telemetry.mode_name = name.into();

        if length > 10 && data.len() >= length as usize {
            let voltage_offset =
                (self.profile.addr_present_voltage - self.profile.addr_present_current) as usize;
            let temp_offset = (self.profile.addr_present_temperature
                - self.profile.addr_present_current) as usize;
            let voltage = u16::from_le_bytes([data[voltage_offset], data[voltage_offset + 1]])
                as f64
                * self.profile.voltage_volts_per_count;
            telemetry.voltage = Some(Volts(voltage));
            // A Dynamixel is an integrated servo: the only temperature it has
            // is the one inside the case, and it stands for both.
            let temperature = Celsius(data[temp_offset] as f64);
            telemetry.motor_temperature = Some(temperature);
            telemetry.board_temperature = Some(temperature);
        }

        if profile != QueryProfile::Fast {
            if let Ok(status) = driver.read(self.id, self.profile.addr_hardware_error, 1) {
                if status.len() == 1 {
                    telemetry.fault = Some(decode_hardware_error(status[0]));
                }
            }
        }

        Ok(Some(telemetry))
    }
}

/// The bus watchdog register, in its own counts.
///
/// It is one byte and rounds down to zero for short timeouts, where zero means
/// "no watchdog at all" — the opposite of what a short timeout is asking for.
/// Clamping into the representable range is the only safe reading of the
/// request.
fn watchdog_counts(seconds: f64, profile: &Profile) -> u8 {
    let counts = (seconds / profile.watchdog_seconds_per_count).round() as i32;
    counts.clamp(1, profile.watchdog_max_count) as u8
}

impl Link for DynamixelLink {
    fn kind(&self) -> &'static str {
        "dynamixel"
    }

    fn describe(&self) -> Vec<(String, String)> {
        vec![
            ("driver".into(), "dynamixel".into()),
            ("port".into(), self.port_name.clone()),
            ("baud".into(), self.baud.to_string()),
            ("id".into(), self.id.to_string()),
            ("profile".into(), self.profile.name.into()),
        ]
    }

    fn open(&mut self) -> Result<()> {
        if self.id > MAX_VALID_MOTOR_ID {
            return Err(Error::Config(format!(
                "servo id {} is out of range (0..{MAX_VALID_MOTOR_ID})",
                self.id
            )));
        }

        let driver = self.driver()?.clone();
        driver.tune_read_timeout();

        let model = driver.ping(self.id).map_err(|error| {
            Error::Refused(format!(
                "no reply from servo id {} on {} at {} baud: {error}",
                self.id, self.port_name, self.baud
            ))
        })?;
        self.resolve_profile(model as u32)?;

        // Clear a latched bus watchdog before anything else is attempted.
        //
        // The watchdog is the servo's own safety net: set it, stop sending, and
        // the motor stops on its own. When it fires it latches — the register
        // reads 0xFF — and while it is latched the servo refuses every goal
        // register with an access error, which is exactly the state a session
        // that was killed while armed leaves behind.
        //
        // Without this, the next bring-up connects, identifies the servo, arms
        // it, and then e-stops on the first command with error 0x07, and stays
        // that way until somebody writes zero to a register they have never
        // heard of. Taking control of a servo means clearing what the last
        // session left, the same way the moteus driver forgets its sticky
        // registers on connect.
        driver
            .write_u8(self.id, self.profile.addr_bus_watchdog, 0)
            .map_err(|error| Error::Refused(format!("clearing the bus watchdog: {error}")))?;
        self.last_watchdog = None;

        // Leave the motor free until the caller arms it, matching every other
        // driver: connecting is not a reason for a machine to become rigid.
        self.set_torque(false)?;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        if self.driver.is_some() {
            let _ = self.set_torque(false);
            self.driver = None;
        }
        Ok(())
    }

    fn identify(&mut self) -> Result<Option<Identity>> {
        let driver = self.driver()?.clone();
        let mut identity = Identity::new("dynamixel");

        let model = driver.ping(self.id)?;
        identity.model_number = Some(model as u32);
        identity.board = match self.model_name {
            Some(name) => format!("Dynamixel {name} ({} control table)", self.profile.name),
            None => format!(
                "Dynamixel model {model} ({} control table, not confirmed)",
                self.profile.name
            ),
        };

        if let Ok(data) = driver.read(self.id, self.profile.addr_firmware_version, 1) {
            if data.len() == 1 {
                identity.firmware = data[0].to_string();
            }
        }

        // A servo this build recognises, or one whose profile the operator
        // named, is driven through a control table somebody stands behind.
        // Anything else is a guess, and saying so is the difference between an
        // operator who knows the current readings are unverified and one who
        // trusts them.
        identity.status = if self.model_name.is_some() || self.profile_declared {
            BoardStatus::Supported
        } else {
            BoardStatus::Unknown
        };

        // A Dynamixel has no configurable travel envelope equivalent to the one
        // a moteus enforces — position limits exist but only apply in the
        // single-turn modes — so no bounds are reported and the host envelope
        // is the only one in force.
        Ok(Some(identity))
    }

    fn capabilities(&self) -> Capabilities {
        Capabilities {
            modes: vec![
                ControlMode::Stop,
                ControlMode::Query,
                ControlMode::Position,
                ControlMode::Velocity,
                ControlMode::Torque,
                ControlMode::Hold,
            ],
            // A Protocol 2.0 bus at 1 Mbit/s completes a write-plus-read for
            // one servo well inside a millisecond, but the serial round trip
            // and the host's scheduling put the practical ceiling far below.
            max_rate_hz: 200,
            has_watchdog: self.profile.addr_bus_watchdog != 0,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }

    fn envelope(&self) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(self.profile.voltage_min)),
            voltage_max: Some(Volts(self.profile.voltage_max)),
            voltage_rated: Some(Volts(self.profile.voltage_rated)),
            velocity_max: Some(RevPerSec(self.profile.max_velocity_rev_per_sec)),
            peak_current: None,
        })
    }

    fn cycle(&mut self, command: &Command, profile: QueryProfile) -> Result<Option<Telemetry>> {
        self.apply_command(command)?;
        self.read_telemetry(profile)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{XL330, X_SERIES};

    fn link(profile: Option<Profile>) -> DynamixelLink {
        struct Silent;
        impl drlabs_transport::serial::SerialPort for Silent {
            fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
                Ok(0)
            }
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                Ok(buf.len())
            }
        }
        DynamixelLink::new(SharedPort::new(Box::new(Silent)), 1, profile)
    }

    /// The ordinary path: a config that names no profile is still driven
    /// correctly, because the servo says what it is on every ping.
    #[test]
    fn an_undeclared_profile_is_taken_from_the_model_number() {
        let mut link = link(None);
        assert_eq!(link.profile.name, X_SERIES.name, "provisional until asked");

        link.resolve_profile(1200).unwrap();
        assert_eq!(link.profile.name, XL330.name);
        assert_eq!(link.profile.current_amps_per_count, 0.001);
        assert_eq!(link.model_name, Some("XL330-M288-T"));
    }

    /// A regression worth pinning. Passing a default profile as though it were
    /// a declaration turned every profile-less config into a contradiction the
    /// driver refused to open — the servo said XL330, the record said X-series,
    /// and nothing on the bench had actually claimed anything.
    #[test]
    fn a_caller_with_nothing_to_say_is_not_recorded_as_having_said_x_series() {
        let mut link = link(None);
        assert!(!link.profile_declared);
        assert!(
            link.resolve_profile(1200).is_ok(),
            "an undeclared profile must never contradict the servo"
        );
    }

    /// But a caller who did state what is on the bus is checked against it.
    /// One of the two is wrong, and carrying on means reporting numbers scaled
    /// for a motor that is not there.
    #[test]
    fn a_declared_profile_that_contradicts_the_servo_is_refused() {
        let mut link = link(Some(X_SERIES));
        let error = link.resolve_profile(1200).unwrap_err();
        let text = format!("{error}");
        assert!(text.contains("XL330-M288-T"), "{text}");
        assert!(text.contains("current scale"), "{text}");
    }

    #[test]
    fn a_model_this_build_does_not_know_leaves_the_profile_alone() {
        let mut link = link(None);
        link.resolve_profile(9999).unwrap();
        assert_eq!(link.profile.name, X_SERIES.name);
        assert_eq!(link.model_name, None);
    }

    /// A torque constant the caller set survives the profile being replaced by
    /// the servo's own — it is a property of the motor, not of the family.
    #[test]
    fn an_explicit_torque_constant_survives_profile_resolution() {
        let mut link = link(None).with_torque_constant(0.35);
        link.resolve_profile(1200).unwrap();
        assert_eq!(link.profile.torque_constant, Some(0.35));
        assert_eq!(link.profile.name, XL330.name);
    }

    /// The watchdog register is one byte. A 5 ms timeout rounds to zero, and
    /// zero means "no watchdog" — the exact opposite of the request.
    #[test]
    fn a_short_watchdog_clamps_rather_than_rounding_to_off() {
        assert_eq!(watchdog_counts(0.005, &X_SERIES), 1);
        assert_eq!(watchdog_counts(0.0, &X_SERIES), 1);
        assert_eq!(watchdog_counts(0.100, &X_SERIES), 5);
        // And the other end saturates rather than wrapping to a short one.
        assert_eq!(watchdog_counts(60.0, &X_SERIES), 127);
    }

    /// What a Dynamixel cannot do is part of the contract, and is reported
    /// rather than approximated.
    #[test]
    fn the_modes_a_dynamixel_lacks_are_refused_by_name() {
        let link = link(None);
        assert!(!link.capabilities().supports(ControlMode::StayWithin));
        assert!(!link.capabilities().supports(ControlMode::Brake));
        assert!(link.capabilities().supports(ControlMode::Position));

        let error = link.operating_mode_for(Mode::Brake).unwrap_err();
        assert!(matches!(error, Error::UnsupportedMode { .. }), "{error}");
    }

    /// The envelope is what let a healthy 5 V servo be armed at all: without a
    /// rated voltage the safety layer keeps a band inherited from a 12 V bus,
    /// which excludes 4.8 V outright.
    #[test]
    fn the_envelope_carries_the_rated_supply() {
        let mut link = link(None);
        link.resolve_profile(1200).unwrap();
        let envelope = link.envelope().expect("a known family has an envelope");
        assert_eq!(envelope.voltage_rated, Some(Volts(5.0)));
        assert_eq!(envelope.voltage_min, Some(Volts(3.7)));
    }
}
