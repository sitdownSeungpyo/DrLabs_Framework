//! One moteus, as the rest of the framework sees it.
//!
//! Everything above this file is in framework units and vendor-neutral modes;
//! everything below is registers. This is where the translation happens, and
//! where the frame budget is spent.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use drlabs_core::command::{Command, ControlMode, Field, Mode, QueryProfile, TorqueLimit};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::{ActiveLimits, Telemetry};
use drlabs_core::units::{Amps, Celsius, Nm, Rev, RevPerSec, RevPerSec2, Volts};
use drlabs_transport::can::{SharedBus, MAX_FD_PAYLOAD};

use crate::controller::Controller;
use crate::frames::{self, PositionCommand, Send, StayWithinCommand};
use crate::hardware;
use crate::multiplex::Resolution;
use crate::query::QueryResolution;
use crate::registers::{self as reg, Register};
use crate::stream::Stream;

/// A [`Link`] backed by one moteus controller.
pub struct MoteusLink {
    controller: Controller,
    bus: Option<Arc<SharedBus>>,

    fast: QueryResolution,
    full: QueryResolution,
    verify: QueryResolution,

    /// What the controller was last told, so an unchanged parameter costs no
    /// frame budget.
    sent: SentParams,
}

/// The command registers that keep their value until they are written again.
///
/// Re-sending them every cycle is not merely wasteful, it is the difference
/// between a frame that fits and one that does not: the full parameter set is
/// about 48 bytes, a wide telemetry query is another 18, and a CAN-FD frame
/// holds 64. The controller answers an over-long frame by not receiving it.
#[derive(Debug, Clone, Copy, Default)]
struct SentParams {
    valid: bool,

    feedforward_torque: f64,
    kp_scale: f64,
    kd_scale: f64,
    ilimit_scale: f64,
    velocity_limit: f64,
    accel_limit: f64,
    stop_position: f64,
    ignore_position_bounds: f64,
}

impl SentParams {
    /// Forgets what the controller was told, so the next command re-sends
    /// everything.
    ///
    /// Used on connect, when the command registers hold whatever a previous
    /// session or a power cycle left in them.
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// Returns a field only when it has to go out: on a full refresh, or when
    /// the value differs from what the controller was last told.
    fn sticky(last: &mut f64, value: Option<f64>, full: bool) -> Send {
        let value = match value {
            Some(value) => value,
            None => return Send::No,
        };
        if !full && *last == value {
            return Send::No;
        }
        *last = value;
        Send::Value(value)
    }
}

impl MoteusLink {
    /// A link on a bus the caller already owns.
    ///
    /// Closing this link does not close that bus — whoever opened it keeps the
    /// responsibility, which is what makes it possible to drive several
    /// controllers over one adapter.
    pub fn new(bus: Arc<SharedBus>, id: u8) -> Self {
        Self {
            controller: Controller::new(id),
            bus: Some(bus),
            fast: QueryResolution::fast(),
            full: QueryResolution::full(),
            verify: QueryResolution::verify(),
            sent: SentParams::default(),
        }
    }

    /// The low-level interface, for callers that need registers or the console.
    pub fn controller(&self) -> &Controller {
        &self.controller
    }

    pub fn controller_mut(&mut self) -> &mut Controller {
        &mut self.controller
    }

    pub fn bus(&self) -> Option<&Arc<SharedBus>> {
        self.bus.as_ref()
    }

    fn require_bus(&self) -> Result<&Arc<SharedBus>> {
        self.bus.as_ref().ok_or(Error::Closed)
    }

    fn query_for(&self, profile: QueryProfile) -> &QueryResolution {
        match profile {
            QueryProfile::Full => &self.full,
            QueryProfile::Verify => &self.verify,
            QueryProfile::Fast => &self.fast,
        }
    }

    /// The widest query that still fits alongside a command of the given size,
    /// narrowing to the fast set and then to nothing.
    ///
    /// The frame budget belongs to this layer, because this is the layer that
    /// knows both halves of it. Leaving it to the caller means every caller has
    /// to know the size of every parameter set — and the one that gets it wrong
    /// produces a frame the adapter simply refuses, which surfaces as a dead
    /// link rather than as missing telemetry.
    ///
    /// Narrowing costs a cycle of the wider registers, which the next rotation
    /// picks up. Overflowing costs the connection.
    fn fit_query<'a>(
        &'a self,
        command_bytes: usize,
        requested: &'a QueryResolution,
    ) -> Option<&'a QueryResolution> {
        if command_bytes + requested.encoded_len() <= MAX_FD_PAYLOAD {
            return Some(requested);
        }
        if command_bytes + self.fast.encoded_len() <= MAX_FD_PAYLOAD {
            return Some(&self.fast);
        }
        // A command this large leaves no room to ask anything. It still has to
        // go out; the cycle simply reports no telemetry.
        None
    }

    /// Renders one command with no telemetry query attached.
    fn command_payload(&mut self, command: &Command) -> Result<Vec<u8>> {
        let resolutions = self.controller.position_resolution;
        Ok(match command.mode {
            Mode::Stop => frames::stop(None),
            Mode::Query => Vec::new(),
            Mode::Brake => frames::brake(None),
            Mode::Hold => frames::zero_velocity(field_send(command.kd_scale), &resolutions, None),
            Mode::SetOutput { position } => {
                frames::set_output(reg::SET_OUTPUT_EXACT, position.value(), None)
            }
            Mode::StayWithin { lower, upper } => {
                frames::stay_within(&self.stay_within(command, lower, upper), &resolutions, None)
            }
            Mode::Position | Mode::Velocity | Mode::Torque => {
                let position = self.position(command);
                frames::position(&position, &resolutions, None)
            }
        })
    }

    /// Translates a framework command into moteus position-mode fields.
    ///
    /// Velocity and torque modes both ride on position mode: a velocity command
    /// is a position command with a NaN target, and a torque command is one
    /// with no position or velocity opinion and a feedforward torque. That is
    /// how the controller itself is built, so expressing it any other way would
    /// mean fighting the firmware.
    fn position(&mut self, command: &Command) -> PositionCommand {
        // Position, velocity, the torque ceiling and the watchdog go out every
        // cycle: the first two are the command, and the last two are the limits
        // that must never be left to chance. Everything else is sticky, and is
        // sent when it changes or when a full refresh is asked for.
        let full = command.full || !self.sent.valid;
        self.sent.valid = true;
        let sent = &mut self.sent;

        let mut out = PositionCommand {
            position: field_send(command.position.get().map(|p| p.value()).into_field()),
            velocity: field_send(command.velocity.get().map(|v| v.value()).into_field()),
            max_torque: torque_limit(command.max_torque),
            watchdog: field_send(command.watchdog),
            feedforward_torque: SentParams::sticky(
                &mut sent.feedforward_torque,
                command.feedforward_torque.get().map(|t| t.value()),
                full,
            ),
            kp_scale: SentParams::sticky(&mut sent.kp_scale, command.kp_scale.get(), full),
            kd_scale: SentParams::sticky(&mut sent.kd_scale, command.kd_scale.get(), full),
            stop_position: SentParams::sticky(
                &mut sent.stop_position,
                command.stop_position.get().map(|p| p.value()),
                full,
            ),
            velocity_limit: SentParams::sticky(
                &mut sent.velocity_limit,
                command.velocity_limit.get().map(|v| v.value()),
                full,
            ),
            accel_limit: SentParams::sticky(
                &mut sent.accel_limit,
                command.accel_limit.get().map(|a| a.value()),
                full,
            ),
            ilimit_scale: SentParams::sticky(
                &mut sent.ilimit_scale,
                command.ilimit_scale.get(),
                full,
            ),
            ignore_position_bounds: SentParams::sticky(
                &mut sent.ignore_position_bounds,
                Some(if command.ignore_position_bounds {
                    1.0
                } else {
                    0.0
                }),
                full,
            ),
        };

        match command.mode {
            Mode::Velocity => {
                // A NaN position must be *sent*, not omitted. It is what tells
                // the controller there is no position target, so the axis holds
                // the commanded velocity; omitting the register would leave the
                // previous position command in force and the axis would servo
                // back to it.
                out.position = Send::Value(f64::NAN);
                if command.velocity.get().is_none() {
                    out.velocity = Send::Value(0.0);
                }
            }
            Mode::Torque => {
                // Neither a position nor a velocity opinion: the feedforward
                // torque is the whole command.
                out.position = Send::Value(f64::NAN);
                out.velocity = Send::Value(f64::NAN);
            }
            _ => {}
        }
        out
    }

    fn stay_within(&self, command: &Command, lower: Rev, upper: Rev) -> StayWithinCommand {
        StayWithinCommand {
            lower: Send::Value(lower.value()),
            upper: Send::Value(upper.value()),
            feedforward_torque: field_send(
                command
                    .feedforward_torque
                    .get()
                    .map(|t| t.value())
                    .into_field(),
            ),
            kp_scale: field_send(command.kp_scale),
            kd_scale: field_send(command.kd_scale),
            max_torque: torque_limit(command.max_torque),
            watchdog: field_send(command.watchdog),
            ilimit_scale: field_send(command.ilimit_scale),
            ignore_position_bounds: Send::Value(if command.ignore_position_bounds {
                1.0
            } else {
                0.0
            }),
        }
    }
}

/// Bridges `Field<f64>` to the driver's `Send`.
fn field_send(field: Field<f64>) -> Send {
    match field {
        Field::Unchanged => Send::No,
        Field::Set(value) => Send::Value(value),
    }
}

/// Resolves the three ways a command can talk about the torque ceiling.
fn torque_limit(limit: TorqueLimit) -> Send {
    match limit {
        TorqueLimit::Unchanged => Send::No,
        TorqueLimit::Nm(value) => Send::Value(value.value()),
        // Transmitted, and transmitted as NaN: that is the sentinel that hands
        // the ceiling back to the controller's own configuration.
        TorqueLimit::Controller => Send::Value(f64::NAN),
    }
}

/// A small convenience so `Option<f64>` reads as a `Field` at the call site.
trait IntoField {
    fn into_field(self) -> Field<f64>;
}

impl IntoField for Option<f64> {
    fn into_field(self) -> Field<f64> {
        match self {
            Some(value) => Field::Set(value),
            None => Field::Unchanged,
        }
    }
}

impl Link for MoteusLink {
    fn kind(&self) -> &'static str {
        "moteus"
    }

    fn describe(&self) -> Vec<(String, String)> {
        let mut out = vec![
            ("driver".into(), "moteus".into()),
            ("can_id".into(), self.controller.id.to_string()),
        ];
        if let Some(bus) = &self.bus {
            out.extend(bus.lock().describe());
        }
        out
    }

    /// Acquires the transport and leaves the controller stopped.
    ///
    /// The opening stop is not a formality. A controller left driving by a
    /// previous session keeps driving until it is told otherwise, and its
    /// command registers still hold that session's limits — so the connect
    /// sequence stops the axis and drains whatever was already queued before
    /// anything else.
    fn open(&mut self) -> Result<()> {
        let bus = self.require_bus()?.clone();
        bus.lock()
            .drain(Duration::from_millis(20))
            .map_err(|error| Error::Transport {
                operation: "drain",
                source: Box::new(error),
            })?;

        // Nothing may be assumed to be already set.
        self.sent.reset();

        let payload = frames::stop(Some(&self.fast));
        match self.controller.transact(&bus, &payload, true)? {
            Some(_) => Ok(()),
            None => Err(Error::Refused(format!(
                "no reply from CAN id {} — check the id, the bus wiring and termination, and that \
                 the controller is powered",
                self.controller.id
            ))),
        }
    }

    /// Stops the motor. The bus belongs to whoever opened it.
    fn close(&mut self) -> Result<()> {
        if let Some(bus) = self.bus.take() {
            // Leave the controller in the safest state it has, but do not let a
            // failure here prevent the caller from tearing down.
            let _ = self.controller.transact(&bus, &frames::stop(None), false);
        }
        Ok(())
    }

    fn identify(&mut self) -> Result<Option<Identity>> {
        let bus = self.require_bus()?.clone();
        let mut identity = Identity::new("moteus");

        let mut want = BTreeMap::new();
        want.insert(reg::MODEL_NUMBER, Resolution::Int32);
        want.insert(reg::FIRMWARE_VERSION, Resolution::Int32);
        if let Ok(values) = self.controller.read_registers(&bus, &want) {
            if let Some(&model) = values.get(&reg::MODEL_NUMBER) {
                if model.is_finite() && model != 0.0 {
                    identity.model_number = Some(model as u32);
                }
            }
            if let Some(&version) = values.get(&reg::FIRMWARE_VERSION) {
                if version.is_finite() {
                    identity.firmware = format_firmware(version as u32);
                }
            }
        }

        let mut want = BTreeMap::new();
        for register in [
            reg::SERIAL_NUMBER_1,
            reg::SERIAL_NUMBER_2,
            reg::SERIAL_NUMBER_3,
        ] {
            want.insert(register, Resolution::Int32);
        }
        if let Ok(values) = self.controller.read_registers(&bus, &want) {
            identity.serial = format_serial(&values);
        }

        // The console is slow, so it is read once here and never inside the
        // control loop.
        //
        // The order matters. A board that was emitting telemetry when we
        // connected is filling the same channel that carries command responses,
        // so the reply to "tel stop" arrives interleaved with binary records and
        // frequently cannot be parsed at all. The command is issued for its
        // effect and its result is discarded; what makes the channel usable is
        // draining on both sides of it.
        {
            let mut console = Stream::new(&self.controller, &bus);
            console.drain();
            let _ = console.command("tel stop");

            // If the console will not go quiet, stop here rather than reading it
            // anyway. Every console read is a read of "whatever comes next", so
            // a controller that is still emitting answers each question with
            // somebody else's output — and a travel bound invented that way is
            // worse than an absent one, because the preflight check would then
            // trust it.
            if console.drain() {
                // The board family and revision have no register; they live only
                // in the firmware telemetry record. Without them the board cannot
                // be checked against a known envelope, and every connection has to
                // be waved through by hand — which trains people to wave through
                // the ones that matter.
                if let Ok(firmware) = console.read_firmware() {
                    identity.family = firmware.family;
                    identity.hardware_revision = firmware.hardware_revision;
                    if let Some(model) = firmware.model {
                        if model >= 0 {
                            identity.model_number = Some(model as u32);
                        }
                    }
                    if identity.firmware.is_empty() {
                        if let Some(version) = firmware.version {
                            if version >= 0 {
                                identity.firmware = format_firmware(version as u32);
                            }
                        }
                    }
                }

                let lower = console
                    .conf_get_float("servopos.position_min")
                    .ok()
                    .flatten();
                let upper = console
                    .conf_get_float("servopos.position_max")
                    .ok()
                    .flatten();
                if let (Some(lower), Some(upper)) = (lower, upper) {
                    identity.bounds = Some((Rev(lower), Rev(upper)));
                }
            }
        }

        identity.board = hardware::board_name(identity.family, identity.hardware_revision);
        identity.status = hardware::board_status(identity.family, identity.hardware_revision);
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
                ControlMode::Brake,
                ControlMode::StayWithin,
                ControlMode::SetOutput,
            ],
            max_rate_hz: hardware::CAN_DOCUMENTED_MAX_CYCLE_HZ,
            has_watchdog: true,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }

    fn envelope(&self) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(hardware::R4_ENVELOPE.input_voltage_min)),
            voltage_max: Some(Volts(hardware::R4_ENVELOPE.input_voltage_max)),
            voltage_rated: Some(Volts(hardware::R4_ENVELOPE.input_voltage_nominal)),
            velocity_max: Some(RevPerSec(hardware::R4_ENVELOPE.velocity_max)),
            peak_current: Some(Amps(hardware::R4_ENVELOPE.peak_phase_current)),
        })
    }

    fn cycle(&mut self, command: &Command, profile: QueryProfile) -> Result<Option<Telemetry>> {
        let bus = self.require_bus()?.clone();

        // Build the command first, then choose a query that fits beside it. The
        // two share one 64-byte frame, and the command is not negotiable.
        let mut payload = self.command_payload(command)?;

        let requested = self.query_for(profile).clone();
        let query = self.fit_query(payload.len(), &requested);
        let expect_reply = query.is_some();
        if let Some(query) = query {
            let (data, _) = query.encode();
            payload.extend_from_slice(&data);
        }

        match self.controller.transact(&bus, &payload, expect_reply)? {
            Some(values) => Ok(Some(decode(&values))),
            None => Ok(None),
        }
    }
}

/// Turns a register map into framework-unit telemetry.
pub fn decode(values: &crate::controller::Values) -> Telemetry {
    let mut telemetry = Telemetry::default();

    let scaled = |register: Register| -> Option<f64> {
        values
            .get(&register)
            .map(|value| reg::scale_value(register, value.raw, value.resolution))
            .filter(|value| value.is_finite())
    };
    let raw = |register: Register| -> Option<i32> {
        values
            .get(&register)
            .map(|value| value.raw)
            .filter(|value| value.is_finite())
            .map(|value| value as i32)
    };

    if let Some(mode) = raw(reg::MODE) {
        telemetry.mode = Some(mode);
        telemetry.mode_name = reg::ControllerMode(mode).name().to_string();
    }
    if let Some(code) = raw(reg::FAULT) {
        telemetry.fault = Some(hardware::fault_text(code));
    }
    telemetry.trajectory_complete = raw(reg::TRAJECTORY_COMPLETE).map(|value| value != 0);
    telemetry.home_state = raw(reg::HOME_STATE);

    telemetry.position = scaled(reg::POSITION).map(Rev);
    telemetry.velocity = scaled(reg::VELOCITY).map(RevPerSec);
    telemetry.torque = scaled(reg::TORQUE).map(Nm);
    telemetry.q_current = scaled(reg::Q_CURRENT).map(Amps);
    telemetry.d_current = scaled(reg::D_CURRENT).map(Amps);
    telemetry.abs_position = scaled(reg::ABS_POSITION).map(Rev);
    telemetry.power_w = scaled(reg::POWER);
    telemetry.voltage = scaled(reg::VOLTAGE).map(Volts);
    telemetry.board_temperature = scaled(reg::TEMPERATURE).map(Celsius);
    telemetry.motor_temperature = scaled(reg::MOTOR_TEMPERATURE).map(Celsius);

    telemetry.active = ActiveLimits {
        max_torque: scaled(reg::COMMAND_MAX_TORQUE).map(Nm),
        watchdog: scaled(reg::COMMAND_TIMEOUT),
        velocity_limit: scaled(reg::COMMAND_VELOCITY_LIMIT).map(RevPerSec),
        accel_limit: scaled(reg::COMMAND_ACCEL_LIMIT).map(RevPerSec2),
    };

    telemetry
}

/// Renders the packed firmware version register.
fn format_firmware(version: u32) -> String {
    format!(
        "{}.{}.{}",
        (version >> 16) & 0xff,
        (version >> 8) & 0xff,
        version & 0xff
    )
}

/// Joins the three serial number registers.
fn format_serial(values: &BTreeMap<Register, f64>) -> String {
    let parts: Vec<u32> = [
        reg::SERIAL_NUMBER_1,
        reg::SERIAL_NUMBER_2,
        reg::SERIAL_NUMBER_3,
    ]
    .iter()
    .map(|register| {
        values
            .get(register)
            .filter(|value| value.is_finite())
            .map(|&value| value as i32 as u32)
            .unwrap_or(0)
    })
    .collect();

    if parts.iter().all(|&part| part == 0) {
        return String::new();
    }
    parts.iter().map(|part| format!("{part:08x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multiplex::RawValue;

    fn link() -> MoteusLink {
        MoteusLink::new(Arc::new(SharedBus::new(Box::new(NullBus))), 1)
    }

    struct NullBus;
    impl drlabs_transport::can::Bus for NullBus {
        fn send(
            &mut self,
            _frame: &drlabs_transport::can::Frame,
        ) -> drlabs_transport::can::Result<()> {
            Ok(())
        }
        fn recv(
            &mut self,
            _timeout: Duration,
        ) -> drlabs_transport::can::Result<drlabs_transport::can::Frame> {
            Err(drlabs_transport::can::Error::Timeout)
        }
        fn drain(&mut self, _quiet: Duration) -> drlabs_transport::can::Result<()> {
            Ok(())
        }
    }

    /// The point of the sticky set: a parameter that has not changed costs no
    /// frame budget, which is what keeps the command small enough for a query
    /// to fit beside it.
    #[test]
    fn an_unchanged_parameter_is_not_re_sent() {
        let mut link = link();
        let command = Command {
            position: Field::Set(Rev(0.25)),
            velocity_limit: Field::Set(RevPerSec(2.0)),
            accel_limit: Field::Set(RevPerSec2(10.0)),
            kp_scale: Field::Set(1.0),
            ..Command::new(Mode::Position)
        };

        let first = link.command_payload(&command).unwrap();
        let second = link.command_payload(&command).unwrap();
        assert!(
            second.len() < first.len(),
            "the second cycle should be smaller: {} then {}",
            first.len(),
            second.len()
        );
    }

    /// Unless the caller asks for a refresh, which is how a host recovers when
    /// it suspects the controller no longer holds what it was told.
    #[test]
    fn a_full_refresh_re_sends_everything() {
        let mut link = link();
        let mut command = Command {
            position: Field::Set(Rev(0.25)),
            velocity_limit: Field::Set(RevPerSec(2.0)),
            kp_scale: Field::Set(1.0),
            ..Command::new(Mode::Position)
        };

        let first = link.command_payload(&command).unwrap();
        command.full = true;
        let refreshed = link.command_payload(&command).unwrap();
        assert_eq!(first.len(), refreshed.len());
    }

    /// Reconnecting to a controller that a previous session left configured is
    /// exactly when nothing may be assumed.
    #[test]
    fn opening_forgets_what_the_controller_was_told() {
        let mut link = link();
        let command = Command {
            position: Field::Set(Rev(0.25)),
            velocity_limit: Field::Set(RevPerSec(2.0)),
            ..Command::new(Mode::Position)
        };

        let first = link.command_payload(&command).unwrap();
        let _ = link.command_payload(&command).unwrap();
        link.sent.reset();
        let after_reset = link.command_payload(&command).unwrap();
        assert_eq!(first.len(), after_reset.len());
    }

    /// A velocity command has to *transmit* a NaN position. Omitting the
    /// register would leave the last position target in force, and the axis
    /// would servo back to it instead of holding the commanded velocity.
    #[test]
    fn velocity_mode_transmits_a_nan_position() {
        let mut link = link();
        let payload = link
            .command_payload(&Command {
                velocity: Field::Set(RevPerSec(1.0)),
                ..Command::new(Mode::Velocity)
            })
            .unwrap();

        // Register 0x020 at int32: the NaN sentinel is the minimum value.
        let hex: String = payload.iter().map(|b| format!("{b:02X}")).collect();
        assert!(hex.contains("00000080"), "no NaN position in {hex}");
    }

    #[test]
    fn a_deferred_torque_limit_is_transmitted_rather_than_omitted() {
        assert_eq!(torque_limit(TorqueLimit::Unchanged), Send::No);
        assert_eq!(torque_limit(TorqueLimit::Nm(Nm(0.5))), Send::Value(0.5));
        match torque_limit(TorqueLimit::Controller) {
            Send::Value(value) => assert!(value.is_nan()),
            other => panic!("expected a transmitted NaN, got {other:?}"),
        }
    }

    /// The budget rule: a wide query is narrowed rather than allowed to
    /// overrun, because an over-long frame is not received at all.
    #[test]
    fn a_query_that_does_not_fit_is_narrowed() {
        let link = link();
        let full = QueryResolution::full();

        // A command with plenty of room keeps what it asked for.
        assert_eq!(
            link.fit_query(8, &full).map(|q| q.encoded_len()),
            Some(full.encoded_len())
        );

        // One that does not gets the fast set instead.
        let narrowed = link.fit_query(48, &full).expect("the fast set still fits");
        assert_eq!(narrowed.encoded_len(), link.fast.encoded_len());

        // And a command with no room at all asks nothing.
        assert!(link.fit_query(60, &full).is_none());
    }

    #[test]
    fn telemetry_comes_back_in_framework_units() {
        let mut values = crate::controller::Values::new();
        values.insert(
            reg::POSITION,
            RawValue {
                resolution: Resolution::Int32,
                raw: 25000.0,
            },
        );
        values.insert(
            reg::MODE,
            RawValue {
                resolution: Resolution::Int8,
                raw: 10.0,
            },
        );
        values.insert(
            reg::FAULT,
            RawValue {
                resolution: Resolution::Int8,
                raw: 102.0,
            },
        );

        let telemetry = decode(&values);
        assert_eq!(telemetry.mode_name, "Position");
        assert!((telemetry.position.unwrap().value() - 0.25).abs() < 1e-9);
        let fault = telemetry.fault.expect("a fault code was reported");
        assert!(!fault.hard, "limiting is information, not a fault");
    }

    /// A register the controller did not report is absent, not zero. The Go
    /// version left these as NaN and every consumer had to remember to check.
    #[test]
    fn an_unreported_register_is_absent_rather_than_zero() {
        let telemetry = decode(&crate::controller::Values::new());
        assert_eq!(telemetry.position, None);
        assert_eq!(telemetry.voltage, None);
        assert_eq!(telemetry.fault, None);
    }

    #[test]
    fn the_firmware_version_register_reads_as_a_version() {
        assert_eq!(format_firmware(0x0001_0203), "1.2.3");
    }

    #[test]
    fn an_unprogrammed_serial_number_is_empty_rather_than_zeroes() {
        let mut values = BTreeMap::new();
        for register in [
            reg::SERIAL_NUMBER_1,
            reg::SERIAL_NUMBER_2,
            reg::SERIAL_NUMBER_3,
        ] {
            values.insert(register, 0.0);
        }
        assert_eq!(format_serial(&values), "");

        values.insert(reg::SERIAL_NUMBER_1, 1.0);
        assert_eq!(format_serial(&values), "000000010000000000000000");
    }
}
