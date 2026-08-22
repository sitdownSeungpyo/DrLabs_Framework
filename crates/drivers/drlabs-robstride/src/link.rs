//! One RobStride actuator, as the rest of the framework sees it.
//!
//! Two translations happen here.
//!
//! The first is units: RobStride speaks radians and rad/s, the framework speaks
//! revolutions and rev/s.
//!
//! The second is control modes, and each framework mode is mapped onto the
//! controller loop that can actually deliver its contract:
//!
//! | Framework mode | Controller loop | Why |
//! |---|---|---|
//! | Position | cyclic synchronous position | its profile generator honours the commanded velocity limit |
//! | Velocity | the dedicated velocity mode | closes a PI loop and holds a setpoint under load, where MIT's damping term sags |
//! | Torque | operation mode, both gains zero | the feedforward torque is the whole command |
//! | Hold | operation mode, damping only | the controller's documented damping configuration |
//! | Stop, Query | the stop frame | it answers with full telemetry and leaves the motor released |

use std::sync::Arc;
use std::time::Duration;

use drlabs_core::command::{Command, ControlMode, Mode, QueryProfile};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::{ActiveLimits, Telemetry};
use drlabs_core::units::{Celsius, Nm, Rev, RevPerSec, Volts};
use drlabs_transport::can::SharedBus;

use crate::controller::Controller;
use crate::models::Model;
use crate::parameters::{Parameter, RunMode};
use crate::protocol::{ControlCommand, Feedback, POSITION_RANGE};

/// How a position command is expressed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionMode {
    /// The controller's cyclic synchronous position mode, whose own profile
    /// generator respects the commanded velocity limit — the behaviour the
    /// framework's position mode promises.
    Csp,
    /// The MIT-style operation mode, where the gain scales and the feedforward
    /// torque take effect and the axis behaves as a spring-damper. That is what
    /// impedance control needs, and it ignores the velocity limit because the
    /// mode has no trajectory generator.
    Operation,
}

/// A [`Link`] backed by one RobStride actuator.
pub struct RobstrideLink {
    controller: Controller,
    bus: Option<Arc<SharedBus>>,
    model: Model,

    /// What a gain scale of 1.0 maps to.
    kp: f64,
    kd: f64,

    /// The mode believed to be selected in the motor. Switching it costs a
    /// parameter write, so it is tracked rather than re-sent.
    run_mode: Option<RunMode>,
    enabled: bool,

    position_mode: PositionMode,

    /// The values last written, so the writes happen on change rather than
    /// every cycle.
    last_torque_limit: Option<f64>,
    last_velocity_limit: Option<f64>,

    /// Tracks full revolutions beyond the controller's ±4π window, so a
    /// multi-turn axis reports a continuous position instead of wrapping.
    turns: f64,
    last_raw_angle: Option<f64>,
}

impl RobstrideLink {
    /// A link on a bus the caller already owns.
    ///
    /// The model is not optional and has no default on purpose: the scaling
    /// differs by up to a factor of twenty across the family, so a guess would
    /// mean commanding an unknown multiple of the intended torque.
    pub fn new(bus: Arc<SharedBus>, id: u8, model: Model) -> Self {
        Self {
            controller: Controller::new(id, model),
            bus: Some(bus),
            model,
            kp: model.default_kp,
            kd: model.default_kd,
            run_mode: None,
            enabled: false,
            position_mode: PositionMode::Csp,
            last_torque_limit: None,
            last_velocity_limit: None,
            turns: 0.0,
            last_raw_angle: None,
        }
    }

    /// Sets the stiffness and damping a gain scale of 1.0 maps to.
    pub fn with_gains(mut self, kp: f64, kd: f64) -> Self {
        self.kp = kp;
        self.kd = kd;
        self
    }

    pub fn with_position_mode(mut self, mode: PositionMode) -> Self {
        self.position_mode = mode;
        self
    }

    /// The low-level interface, for callers that need the parameter table or
    /// the run modes this adapter does not use.
    pub fn controller(&self) -> &Controller {
        &self.controller
    }

    fn require_bus(&self) -> Result<&Arc<SharedBus>> {
        self.bus.as_ref().ok_or(Error::Closed)
    }

    /// Switches the controller's active loop if needed.
    ///
    /// The motor must be stopped for the switch: changing which loop is in
    /// charge while it is driving hands the output to a controller whose state
    /// has not been initialised for the current position.
    fn ensure_run_mode(&mut self, bus: &SharedBus, mode: RunMode) -> Result<()> {
        if self.run_mode == Some(mode) {
            return Ok(());
        }
        if self.enabled {
            self.controller.stop(bus, false)?;
            self.enabled = false;
        }
        self.controller.set_run_mode(bus, mode)?;
        self.run_mode = Some(mode);
        Ok(())
    }

    fn ensure_enabled(&mut self, bus: &SharedBus) -> Result<()> {
        if self.enabled {
            return Ok(());
        }
        self.controller.enable(bus)?;
        self.enabled = true;
        Ok(())
    }

    /// Writes `limit_torque` when it changes.
    ///
    /// This is the parameter that actually bounds what the machine can do,
    /// whatever the gains are, so it is worth a transaction whenever the
    /// operator moves it — and worth not spending one when they have not.
    fn apply_torque_limit(&mut self, bus: &SharedBus, command: &Command) -> Result<()> {
        let requested = match command.max_torque.nm() {
            Some(value) => value.value(),
            None => return Ok(()),
        };
        let limit = requested.clamp(0.0, self.model.torque_limit_max);
        if !command.full && self.last_torque_limit == Some(limit) {
            return Ok(());
        }
        self.controller
            .write_parameter(bus, Parameter::LIMIT_TORQUE, limit)?;
        self.last_torque_limit = Some(limit);
        Ok(())
    }

    /// Writes `limit_spd` when it changes.
    fn apply_velocity_limit(&mut self, bus: &SharedBus, command: &Command) -> Result<()> {
        let requested = match command.velocity_limit.get() {
            Some(value) => value.to_rad_per_sec().value(),
            None => return Ok(()),
        };
        let limit = requested.clamp(0.0, self.model.speed_limit_max);
        if !command.full && self.last_velocity_limit == Some(limit) {
            return Ok(());
        }
        self.controller
            .write_parameter(bus, Parameter::LIMIT_SPD, limit)?;
        self.last_velocity_limit = Some(limit);
        Ok(())
    }

    /// Drives a position target through cyclic synchronous position mode.
    ///
    /// This is the mapping that honours the framework's contract for position
    /// mode: "drive to this position, respecting the configured velocity
    /// limit". The controller's own profile generator does the limiting, so a
    /// step change in the setpoint produces a bounded move rather than a
    /// full-torque lunge.
    ///
    /// The operation mode cannot do that — it has no trajectory generator, so a
    /// step in the target is a step in the error and the axis accelerates until
    /// the torque limit stops it. That is the right behaviour for impedance
    /// control and the wrong one for "go here at no more than this speed".
    fn apply_csp(&mut self, bus: &SharedBus, command: &Command) -> Result<Option<Feedback>> {
        self.ensure_run_mode(bus, RunMode::PositionCsp)?;
        self.apply_velocity_limit(bus, command)?;
        self.ensure_enabled(bus)?;

        let target = command.position.get().unwrap_or(Rev(0.0)).to_rad().value();
        self.controller
            .write_parameter(bus, Parameter::LOC_REF, target)
    }

    /// Builds the operation-mode frame for a command.
    fn control_command(&self, command: &Command) -> ControlCommand {
        // Gain scales are 0..1 in the framework; here they select a fraction of
        // the configured stiffness rather than of the codec's full scale, which
        // would be far too stiff to use as a default.
        let kp_scale = command.kp_scale.or(1.0);
        let kd_scale = command.kd_scale.or(1.0);

        let mut out = ControlCommand::default();
        match command.mode {
            Mode::Position => {
                // The controller's position field wraps at ±4π, so a target
                // beyond that has to be folded back into the window it can
                // express. The turn counter on the feedback side undoes the
                // fold when reporting.
                out.angle = wrap_angle(command.position.get().unwrap_or(Rev(0.0)).to_rad().value());
                out.kp = self.kp * kp_scale;
                out.kd = self.kd * kd_scale;
            }
            Mode::Hold => {
                // Damping without a position target: zero stiffness, zero
                // velocity target, damping only. The controller's documented
                // damping configuration.
                out.kd = self.kd * kd_scale;
            }
            // Torque: pure feedforward, no stiffness and no damping.
            _ => {}
        }
        if let Some(torque) = command.feedforward_torque.get() {
            out.torque = torque.value();
        }
        out
    }

    /// The one transaction this command amounts to.
    fn apply(&mut self, bus: &SharedBus, command: &Command) -> Result<Option<Feedback>> {
        match command.mode {
            Mode::Stop => {
                self.enabled = false;
                return self.controller.stop(bus, false);
            }
            // A stop frame answers with a full feedback frame and leaves the
            // motor released, which is exactly what a disarmed session wants:
            // watch the axis without commanding it.
            Mode::Query => return self.controller.stop(bus, false),
            Mode::SetOutput { position } => {
                // The controller can only declare the *current* position to be
                // zero; it cannot be told "you are at 0.3 rev". Anything else
                // is refused rather than approximated.
                if position.value().abs() > 1e-9 {
                    return Err(Error::Refused(format!(
                        "the output can only be re-referenced to zero, not to {:.4} rev",
                        position.value()
                    )));
                }
                if self.enabled {
                    return Err(Error::Refused(
                        "stop the motor before re-referencing the output".into(),
                    ));
                }
                self.turns = 0.0;
                self.last_raw_angle = None;
                return self.controller.set_mechanical_zero(bus);
            }
            Mode::Brake | Mode::StayWithin { .. } => {
                return Err(Error::UnsupportedMode {
                    driver: "robstride",
                    mode: command.mode.control(),
                })
            }
            _ => {}
        }

        self.apply_torque_limit(bus, command)?;

        match command.mode {
            Mode::Velocity => {
                self.ensure_run_mode(bus, RunMode::Velocity)?;
                self.ensure_enabled(bus)?;
                let velocity = command
                    .velocity
                    .get()
                    .map(|value| value.to_rad_per_sec().value())
                    .unwrap_or(0.0)
                    .clamp(-self.model.speed_ref_max, self.model.speed_ref_max);
                self.controller
                    .write_parameter(bus, Parameter::SPD_REF, velocity)
            }
            Mode::Position if self.position_mode == PositionMode::Csp => {
                self.apply_csp(bus, command)
            }
            Mode::Position | Mode::Torque | Mode::Hold => {
                self.ensure_run_mode(bus, RunMode::Operation)?;
                self.ensure_enabled(bus)?;
                let frame = self.control_command(command);
                self.controller.control(bus, &frame)
            }
            other => Err(Error::UnsupportedMode {
                driver: "robstride",
                mode: other.control(),
            }),
        }
    }

    /// Converts a feedback frame into framework units.
    fn telemetry(&mut self, feedback: &Feedback) -> Telemetry {
        let (name, detail) = feedback.fault_text();
        let temperature = Celsius(feedback.temperature);

        Telemetry {
            mode: Some(feedback.mode as i32),
            mode_name: feedback.mode.name().to_string(),
            fault: Some(drlabs_core::telemetry::Fault {
                code: feedback.flags as i32,
                name: name.into(),
                detail: detail.into(),
                hard: feedback.faulted(),
            }),
            position: Some(Rev(self.unwrap(feedback.angle) / std::f64::consts::TAU)),
            velocity: Some(RevPerSec(feedback.velocity / std::f64::consts::TAU)),
            torque: Some(Nm(feedback.torque)),
            // A RobStride is an integrated actuator: the temperature it reports
            // is the one inside the case, and it stands for both.
            board_temperature: Some(temperature),
            motor_temperature: Some(temperature),
            ..Default::default()
        }
    }

    /// Turns the controller's ±4π position into a continuous one.
    ///
    /// The reported angle wraps at the ends of its range, so a multi-turn axis
    /// would otherwise appear to jump eight π backwards every four revolutions
    /// — and a position envelope checked against that would trip on a
    /// perfectly healthy machine. A jump larger than half the window is read as
    /// a wrap.
    fn unwrap(&mut self, raw: f64) -> f64 {
        let last = match self.last_raw_angle {
            Some(last) => last,
            None => {
                self.last_raw_angle = Some(raw);
                return raw;
            }
        };
        let delta = raw - last;
        let window = 2.0 * POSITION_RANGE;
        if delta > POSITION_RANGE {
            self.turns -= window;
        } else if delta < -POSITION_RANGE {
            self.turns += window;
        }
        self.last_raw_angle = Some(raw);
        raw + self.turns
    }
}

/// Folds a commanded angle into the ±4π window the protocol can express.
fn wrap_angle(angle: f64) -> f64 {
    let window = 2.0 * POSITION_RANGE;
    let mut wrapped = (angle + POSITION_RANGE) % window;
    if wrapped < 0.0 {
        wrapped += window;
    }
    wrapped - POSITION_RANGE
}

impl Link for RobstrideLink {
    fn kind(&self) -> &'static str {
        "robstride"
    }

    fn describe(&self) -> Vec<(String, String)> {
        let mut out = vec![
            ("driver".into(), "robstride".into()),
            ("model".into(), self.model.name.into()),
            ("can_id".into(), self.controller.id.to_string()),
            (
                "position_mode".into(),
                match self.position_mode {
                    PositionMode::Csp => "csp".into(),
                    PositionMode::Operation => "operation".into(),
                },
            ),
        ];
        if let Some(bus) = &self.bus {
            out.extend(bus.lock().describe());
        }
        out
    }

    fn open(&mut self) -> Result<()> {
        let bus = self.require_bus()?.clone();
        bus.lock()
            .drain(Duration::from_millis(20))
            .map_err(|error| Error::Transport {
                operation: "drain",
                source: Box::new(error),
            })?;

        // A motor left enabled by a previous session keeps acting on its last
        // command until told otherwise, so the connect sequence stops it first.
        match self.controller.stop(&bus, false)? {
            Some(_) => {}
            None => {
                return Err(Error::Refused(format!(
                    "no reply from CAN id {} — check the id, the bus wiring and termination, the \
                     1 Mbit/s bit rate, and that the motor is powered",
                    self.controller.id
                )))
            }
        }
        self.enabled = false;
        self.run_mode = None;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        if let Some(bus) = self.bus.take() {
            let _ = self.controller.stop(&bus, false);
        }
        self.enabled = false;
        Ok(())
    }

    /// A RobStride does not report its own model over the bus, so the board
    /// name here is the *configured* model, and the status is unverified.
    ///
    /// That distinction matters: if the configuration says RS01 and an RS04 is
    /// attached, every torque command is scaled by a factor of seven and
    /// nothing on the wire would reveal it — so the operator is told the
    /// identification is an assumption rather than a reading.
    fn identify(&mut self) -> Result<Option<Identity>> {
        let bus = self.require_bus()?.clone();
        let mut identity = Identity::new("robstride");
        identity.board = format!("RobStride {} (configured, not read back)", self.model.name);
        identity.status = BoardStatus::Unknown;

        if let Ok(Some(serial)) = self.controller.device_id(&bus) {
            identity.serial = serial;
        }
        Ok(Some(identity))
    }

    /// Brake and stay-within are absent because the controller has no
    /// equivalent: there is no phase-short brake mode, and no bounded-region
    /// mode. Claiming them and approximating would be worse than saying so — a
    /// caller that asks for a brake and silently gets a damped hold has a
    /// machine that coasts further than it was told to.
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            modes: vec![
                ControlMode::Stop,
                ControlMode::Query,
                ControlMode::Position,
                ControlMode::Velocity,
                ControlMode::Torque,
                ControlMode::Hold,
                ControlMode::SetOutput,
            ],
            max_rate_hz: 500,
            has_watchdog: true,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }

    fn envelope(&self) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(self.model.voltage_min)),
            voltage_max: Some(Volts(self.model.voltage_max)),
            voltage_rated: Some(Volts(self.model.voltage_rated)),
            velocity_max: Some(RevPerSec(self.model.velocity_range / std::f64::consts::TAU)),
            peak_current: Some(drlabs_core::units::Amps(self.model.current_limit_max)),
        })
    }

    fn cycle(&mut self, command: &Command, profile: QueryProfile) -> Result<Option<Telemetry>> {
        let bus = self.require_bus()?.clone();

        let feedback = match self.apply(&bus, command)? {
            Some(feedback) => feedback,
            None => return Ok(None),
        };
        let mut telemetry = self.telemetry(&feedback);

        // The feedback frame carries no bus voltage and no current; both live
        // in the parameter table. They are read on the wider profiles so the
        // fast path stays a single transaction, which is what lets the loop
        // keep its period.
        if profile != QueryProfile::Fast {
            if let Ok(Some(value)) = self.controller.read_parameter(&bus, Parameter::VBUS) {
                telemetry.voltage = Some(Volts(value));
            }
            if let Ok(Some(value)) = self.controller.read_parameter(&bus, Parameter::IQ_FILT) {
                telemetry.q_current = Some(drlabs_core::units::Amps(value));
            }
        }

        if profile == QueryProfile::Verify {
            let mut active = ActiveLimits::default();
            // limit_torque governs in every run mode, so it is always worth
            // verifying against what the host believes it set.
            if let Ok(Some(value)) = self
                .controller
                .read_parameter(&bus, Parameter::LIMIT_TORQUE)
            {
                active.max_torque = Some(Nm(value));
            }
            // limit_spd only governs in CSP position mode. Reporting it while
            // the operation mode is in charge would have the host compare its
            // own velocity limit against a parameter that is not doing
            // anything, and warn about a mismatch on every verify cycle for
            // ever.
            if self.run_mode == Some(RunMode::PositionCsp) {
                if let Ok(Some(value)) = self.controller.read_parameter(&bus, Parameter::LIMIT_SPD)
                {
                    active.velocity_limit = Some(RevPerSec(value / std::f64::consts::TAU));
                }
            }
            telemetry.active = active;
        }

        Ok(Some(telemetry))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{RS01, RS04};
    use drlabs_core::command::Field;
    use drlabs_transport::can::{self, Frame};

    struct NullBus;
    impl can::Bus for NullBus {
        fn send(&mut self, _frame: &Frame) -> can::Result<()> {
            Ok(())
        }
        fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
            Err(can::Error::Timeout)
        }
        fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
            Ok(())
        }
    }

    fn link(model: Model) -> RobstrideLink {
        RobstrideLink::new(Arc::new(SharedBus::new(Box::new(NullBus))), 1, model)
    }

    /// Four revolutions is the edge of the window. Without unwrapping, an axis
    /// that crosses it appears to jump eight π backwards, and a position
    /// envelope checked against that trips on a healthy machine.
    #[test]
    fn a_multi_turn_axis_reports_a_continuous_position() {
        let mut link = link(RS01);
        let just_under = POSITION_RANGE - 0.1;
        let just_over = -POSITION_RANGE + 0.1;

        assert!((link.unwrap(just_under) - just_under).abs() < 1e-9);
        let continued = link.unwrap(just_over);
        assert!(
            continued > just_under,
            "position went backwards across the wrap: {just_under} then {continued}"
        );
        assert!((continued - (just_under + 0.2)).abs() < 1e-9);
    }

    #[test]
    fn unwrapping_works_in_both_directions() {
        let mut link = link(RS01);
        link.unwrap(-POSITION_RANGE + 0.1);
        let back = link.unwrap(POSITION_RANGE - 0.1);
        assert!(back < 0.0, "expected to keep counting down, got {back}");
    }

    /// A target past the window is folded into it, and the turn counter undoes
    /// the fold when reporting.
    #[test]
    fn a_commanded_angle_is_folded_into_the_window() {
        assert!((wrap_angle(0.5) - 0.5).abs() < 1e-9);
        assert!(wrap_angle(3.0 * POSITION_RANGE).abs() <= POSITION_RANGE);
        assert!(wrap_angle(-3.0 * POSITION_RANGE).abs() <= POSITION_RANGE);
    }

    /// Position mode is a spring-damper; torque mode is neither.
    #[test]
    fn each_mode_sets_the_gains_its_contract_needs() {
        let link = link(RS01).with_gains(30.0, 2.0);

        let position = link.control_command(&Command {
            position: Field::Set(Rev(0.25)),
            ..Command::new(Mode::Position)
        });
        assert_eq!(position.kp, 30.0);
        assert_eq!(position.kd, 2.0);

        let hold = link.control_command(&Command::new(Mode::Hold));
        assert_eq!(hold.kp, 0.0, "a hold has no position to pull towards");
        assert_eq!(hold.kd, 2.0);

        let torque = link.control_command(&Command {
            feedforward_torque: Field::Set(Nm(1.5)),
            ..Command::new(Mode::Torque)
        });
        assert_eq!(torque.kp, 0.0);
        assert_eq!(torque.kd, 0.0);
        assert_eq!(torque.torque, 1.5);
    }

    /// The gain scale selects a fraction of the *configured* stiffness, not of
    /// the codec's full scale — which would be far too stiff to use.
    #[test]
    fn a_gain_scale_is_a_fraction_of_the_configured_stiffness() {
        let link = link(RS01).with_gains(30.0, 2.0);
        let command = link.control_command(&Command {
            position: Field::Set(Rev(0.0)),
            kp_scale: Field::Set(0.5),
            kd_scale: Field::Set(0.25),
            ..Command::new(Mode::Position)
        });
        assert_eq!(command.kp, 15.0);
        assert_eq!(command.kd, 0.5);
        assert!(command.kp < RS01.kp_max, "nowhere near the codec's ceiling");
    }

    /// What the controller cannot do is reported rather than approximated. A
    /// caller who asks for a brake and silently gets a damped hold has a
    /// machine that coasts further than it was told to.
    #[test]
    fn the_modes_the_controller_lacks_are_refused() {
        let link = link(RS01);
        assert!(!link.capabilities().supports(ControlMode::Brake));
        assert!(!link.capabilities().supports(ControlMode::StayWithin));
        assert!(link.capabilities().supports(ControlMode::Position));
    }

    /// The default gains come from the model, and the two halves of the family
    /// are an order of magnitude apart.
    #[test]
    fn the_default_gains_follow_the_model() {
        assert_eq!(link(RS01).kp, 20.0);
        assert_eq!(link(RS04).kp, 200.0);
    }

    /// A RobStride cannot be told "you are at 0.3 rev", only "you are at zero".
    #[test]
    fn re_referencing_to_anything_but_zero_is_refused() {
        let mut link = link(RS01);
        let bus = SharedBus::new(Box::new(NullBus));
        let error = link
            .apply(&bus, &Command::new(Mode::SetOutput { position: Rev(0.3) }))
            .unwrap_err();
        assert!(
            format!("{error}").contains("re-referenced to zero"),
            "{error}"
        );
    }

    /// And not while it is driving.
    #[test]
    fn re_referencing_a_driving_motor_is_refused() {
        let mut link = link(RS01);
        link.enabled = true;
        let bus = SharedBus::new(Box::new(NullBus));
        let error = link
            .apply(&bus, &Command::new(Mode::SetOutput { position: Rev(0.0) }))
            .unwrap_err();
        assert!(format!("{error}").contains("stop the motor"), "{error}");
    }

    /// The envelope is per-model, and it is what the safety layer checks a
    /// configured limit against.
    #[test]
    fn the_envelope_follows_the_model() {
        let envelope = link(RS01).envelope().unwrap();
        assert_eq!(envelope.voltage_rated, Some(Volts(36.0)));

        let envelope = link(RS04).envelope().unwrap();
        assert_eq!(envelope.voltage_rated, Some(Volts(48.0)));
    }
}
