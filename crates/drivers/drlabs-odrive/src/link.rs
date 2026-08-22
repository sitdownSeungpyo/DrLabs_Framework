//! One ODrive axis on a CAN bus.
//!
//! **This driver has never been run against an ODrive.** No such board has been
//! on this bench. Everything here follows the published CAN protocol and is
//! pinned by tests against the documented message layouts, which is a real
//! standard — the moteus driver's byte-exact tests caught a genuine ordering
//! bug the same way — but it is not the same as having watched an axis move.
//! [`Link::identify`] says so, so an operator is told rather than left to
//! assume.

use std::sync::Arc;
use std::time::Duration;

use drlabs_core::command::{
    Command as MotorCommand, ControlMode as FrameworkMode, Mode, QueryProfile,
};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::{Fault, Telemetry};
use drlabs_core::units::{Amps, Celsius, Nm, Rev, RevPerSec, Volts};
use drlabs_transport::can::{self, Frame, SharedBus};

use crate::protocol::{
    arbitration_id, axis_error_text, decode_bus_voltage_current, decode_encoder_estimates,
    decode_heartbeat, decode_iq, decode_temperature, encode_axis_state, encode_controller_mode,
    encode_input_pos, encode_input_torque, encode_input_vel, encode_limits, parse_arbitration_id,
    AxisState, Command, ControlMode, InputMode, MAX_NODE_ID,
};

/// How long to wait for a reply to a request.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// A [`Link`] backed by one ODrive axis over CANSimple.
pub struct OdriveLink {
    node_id: u8,
    bus: Option<Arc<SharedBus>>,
    pub timeout: Duration,

    /// The controller mode believed to be selected. Switching it costs a frame,
    /// so it is tracked rather than re-sent every cycle.
    control_mode: Option<(ControlMode, InputMode)>,
    axis_state: Option<AxisState>,

    /// The limits last written, so they cost a frame only when they change.
    last_velocity_limit: Option<f64>,
    last_current_limit: Option<f64>,

    /// The current limit to pair with a velocity limit, since the message
    /// carries both and neither can be written alone.
    current_limit: f64,

    /// Nm per amp, so a torque command can be expressed as one. `None` means
    /// torque is commanded directly, which is what ODrive expects anyway.
    torque_constant: Option<f64>,
}

impl OdriveLink {
    pub fn new(bus: Arc<SharedBus>, node_id: u8) -> Self {
        Self {
            node_id,
            bus: Some(bus),
            timeout: DEFAULT_TIMEOUT,
            control_mode: None,
            axis_state: None,
            last_velocity_limit: None,
            last_current_limit: None,
            current_limit: 10.0,
            torque_constant: None,
        }
    }

    /// Sets the current limit that accompanies every velocity limit.
    pub fn with_current_limit(mut self, amps: f64) -> Self {
        self.current_limit = amps;
        self
    }

    pub fn with_torque_constant(mut self, nm_per_amp: f64) -> Self {
        self.torque_constant = Some(nm_per_amp);
        self
    }

    fn require_bus(&self) -> Result<&Arc<SharedBus>> {
        self.bus.as_ref().ok_or(Error::Closed)
    }

    /// Sends one message with no reply expected.
    ///
    /// Most ODrive commands are one-way: there is nothing to wait for, and
    /// waiting would cost a timeout on every cycle.
    fn send(&self, bus: &SharedBus, command: Command, data: Vec<u8>) -> Result<()> {
        let mut payload = data;
        payload.resize(8, 0);
        let frame = Frame {
            id: arbitration_id(self.node_id, command),
            data: payload,
            extended: false,
            fd: false,
            brs: false,
        };
        bus.lock().send(&frame).map_err(|error| Error::Transport {
            operation: "send",
            source: Box::new(error),
        })
    }

    /// Asks for one message and waits for the answer.
    ///
    /// A request is the same identifier as its reply — the remote-transmission
    /// convention — so the answer is recognised by which command it carries and
    /// which node it came from.
    fn request(&self, bus: &SharedBus, command: Command) -> Result<Option<Vec<u8>>> {
        let mut bus = bus.lock();
        let frame = Frame {
            id: arbitration_id(self.node_id, command),
            data: vec![0u8; 8],
            extended: false,
            fd: false,
            brs: false,
        };
        bus.send(&frame).map_err(|error| Error::Transport {
            operation: "send",
            source: Box::new(error),
        })?;

        let deadline = std::time::Instant::now() + self.timeout;
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            let reply = match bus.recv(remaining) {
                Ok(reply) => reply,
                Err(can::Error::Timeout) => return Ok(None),
                Err(error) => {
                    return Err(Error::Transport {
                        operation: "receive",
                        source: Box::new(error),
                    })
                }
            };
            let (node, command_id) = parse_arbitration_id(reply.id);
            if node == self.node_id && command_id == command as u8 && reply.data.len() >= 8 {
                return Ok(Some(reply.data));
            }
        }
    }

    fn ensure_axis_state(&mut self, bus: &SharedBus, state: AxisState) -> Result<()> {
        if self.axis_state == Some(state) {
            return Ok(());
        }
        self.send(bus, Command::SetAxisState, encode_axis_state(state))?;
        self.axis_state = Some(state);
        Ok(())
    }

    fn ensure_control_mode(
        &mut self,
        bus: &SharedBus,
        control: ControlMode,
        input: InputMode,
    ) -> Result<()> {
        if self.control_mode == Some((control, input)) {
            return Ok(());
        }
        self.send(
            bus,
            Command::SetControllerMode,
            encode_controller_mode(control, input),
        )?;
        self.control_mode = Some((control, input));
        Ok(())
    }

    /// Writes the velocity and current limits when either changes.
    ///
    /// They share one message, so neither can be written alone — which is why
    /// the current limit is carried on this struct rather than taken from the
    /// command.
    fn apply_limits(&mut self, bus: &SharedBus, command: &MotorCommand) -> Result<()> {
        let velocity_limit = match command.velocity_limit.get() {
            Some(value) => value.value(),
            None => return Ok(()),
        };
        if !command.full
            && self.last_velocity_limit == Some(velocity_limit)
            && self.last_current_limit == Some(self.current_limit)
        {
            return Ok(());
        }
        self.send(
            bus,
            Command::SetLimits,
            encode_limits(velocity_limit, self.current_limit),
        )?;
        self.last_velocity_limit = Some(velocity_limit);
        self.last_current_limit = Some(self.current_limit);
        Ok(())
    }

    /// The torque a command asks for, in Nm.
    ///
    /// ODrive commands torque directly, so a torque constant is only needed if
    /// the caller wants to think in amps. It is here for the same reason the
    /// Dynamixel driver has one: without it, a number would have to be invented.
    fn torque_for(&self, torque: Nm) -> f64 {
        torque.value()
    }
}

impl Link for OdriveLink {
    fn kind(&self) -> &'static str {
        "odrive"
    }

    fn describe(&self) -> Vec<(String, String)> {
        let mut out = vec![
            ("driver".into(), "odrive".into()),
            ("node_id".into(), self.node_id.to_string()),
        ];
        if let Some(bus) = &self.bus {
            out.extend(bus.lock().describe());
        }
        out
    }

    fn open(&mut self) -> Result<()> {
        if self.node_id > MAX_NODE_ID {
            return Err(Error::Config(format!(
                "node id {} is out of range (0..{MAX_NODE_ID}) — a standard CAN identifier has \
                 only six bits for it",
                self.node_id
            )));
        }
        let bus = self.require_bus()?.clone();
        bus.lock()
            .drain(Duration::from_millis(20))
            .map_err(|error| Error::Transport {
                operation: "drain",
                source: Box::new(error),
            })?;

        // An axis left in closed-loop control by a previous session keeps
        // acting on its last setpoint, so the connect sequence idles it first.
        self.axis_state = None;
        self.control_mode = None;
        self.last_velocity_limit = None;
        self.last_current_limit = None;
        self.ensure_axis_state(&bus, AxisState::IDLE)?;

        // A board that does not answer a heartbeat is not there. This is the
        // one thing a connect can actually confirm, and skipping it means the
        // first sign of a wrong node id is an axis that never moves.
        match self.request(&bus, Command::Heartbeat)? {
            Some(_) => Ok(()),
            None => Err(Error::Refused(format!(
                "no heartbeat from node {} — check the node id, the bus wiring and termination, \
                 and that the board is powered",
                self.node_id
            ))),
        }
    }

    fn close(&mut self) -> Result<()> {
        if let Some(bus) = self.bus.take() {
            let _ = self.send(
                &bus,
                Command::SetAxisState,
                encode_axis_state(AxisState::IDLE),
            );
        }
        self.axis_state = None;
        Ok(())
    }

    fn identify(&mut self) -> Result<Option<Identity>> {
        let bus = self.require_bus()?.clone();
        let mut identity = Identity::new("odrive");
        identity.board = format!(
            "ODrive node {} (protocol only, never bench tested)",
            self.node_id
        );
        // Never verified against hardware, and saying so is the whole point of
        // the field.
        identity.status = BoardStatus::Unknown;

        if let Some(data) = self.request(&bus, Command::GetVersion)? {
            // hw_version major.minor.variant, then fw_version major.minor.rev.
            identity.firmware = format!("{}.{}.{}", data[4], data[5], data[6]);
        }
        Ok(Some(identity))
    }

    /// Stay-within has no ODrive equivalent, and brake is not a mode the
    /// controller has — idling the axis is a release, not a brake, and calling
    /// it one would promise a machine that stops sooner than it does.
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            modes: vec![
                FrameworkMode::Stop,
                FrameworkMode::Query,
                FrameworkMode::Position,
                FrameworkMode::Velocity,
                FrameworkMode::Torque,
                FrameworkMode::Hold,
            ],
            max_rate_hz: 500,
            has_watchdog: true,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }

    /// No envelope: an ODrive is a controller, not a motor, and its ratings
    /// depend on which motor is bolted to it. Reporting the board's own limits
    /// as the axis's would let the safety layer allow a setting the mechanism
    /// cannot take.
    fn envelope(&self) -> Option<Envelope> {
        None
    }

    fn cycle(
        &mut self,
        command: &MotorCommand,
        profile: QueryProfile,
    ) -> Result<Option<Telemetry>> {
        let bus = self.require_bus()?.clone();

        match command.mode {
            Mode::Stop => {
                self.ensure_axis_state(&bus, AxisState::IDLE)?;
            }
            Mode::Query => {}
            Mode::Brake | Mode::StayWithin { .. } | Mode::SetOutput { .. } => {
                return Err(Error::UnsupportedMode {
                    driver: "odrive",
                    mode: command.mode.control(),
                })
            }
            Mode::Position => {
                self.apply_limits(&bus, command)?;
                // A trapezoidal input mode is what makes a step change in the
                // target a bounded move rather than a lunge at the current
                // limit — the same contract the RobStride driver gets from its
                // cyclic synchronous position mode.
                self.ensure_control_mode(&bus, ControlMode::Position, InputMode::TrapTraj)?;
                self.ensure_axis_state(&bus, AxisState::CLOSED_LOOP_CONTROL)?;

                let position = command.position.get().unwrap_or(Rev(0.0)).value();
                let velocity_ff = command.velocity.get().map(|v| v.value()).unwrap_or(0.0);
                let torque_ff = command
                    .feedforward_torque
                    .get()
                    .map(|t| self.torque_for(t))
                    .unwrap_or(0.0);
                self.send(
                    &bus,
                    Command::SetInputPos,
                    encode_input_pos(position, velocity_ff, torque_ff),
                )?;
            }
            Mode::Velocity => {
                self.apply_limits(&bus, command)?;
                self.ensure_control_mode(&bus, ControlMode::Velocity, InputMode::Passthrough)?;
                self.ensure_axis_state(&bus, AxisState::CLOSED_LOOP_CONTROL)?;

                let velocity = command.velocity.get().unwrap_or(RevPerSec(0.0)).value();
                let torque_ff = command
                    .feedforward_torque
                    .get()
                    .map(|t| self.torque_for(t))
                    .unwrap_or(0.0);
                self.send(
                    &bus,
                    Command::SetInputVel,
                    encode_input_vel(velocity, torque_ff),
                )?;
            }
            Mode::Hold => {
                // Zero velocity in velocity mode: the axis resists motion
                // without holding a position target, which is what a hold is.
                self.ensure_control_mode(&bus, ControlMode::Velocity, InputMode::Passthrough)?;
                self.ensure_axis_state(&bus, AxisState::CLOSED_LOOP_CONTROL)?;
                self.send(&bus, Command::SetInputVel, encode_input_vel(0.0, 0.0))?;
            }
            Mode::Torque => {
                self.ensure_control_mode(&bus, ControlMode::Torque, InputMode::Passthrough)?;
                self.ensure_axis_state(&bus, AxisState::CLOSED_LOOP_CONTROL)?;
                let torque = command
                    .feedforward_torque
                    .get()
                    .map(|t| self.torque_for(t))
                    .unwrap_or(0.0);
                self.send(&bus, Command::SetInputTorque, encode_input_torque(torque))?;
            }
        }

        // Telemetry is a separate request per message, which is what makes an
        // ODrive expensive to poll compared with a moteus: there is no way to
        // ask for several at once.
        let mut telemetry = Telemetry::default();

        match self.request(&bus, Command::GetEncoderEstimates)? {
            Some(data) => {
                if let Some((position, velocity)) = decode_encoder_estimates(&data) {
                    telemetry.position = Some(Rev(position));
                    telemetry.velocity = Some(RevPerSec(velocity));
                }
            }
            // No estimates means the axis did not answer this period. One is a
            // dropped cycle, not a failure.
            None => return Ok(None),
        }

        if let Some(data) = self.request(&bus, Command::Heartbeat)? {
            if let Some(heartbeat) = decode_heartbeat(&data) {
                let state = AxisState(heartbeat.axis_state);
                telemetry.mode = Some(heartbeat.axis_state as i32);
                telemetry.mode_name = state.name().to_string();
                let (name, detail) = axis_error_text(heartbeat.axis_error);
                telemetry.fault = Some(Fault {
                    code: heartbeat.axis_error as i32,
                    name: name.into(),
                    detail: detail.into(),
                    hard: heartbeat.axis_error != 0,
                });
                telemetry.trajectory_complete = heartbeat.trajectory_done;
            }
        }

        if profile != QueryProfile::Fast {
            if let Some(data) = self.request(&bus, Command::GetIq)? {
                if let Some((_, measured)) = decode_iq(&data) {
                    telemetry.q_current = Some(Amps(measured));
                }
            }
            if let Some(data) = self.request(&bus, Command::GetBusVoltageCurrent)? {
                if let Some((voltage, _)) = decode_bus_voltage_current(&data) {
                    telemetry.voltage = Some(Volts(voltage));
                }
            }
            if let Some(data) = self.request(&bus, Command::GetTemperature)? {
                if let Some((board, motor)) = decode_temperature(&data) {
                    telemetry.board_temperature = Some(Celsius(board));
                    telemetry.motor_temperature = Some(Celsius(motor));
                }
            }
        }

        Ok(Some(telemetry))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::command::Field;
    use std::sync::{Arc as StdArc, Mutex};

    #[derive(Clone, Default)]
    struct FakeOdrive {
        sent: StdArc<Mutex<Vec<Frame>>>,
        replies: StdArc<Mutex<Vec<Frame>>>,
    }

    impl FakeOdrive {
        fn will_reply(&self, node: u8, command: Command, data: Vec<u8>) {
            let mut payload = data;
            payload.resize(8, 0);
            self.replies.lock().unwrap().push(Frame {
                id: arbitration_id(node, command),
                data: payload,
                ..Default::default()
            });
        }

        fn commands_sent(&self) -> Vec<u8> {
            self.sent
                .lock()
                .unwrap()
                .iter()
                .map(|frame| parse_arbitration_id(frame.id).1)
                .collect()
        }
    }

    impl can::Bus for FakeOdrive {
        fn send(&mut self, frame: &Frame) -> can::Result<()> {
            self.sent.lock().unwrap().push(frame.clone());
            Ok(())
        }
        fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
            let mut replies = self.replies.lock().unwrap();
            if replies.is_empty() {
                return Err(can::Error::Timeout);
            }
            Ok(replies.remove(0))
        }
        fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
            Ok(())
        }
    }

    fn fixture() -> (OdriveLink, FakeOdrive) {
        let board = FakeOdrive::default();
        let bus = Arc::new(SharedBus::new(Box::new(board.clone())));
        let mut link = OdriveLink::new(bus, 1);
        link.timeout = Duration::from_millis(20);
        (link, board)
    }

    /// CANSimple is standard-identifier classic CAN, not FD and not extended.
    #[test]
    fn every_frame_is_a_standard_classic_can_frame() {
        let (link, board) = fixture();
        let bus = link.require_bus().unwrap().clone();
        link.send(&bus, Command::ClearErrors, vec![]).unwrap();

        let frame = board.sent.lock().unwrap()[0].clone();
        assert!(!frame.extended);
        assert!(!frame.fd);
        assert_eq!(frame.id, 0x038);
        assert_eq!(frame.data.len(), 8, "always padded to eight bytes");
    }

    /// A board that does not answer a heartbeat is not there, and saying so at
    /// connect is better than an axis that silently never moves.
    #[test]
    fn opening_without_a_heartbeat_is_refused() {
        let (mut link, _board) = fixture();
        let error = link.open().unwrap_err();
        assert!(format!("{error}").contains("no heartbeat"), "{error}");
    }

    #[test]
    fn opening_idles_the_axis_before_anything_else() {
        let (mut link, board) = fixture();
        board.will_reply(1, Command::Heartbeat, vec![0, 0, 0, 0, 1, 0, 0, 0]);
        link.open().unwrap();

        let commands = board.commands_sent();
        assert_eq!(commands[0], Command::SetAxisState as u8);
        let state = board.sent.lock().unwrap()[0].data.clone();
        assert_eq!(&state[0..4], &1u32.to_le_bytes(), "idle");
    }

    /// Six bits of node id is the whole of the addressing space.
    #[test]
    fn a_node_id_that_cannot_fit_the_identifier_is_refused() {
        let board = FakeOdrive::default();
        let bus = Arc::new(SharedBus::new(Box::new(board)));
        let mut link = OdriveLink::new(bus, 64);
        let error = link.open().unwrap_err();
        assert!(matches!(error, Error::Config(_)), "{error}");
    }

    /// A position command has to arrive in a mode whose profile generator
    /// bounds the move, or a step in the target is a lunge at the current
    /// limit.
    #[test]
    fn a_position_command_selects_a_trajectory_input_mode() {
        let (mut link, board) = fixture();
        board.will_reply(1, Command::GetEncoderEstimates, vec![0; 8]);

        link.cycle(
            &MotorCommand {
                position: Field::Set(Rev(1.5)),
                ..MotorCommand::new(Mode::Position)
            },
            QueryProfile::Fast,
        )
        .unwrap();

        let commands = board.commands_sent();
        assert!(commands.contains(&(Command::SetControllerMode as u8)));
        assert!(commands.contains(&(Command::SetInputPos as u8)));

        let mode_frame = board
            .sent
            .lock()
            .unwrap()
            .iter()
            .find(|frame| parse_arbitration_id(frame.id).1 == Command::SetControllerMode as u8)
            .unwrap()
            .data
            .clone();
        assert_eq!(
            &mode_frame[0..4],
            &(ControlMode::Position as u32).to_le_bytes()
        );
        assert_eq!(
            &mode_frame[4..8],
            &(InputMode::TrapTraj as u32).to_le_bytes()
        );
    }

    /// Switching modes costs a frame, so an unchanged mode does not pay for one.
    #[test]
    fn an_unchanged_control_mode_is_not_re_sent() {
        let (mut link, board) = fixture();
        for _ in 0..2 {
            board.will_reply(1, Command::GetEncoderEstimates, vec![0; 8]);
        }
        let command = MotorCommand {
            position: Field::Set(Rev(1.0)),
            ..MotorCommand::new(Mode::Position)
        };

        link.cycle(&command, QueryProfile::Fast).unwrap();
        let after_first = board.commands_sent().len();
        link.cycle(&command, QueryProfile::Fast).unwrap();
        let second_cycle = board.commands_sent().len() - after_first;

        // The second cycle is the setpoint and the telemetry request, with no
        // mode or state frames.
        assert!(
            second_cycle < after_first,
            "the second cycle sent {second_cycle} frames, the first {after_first}"
        );
    }

    /// An axis that does not answer is a dropped cycle, not a failure.
    #[test]
    fn a_silent_axis_reports_no_telemetry_rather_than_an_error() {
        let (mut link, _board) = fixture();
        let telemetry = link
            .cycle(&MotorCommand::new(Mode::Query), QueryProfile::Fast)
            .unwrap();
        assert_eq!(telemetry, None);
    }

    #[test]
    fn telemetry_comes_back_in_framework_units() {
        let (mut link, board) = fixture();
        let mut estimates = Vec::new();
        estimates.extend_from_slice(&2.5f32.to_le_bytes());
        estimates.extend_from_slice(&(-1.25f32).to_le_bytes());
        board.will_reply(1, Command::GetEncoderEstimates, estimates);
        board.will_reply(1, Command::Heartbeat, vec![0, 0, 0, 0, 8, 0, 1, 0]);

        let telemetry = link
            .cycle(&MotorCommand::new(Mode::Query), QueryProfile::Fast)
            .unwrap()
            .expect("a reply");

        assert_eq!(telemetry.position, Some(Rev(2.5)));
        assert_eq!(telemetry.velocity, Some(RevPerSec(-1.25)));
        assert_eq!(telemetry.mode_name, "Closed loop control");
        assert_eq!(telemetry.trajectory_complete, Some(true));
        assert!(!telemetry.fault.unwrap().hard);
    }

    /// An ODrive is a controller, not a motor. Its board ratings are not the
    /// axis's, and reporting them as such would let the safety layer allow a
    /// setting the mechanism cannot take.
    #[test]
    fn no_envelope_is_claimed_for_a_motor_this_driver_cannot_see() {
        let (link, _board) = fixture();
        assert_eq!(link.envelope(), None);
    }

    /// The status says what this driver actually is: protocol, never bench
    /// tested.
    #[test]
    fn identification_admits_it_has_never_seen_hardware() {
        let (mut link, board) = fixture();
        board.will_reply(1, Command::GetVersion, vec![0, 0, 0, 0, 0, 6, 10, 0]);

        let identity = link.identify().unwrap().expect("an identity");
        assert_eq!(identity.status, BoardStatus::Unknown);
        assert!(
            identity.board.contains("never bench tested"),
            "{}",
            identity.board
        );
    }

    #[test]
    fn the_modes_an_odrive_lacks_are_refused() {
        let (mut link, _board) = fixture();
        assert!(!link.capabilities().supports(FrameworkMode::StayWithin));
        assert!(!link.capabilities().supports(FrameworkMode::Brake));

        let error = link
            .cycle(&MotorCommand::new(Mode::Brake), QueryProfile::Fast)
            .unwrap_err();
        assert!(matches!(error, Error::UnsupportedMode { .. }), "{error}");
    }
}
