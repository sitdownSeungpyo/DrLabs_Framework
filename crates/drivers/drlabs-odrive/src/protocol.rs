//! The two ways an ODrive can be spoken to.
//!
//! **CANSimple** is a fixed message layout on standard 11-bit identifiers:
//! `(node_id << 5) | command_id`. Five bits for the command means 32 of them
//! and 64 nodes, which is the whole addressing scheme.
//!
//! **The ASCII protocol** runs over UART or USB CDC and is line-oriented:
//! `w axis0.requested_state 8`, `r vbus_voltage`, `p 0 1.5 0 0`. It is slower
//! and chattier, and it is what you have when there is no CAN adapter.
//!
//! One thing this driver does *not* have to do is convert units. ODrive counts
//! position in turns and velocity in turns per second, which are the
//! framework's revolutions and rev/s exactly.

/// CANSimple command identifiers.
///
/// Numbering is from the ODrive CAN protocol documentation. The layout of
/// `Heartbeat` changed between firmware 0.5.x and 0.6.x — see
/// [`decode_heartbeat`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    GetVersion = 0x00,
    Heartbeat = 0x01,
    Estop = 0x02,
    GetError = 0x03,
    RxSdo = 0x04,
    TxSdo = 0x05,
    SetAxisNodeId = 0x06,
    SetAxisState = 0x07,
    GetEncoderEstimates = 0x09,
    SetControllerMode = 0x0A,
    SetInputPos = 0x0B,
    SetInputVel = 0x0C,
    SetInputTorque = 0x0D,
    SetLimits = 0x0F,
    StartAnticogging = 0x10,
    SetTrajVelLimit = 0x11,
    SetTrajAccelLimits = 0x12,
    SetTrajInertia = 0x13,
    GetIq = 0x14,
    GetTemperature = 0x15,
    Reboot = 0x16,
    GetBusVoltageCurrent = 0x17,
    ClearErrors = 0x18,
    SetAbsolutePosition = 0x19,
    SetPosGain = 0x1A,
    SetVelGains = 0x1B,
}

/// The number of bits the node id is shifted by, which is also the number of
/// command ids.
const COMMAND_BITS: u32 = 5;
const COMMAND_MASK: u32 = 0x1F;

/// The highest node id an 11-bit identifier can carry.
pub const MAX_NODE_ID: u8 = 0x3F;

/// Builds the arbitration identifier for a message to a node.
pub fn arbitration_id(node_id: u8, command: Command) -> u32 {
    ((node_id as u32) << COMMAND_BITS) | (command as u32 & COMMAND_MASK)
}

/// Splits a received identifier into its node and command halves.
pub fn parse_arbitration_id(id: u32) -> (u8, u8) {
    (
        ((id >> COMMAND_BITS) & 0x3F) as u8,
        (id & COMMAND_MASK) as u8,
    )
}

/// The axis states worth naming.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisState(pub u32);

impl AxisState {
    pub const UNDEFINED: Self = Self(0);
    pub const IDLE: Self = Self(1);
    pub const STARTUP_SEQUENCE: Self = Self(2);
    pub const FULL_CALIBRATION_SEQUENCE: Self = Self(3);
    pub const MOTOR_CALIBRATION: Self = Self(4);
    pub const ENCODER_INDEX_SEARCH: Self = Self(6);
    pub const ENCODER_OFFSET_CALIBRATION: Self = Self(7);
    pub const CLOSED_LOOP_CONTROL: Self = Self(8);
    pub const LOCKIN_SPIN: Self = Self(9);
    pub const ENCODER_DIR_FIND: Self = Self(10);
    pub const HOMING: Self = Self(11);

    pub fn name(self) -> &'static str {
        match self.0 {
            0 => "Undefined",
            1 => "Idle",
            2 => "Startup sequence",
            3 => "Full calibration",
            4 => "Motor calibration",
            6 => "Encoder index search",
            7 => "Encoder offset calibration",
            8 => "Closed loop control",
            9 => "Lockin spin",
            10 => "Encoder direction find",
            11 => "Homing",
            _ => "Unknown",
        }
    }

    /// Whether the axis is actually driving.
    pub fn is_driving(self) -> bool {
        self == Self::CLOSED_LOOP_CONTROL
    }
}

/// Which loop the controller closes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum ControlMode {
    Voltage = 0,
    Torque = 1,
    Velocity = 2,
    Position = 3,
}

/// How a setpoint is fed into that loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum InputMode {
    Inactive = 0,
    /// The setpoint is used as given.
    Passthrough = 1,
    VelRamp = 2,
    /// The controller runs a trapezoidal profile to the target, which is what
    /// makes a step change in position a bounded move rather than a lunge.
    TrapTraj = 3,
    TorqueRamp = 4,
    Mirror = 5,
    TuningPosFilter = 6,
}

/// What the axis said about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Heartbeat {
    pub axis_error: u32,
    pub axis_state: u32,
    /// Firmware 0.6 and later only.
    pub procedure_result: Option<u8>,
    /// Firmware 0.6 and later only.
    pub trajectory_done: Option<bool>,
}

/// Decodes a heartbeat, which changed shape between firmware versions.
///
/// In 0.5.x the frame is two 32-bit words: the error and the current state. In
/// 0.6 the state shrank to a byte and two status bytes were added after it.
/// The first four bytes mean the same thing either way, and the state is in
/// byte 4 in both — so the common part is read positionally and the extra
/// bytes are reported as `Some` only when the frame is long enough to have
/// carried them deliberately.
pub fn decode_heartbeat(data: &[u8]) -> Option<Heartbeat> {
    if data.len() < 8 {
        return None;
    }
    Some(Heartbeat {
        axis_error: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        axis_state: data[4] as u32,
        procedure_result: Some(data[5]),
        trajectory_done: Some(data[6] != 0),
    })
}

/// A pair of 32-bit floats, which is the shape of most of these messages.
fn decode_two_f32(data: &[u8]) -> Option<(f64, f64)> {
    if data.len() < 8 {
        return None;
    }
    Some((
        f32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64,
        f32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64,
    ))
}

fn encode_two_f32(first: f64, second: f64) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&(first as f32).to_le_bytes());
    data.extend_from_slice(&(second as f32).to_le_bytes());
    data
}

/// Position in turns, velocity in turns per second — the framework's own units.
pub fn decode_encoder_estimates(data: &[u8]) -> Option<(f64, f64)> {
    decode_two_f32(data)
}

/// Setpoint current and measured current, in amps.
pub fn decode_iq(data: &[u8]) -> Option<(f64, f64)> {
    decode_two_f32(data)
}

/// Board and motor temperature, in Celsius.
pub fn decode_temperature(data: &[u8]) -> Option<(f64, f64)> {
    decode_two_f32(data)
}

/// Bus voltage in volts and bus current in amps.
pub fn decode_bus_voltage_current(data: &[u8]) -> Option<(f64, f64)> {
    decode_two_f32(data)
}

pub fn encode_axis_state(state: AxisState) -> Vec<u8> {
    let mut data = vec![0u8; 8];
    data[0..4].copy_from_slice(&state.0.to_le_bytes());
    data
}

pub fn encode_controller_mode(control: ControlMode, input: InputMode) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&(control as u32).to_le_bytes());
    data.extend_from_slice(&(input as u32).to_le_bytes());
    data
}

/// A position command, with optional velocity and torque feedforward.
///
/// The two feedforward terms are 16-bit fixed point at 0.001 of their unit —
/// not floats. A torque feedforward beyond ±32.767 Nm cannot be expressed, so
/// it is clamped rather than allowed to wrap into full reverse.
pub fn encode_input_pos(position: f64, velocity_ff: f64, torque_ff: f64) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&(position as f32).to_le_bytes());
    data.extend_from_slice(&fixed_milli(velocity_ff).to_le_bytes());
    data.extend_from_slice(&fixed_milli(torque_ff).to_le_bytes());
    data
}

/// Scales into the protocol's 0.001-per-count 16-bit field, saturating.
fn fixed_milli(value: f64) -> i16 {
    if value.is_nan() {
        return 0;
    }
    (value * 1000.0).round().clamp(-32768.0, 32767.0) as i16
}

pub fn encode_input_vel(velocity: f64, torque_ff: f64) -> Vec<u8> {
    encode_two_f32(velocity, torque_ff)
}

pub fn encode_input_torque(torque: f64) -> Vec<u8> {
    let mut data = vec![0u8; 8];
    data[0..4].copy_from_slice(&(torque as f32).to_le_bytes());
    data
}

/// Velocity limit in turns/s and current limit in amps.
pub fn encode_limits(velocity_limit: f64, current_limit: f64) -> Vec<u8> {
    encode_two_f32(velocity_limit, current_limit)
}

pub fn encode_traj_accel_limits(accel: f64, decel: f64) -> Vec<u8> {
    encode_two_f32(accel, decel)
}

/// The axis error word, as a cause rather than a bitmask.
pub fn axis_error_text(error: u32) -> (&'static str, &'static str) {
    // Most severe first, so the reported cause is the one worth acting on.
    const ERRORS: &[(u32, &str, &str)] = &[
        (
            0x100,
            "Motor failed",
            "The motor reported its own error. Read the motor error word for the cause.",
        ),
        (
            0x200,
            "Sensorless estimator failed",
            "The sensorless estimator lost track of the rotor.",
        ),
        (
            0x400,
            "Encoder failed",
            "The encoder reported an error. Check the wiring and the index signal.",
        ),
        (
            0x800,
            "Controller failed",
            "The controller tripped, most often on a velocity or position error limit.",
        ),
        (
            0x40,
            "Watchdog timer expired",
            "No command arrived inside the configured watchdog period, so the axis stopped itself.",
        ),
        (
            0x80,
            "Min endstop pressed",
            "The axis reached its minimum endstop.",
        ),
        (
            0x1000,
            "Max endstop pressed",
            "The axis reached its maximum endstop.",
        ),
        (
            0x2000,
            "Estop requested",
            "An emergency stop message was received on the bus.",
        ),
        (
            0x10000,
            "Over temperature",
            "The board or the motor exceeded its temperature limit.",
        ),
        (
            0x20000,
            "Unknown position",
            "The axis does not know where it is; it needs homing or an index search.",
        ),
    ];

    if error == 0 {
        return ("No fault", "");
    }
    for &(bit, name, detail) in ERRORS {
        if error & bit != 0 {
            return (name, detail);
        }
    }
    (
        "Axis error",
        "The axis reported an error this build does not recognise.",
    )
}

/// The line-oriented ASCII protocol, for a board on a serial port.
///
/// Every command is one line. Reads answer with one line; writes answer with
/// nothing at all, which is why a write cannot be confirmed and a read of the
/// same property afterwards is the only way to check one landed.
pub mod ascii {
    /// `p <axis> <position> <velocity_ff> <torque_ff>`
    pub fn set_position(axis: u8, position: f64, velocity_ff: f64, torque_ff: f64) -> String {
        format!("p {axis} {position:.6} {velocity_ff:.6} {torque_ff:.6}\n")
    }

    /// `v <axis> <velocity> <torque_ff>`
    pub fn set_velocity(axis: u8, velocity: f64, torque_ff: f64) -> String {
        format!("v {axis} {velocity:.6} {torque_ff:.6}\n")
    }

    /// `c <axis> <torque>`
    pub fn set_torque(axis: u8, torque: f64) -> String {
        format!("c {axis} {torque:.6}\n")
    }

    /// `f <axis>` — asks for position and velocity on one line.
    pub fn feedback(axis: u8) -> String {
        format!("f {axis}\n")
    }

    /// `r <property>`
    pub fn read(property: &str) -> String {
        format!("r {property}\n")
    }

    /// `w <property> <value>`
    pub fn write(property: &str, value: &str) -> String {
        format!("w {property} {value}\n")
    }

    /// Parses the `<position> <velocity>` line a feedback request answers with.
    pub fn parse_feedback(line: &str) -> Option<(f64, f64)> {
        let mut fields = line.split_whitespace();
        let position = fields.next()?.parse().ok()?;
        let velocity = fields.next()?.parse().ok()?;
        Some((position, velocity))
    }

    /// Parses the single value a read answers with.
    pub fn parse_value(line: &str) -> Option<f64> {
        line.trim().parse().ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The whole addressing scheme: five bits of command, six of node.
    #[test]
    fn an_identifier_packs_the_node_and_the_command() {
        assert_eq!(arbitration_id(0, Command::Heartbeat), 0x001);
        assert_eq!(arbitration_id(1, Command::Heartbeat), 0x021);
        assert_eq!(arbitration_id(1, Command::SetInputPos), 0x02B);
        assert_eq!(arbitration_id(3, Command::GetEncoderEstimates), 0x069);

        assert_eq!(parse_arbitration_id(0x02B), (1, Command::SetInputPos as u8));
    }

    /// Node 63 is the last one an 11-bit identifier can carry.
    #[test]
    fn the_highest_node_still_fits_eleven_bits() {
        let id = arbitration_id(MAX_NODE_ID, Command::SetVelGains);
        assert!(id <= 0x7FF, "{id:#05x} does not fit a standard identifier");
        assert_eq!(parse_arbitration_id(id).0, MAX_NODE_ID);
    }

    #[test]
    fn a_heartbeat_reports_the_state_and_the_error() {
        let mut data = vec![0u8; 8];
        data[0..4].copy_from_slice(&0x40u32.to_le_bytes()); // watchdog expired
        data[4] = 8; // closed loop control

        let heartbeat = decode_heartbeat(&data).expect("a full frame");
        assert_eq!(heartbeat.axis_error, 0x40);
        assert!(AxisState(heartbeat.axis_state).is_driving());
        assert_eq!(
            axis_error_text(heartbeat.axis_error).0,
            "Watchdog timer expired"
        );
    }

    #[test]
    fn a_short_frame_is_refused_rather_than_padded() {
        assert_eq!(decode_heartbeat(&[0u8; 4]), None);
        assert_eq!(decode_encoder_estimates(&[0u8; 4]), None);
    }

    /// ODrive counts turns; the framework counts revolutions. They are the same
    /// unit, so there is nothing to convert and nothing to get wrong.
    #[test]
    fn encoder_estimates_arrive_in_turns_and_turns_per_second() {
        let mut data = Vec::new();
        data.extend_from_slice(&1.25f32.to_le_bytes());
        data.extend_from_slice(&(-0.5f32).to_le_bytes());

        let (position, velocity) = decode_encoder_estimates(&data).unwrap();
        assert!((position - 1.25).abs() < 1e-6);
        assert!((velocity + 0.5).abs() < 1e-6);
    }

    /// The feedforward fields are 16-bit fixed point, not floats. Overflowing
    /// one would wrap a large forward torque into full reverse.
    #[test]
    fn a_feedforward_term_saturates_rather_than_wrapping() {
        let data = encode_input_pos(0.0, 0.0, 1000.0);
        let torque_ff = i16::from_le_bytes([data[6], data[7]]);
        assert_eq!(torque_ff, 32767);

        let data = encode_input_pos(0.0, 0.0, -1000.0);
        let torque_ff = i16::from_le_bytes([data[6], data[7]]);
        assert_eq!(torque_ff, -32768);
    }

    #[test]
    fn a_position_command_carries_its_feedforward_terms_at_a_thousandth() {
        let data = encode_input_pos(2.5, 1.5, 0.25);
        assert_eq!(data.len(), 8);
        assert!((f32::from_le_bytes([data[0], data[1], data[2], data[3]]) - 2.5).abs() < 1e-6);
        assert_eq!(i16::from_le_bytes([data[4], data[5]]), 1500);
        assert_eq!(i16::from_le_bytes([data[6], data[7]]), 250);
    }

    #[test]
    fn a_controller_mode_carries_both_halves() {
        let data = encode_controller_mode(ControlMode::Position, InputMode::TrapTraj);
        assert_eq!(&data[0..4], &3u32.to_le_bytes());
        assert_eq!(&data[4..8], &3u32.to_le_bytes());
    }

    #[test]
    fn an_axis_state_names_itself() {
        assert_eq!(AxisState::CLOSED_LOOP_CONTROL.name(), "Closed loop control");
        assert_eq!(AxisState::IDLE.name(), "Idle");
        assert!(!AxisState::IDLE.is_driving());
        assert_eq!(AxisState(99).name(), "Unknown");
    }

    #[test]
    fn a_clear_error_word_is_not_a_fault() {
        assert_eq!(axis_error_text(0).0, "No fault");
        assert_eq!(axis_error_text(0x400).0, "Encoder failed");
    }

    #[test]
    fn the_ascii_commands_are_one_line_each() {
        assert_eq!(
            ascii::set_position(0, 1.5, 0.0, 0.0),
            "p 0 1.500000 0.000000 0.000000\n"
        );
        assert_eq!(ascii::set_torque(1, -0.25), "c 1 -0.250000\n");
        assert_eq!(ascii::read("vbus_voltage"), "r vbus_voltage\n");
        assert_eq!(
            ascii::write("axis0.requested_state", "8"),
            "w axis0.requested_state 8\n"
        );
    }

    #[test]
    fn an_ascii_feedback_line_parses_to_a_pair() {
        assert_eq!(ascii::parse_feedback("1.5 -0.25\r\n"), Some((1.5, -0.25)));
        assert_eq!(ascii::parse_value(" 24.1 \n"), Some(24.1));
        // A line that is not a number is not a reading.
        assert_eq!(ascii::parse_feedback("invalid"), None);
        assert_eq!(ascii::parse_value("ERR"), None);
    }
}
