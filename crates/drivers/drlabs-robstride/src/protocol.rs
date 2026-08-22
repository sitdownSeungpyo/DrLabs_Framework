//! The RobStride wire protocol: CAN 2.0B at 1 Mbit/s, 29-bit extended
//! identifiers, an 8-byte payload.
//!
//! Every detail here is taken from the official RobStride user manuals
//! (`Product_Information`, "User Manual260713" for each model). The per-model
//! scaling table in [`crate::models`] is the part that differs between them,
//! and getting it wrong is the difference between commanding 17 Nm and
//! commanding 120.

use crate::models::Model;

/// The 5-bit function code in bits 28..24 of the arbitration identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommType {
    /// Asks for the 64-bit MCU unique identifier.
    GetDeviceId = 0,
    /// The operation-control ("MIT") command: position, velocity, Kp and Kd in
    /// the payload, feedforward torque in the identifier.
    Control = 1,
    /// The motor's reply to almost every command.
    Feedback = 2,
    /// Puts the motor into run mode.
    Enable = 3,
    /// Takes it out of run mode, optionally clearing faults.
    Stop = 4,
    /// Declares the current shaft position to be mechanical zero.
    SetZero = 6,
    /// Changes the motor's own address, effective immediately.
    SetCanId = 7,
    /// Reads the 0x70xx parameter table.
    ReadParam = 17,
    /// Writes it.
    WriteParam = 18,
    /// Carries the detailed fault and warning words.
    FaultFeedback = 21,
    /// Commits parameters so they survive a power cycle.
    SaveParams = 22,
    /// Takes effect on the next power-up.
    SetBaudRate = 23,
    SetReporting = 24,
}

impl CommType {
    pub fn from_bits(bits: u8) -> Option<Self> {
        Some(match bits {
            0 => Self::GetDeviceId,
            1 => Self::Control,
            2 => Self::Feedback,
            3 => Self::Enable,
            4 => Self::Stop,
            6 => Self::SetZero,
            7 => Self::SetCanId,
            17 => Self::ReadParam,
            18 => Self::WriteParam,
            21 => Self::FaultFeedback,
            22 => Self::SaveParams,
            23 => Self::SetBaudRate,
            24 => Self::SetReporting,
            _ => return None,
        })
    }
}

const TYPE_SHIFT: u32 = 24;
const TYPE_MASK: u32 = 0x1f;
const DATA_AREA2_SHIFT: u32 = 8;
const DATA_AREA2_MASK: u32 = 0xffff;
const ADDRESS_MASK: u32 = 0xff;

/// Builds the identifier for a frame sent to a motor.
///
/// ```text
/// bits 28..24   communication type
/// bits 23..8    data area 2 — usually the host id in bits 15..8, but the
///               control command puts the whole feedforward torque there
/// bits  7..0    destination address
/// ```
///
/// Replies invert the addressing: the motor's id lands in bits 15..8 and the
/// host's in bits 7..0.
pub fn arbitration_id(comm_type: CommType, data_area2: u16, target: u8) -> u32 {
    ((comm_type as u32) & TYPE_MASK) << TYPE_SHIFT
        | (data_area2 as u32) << DATA_AREA2_SHIFT
        | target as u32
}

/// Splits a received identifier.
///
/// In a reply, data area 2 carries the responding motor's id in its low byte
/// and — for a feedback frame — the fault and mode bits above it.
pub fn parse_arbitration_id(id: u32) -> (Option<CommType>, u16, u8) {
    (
        CommType::from_bits(((id >> TYPE_SHIFT) & TYPE_MASK) as u8),
        ((id >> DATA_AREA2_SHIFT) & DATA_AREA2_MASK) as u16,
        (id & ADDRESS_MASK) as u8,
    )
}

/// The address this host answers to.
///
/// 0xFD is the value the manuals use in their worked examples; any value the
/// bus does not otherwise use will do.
pub const DEFAULT_HOST_ID: u8 = 0xFD;

/// The fixed address a device-id reply is sent to.
pub const BROADCAST_REPLY_ADDRESS: u8 = 0xFE;

/// The full-scale range of the position field, ±4π radians, on every model.
///
/// The controller counts turns within it and wraps, so a multi-turn axis has to
/// track revolutions on the host.
pub const POSITION_RANGE: f64 = 4.0 * std::f64::consts::PI;

/// The motor's own state, reported in bits 23..22 of a feedback frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    #[default]
    Reset,
    Calibration,
    Run,
    Unknown,
}

impl Mode {
    pub fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Mode::Reset,
            1 => Mode::Calibration,
            2 => Mode::Run,
            _ => Mode::Unknown,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Mode::Reset => "Reset",
            Mode::Calibration => "Calibration",
            Mode::Run => "Run",
            Mode::Unknown => "unknown",
        }
    }
}

// Feedback flag bits, in bits 21..16 of a feedback frame's identifier. These
// are a summary; the detailed words come from a fault feedback frame (type 21).
const FLAG_UNDERVOLTAGE: u16 = 1 << 0;
const FLAG_OVERCURRENT: u16 = 1 << 1;
const FLAG_OVERTEMPERATURE: u16 = 1 << 2;
const FLAG_MAGNETIC_ENCODER: u16 = 1 << 3;
const FLAG_OVERLOAD_STALL: u16 = 1 << 4;
const FLAG_UNCALIBRATED: u16 = 1 << 5;

/// Each summary bit and its description, most severe first, so the reported
/// cause is the one worth acting on.
const SUMMARY_FAULTS: &[(u16, &str, &str)] = &[
    (
        FLAG_UNCALIBRATED,
        "Uncalibrated",
        "The motor's encoder has not been calibrated. It cannot be driven until it is.",
    ),
    (
        FLAG_MAGNETIC_ENCODER,
        "Magnetic encoder fault",
        "The encoder is not reporting usable positions. Check the magnet and the shaft coupling.",
    ),
    (
        FLAG_OVERTEMPERATURE,
        "Over temperature",
        "The motor exceeded its temperature limit and has derated or stopped.",
    ),
    (
        FLAG_OVERCURRENT,
        "Three-phase over current",
        "The phase current exceeded the hardware limit — usually a short, a stall, or too high a \
         torque command.",
    ),
    (
        FLAG_UNDERVOLTAGE,
        "Under voltage",
        "The bus voltage dropped below the protection threshold. Check the supply and its current \
         capability.",
    ),
    (
        FLAG_OVERLOAD_STALL,
        "Stall overload protection",
        "The motor's own stall-overload algorithm tripped: it has been drawing high current \
         without turning.",
    ),
];

/// A decoded type-2 frame, in the motor's native units.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Feedback {
    pub motor_id: u8,
    /// The raw summary word from identifier bits 21..16.
    pub flags: u16,
    pub mode: Mode,

    /// Radians, within ±4π.
    pub angle: f64,
    /// Radians per second.
    pub velocity: f64,
    /// Newton-metres.
    pub torque: f64,
    /// Degrees Celsius.
    pub temperature: f64,
}

impl Feedback {
    pub fn faulted(&self) -> bool {
        self.flags != 0
    }

    /// The most significant fault present.
    pub fn fault_text(&self) -> (&'static str, &'static str) {
        for &(bit, name, detail) in SUMMARY_FAULTS {
            if self.flags & bit != 0 {
                return (name, detail);
            }
        }
        ("No fault", "")
    }
}

/// Maps a physical value onto the protocol's 0..65535 field.
///
/// Every field in the control and feedback frames is a 16-bit linear map over a
/// symmetric range, and the range is per-model — which is why this takes it as
/// an argument rather than baking one in. Out-of-range values are clamped, not
/// wrapped: commanding more torque than the codec can express must not come out
/// the other side as full reverse.
///
/// One consequence is worth knowing before it surprises someone. A symmetric
/// range spread over 65536 codes has no code at its exact centre, so zero is
/// not representable: it encodes to the nearest code and decodes back half a
/// count away. On an RS01 that is 0.26 milli-newton-metres and on an RS04 it is
/// 1.8, far below anything that matters physically — but a comparison against
/// an exact zero will never hold, so telemetry should be compared against a
/// count rather than against 0.
pub fn encode_uint16(value: f64, limit: f64) -> u16 {
    let value = if value.is_nan() { 0.0 } else { value };
    let scaled = (value + limit) / (2.0 * limit) * 65535.0;
    if scaled <= 0.0 {
        0
    } else if scaled >= 65535.0 {
        65535
    } else {
        scaled.round() as u16
    }
}

/// The inverse of [`encode_uint16`].
pub fn decode_uint16(raw: u16, limit: f64) -> f64 {
    raw as f64 / 65535.0 * (2.0 * limit) - limit
}

/// Maps a gain onto its 0..max field.
///
/// Gains are one-sided, so this is a plain scale rather than the symmetric map
/// the other fields use.
pub fn encode_gain(value: f64, max: f64) -> u16 {
    if value.is_nan() || value <= 0.0 {
        return 0;
    }
    let scaled = value / max * 65535.0;
    if scaled >= 65535.0 {
        65535
    } else {
        scaled.round() as u16
    }
}

/// One operation-control ("MIT") frame, in the motor's native units.
///
/// The controller computes torque as
///
/// ```text
/// t = Kp·(angle − measured) + Kd·(velocity − measured) + torque
/// ```
///
/// so this one frame expresses position control, velocity control, pure torque
/// and pure damping, depending on which gains are non-zero.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ControlCommand {
    /// Radians.
    pub angle: f64,
    /// Radians per second.
    pub velocity: f64,
    /// Nm/rad.
    pub kp: f64,
    /// Nm/(rad/s).
    pub kd: f64,
    /// Newton-metres, feedforward.
    pub torque: f64,
}

/// Builds the identifier and payload for a control command.
///
/// The payload is big-endian — "the high byte is in front" in the manuals —
/// while the parameter frames are little-endian. Mixing the two up produces
/// positions that look like noise, so the two encoders are kept apart.
pub fn encode_control(command: &ControlCommand, model: &Model, target: u8) -> (u32, Vec<u8>) {
    let id = arbitration_id(
        CommType::Control,
        encode_uint16(command.torque, model.torque_range),
        target,
    );

    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&encode_uint16(command.angle, POSITION_RANGE).to_be_bytes());
    data.extend_from_slice(&encode_uint16(command.velocity, model.velocity_range).to_be_bytes());
    data.extend_from_slice(&encode_gain(command.kp, model.kp_max).to_be_bytes());
    data.extend_from_slice(&encode_gain(command.kd, model.kd_max).to_be_bytes());
    (id, data)
}

/// Parses a type-2 frame.
pub fn decode_feedback(id: u32, data: &[u8], model: &Model) -> Option<Feedback> {
    if data.len() < 8 {
        return None;
    }
    let (_, data_area2, _) = parse_arbitration_id(id);

    Some(Feedback {
        motor_id: (data_area2 & 0xff) as u8,
        // Identifier bits 21..16 are bits 13..8 of data area 2.
        flags: (data_area2 >> 8) & 0x3f,
        // Identifier bits 23..22 are bits 15..14 of data area 2.
        mode: Mode::from_bits(((data_area2 >> 14) & 0x03) as u8),
        angle: decode_uint16(u16::from_be_bytes([data[0], data[1]]), POSITION_RANGE),
        velocity: decode_uint16(u16::from_be_bytes([data[2], data[3]]), model.velocity_range),
        torque: decode_uint16(u16::from_be_bytes([data[4], data[5]]), model.torque_range),
        // Temperature is a plain fixed-point value, not a ranged map, and is
        // read as signed so a below-zero ambient does not come back as several
        // thousand degrees.
        temperature: i16::from_be_bytes([data[6], data[7]]) as f64 / 10.0,
    })
}

/// A decoded type-21 frame: the full fault and warning words, which carry more
/// detail than the six summary bits in a feedback frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaultReport {
    pub fault: u32,
    pub warning: u32,
}

const DETAILED_FAULTS: &[(u32, &str, &str)] = &[
    (
        1 << 16,
        "A-phase current sampling over current",
        "The A-phase current sensor read beyond its range.",
    ),
    (
        1 << 14,
        "Stall overload protection",
        "The motor's stall-overload algorithm tripped.",
    ),
    (
        1 << 9,
        "Position initialisation fault",
        "The controller could not establish a starting position.",
    ),
    (
        1 << 8,
        "Hardware identification fault",
        "The controller did not recognise its own hardware.",
    ),
    (
        1 << 7,
        "Encoder uncalibrated",
        "The motor encoder has not been calibrated.",
    ),
    (
        1 << 5,
        "C-phase current sampling over current",
        "The C-phase current sensor read beyond its range.",
    ),
    (
        1 << 4,
        "B-phase current sampling over current",
        "The B-phase current sensor read beyond its range.",
    ),
    (
        1 << 3,
        "Over voltage",
        "The bus voltage exceeded the protection threshold.",
    ),
    (
        1 << 2,
        "Under voltage",
        "The bus voltage fell below the protection threshold.",
    ),
    (
        1 << 1,
        "Driver chip fault",
        "The gate driver reported a fault.",
    ),
    (
        1 << 0,
        "Motor over temperature",
        "The motor exceeded its temperature fault threshold.",
    ),
];

impl FaultReport {
    /// The most significant fault in the report.
    pub fn text(&self) -> (&'static str, &'static str) {
        for &(bit, name, detail) in DETAILED_FAULTS {
            if self.fault & bit != 0 {
                return (name, detail);
            }
        }
        if self.warning & 1 != 0 {
            return (
                "Motor over temperature warning",
                "The motor is above its warning threshold but below the fault threshold.",
            );
        }
        ("No fault", "")
    }

    pub fn is_clear(&self) -> bool {
        self.fault == 0 && self.warning == 0
    }
}

/// Parses a type-21 frame.
pub fn decode_fault_report(data: &[u8]) -> Option<FaultReport> {
    if data.len() < 8 {
        return None;
    }
    Some(FaultReport {
        fault: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        warning: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{RS01, RS04};

    #[test]
    fn an_identifier_carries_the_type_the_host_and_the_target() {
        let id = arbitration_id(CommType::Enable, DEFAULT_HOST_ID as u16, 0x7F);
        assert_eq!(id >> 24, 3);
        assert_eq!((id >> 8) & 0xffff, 0xFD);
        assert_eq!(id & 0xff, 0x7F);

        let (comm_type, data_area2, address) = parse_arbitration_id(id);
        assert_eq!(comm_type, Some(CommType::Enable));
        assert_eq!(data_area2, 0xFD);
        assert_eq!(address, 0x7F);
    }

    /// The clamp, and why it is a clamp. Wrapping would turn "more torque than
    /// I can express" into full reverse.
    #[test]
    fn an_out_of_range_value_clamps_rather_than_wrapping() {
        assert_eq!(encode_uint16(1000.0, 17.0), 65535);
        assert_eq!(encode_uint16(-1000.0, 17.0), 0);
        assert!((decode_uint16(65535, 17.0) - 17.0).abs() < 1e-9);
        assert!((decode_uint16(0, 17.0) + 17.0).abs() < 1e-9);
    }

    /// Zero is not representable, and the error is far below anything
    /// physical — but it is not nothing, so telemetry is compared against a
    /// count rather than against an exact zero.
    #[test]
    fn zero_is_not_exactly_representable() {
        let round_tripped = decode_uint16(encode_uint16(0.0, RS01.torque_range), RS01.torque_range);
        assert_ne!(round_tripped, 0.0);
        assert!(round_tripped.abs() < 0.001, "{round_tripped} Nm off zero");
    }

    #[test]
    fn a_value_survives_the_round_trip_within_a_count() {
        for value in [-16.0, -0.5, 0.25, 8.0, 16.9] {
            let back = decode_uint16(encode_uint16(value, RS01.torque_range), RS01.torque_range);
            let one_count = 2.0 * RS01.torque_range / 65535.0;
            assert!((back - value).abs() <= one_count, "{value} -> {back}");
        }
    }

    /// The reason the model is a required setting rather than a default: the
    /// same number means seven times the torque on a different motor.
    #[test]
    fn the_same_code_means_different_torque_on_different_models() {
        let code = encode_uint16(17.0, RS01.torque_range);
        assert_eq!(code, 65535, "full scale on an RS01");
        // That same full-scale code on an RS04 is 120 Nm.
        assert!((decode_uint16(code, RS04.torque_range) - 120.0).abs() < 1e-9);
    }

    #[test]
    fn a_gain_is_one_sided() {
        assert_eq!(encode_gain(0.0, 500.0), 0);
        assert_eq!(encode_gain(-5.0, 500.0), 0, "a negative gain is no gain");
        assert_eq!(encode_gain(500.0, 500.0), 65535);
        assert_eq!(encode_gain(1000.0, 500.0), 65535);
    }

    /// The control frame is big-endian and the parameter frames are not.
    /// Mixing them up produces positions that look like noise.
    #[test]
    fn a_control_frame_is_big_endian() {
        let command = ControlCommand {
            angle: POSITION_RANGE, // full scale
            ..Default::default()
        };
        let (_, data) = encode_control(&command, &RS01, 1);
        assert_eq!(data.len(), 8);
        assert_eq!(&data[0..2], &[0xFF, 0xFF]);
    }

    /// The feedforward torque rides in the identifier, not the payload — the
    /// one field of a control command that does.
    #[test]
    fn the_feedforward_torque_rides_in_the_identifier() {
        let command = ControlCommand {
            torque: RS01.torque_range,
            ..Default::default()
        };
        let (id, _) = encode_control(&command, &RS01, 1);
        assert_eq!((id >> 8) & 0xffff, 0xFFFF);
    }

    #[test]
    fn a_feedback_frame_decodes_to_its_mode_and_flags() {
        // Motor 1, mode Run, undervoltage flag set.
        let data_area2 = (2u16 << 14) | (FLAG_UNDERVOLTAGE << 8) | 0x01;
        let id = arbitration_id(CommType::Feedback, data_area2, DEFAULT_HOST_ID);

        let mut data = vec![0u8; 8];
        data[0..2].copy_from_slice(&encode_uint16(0.0, POSITION_RANGE).to_be_bytes());
        data[6..8].copy_from_slice(&255i16.to_be_bytes()); // 25.5 °C

        let feedback = decode_feedback(id, &data, &RS01).expect("a full frame");
        assert_eq!(feedback.motor_id, 1);
        assert_eq!(feedback.mode, Mode::Run);
        assert!(feedback.faulted());
        assert_eq!(feedback.fault_text().0, "Under voltage");
        assert!((feedback.temperature - 25.5).abs() < 1e-9);
    }

    /// A cold room is not several thousand degrees.
    #[test]
    fn a_below_zero_temperature_reads_as_negative() {
        let id = arbitration_id(CommType::Feedback, 1, DEFAULT_HOST_ID);
        let mut data = vec![0u8; 8];
        data[6..8].copy_from_slice(&(-55i16).to_be_bytes());
        let feedback = decode_feedback(id, &data, &RS01).unwrap();
        assert!((feedback.temperature + 5.5).abs() < 1e-9);
    }

    #[test]
    fn a_short_frame_is_refused_rather_than_padded() {
        let id = arbitration_id(CommType::Feedback, 1, DEFAULT_HOST_ID);
        assert_eq!(decode_feedback(id, &[0u8; 4], &RS01), None);
        assert_eq!(decode_fault_report(&[0u8; 4]), None);
    }

    #[test]
    fn the_detailed_fault_word_names_the_worst_bit_present() {
        let report = FaultReport {
            fault: (1 << 2) | (1 << 14),
            warning: 0,
        };
        // Stall overload is listed above under voltage, so it is reported.
        assert_eq!(report.text().0, "Stall overload protection");
        assert!(!report.is_clear());

        assert_eq!(FaultReport::default().text().0, "No fault");
        assert!(FaultReport::default().is_clear());
    }
}
