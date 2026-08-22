//! The 0x70xx parameter table, read with communication type 17 and written
//! with type 18.
//!
//! A type-18 write is volatile — it is lost on power-down unless a save frame
//! (type 22) commits it. Parameter frames are little-endian, unlike the control
//! and feedback frames.

/// An index into the motor's parameter table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Parameter(pub u16);

impl Parameter {
    /// uint8, see [`RunMode`].
    pub const RUN_MODE: Self = Self(0x7005);
    /// float, A — current-mode setpoint.
    pub const IQ_REF: Self = Self(0x7006);
    /// float, rad/s — velocity-mode setpoint.
    pub const SPD_REF: Self = Self(0x700A);
    /// float, Nm — the torque ceiling in every mode.
    pub const LIMIT_TORQUE: Self = Self(0x700B);
    pub const CUR_KP: Self = Self(0x7010);
    pub const CUR_KI: Self = Self(0x7011);
    /// float, 0..1.
    pub const CUR_FILT_GAIN: Self = Self(0x7014);
    /// float, rad — position-mode setpoint.
    pub const LOC_REF: Self = Self(0x7016);
    /// float, rad/s — the CSP speed limit.
    pub const LIMIT_SPD: Self = Self(0x7017);
    /// float, A — the velocity and position current limit.
    pub const LIMIT_CUR: Self = Self(0x7018);
    /// float, rad — read only.
    pub const MECH_POS: Self = Self(0x7019);
    /// float, A — read only, filtered q current.
    pub const IQ_FILT: Self = Self(0x701A);
    /// float, rad/s — read only.
    pub const MECH_VEL: Self = Self(0x701B);
    /// float, V — read only.
    pub const VBUS: Self = Self(0x701C);
    pub const LOC_KP: Self = Self(0x701E);
    pub const SPD_KP: Self = Self(0x701F);
    pub const SPD_KI: Self = Self(0x7020);
    pub const SPD_FILT_GAIN: Self = Self(0x7021);
    /// float, rad/s² — velocity-mode acceleration.
    pub const ACC_RAD: Self = Self(0x7022);
    /// float, rad/s² — PP-mode speed.
    pub const VEL_MAX: Self = Self(0x7024);
    /// float, rad/s² — PP-mode acceleration.
    pub const ACC_SET: Self = Self(0x7025);
    /// uint16 — the active reporting interval.
    pub const REPORT_TIME: Self = Self(0x7026);
    /// uint32 — the command timeout; 20000 is one second.
    pub const CAN_TIMEOUT: Self = Self(0x7028);

    /// What is known about this index.
    pub fn info(self) -> Option<ParamInfo> {
        PARAM_TABLE.iter().find(|info| info.index == self).copied()
    }
}

impl std::fmt::Display for Parameter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.info() {
            Some(info) => f.write_str(info.name),
            None => write!(f, "0x{:04X}", self.0),
        }
    }
}

/// The on-wire type of a parameter's value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamKind {
    Float,
    Uint8,
    Uint16,
    Uint32,
}

/// One parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamInfo {
    pub index: Parameter,
    pub name: &'static str,
    pub kind: ParamKind,
    pub writable: bool,
    pub unit: &'static str,
}

const fn param(
    index: Parameter,
    name: &'static str,
    kind: ParamKind,
    writable: bool,
    unit: &'static str,
) -> ParamInfo {
    ParamInfo {
        index,
        name,
        kind,
        writable,
        unit,
    }
}

use ParamKind::{Float, Uint16, Uint32, Uint8};

pub const PARAM_TABLE: &[ParamInfo] = &[
    param(Parameter::RUN_MODE, "run_mode", Uint8, true, ""),
    param(Parameter::IQ_REF, "iq_ref", Float, true, "A"),
    param(Parameter::SPD_REF, "spd_ref", Float, true, "rad/s"),
    param(Parameter::LIMIT_TORQUE, "limit_torque", Float, true, "Nm"),
    param(Parameter::CUR_KP, "cur_kp", Float, true, ""),
    param(Parameter::CUR_KI, "cur_ki", Float, true, ""),
    param(Parameter::CUR_FILT_GAIN, "cur_filt_gain", Float, true, ""),
    param(Parameter::LOC_REF, "loc_ref", Float, true, "rad"),
    param(Parameter::LIMIT_SPD, "limit_spd", Float, true, "rad/s"),
    param(Parameter::LIMIT_CUR, "limit_cur", Float, true, "A"),
    param(Parameter::MECH_POS, "mechPos", Float, false, "rad"),
    param(Parameter::IQ_FILT, "iqf", Float, false, "A"),
    param(Parameter::MECH_VEL, "mechVel", Float, false, "rad/s"),
    param(Parameter::VBUS, "VBUS", Float, false, "V"),
    param(Parameter::LOC_KP, "loc_kp", Float, true, ""),
    param(Parameter::SPD_KP, "spd_kp", Float, true, ""),
    param(Parameter::SPD_KI, "spd_ki", Float, true, ""),
    param(Parameter::SPD_FILT_GAIN, "spd_filt_gain", Float, true, ""),
    param(Parameter::ACC_RAD, "acc_rad", Float, true, "rad/s^2"),
    param(Parameter::VEL_MAX, "vel_max", Float, true, "rad/s"),
    param(Parameter::ACC_SET, "acc_set", Float, true, "rad/s^2"),
    param(Parameter::REPORT_TIME, "EPScan_time", Uint16, true, ""),
    param(Parameter::CAN_TIMEOUT, "canTimeout", Uint32, true, ""),
];

/// Finds a parameter by its documented name.
pub fn lookup_param(name: &str) -> Option<ParamInfo> {
    PARAM_TABLE.iter().find(|info| info.name == name).copied()
}

/// The known parameter names, sorted.
pub fn param_names() -> Vec<&'static str> {
    let mut names: Vec<&'static str> = PARAM_TABLE.iter().map(|info| info.name).collect();
    names.sort_unstable();
    names
}

/// Which of the controller's loops is in charge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    /// The "MIT" mode: one frame carries position, velocity, both gains and a
    /// feedforward torque, and the controller computes
    /// `torque = Kp·(p−p̂) + Kd·(v−v̂) + t_ff`. It is the mode a host control
    /// loop wants, because a whole command fits in one frame.
    Operation,
    /// A trapezoidal profile move to `loc_ref`. Speed and acceleration cannot
    /// be changed while it runs.
    PositionPp,
    /// A closed-loop velocity controller tracking `spd_ref`.
    Velocity,
    /// Commands `iq_ref` directly.
    Current,
    /// Cyclic synchronous position: `loc_ref` is a setpoint the host updates
    /// every cycle, limited by `limit_spd`.
    PositionCsp,
}

impl RunMode {
    pub fn code(self) -> u8 {
        match self {
            RunMode::Operation => 0,
            RunMode::PositionPp => 1,
            RunMode::Velocity => 2,
            RunMode::Current => 3,
            RunMode::PositionCsp => 5,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            RunMode::Operation => "operation",
            RunMode::PositionPp => "position-pp",
            RunMode::Velocity => "velocity",
            RunMode::Current => "current",
            RunMode::PositionCsp => "position-csp",
        }
    }
}

/// Why a parameter could not be written.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ParamError {
    #[error("parameter {0} is read only")]
    ReadOnly(String),
    #[error("parameter {name} cannot be set to {value}")]
    NotFinite { name: String, value: String },
}

/// Builds the payload of a type-18 write.
///
/// ```text
/// bytes 0..1  parameter index, little-endian
/// bytes 2..3  zero
/// bytes 4..7  value, little-endian
/// ```
pub fn encode_param_write(index: Parameter, value: f64) -> Result<Vec<u8>, ParamError> {
    // An index this build does not know is still writable — the table lags
    // firmware — but it can only be sent as a float.
    let info = index.info().unwrap_or(ParamInfo {
        index,
        name: "",
        kind: ParamKind::Float,
        writable: true,
        unit: "",
    });
    if !info.writable {
        return Err(ParamError::ReadOnly(index.to_string()));
    }
    if !value.is_finite() {
        return Err(ParamError::NotFinite {
            name: index.to_string(),
            value: value.to_string(),
        });
    }

    let mut data = vec![0u8; 8];
    data[0..2].copy_from_slice(&index.0.to_le_bytes());
    match info.kind {
        ParamKind::Uint8 => data[4] = value as u8,
        ParamKind::Uint16 => data[4..6].copy_from_slice(&(value as u16).to_le_bytes()),
        ParamKind::Uint32 => data[4..8].copy_from_slice(&(value as u32).to_le_bytes()),
        ParamKind::Float => data[4..8].copy_from_slice(&(value as f32).to_le_bytes()),
    }
    Ok(data)
}

/// Builds the payload of a type-17 read.
pub fn encode_param_read(index: Parameter) -> Vec<u8> {
    let mut data = vec![0u8; 8];
    data[0..2].copy_from_slice(&index.0.to_le_bytes());
    data
}

/// Parses the reply to a type-17 read.
///
/// It returns the index the motor answered with as well as the value, because a
/// reply that does not match the request means the bus delivered someone else's
/// answer — and quietly using it would put another parameter's value into this
/// one's variable.
pub fn decode_param_reply(data: &[u8]) -> Option<(Parameter, f64)> {
    if data.len() < 8 {
        return None;
    }
    let index = Parameter(u16::from_le_bytes([data[0], data[1]]));
    let kind = index
        .info()
        .map(|info| info.kind)
        .unwrap_or(ParamKind::Float);

    let value = match kind {
        ParamKind::Uint8 => data[4] as f64,
        ParamKind::Uint16 => u16::from_le_bytes([data[4], data[5]]) as f64,
        ParamKind::Uint32 => u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64,
        ParamKind::Float => f32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64,
    };
    Some((index, value))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parameter frames are little-endian. The control frames are not, and the
    /// two are kept apart for exactly that reason.
    #[test]
    fn a_parameter_frame_is_little_endian() {
        let data = encode_param_read(Parameter::LOC_REF);
        assert_eq!(&data[0..2], &[0x16, 0x70]);
    }

    #[test]
    fn a_float_parameter_round_trips() {
        let written = encode_param_write(Parameter::LOC_REF, 1.5).unwrap();
        let (index, value) = decode_param_reply(&written).unwrap();
        assert_eq!(index, Parameter::LOC_REF);
        assert!((value - 1.5).abs() < 1e-6);
    }

    /// The kinds are not interchangeable: run_mode is one byte and canTimeout
    /// is four, and writing one as the other sets a different value.
    #[test]
    fn each_kind_is_written_at_its_own_width() {
        let run_mode = encode_param_write(Parameter::RUN_MODE, 5.0).unwrap();
        assert_eq!(&run_mode[4..8], &[5, 0, 0, 0]);

        let timeout = encode_param_write(Parameter::CAN_TIMEOUT, 20000.0).unwrap();
        assert_eq!(&timeout[4..8], &20000u32.to_le_bytes());

        let (_, value) = decode_param_reply(&run_mode).unwrap();
        assert_eq!(value, 5.0);
    }

    #[test]
    fn a_read_only_parameter_is_refused() {
        let error = encode_param_write(Parameter::VBUS, 24.0).unwrap_err();
        assert!(matches!(error, ParamError::ReadOnly(_)), "{error}");
        // And the name is in the message, not the number.
        assert!(format!("{error}").contains("VBUS"));
    }

    /// NaN into a motor's parameter table is not a value, it is a fault waiting
    /// for the next power-up.
    #[test]
    fn a_non_finite_value_never_reaches_the_wire() {
        assert!(encode_param_write(Parameter::LOC_REF, f64::NAN).is_err());
        assert!(encode_param_write(Parameter::LOC_REF, f64::INFINITY).is_err());
    }

    /// The table lags firmware, so an unknown index is still writable — as a
    /// float, which is what all but four of the known ones are.
    #[test]
    fn an_unknown_index_is_still_writable_as_a_float() {
        let data = encode_param_write(Parameter(0x70FF), 2.5).unwrap();
        assert_eq!(&data[0..2], &[0xFF, 0x70]);
        assert_eq!(&data[4..8], &2.5f32.to_le_bytes());
    }

    #[test]
    fn a_parameter_prints_as_its_documented_name() {
        assert_eq!(Parameter::LIMIT_TORQUE.to_string(), "limit_torque");
        assert_eq!(Parameter(0x70FF).to_string(), "0x70FF");
        assert_eq!(lookup_param("limit_torque").unwrap().unit, "Nm");
        assert_eq!(lookup_param("nonsense"), None);
    }

    /// Position-CSP is 5, not 4. The gap is in the firmware's own numbering.
    #[test]
    fn the_run_mode_codes_match_the_manual() {
        assert_eq!(RunMode::Operation.code(), 0);
        assert_eq!(RunMode::PositionPp.code(), 1);
        assert_eq!(RunMode::Velocity.code(), 2);
        assert_eq!(RunMode::Current.code(), 3);
        assert_eq!(RunMode::PositionCsp.code(), 5);
    }

    #[test]
    fn a_short_reply_is_refused_rather_than_padded() {
        assert_eq!(decode_param_reply(&[0u8; 4]), None);
    }
}
