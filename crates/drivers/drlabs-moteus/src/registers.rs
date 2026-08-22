//! Register numbers and the scale factors their values are carried at.
//!
//! Integer registers are fixed point, and the scale depends on both the
//! register family and the width it was sent at. Getting one wrong is not a
//! visible failure — the motor still moves, the numbers are just silently out.

use crate::multiplex::{Resolution, Scales};

/// A moteus register number.
pub type Register = u32;

pub const MODE: Register = 0x000;
pub const POSITION: Register = 0x001;
pub const VELOCITY: Register = 0x002;
pub const TORQUE: Register = 0x003;
pub const Q_CURRENT: Register = 0x004;
pub const D_CURRENT: Register = 0x005;
pub const ABS_POSITION: Register = 0x006;
pub const POWER: Register = 0x007;

pub const MOTOR_TEMPERATURE: Register = 0x00a;
pub const TRAJECTORY_COMPLETE: Register = 0x00b;
pub const HOME_STATE: Register = 0x00c;
pub const VOLTAGE: Register = 0x00d;
pub const TEMPERATURE: Register = 0x00e;
pub const FAULT: Register = 0x00f;

// Command registers. These are sticky: a value written here stays in force
// until it is written again, which is why the per-cycle frame only needs to
// carry the handful that actually change every period.
pub const COMMAND_POSITION: Register = 0x020;
pub const COMMAND_VELOCITY: Register = 0x021;
pub const COMMAND_FEEDFORWARD_TORQUE: Register = 0x022;
pub const COMMAND_KP_SCALE: Register = 0x023;
pub const COMMAND_KD_SCALE: Register = 0x024;
pub const COMMAND_MAX_TORQUE: Register = 0x025;
pub const COMMAND_STOP_POSITION: Register = 0x026;
pub const COMMAND_TIMEOUT: Register = 0x027;
pub const COMMAND_VELOCITY_LIMIT: Register = 0x028;
pub const COMMAND_ACCEL_LIMIT: Register = 0x029;
pub const COMMAND_FIXED_VOLTAGE_OVERRIDE: Register = 0x02a;
pub const COMMAND_ILIMIT_SCALE: Register = 0x02b;
pub const COMMAND_FIXED_CURRENT_OVERRIDE: Register = 0x02c;
pub const COMMAND_IGNORE_POSITION_BOUNDS: Register = 0x02d;

pub const WITHIN_LOWER_BOUND: Register = 0x040;
pub const WITHIN_UPPER_BOUND: Register = 0x041;
pub const WITHIN_FEEDFORWARD_TORQUE: Register = 0x042;
pub const WITHIN_KP_SCALE: Register = 0x043;
pub const WITHIN_KD_SCALE: Register = 0x044;
pub const WITHIN_MAX_TORQUE: Register = 0x045;
pub const WITHIN_TIMEOUT: Register = 0x046;
pub const WITHIN_ILIMIT_SCALE: Register = 0x047;
pub const WITHIN_IGNORE_POSITION_BOUNDS: Register = 0x048;

pub const MODEL_NUMBER: Register = 0x100;
pub const FIRMWARE_VERSION: Register = 0x102;
pub const SERIAL_NUMBER_1: Register = 0x120;
pub const SERIAL_NUMBER_2: Register = 0x121;
pub const SERIAL_NUMBER_3: Register = 0x122;

/// Re-references the output to the nearest equivalent of the given position.
pub const SET_OUTPUT_NEAREST: Register = 0x130;
/// Re-references the output verbatim.
pub const SET_OUTPUT_EXACT: Register = 0x131;

// Scale factors per register family, indexed by width.
pub const POSITION_SCALES: Scales = Scales::new(0.01, 0.0001, 0.00001);
pub const VELOCITY_SCALES: Scales = Scales::new(0.1, 0.00025, 0.00001);
pub const ACCEL_SCALES: Scales = Scales::new(0.05, 0.001, 0.00001);
pub const TORQUE_SCALES: Scales = Scales::new(0.5, 0.01, 0.001);
pub const PWM_SCALES: Scales = Scales::new(1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0);
pub const VOLTAGE_SCALES: Scales = Scales::new(0.5, 0.1, 0.001);
pub const TEMPERATURE_SCALES: Scales = Scales::new(1.0, 0.1, 0.001);
pub const TIME_SCALES: Scales = Scales::new(0.01, 0.001, 0.000001);
pub const CURRENT_SCALES: Scales = Scales::new(1.0, 0.1, 0.001);
pub const POWER_SCALES: Scales = Scales::new(10.0, 0.05, 0.0001);

/// The scale table a register's value uses, or `None` when the raw value is
/// the meaning — mode, fault, trajectory-complete, home state.
pub fn scales_for(register: Register) -> Option<Scales> {
    Some(match register {
        POSITION
        | ABS_POSITION
        | COMMAND_POSITION
        | COMMAND_STOP_POSITION
        | WITHIN_LOWER_BOUND
        | WITHIN_UPPER_BOUND => POSITION_SCALES,

        VELOCITY | COMMAND_VELOCITY | COMMAND_VELOCITY_LIMIT => VELOCITY_SCALES,

        COMMAND_ACCEL_LIMIT => ACCEL_SCALES,

        TORQUE
        | COMMAND_FEEDFORWARD_TORQUE
        | COMMAND_MAX_TORQUE
        | WITHIN_FEEDFORWARD_TORQUE
        | WITHIN_MAX_TORQUE => TORQUE_SCALES,

        COMMAND_KP_SCALE | COMMAND_KD_SCALE | COMMAND_ILIMIT_SCALE | WITHIN_KP_SCALE
        | WITHIN_KD_SCALE | WITHIN_ILIMIT_SCALE => PWM_SCALES,

        Q_CURRENT | D_CURRENT => CURRENT_SCALES,
        POWER => POWER_SCALES,
        VOLTAGE => VOLTAGE_SCALES,
        TEMPERATURE | MOTOR_TEMPERATURE => TEMPERATURE_SCALES,
        COMMAND_TIMEOUT | WITHIN_TIMEOUT => TIME_SCALES,

        _ => return None,
    })
}

/// Turns a raw register value into its physical meaning.
pub fn scale_value(register: Register, raw: f64, resolution: Resolution) -> f64 {
    match scales_for(register) {
        Some(scales) => raw * scales.for_resolution(resolution),
        None => raw,
    }
}

/// The controller's own mode register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControllerMode(pub i32);

impl ControllerMode {
    pub const STOPPED: Self = Self(0);
    pub const FAULT: Self = Self(1);
    pub const PWM: Self = Self(5);
    pub const VOLTAGE: Self = Self(6);
    pub const VOLTAGE_FOC: Self = Self(7);
    pub const VOLTAGE_DQ: Self = Self(8);
    pub const CURRENT: Self = Self(9);
    pub const POSITION: Self = Self(10);
    pub const TIMEOUT: Self = Self(11);
    pub const ZERO_VELOCITY: Self = Self(12);
    pub const STAY_WITHIN: Self = Self(13);
    pub const MEASURE_INDUCTANCE: Self = Self(14);
    pub const BRAKE: Self = Self(15);

    pub fn name(self) -> &'static str {
        match self.0 {
            0 => "Stopped",
            1 => "Fault",
            5 => "PWM",
            6 => "Voltage",
            7 => "Voltage FOC",
            8 => "Voltage DQ",
            9 => "Current",
            10 => "Position",
            11 => "Timeout",
            12 => "Zero velocity",
            13 => "Stay within",
            14 => "Measuring inductance",
            15 => "Brake",
            _ => "Unknown",
        }
    }
}

/// The widths each field of a position command is sent at.
///
/// Narrower fields leave more of the frame for the query beside them. The
/// default is tuned for a small motor on a fast loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionResolution {
    pub position: Resolution,
    pub velocity: Resolution,
    pub feedforward_torque: Resolution,
    pub kp_scale: Resolution,
    pub kd_scale: Resolution,
    pub max_torque: Resolution,
    pub stop_position: Resolution,
    pub watchdog: Resolution,
    pub velocity_limit: Resolution,
    pub accel_limit: Resolution,
    pub ilimit_scale: Resolution,
    pub ignore_position_bounds: Resolution,
}

impl Default for PositionResolution {
    fn default() -> Self {
        Self {
            position: Resolution::Int32,           // 0.00001 rev
            velocity: Resolution::Int32,           // 0.00001 rev/s
            feedforward_torque: Resolution::Int16, // 0.01 Nm
            kp_scale: Resolution::Int16,
            kd_scale: Resolution::Int16,
            max_torque: Resolution::Int32, // 0.001 Nm
            stop_position: Resolution::Int32,
            watchdog: Resolution::Int16, // 0.001 s
            velocity_limit: Resolution::Int32,
            accel_limit: Resolution::Int32,
            ilimit_scale: Resolution::Int16,
            ignore_position_bounds: Resolution::Int8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_register_without_a_scale_means_its_raw_value() {
        assert_eq!(scales_for(MODE), None);
        assert_eq!(scales_for(FAULT), None);
        assert_eq!(scale_value(MODE, 10.0, Resolution::Int8), 10.0);
    }

    /// The scale depends on the width, not only the register.
    #[test]
    fn position_scales_differ_by_width() {
        // Compared with a tolerance: 2500 * 0.0001 is not exactly 0.25 in
        // binary floating point, and pinning the exact bits would be testing
        // the arithmetic rather than the scale.
        let close = |got: f64, want: f64| (got - want).abs() < 1e-12;
        assert!(close(
            scale_value(POSITION, 24999.0, Resolution::Int32),
            0.24999
        ));
        assert!(close(
            scale_value(POSITION, 2500.0, Resolution::Int16),
            0.25
        ));
        assert!(close(scale_value(POSITION, 25.0, Resolution::Int8), 0.25));
    }

    #[test]
    fn the_command_registers_share_their_measurement_scales() {
        assert_eq!(scales_for(COMMAND_POSITION), scales_for(POSITION));
        assert_eq!(scales_for(COMMAND_MAX_TORQUE), scales_for(TORQUE));
        assert_eq!(scales_for(COMMAND_VELOCITY_LIMIT), scales_for(VELOCITY));
    }

    #[test]
    fn the_mode_register_names_what_the_controller_is_doing() {
        assert_eq!(ControllerMode::POSITION.name(), "Position");
        assert_eq!(ControllerMode::STOPPED.name(), "Stopped");
        assert_eq!(ControllerMode(99).name(), "Unknown");
    }
}
