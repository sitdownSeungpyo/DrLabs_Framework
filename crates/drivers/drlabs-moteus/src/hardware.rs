//! Hardware facts for the moteus controller family.
//!
//! Every number here comes from the mjbots/moteus reference documentation.
//! They live in one file so the driver, the safety monitor and any UI agree on
//! the envelope rather than each carrying its own copy of it.

use drlabs_core::link::BoardStatus;
use drlabs_core::telemetry::Fault;

/// Family 0: the only family whose electrical envelope this build carries.
///
/// Boards from the other families are recognised by name but are not
/// characterised here, so they are reported as unknown rather than accepted.
pub const FAMILY_R4: i32 = 0;

/// Board families, as the firmware telemetry channel reports them.
fn family_name(family: i32) -> Option<&'static str> {
    Some(match family {
        0 => "moteus (r4 / r5 series)",
        1 => "moteus-n1",
        2 => "moteus-c1",
        3 => "moteus-x1",
        _ => return None,
    })
}

/// Family 0 hardware revisions, from the firmware's own table. Revisions 0-2
/// are pre-r4 prototypes; 3-8 are the r4 boards.
fn r4_revision_name(revision: i32) -> Option<&'static str> {
    Some(match revision {
        0 => "r1",
        1 => "r2",
        2 => "r3",
        3 => "r4.1",
        4 => "r4.2/r4.3",
        5 => "r4.4",
        6 => "r4.5",
        7 => "r4.5b-r4.8",
        8 => "r4.11",
        _ => return None,
    })
}

const R4_REVISION_MIN: i32 = 3;
const R4_REVISION_MAX: i32 = 8;

/// The electrical and motion envelope of a controller family.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HardwareEnvelope {
    /// Volts.
    pub input_voltage_min: f64,
    pub input_voltage_max: f64,
    pub input_voltage_nominal: f64,
    /// Amps, peak phase current.
    pub peak_phase_current: f64,
    /// The switching frequency, which equals the control rate.
    pub pwm_rate_hz: f64,
    /// Revolutions.
    pub position_abs_max: f64,
    /// Revolutions per second.
    pub velocity_max: f64,
}

/// The documented envelope of the moteus r4.
pub const R4_ENVELOPE: HardwareEnvelope = HardwareEnvelope {
    input_voltage_min: 10.0, // "10V to 44V (<= 10S LiPo)"
    input_voltage_max: 44.0,
    input_voltage_nominal: 24.0,
    peak_phase_current: 100.0,
    pwm_rate_hz: 30_000.0,
    position_abs_max: 32_768.0,
    velocity_max: 467.0,
};

pub const CAN_NOMINAL_BITRATE: u32 = 1_000_000;
pub const CAN_DATA_BITRATE: u32 = 5_000_000;

/// The rate the documentation calls out as achievable.
///
/// Whether it is achievable *here* depends on the adapter and the host's
/// scheduling, which is why the session measures the rate it actually attains
/// rather than trusting this number.
pub const CAN_DOCUMENTED_MAX_CYCLE_HZ: u32 = 1000;

/// A best-effort human readable name for a family and revision.
pub fn board_name(family: Option<i32>, revision: Option<i32>) -> String {
    let family = match family {
        Some(family) => family,
        None => return "unknown".into(),
    };
    if family == FAMILY_R4 {
        return match revision {
            Some(revision) => match r4_revision_name(revision) {
                Some(name) => format!("moteus {name}"),
                None => format!("moteus family 0, hwrev {revision}"),
            },
            None => "moteus (family 0, revision not reported)".into(),
        };
    }
    match family_name(family) {
        Some(name) => name.into(),
        None => format!("family {family}"),
    }
}

/// Whether this build knows enough about a board to drive it safely.
///
/// Only family 0 revisions 3-8 are positively characterised. Anything else is
/// reported as unknown rather than rejected outright: a revision table always
/// lags new hardware, and a stale table must not lock a genuine controller out.
/// The pre-r4 prototypes are the one hard rejection, because their electrical
/// limits are known to differ.
pub fn board_status(family: Option<i32>, revision: Option<i32>) -> BoardStatus {
    match (family, revision) {
        (Some(FAMILY_R4), Some(revision)) => {
            if (R4_REVISION_MIN..=R4_REVISION_MAX).contains(&revision) {
                BoardStatus::Supported
            } else if revision < R4_REVISION_MIN {
                BoardStatus::Rejected // r1/r2/r3 prototypes
            } else {
                BoardStatus::Unknown // newer than this build knows about
            }
        }
        _ => BoardStatus::Unknown,
    }
}

/// The first code that reports limiting rather than a latching fault.
pub const LIMITING_FAULT_MIN: i32 = 96;

/// Decodes register 0x00f.
///
/// Codes at or above [`LIMITING_FAULT_MIN`] are not faults at all: they say
/// "the output is being limited, and here is by what", which is the single most
/// useful thing to show an operator whose motor is accepting commands but not
/// moving.
pub fn fault_text(code: i32) -> Fault {
    let (name, detail) = match code {
        0 => ("No fault", ""),
        32 => (
            "Calibration fault",
            "The encoder could not sense the magnet during calibration.",
        ),
        33 => (
            "Motor driver fault",
            "Usually undervoltage, or a supply that cannot deliver the demanded current.",
        ),
        34 => (
            "Over voltage",
            "Bus voltage exceeded servo.max_voltage — most often regeneration with no battery on the bus.",
        ),
        35 => ("Encoder fault", "The encoder reported inconsistent readings."),
        36 => (
            "Motor not configured",
            "moteus_tool --calibrate has not been run for this motor.",
        ),
        37 => ("PWM cycle overrun", "Firmware level timing error."),
        38 => (
            "Over temperature",
            "The board exceeded its configured fault temperature.",
        ),
        39 => (
            "Outside limit",
            "Position control was started while the axis was already outside \
             servopos.position_min/max. Turn the shaft back inside by hand (the driver is off, so \
             it is free), re-reference with a set-output while disarmed, or set \
             ignore_position_bounds on the command.",
        ),
        40 => (
            "Under voltage",
            "Bus voltage dropped below the configured limit.",
        ),
        41 => (
            "Config changed",
            "A configuration value changed that requires a stop.",
        ),
        42 => ("Theta invalid", "No valid commutation encoder."),
        43 => ("Position invalid", "No valid output encoder."),
        44 => ("Driver enable fault", "The gate driver failed to enable."),
        45 => (
            "Stop position deprecated",
            "Use velocity and acceleration limits instead of stop position.",
        ),
        46 => (
            "Timing violation",
            "An internal timing constraint was violated.",
        ),
        47 => (
            "BEMF feedforward without accel limit",
            "An acceleration limit is required for this feature.",
        ),
        48 => (
            "Invalid limits",
            "Configured position bounds are outside the valid range.",
        ),
        49 => (
            "Position control error",
            "Position error exceeded the configured threshold.",
        ),
        50 => (
            "Velocity control error",
            "Velocity error exceeded the configured threshold.",
        ),

        96 => (
            "Limiting: max velocity",
            "The trajectory is capped by servo.max_velocity or the commanded velocity limit.",
        ),
        97 => ("Limiting: max power", "Output is capped by servo.max_power_W."),
        98 => (
            "Limiting: max voltage",
            "Output is capped because the bus voltage is at its limit.",
        ),
        99 => (
            "Limiting: max current",
            "Output is capped by servo.max_current_A.",
        ),
        100 => (
            "Limiting: FET temperature",
            "Output derated because the board is hot.",
        ),
        101 => (
            "Limiting: motor temperature",
            "Output derated because the motor is hot.",
        ),
        102 => (
            "Limiting: max torque",
            "The commanded maximum torque is the active limit. If the axis will not move, raise \
             the torque limit — this is the most common reason a moteus sits still while accepting \
             commands.",
        ),
        103 => (
            "Limiting: position bounds",
            "The target is outside servopos.position_min/max, so the firmware clamps it.",
        ),
        104 => (
            "Limiting: flux braking",
            "Flux braking is active to absorb regenerated energy.",
        ),
        105 => (
            "Limiting: field weakening",
            "Field weakening is active at high speed.",
        ),
        _ => {
            return Fault {
                code,
                name: format!("Fault {code}"),
                detail: String::new(),
                hard: is_hard_fault(code),
            }
        }
    };

    Fault {
        code,
        name: name.into(),
        detail: detail.into(),
        hard: is_hard_fault(code),
    }
}

/// Whether a code is a latching fault, as opposed to an informational limiting
/// state or no fault at all.
pub fn is_hard_fault(code: i32) -> bool {
    code > 0 && code < LIMITING_FAULT_MIN
}

/// Decodes register 0x00c.
pub fn home_state_name(state: i32) -> &'static str {
    match state {
        0 => "relative",
        1 => "rotor",
        2 => "output",
        _ => "",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_characterised_revisions_are_accepted() {
        assert_eq!(board_status(Some(0), Some(8)), BoardStatus::Supported);
        assert_eq!(board_name(Some(0), Some(8)), "moteus r4.11");
    }

    /// The prototypes are the one hard rejection: their electrical limits are
    /// known to differ from the table above.
    #[test]
    fn the_pre_r4_prototypes_are_rejected() {
        assert_eq!(board_status(Some(0), Some(0)), BoardStatus::Rejected);
        assert_eq!(board_status(Some(0), Some(2)), BoardStatus::Rejected);
    }

    /// A revision table always lags new hardware, so anything past the end is
    /// unknown rather than refused. Refusing would mean a stale build locks out
    /// a genuine controller.
    #[test]
    fn newer_hardware_is_unknown_rather_than_refused() {
        assert_eq!(board_status(Some(0), Some(9)), BoardStatus::Unknown);
        assert_eq!(board_status(Some(1), Some(0)), BoardStatus::Unknown);
        assert_eq!(board_status(None, None), BoardStatus::Unknown);
        assert_eq!(board_name(Some(1), None), "moteus-n1");
    }

    #[test]
    fn a_limiting_code_is_not_a_fault() {
        assert!(!fault_text(0).hard);
        assert!(fault_text(39).hard);
        assert!(!fault_text(102).hard, "max torque limiting is information");
        assert!(fault_text(102).name.starts_with("Limiting"));
    }

    /// The two codes an operator meets most often, and the reason each message
    /// says what to do rather than only what happened.
    #[test]
    fn the_common_codes_explain_the_way_out() {
        assert!(fault_text(102).detail.contains("raise the torque limit"));
        assert!(fault_text(39).detail.contains("by hand"));
    }

    #[test]
    fn an_unknown_code_is_still_reported() {
        let fault = fault_text(77);
        assert_eq!(fault.name, "Fault 77");
        assert!(
            fault.hard,
            "an unknown code below 96 is treated as latching"
        );
    }
}
