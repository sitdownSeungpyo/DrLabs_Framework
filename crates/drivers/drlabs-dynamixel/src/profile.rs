//! Control table addresses and unit scales, per motor family.
//!
//! A Dynamixel reports everything in raw encoder and driver counts, and the
//! counts mean different things on different models. The framework speaks
//! physical units, so the conversion has to live somewhere — it lives here, one
//! struct per family, rather than scattered through the driver.
//!
//! Addresses are from the ROBOTIS e-Manual control tables for Protocol 2.0.

use drlabs_core::telemetry::Fault;

/// One motor family's control table and units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Profile {
    pub name: &'static str,

    // EEPROM area.
    pub addr_model_number: u16,
    pub addr_firmware_version: u16,
    pub addr_operating_mode: u16,

    // RAM area.
    pub addr_torque_enable: u16,
    pub addr_led: u16,
    pub addr_hardware_error: u16,
    pub addr_bus_watchdog: u16,
    pub addr_goal_pwm: u16,
    pub addr_goal_current: u16,
    pub addr_goal_velocity: u16,
    pub addr_profile_accel: u16,
    pub addr_profile_velocity: u16,
    pub addr_goal_position: u16,
    pub addr_present_current: u16,
    pub addr_present_velocity: u16,
    pub addr_present_position: u16,
    pub addr_present_voltage: u16,
    pub addr_present_temperature: u16,

    /// Converts the position register to revolutions.
    pub counts_per_rev: f64,

    /// Converts the velocity register to RPM.
    pub velocity_rpm_per_count: f64,

    /// Converts the current register to amperes.
    ///
    /// The value that varies most between models — 2.69 mA on an XM430, 1.0 mA
    /// on an XL330 — so a wrong profile shows up as a current reading out by a
    /// factor of nearly three, which is exactly the sort of error that makes a
    /// current trip useless.
    pub current_amps_per_count: f64,

    /// Converts the input voltage register to volts.
    pub voltage_volts_per_count: f64,

    /// Converts the profile acceleration register.
    pub accel_rev_per_min2_per_count: f64,

    /// Converts the bus watchdog register. It is one byte, so the longest
    /// timeout it can express is 127 counts — a little over two and a half
    /// seconds.
    pub watchdog_seconds_per_count: f64,
    pub watchdog_max_count: i32,

    /// Converts current to torque, in Nm/A.
    ///
    /// `None` means the motor's constant is unknown, and torque is reported as
    /// unmeasured rather than as a number derived from a guess.
    pub torque_constant: Option<f64>,

    /// The family's no-load speed, in rev/s, used as the hardware envelope the
    /// host safety limits are checked against.
    pub max_velocity_rev_per_sec: f64,

    /// The family's supply range.
    pub voltage_min: f64,
    pub voltage_max: f64,

    /// The supply the family is designed to run on.
    ///
    /// Without it the safety layer cannot tell whether its voltage trip band is
    /// plausible for the motor in front of it, and keeps the band it inherited
    /// from a 12 V bus. On a 5 V XL330 that band excludes the rated supply
    /// outright, so a servo that is working perfectly reports 4.8 V, fails
    /// preflight, and cannot be armed at all. That is not a hypothetical — it
    /// is what happened on this bench.
    pub voltage_rated: f64,
}

/// The X-series (XM430, XC430, XH430, XW430 and relatives): a 4096-count
/// encoder and the standard 2.69 mA current unit.
pub const X_SERIES: Profile = Profile {
    name: "X-series",
    addr_model_number: 0,
    addr_firmware_version: 6,
    addr_operating_mode: 11,
    addr_torque_enable: 64,
    addr_led: 65,
    addr_hardware_error: 70,
    addr_bus_watchdog: 98,
    addr_goal_pwm: 100,
    addr_goal_current: 102,
    addr_goal_velocity: 104,
    addr_profile_accel: 108,
    addr_profile_velocity: 112,
    addr_goal_position: 116,
    addr_present_current: 126,
    addr_present_velocity: 128,
    addr_present_position: 132,
    addr_present_voltage: 144,
    addr_present_temperature: 146,

    counts_per_rev: 4096.0,
    velocity_rpm_per_count: 0.229,
    current_amps_per_count: 0.00269,
    voltage_volts_per_count: 0.1,
    accel_rev_per_min2_per_count: 214.577,
    watchdog_seconds_per_count: 0.020,
    watchdog_max_count: 127,

    torque_constant: None, // varies per model; set it explicitly to get torque
    max_velocity_rev_per_sec: 4.0,
    voltage_min: 10.0,
    voltage_max: 14.8,
    voltage_rated: 12.0,
};

/// The 5 V X-series family. Shares the control table but uses a 1 mA current
/// unit and a lower supply range.
pub const XL330: Profile = Profile {
    name: "XL330",
    current_amps_per_count: 0.001,
    max_velocity_rev_per_sec: 6.0,
    voltage_min: 3.7,
    voltage_max: 6.0,
    voltage_rated: 5.0,
    ..X_SERIES
};

/// The families that can be named in a configuration.
pub fn profile_named(name: &str) -> Option<Profile> {
    Some(match name {
        "x" | "xseries" => X_SERIES,
        "xl330" => XL330,
        _ => return None,
    })
}

/// What a reported model number identifies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KnownModel {
    /// The model as ROBOTIS writes it, for logs and status.
    pub name: &'static str,
    /// Which entry in [`profile_named`] actually describes this model.
    pub profile: &'static str,
}

/// Maps the model number a servo reports to the profile that describes it.
///
/// A Dynamixel announces its model number on every ping, so the profile never
/// had to be guessed — and guessing it wrong is not a visible failure. An XL330
/// driven through the X-series profile still moves; it just reports every
/// current 2.69 times too high and claims a 12 V supply range, which is enough
/// to make a current trip meaningless and to refuse a perfectly healthy 5 V
/// servo at preflight.
///
/// A number missing from this table is not an error. It means this build cannot
/// confirm the profile, so the operator has to name one — the same bargain the
/// moteus driver strikes with an unrecognised board revision.
pub fn known_model(model: u32) -> Option<KnownModel> {
    let (name, profile) = match model {
        // 5 V families. These are the ones where the distinction matters most:
        // they count current in 1 mA rather than 2.69 mA, and they run nowhere
        // near a 12 V bus.
        1190 => ("XL330-M077-T", "xl330"),
        1200 => ("XL330-M288-T", "xl330"),

        // 12 V X-series.
        1000 => ("XH430-W350", "x"),
        1010 => ("XH430-W210", "x"),
        1020 => ("XM430-W350", "x"),
        1030 => ("XM430-W210", "x"),
        1040 => ("XH430-V350", "x"),
        1050 => ("XH430-V210", "x"),
        1060 => ("XL430-W250", "x"),
        1070 => ("XC430-W150", "x"),
        1080 => ("XC430-W240", "x"),
        1090 => ("2XL430-W250", "x"),
        1120 => ("XM540-W270", "x"),
        1130 => ("XM540-W150", "x"),
        _ => return None,
    };
    Some(KnownModel { name, profile })
}

/// The operating modes.
pub const OP_MODE_CURRENT: u8 = 0;
pub const OP_MODE_VELOCITY: u8 = 1;
pub const OP_MODE_POSITION: u8 = 3;
pub const OP_MODE_EXTENDED_POSITION: u8 = 4;
pub const OP_MODE_CURRENT_BASED_POSITION: u8 = 5;
pub const OP_MODE_PWM: u8 = 16;

/// Turns the hardware error status register into a cause rather than a bitmask.
pub fn decode_hardware_error(status: u8) -> Fault {
    let (name, detail) = match status {
        0 => ("No fault", ""),
        _ if status & 0x01 != 0 => (
            "Input voltage error",
            "The supply is outside the motor's rated range.",
        ),
        _ if status & 0x04 != 0 => (
            "Overheating error",
            "The internal temperature exceeded its limit.",
        ),
        _ if status & 0x08 != 0 => (
            "Motor encoder error",
            "The encoder is not reporting usable positions.",
        ),
        _ if status & 0x10 != 0 => (
            "Electrical shock error",
            "A short circuit or an over-current on the power line.",
        ),
        _ if status & 0x20 != 0 => (
            "Overload error",
            "A load beyond the motor's continuous rating was sustained.",
        ),
        _ => (
            "Hardware error",
            "The motor reports an error this build does not recognise.",
        ),
    };
    Fault {
        code: status as i32,
        name: name.into(),
        detail: detail.into(),
        hard: status != 0,
    }
}

/// Explains a Protocol 2.0 status packet error code.
///
/// These arrive as a bare number, and a bare number is not something anyone can
/// act on. The one that matters most is 0x07: it is what a servo answers to
/// every goal register while its bus watchdog is latched, and the cause is
/// nowhere near the symptom.
pub fn packet_error_text(code: u8) -> (&'static str, &'static str) {
    match code {
        0x01 => (
            "Result fail",
            "The servo could not carry out the instruction.",
        ),
        0x02 => (
            "Instruction error",
            "An undefined instruction, or an Action with no prior Reg Write.",
        ),
        0x03 => ("CRC error", "The servo's CRC check of the packet failed."),
        0x04 => (
            "Data range error",
            "The value is outside what the register accepts.",
        ),
        0x05 => (
            "Data length error",
            "The write is shorter or longer than the register.",
        ),
        0x06 => (
            "Data limit error",
            "The value is outside the configured limit for that register.",
        ),
        0x07 => (
            "Access error",
            "Writing to a read-only register, writing EEPROM while torque is enabled, or — most \
             often — the bus watchdog has latched, which makes every goal register read-only until \
             zero is written to it.",
        ),
        _ => (
            "Unknown error",
            "The servo reported an error code this build does not recognise.",
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two families differ in exactly the places that matter, and agree
    /// everywhere else — which is why one is written as a delta of the other.
    #[test]
    fn the_five_volt_family_differs_where_it_has_to() {
        assert_eq!(XL330.addr_goal_position, X_SERIES.addr_goal_position);
        assert_eq!(XL330.counts_per_rev, X_SERIES.counts_per_rev);

        // And differs where the difference is the whole point.
        assert_eq!(XL330.current_amps_per_count, 0.001);
        assert_eq!(X_SERIES.current_amps_per_count, 0.00269);
        assert_eq!(XL330.voltage_rated, 5.0);
        assert_eq!(X_SERIES.voltage_rated, 12.0);
    }

    /// The bug this table exists to prevent: an XL330 read through the
    /// X-series profile reports its current nearly three times too high.
    #[test]
    fn the_wrong_profile_misreports_current_by_a_factor_of_three() {
        let counts = 100.0;
        let correct = counts * XL330.current_amps_per_count;
        let wrong = counts * X_SERIES.current_amps_per_count;
        assert!((wrong / correct - 2.69).abs() < 0.01);
    }

    #[test]
    fn a_model_number_names_its_own_profile() {
        let model = known_model(1200).expect("the XL330-M288-T is known");
        assert_eq!(model.name, "XL330-M288-T");
        assert_eq!(profile_named(model.profile), Some(XL330));

        assert_eq!(
            profile_named(known_model(1020).unwrap().profile),
            Some(X_SERIES)
        );
    }

    /// An unknown number is not an error. It means this build cannot confirm
    /// the profile, and the operator has to name one.
    #[test]
    fn an_unrecognised_model_is_not_an_error() {
        assert_eq!(known_model(9999), None);
    }

    /// The code an operator actually meets, and the reason its message names a
    /// register they have never heard of.
    #[test]
    fn the_access_error_explains_the_latched_watchdog() {
        let (name, detail) = packet_error_text(0x07);
        assert_eq!(name, "Access error");
        assert!(detail.contains("bus watchdog has latched"));
    }

    #[test]
    fn the_hardware_error_register_reads_as_a_cause() {
        assert!(!decode_hardware_error(0).hard);
        let fault = decode_hardware_error(0x01);
        assert_eq!(fault.name, "Input voltage error");
        assert!(fault.hard);
        assert_eq!(decode_hardware_error(0x40).name, "Hardware error");
    }
}
