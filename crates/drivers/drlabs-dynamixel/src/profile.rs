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
    /// Return Delay Time: the pause the servo inserts before every status
    /// packet, in 2 microsecond counts. EEPROM. Zero here means the family
    /// has no such register and the link leaves the subject alone.
    pub addr_return_delay: u16,
    pub addr_operating_mode: u16,
    /// Velocity Limit: the ceiling the servo itself enforces on the velocity
    /// registers. Four bytes of EEPROM, so it is whatever somebody last set on
    /// that particular servo — not a family constant.
    pub addr_velocity_limit: u16,

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

    /// Whether the register at `addr_present_current` really is a current.
    ///
    /// It is not always. MX-28, XL430, 2XL430 and XC430 have no current sensor
    /// at all: address 126 on those is Present Load, a signed 0.1 % of the PWM
    /// duty, and there is no Goal Current register to write. Scaling that by a
    /// current unit produces a plausible number in amperes that is not a
    /// current, which is worse than reporting nothing — so when this is false
    /// the driver reports current and torque as unmeasured and refuses torque
    /// control outright.
    pub has_current_sense: bool,

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
    addr_return_delay: 9,
    addr_operating_mode: 11,
    addr_velocity_limit: 44,
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

    has_current_sense: true,
    torque_constant: None, // varies per model; set it explicitly to get torque
    // A family bound, not a measurement: the members verified against the
    // e-Manual are all slower than this. Only XM/XH/XM540 belong here now.
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
    // 103 rev/min at 5.0 V (XL330-M288-T e-Manual).
    max_velocity_rev_per_sec: 1.717,
    voltage_min: 3.7,
    voltage_max: 6.0,
    voltage_rated: 5.0,
    ..X_SERIES
};

/// The 12 V X-series models that have no current sensor.
///
/// XL430-W250 and 2XL430-W250. Three things separate them from [`X_SERIES`],
/// and every one of them is silent if it is got wrong: there is no current
/// register, the supply is a 3-cell lithium pack rather than a nominal 12 V
/// bus, and the shaft turns at a quarter of the speed the X-series bound
/// claims.
///
/// Addresses are identical to [`X_SERIES`], so the wide telemetry read is
/// unchanged — only the meaning of the first two bytes differs.
pub const XL430: Profile = Profile {
    name: "XL430",
    has_current_sense: false,
    // 6.5 ~ 12.0 V, recommended 11.1 V (XL430-W250-T e-Manual).
    voltage_min: 6.5,
    voltage_max: 12.0,
    voltage_rated: 11.1,
    // 57 rev/min at 11.1 V.
    max_velocity_rev_per_sec: 0.95,
    ..X_SERIES
};

/// XC430-W150 and XC430-W240. No current sensor either, and a wider supply
/// range than the XL430.
///
/// The two members differ in speed — 106 rev/min for the W150 against
/// 70 rev/min for the W240 — and the envelope here is the slower of them, so
/// that it never permits more than the motor in front of it can do. A W150
/// that genuinely needs the extra speed wants its own profile, not a raised
/// bound on this one.
pub const XC430: Profile = Profile {
    name: "XC430",
    has_current_sense: false,
    // 6.5 ~ 14.8 V, recommended 12.0 V (XC430-W150-T / W240-T e-Manual).
    voltage_min: 6.5,
    voltage_max: 14.8,
    voltage_rated: 12.0,
    // 70 rev/min at 12.0 V, the W240.
    max_velocity_rev_per_sec: 1.167,
    ..X_SERIES
};

/// MX-64T/R/AT/AR (2.0), model 311.
///
/// The MX family running Protocol 2.0 firmware shares the X-series control
/// table address for address — the e-Manual lists the same 64, 98,
/// 100..116 and 126..146 layout — so only the scales and the envelope move.
/// The one that matters is current: an MX counts it in 3.36 mA against the
/// X-series 2.69 mA, a 25 % error in every reading and in every current trip
/// derived from one.
///
/// Protocol 1.0 firmware is a different motor as far as this build is
/// concerned. It reports a different model number, it does not speak the
/// packet format in [`crate::protocol`], and it will simply not answer.
pub const MX64: Profile = Profile {
    name: "MX64",
    has_current_sense: true,
    // "about 3.36 [mA]" (MX-64(2.0) e-Manual).
    current_amps_per_count: 0.00336,
    // 63 rev/min at 12 V.
    max_velocity_rev_per_sec: 1.05,
    // 10.0 ~ 14.8 V, recommended 12.0 V.
    voltage_min: 10.0,
    voltage_max: 14.8,
    voltage_rated: 12.0,
    ..X_SERIES
};

/// MX-106T/R (2.0), model 321. The MX-64's control table and current unit on a
/// bigger, slower gearbox.
pub const MX106: Profile = Profile {
    name: "MX106",
    // 45 rev/min at 12 V.
    max_velocity_rev_per_sec: 0.75,
    ..MX64
};

/// MX-28T/R/AT/AR (2.0), model 30.
///
/// The member of the family without a current sensor. Address 126 is Present
/// Load in units of 0.1 %, there is no Goal Current at 102, and Operating Mode
/// offers only velocity, position, extended position and PWM — no current
/// control and no current-based position control. Driving it through
/// [`MX64`] would report a load percentage as amperes and write a torque
/// command into a register that does not exist.
pub const MX28: Profile = Profile {
    name: "MX28",
    has_current_sense: false,
    // 55 rev/min at 12 V.
    max_velocity_rev_per_sec: 0.917,
    ..MX64
};

/// PH54-200-S500-R / PH54-100-S500-R: the P series, Protocol 2.0, RS-485.
///
/// **Unverified.** No P-series servo has been on this bench. The addresses
/// marked below were read from the PH54-200-S500-R e-Manual control table
/// in this session; the rest of the layout and every scale are from memory
/// of that table and have not been checked against hardware. A P profile
/// is selected only by name, never from a model number, so nothing is
/// driven through it by accident. The first thing to do with a real P
/// servo is `dxl_diag dump` against this table.
///
/// What is different from the X family: the control table moves to the
/// 500s, the encoder counts a million pulses a revolution, velocity is in
/// 0.01 rev/min and the supply is 24 V.
pub const PH54: Profile = Profile {
    name: "PH54",
    addr_model_number: 0,
    addr_firmware_version: 6,
    addr_return_delay: 9,            // e-Manual: Return Delay Time (9), 2 µs units
    addr_operating_mode: 11,
    addr_velocity_limit: 44,
    addr_torque_enable: 512,
    addr_led: 513,
    addr_hardware_error: 518,
    addr_bus_watchdog: 546,          // e-Manual: Bus Watchdog (546), 20 ms units
    addr_goal_pwm: 548,
    addr_goal_current: 550,          // e-Manual: Goal Current (550), 2 bytes, 1 mA
    addr_goal_velocity: 552,
    addr_profile_accel: 556,
    addr_profile_velocity: 560,
    addr_goal_position: 564,         // e-Manual: Goal Position (564), 4 bytes
    addr_present_current: 574,       // e-Manual: Present Current (574), 2 bytes, 1 mA
    addr_present_velocity: 576,      // e-Manual: Present Velocity (576), 4 bytes, 0.01 rev/min
    addr_present_position: 580,      // e-Manual: Present Position (580), 4 bytes
    addr_present_voltage: 592,       // e-Manual: Present Input Voltage (592), 0.1 V
    addr_present_temperature: 594,   // e-Manual: Present Temperature (594), 1 °C

    // PH54-200-S500 and PH54-100-S500: 1,003,846 pulses per revolution.
    counts_per_rev: 1_003_846.0,
    velocity_rpm_per_count: 0.01,
    current_amps_per_count: 0.001,
    voltage_volts_per_count: 0.1,
    accel_rev_per_min2_per_count: 1.0,
    watchdog_seconds_per_count: 0.020,
    watchdog_max_count: 127,

    has_current_sense: true,
    torque_constant: None,
    // 29.2 rev/min no-load at 24 V for the -200; the -100 is a little faster.
    max_velocity_rev_per_sec: 0.5,
    voltage_min: 10.0,
    voltage_max: 30.0,
    voltage_rated: 24.0,
};

/// PM54-060-S250-R / PM54-040-S250-R: the smaller P-series frame. Same
/// table as [`PH54`], half the encoder resolution. **Unverified**, as above.
pub const PM54: Profile = Profile {
    name: "PM54",
    counts_per_rev: 502_834.0,
    // 55 rev/min no-load at 24 V for the -040.
    max_velocity_rev_per_sec: 0.9,
    ..PH54
};

/// Every name [`profile_named`] accepts, for error messages that tell an
/// operator what the alternatives were.
pub const PROFILE_NAMES: &[&str] = &[
    "x", "xseries", "xl330", "xl430", "2xl430", "xc430", "mx64", "mx106", "mx28", "ph54", "pm54",
];

/// The families that can be named in a configuration.
pub fn profile_named(name: &str) -> Option<Profile> {
    Some(match name {
        "x" | "xseries" => X_SERIES,
        "xl330" => XL330,
        "xl430" | "2xl430" => XL430,
        "xc430" => XC430,
        // No bare "mx": the family splits on whether there is a current sensor
        // at all, and defaulting that choice is how a load percentage ends up
        // being read as amperes.
        "mx64" => MX64,
        "mx106" => MX106,
        "mx28" => MX28,
        // By name only: see the profiles' own notes.
        "ph54" => PH54,
        "pm54" => PM54,
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

        // 12 V X-series with a current sensor.
        1000 => ("XH430-W350", "x"),
        1010 => ("XH430-W210", "x"),
        1020 => ("XM430-W350", "x"),
        1030 => ("XM430-W210", "x"),
        1040 => ("XH430-V350", "x"),
        1050 => ("XH430-V210", "x"),
        1120 => ("XM540-W270", "x"),
        1130 => ("XM540-W150", "x"),

        // X-series without one. Address 126 on these is Present Load, not
        // Present Current, so they cannot share the X-series profile however
        // much of the control table they have in common.
        1060 => ("XL430-W250", "xl430"),
        1090 => ("2XL430-W250", "xl430"),
        1070 => ("XC430-W150", "xc430"),
        1080 => ("XC430-W240", "xc430"),

        // MX family, Protocol 2.0 firmware only. The Protocol 1.0 models
        // report different numbers and are deliberately absent: this build
        // cannot talk to them, and naming them here would promise otherwise.
        30 => ("MX-28(2.0)", "mx28"),
        311 => ("MX-64(2.0)", "mx64"),
        321 => ("MX-106(2.0)", "mx106"),
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

    /// Every name this table hands out has to resolve. A model that maps to a
    /// profile string with a typo in it falls through `resolve_profile` in
    /// silence and leaves the servo on whatever profile it started with —
    /// which is the exact failure the table was written to prevent.
    #[test]
    fn every_known_model_names_a_profile_that_exists() {
        for model in 0..=2000u32 {
            if let Some(known) = known_model(model) {
                assert!(
                    profile_named(known.profile).is_some(),
                    "model {model} ({}) names profile {:?}, which does not exist",
                    known.name,
                    known.profile
                );
            }
        }
    }

    /// The families that have no current sensor at all. Address 126 on these
    /// is Present Load in units of 0.1 %, and reading it as a current is not a
    /// scaling error — it is a different quantity.
    #[test]
    fn the_families_without_a_current_sensor_say_so() {
        for profile in [MX28, XL430, XC430] {
            assert!(
                !profile.has_current_sense,
                "{} has no current register",
                profile.name
            );
        }
        for profile in [X_SERIES, XL330, MX64, MX106] {
            assert!(profile.has_current_sense, "{} does have one", profile.name);
        }
    }

    /// The MX counts current in 3.36 mA. Driving one through the X-series
    /// profile understates every reading by a quarter, which is enough to make
    /// a current trip fire late or not at all.
    #[test]
    fn the_mx_current_unit_is_not_the_x_series_one() {
        assert_eq!(MX64.current_amps_per_count, 0.00336);
        assert_eq!(MX106.current_amps_per_count, 0.00336);
        assert_eq!(X_SERIES.current_amps_per_count, 0.00269);

        // The control table itself is shared, which is why the MX profiles are
        // written as a delta of the X-series one.
        assert_eq!(MX64.addr_goal_position, X_SERIES.addr_goal_position);
        assert_eq!(MX64.addr_present_current, X_SERIES.addr_present_current);
        assert_eq!(MX64.counts_per_rev, 4096.0);
    }

    /// The MX models this build claims are the Protocol 2.0 ones, because they
    /// are the only ones it can speak to.
    #[test]
    fn the_mx_family_resolves_from_its_model_number() {
        assert_eq!(known_model(30).unwrap().name, "MX-28(2.0)");
        assert_eq!(profile_named(known_model(30).unwrap().profile), Some(MX28));
        assert_eq!(known_model(311).unwrap().name, "MX-64(2.0)");
        assert_eq!(profile_named(known_model(311).unwrap().profile), Some(MX64));
        assert_eq!(known_model(321).unwrap().name, "MX-106(2.0)");
        assert_eq!(
            profile_named(known_model(321).unwrap().profile),
            Some(MX106)
        );

        // The Protocol 1.0 MX-64 is a motor this build cannot drive, so it is
        // not claimed as known.
        assert_eq!(known_model(310), None);
    }

    /// The XL430 is not an X-series motor for any purpose this profile serves:
    /// no current sensor, a 3-cell lithium supply, and a quarter of the speed.
    #[test]
    fn the_xl430_no_longer_borrows_the_x_series_profile() {
        assert_eq!(
            profile_named(known_model(1060).unwrap().profile),
            Some(XL430)
        );
        assert_eq!(
            profile_named(known_model(1090).unwrap().profile),
            Some(XL430)
        );
        assert_eq!(XL430.voltage_rated, 11.1);
        assert_eq!(XL430.voltage_max, 12.0);
        // 57 rev/min against a family bound of 240.
        assert_eq!(XL430.max_velocity_rev_per_sec, 0.95);
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

    /// The P-series table is laid out so that the same two contiguous
    /// reads the X family uses — ten bytes through position, twenty-one
    /// through temperature — work unchanged. Nothing here is verified on
    /// hardware; the test pins the arithmetic, not the servo.
    #[test]
    fn the_p_series_table_keeps_the_present_block_contiguous() {
        for profile in [PH54, PM54] {
            assert_eq!(
                profile.addr_present_position + 4 - profile.addr_present_current,
                10
            );
            assert_eq!(
                profile.addr_present_temperature + 1 - profile.addr_present_current,
                21
            );
            assert_eq!(profile_named(&profile.name.to_lowercase()), Some(profile));
        }
        // And no model number resolves to one: unverified tables are chosen
        // on purpose or not at all.
        for model in 0..=4000u32 {
            if let Some(known) = known_model(model) {
                assert!(!known.profile.starts_with('p'), "{}", known.name);
            }
        }
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
