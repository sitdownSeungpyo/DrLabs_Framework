//! What a controller reports, in framework units.
//!
//! Every optional measurement is an `Option`, not a NaN. The Go framework used
//! NaN and a `Valid()` helper, and the documentation had to keep saying why:
//!
//! > Optional measurements are null when the driver did not read them, never
//! > 0. A caller that reads a missing temperature as 0 °C will draw the wrong
//! > conclusion from it.
//!
//! An `Option` says that in the type. A caller cannot do arithmetic on a
//! missing reading without first deciding what to do about it being missing.

use crate::units::{Amps, Celsius, Nm, Rev, RevPerSec, Volts};

/// One controller's reported state.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Telemetry {
    /// The vendor's own mode register, kept raw so that driver-specific names
    /// and fault semantics stay meaningful.
    pub mode: Option<i32>,
    pub mode_name: String,

    pub fault: Option<Fault>,

    pub position: Option<Rev>,
    pub velocity: Option<RevPerSec>,
    pub torque: Option<Nm>,
    pub q_current: Option<Amps>,
    pub d_current: Option<Amps>,
    pub abs_position: Option<Rev>,
    pub power_w: Option<f64>,

    pub voltage: Option<Volts>,
    pub board_temperature: Option<Celsius>,
    pub motor_temperature: Option<Celsius>,

    pub trajectory_complete: Option<bool>,
    pub home_state: Option<i32>,

    /// The controller's *active* command registers, read back so the host can
    /// verify that the limits it believes it set are the ones in force. Only a
    /// [`crate::QueryProfile::Verify`] cycle fills these in.
    pub active: ActiveLimits,
}

/// A fault code and what it means.
///
/// `hard` is the distinction that matters most in practice: a moteus reporting
/// "limiting: max torque" is not faulted, it is telling you what it is capping,
/// and stopping the machine for it would make a correctly torque-limited axis
/// unusable.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Fault {
    pub code: i32,
    pub name: String,
    pub detail: String,
    pub hard: bool,
}

impl Fault {
    pub fn none() -> Self {
        Self {
            code: 0,
            name: "No fault".into(),
            detail: String::new(),
            hard: false,
        }
    }

    pub fn is_clear(&self) -> bool {
        self.code == 0
    }
}

/// The command registers as the controller currently holds them.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActiveLimits {
    pub max_torque: Option<Nm>,
    pub watchdog: Option<f64>,
    pub velocity_limit: Option<RevPerSec>,
    pub accel_limit: Option<RevPerSec2Compat>,
}

/// Acceleration read back from a controller.
///
/// Named separately only so that [`ActiveLimits`] stays `Copy` without pulling
/// the whole unit module into its signature; it is the same quantity as
/// [`crate::units::RevPerSec2`].
pub type RevPerSec2Compat = crate::units::RevPerSec2;

impl Telemetry {
    /// Overlays the fields `other` actually reported, leaving the rest alone.
    ///
    /// Rotating query profiles means one cycle only refreshes part of the
    /// picture. Merging keeps the last known value for the rest instead of
    /// blanking it — but only for values that were *measured*, so a field that
    /// has never been read stays unread rather than becoming plausible.
    pub fn merge(&mut self, other: &Telemetry) {
        if other.mode.is_some() {
            self.mode = other.mode;
            self.mode_name = other.mode_name.clone();
        }
        if other.fault.is_some() {
            self.fault = other.fault.clone();
        }
        if other.trajectory_complete.is_some() {
            self.trajectory_complete = other.trajectory_complete;
        }
        if other.home_state.is_some() {
            self.home_state = other.home_state;
        }

        merge(&mut self.position, other.position);
        merge(&mut self.velocity, other.velocity);
        merge(&mut self.torque, other.torque);
        merge(&mut self.q_current, other.q_current);
        merge(&mut self.d_current, other.d_current);
        merge(&mut self.abs_position, other.abs_position);
        merge(&mut self.power_w, other.power_w);
        merge(&mut self.voltage, other.voltage);
        merge(&mut self.board_temperature, other.board_temperature);
        merge(&mut self.motor_temperature, other.motor_temperature);

        merge(&mut self.active.max_torque, other.active.max_torque);
        merge(&mut self.active.watchdog, other.active.watchdog);
        merge(&mut self.active.velocity_limit, other.active.velocity_limit);
        merge(&mut self.active.accel_limit, other.active.accel_limit);
    }
}

#[inline]
fn merge<T: Copy>(destination: &mut Option<T>, source: Option<T>) {
    if source.is_some() {
        *destination = source;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_measurement_is_not_zero() {
        let telemetry = Telemetry::default();
        assert_eq!(telemetry.torque, None);
        assert_ne!(telemetry.torque, Some(Nm::ZERO));
    }

    /// A fast cycle refreshes part of the picture; the rest must survive it.
    #[test]
    fn merging_keeps_what_the_narrow_profile_did_not_report() {
        let mut wide = Telemetry {
            position: Some(Rev(0.25)),
            voltage: Some(Volts(24.0)),
            motor_temperature: Some(Celsius(31.0)),
            ..Default::default()
        };

        let narrow = Telemetry {
            position: Some(Rev(0.30)),
            ..Default::default()
        };
        wide.merge(&narrow);

        assert_eq!(wide.position, Some(Rev(0.30)), "the fresh value wins");
        assert_eq!(wide.voltage, Some(Volts(24.0)), "the old value survives");
        assert_eq!(wide.motor_temperature, Some(Celsius(31.0)));
    }

    /// Merging must not invent a reading for a field nobody has ever read.
    #[test]
    fn merging_does_not_conjure_an_unread_field() {
        let mut blank = Telemetry::default();
        blank.merge(&Telemetry {
            position: Some(Rev(0.1)),
            ..Default::default()
        });
        assert_eq!(blank.torque, None);
        assert_eq!(blank.voltage, None);
    }

    /// The moteus limiting states are not faults, and treating them as faults
    /// makes a torque-limited axis unusable.
    #[test]
    fn a_limiting_state_is_not_a_hard_fault() {
        let limiting = Fault {
            code: 102,
            name: "Limiting: max torque".into(),
            detail: String::new(),
            hard: false,
        };
        assert!(!limiting.is_clear());
        assert!(!limiting.hard);
    }
}
