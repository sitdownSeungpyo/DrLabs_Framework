//! The units every driver and every layer above them agrees on.
//!
//! The Go framework carried these as bare `f64` and relied on comments and
//! field names to keep motor space and joint space apart. That worked until it
//! did not: a message built at the safety layer reported a joint's position in
//! revolutions to somebody who had written the limit in radians, and the two
//! numbers agreed to the digits printed while differing where it mattered.
//!
//! Here the space is part of the type. A [`Rev`] cannot be handed to something
//! expecting [`Rad`] — the conversion has to be written, and the only place
//! that should be writing it is a joint's mapping.

use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// Builds a newtype over `f64` with the arithmetic a physical quantity needs
/// and none of the arithmetic it does not: two positions may be subtracted,
/// but a position times a position is not a thing.
macro_rules! quantity {
    ($name:ident, $unit:literal, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
        pub struct $name(pub f64);

        impl $name {
            pub const ZERO: Self = Self(0.0);

            #[inline]
            pub fn value(self) -> f64 {
                self.0
            }

            #[inline]
            pub fn abs(self) -> Self {
                Self(self.0.abs())
            }

            #[inline]
            pub fn is_finite(self) -> bool {
                self.0.is_finite()
            }

            /// Constrains the value to `[lo, hi]`.
            #[inline]
            pub fn clamp_to(self, lo: Self, hi: Self) -> Self {
                Self(self.0.clamp(lo.0, hi.0))
            }

            /// Clamps, and says whether the value was altered — so a caller can
            /// tell the operator their request was limited rather than
            /// silently obeying something they did not ask for.
            #[inline]
            pub fn clamp_changed(self, lo: Self, hi: Self) -> (Self, bool) {
                let out = self.clamp_to(lo, hi);
                (out, out.0 != self.0)
            }
        }

        impl Add for $name {
            type Output = Self;
            #[inline]
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $name {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        impl Neg for $name {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }

        /// Scaling by a bare number is allowed: a gear ratio is a number.
        impl Mul<f64> for $name {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: f64) -> Self {
                Self(self.0 * rhs)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;
            #[inline]
            fn div(self, rhs: f64) -> Self {
                Self(self.0 / rhs)
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                let digits = f.precision().unwrap_or(4);
                write!(f, "{:.*} {}", digits, self.0, $unit)
            }
        }
    };
}

quantity!(Rev, "rev", "A position or a distance in motor revolutions.");
quantity!(Rad, "rad", "A position or a distance in joint radians.");
quantity!(RevPerSec, "rev/s", "A speed at the motor shaft.");
quantity!(RadPerSec, "rad/s", "A speed at the joint output.");
quantity!(RevPerSec2, "rev/s²", "An acceleration at the motor shaft.");
quantity!(RadPerSec2, "rad/s²", "An acceleration at the joint output.");
quantity!(Nm, "Nm", "A torque. Which side of a gearbox it refers to is the caller's to track.");
quantity!(Amps, "A", "A current.");
quantity!(Volts, "V", "A voltage.");
quantity!(Celsius, "°C", "A temperature.");

const TAU: f64 = std::f64::consts::TAU;

impl Rev {
    #[inline]
    pub fn to_rad(self) -> Rad {
        Rad(self.0 * TAU)
    }
    #[inline]
    pub fn to_deg(self) -> f64 {
        self.0 * 360.0
    }
}

impl Rad {
    #[inline]
    pub fn to_rev(self) -> Rev {
        Rev(self.0 / TAU)
    }
    #[inline]
    pub fn to_deg(self) -> f64 {
        self.0.to_degrees()
    }
}

impl RevPerSec {
    #[inline]
    pub fn to_rad_per_sec(self) -> RadPerSec {
        RadPerSec(self.0 * TAU)
    }
    #[inline]
    pub fn to_rpm(self) -> f64 {
        self.0 * 60.0
    }
    #[inline]
    pub fn from_rpm(rpm: f64) -> Self {
        Self(rpm / 60.0)
    }
}

impl RadPerSec {
    #[inline]
    pub fn to_rev_per_sec(self) -> RevPerSec {
        RevPerSec(self.0 / TAU)
    }
}

impl RevPerSec2 {
    #[inline]
    pub fn to_rad_per_sec2(self) -> RadPerSec2 {
        RadPerSec2(self.0 * TAU)
    }
}

impl RadPerSec2 {
    #[inline]
    pub fn to_rev_per_sec2(self) -> RevPerSec2 {
        RevPerSec2(self.0 / TAU)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_revolution_is_tau_radians() {
        assert!((Rev(1.0).to_rad().value() - TAU).abs() < 1e-12);
        assert!((Rad(TAU).to_rev().value() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn conversions_round_trip() {
        for raw in [-5.25, -0.001, 0.0, 0.4768, 3.0, 91.5] {
            let there_and_back = Rev(raw).to_rad().to_rev();
            assert!((there_and_back.value() - raw).abs() < 1e-12, "{raw}");
        }
    }

    /// The bound that could not be printed as itself.
    ///
    /// A joint limit of 3.0 rad is 0.477465 rev, and the axis settled at
    /// 0.476810 — a difference the old `%.3f` rendering hid by printing both
    /// as "0.477".
    #[test]
    fn a_joint_limit_and_a_settled_position_are_distinguishable() {
        let limit = Rad(3.0).to_rev();
        let settled = Rev(0.476810);
        assert!(settled < limit);
        assert_ne!(format!("{limit:.4}"), format!("{settled:.4}"));
    }

    #[test]
    fn clamping_reports_whether_it_bit() {
        let (value, changed) = Rad(99.0).clamp_changed(Rad(-1.2), Rad(1.2));
        assert_eq!(value, Rad(1.2));
        assert!(changed);

        let (value, changed) = Rad(0.5).clamp_changed(Rad(-1.2), Rad(1.2));
        assert_eq!(value, Rad(0.5));
        assert!(!changed);
    }

    #[test]
    fn rpm_matches_the_dynamixel_register_scale() {
        // 0.229 RPM per count; one rev/s is 60 RPM, about 262 counts.
        let counts = RevPerSec(1.0).to_rpm() / 0.229;
        assert!((counts - 262.0).abs() < 1.0, "{counts}");
    }
}
