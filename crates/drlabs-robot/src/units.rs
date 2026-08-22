//! Saying a motor-space number in joint space.
//!
//! A joint's limits were written in radians at the output. When the safety
//! layer refuses to arm one, the sentence it produces has to be in those units
//! — otherwise the operator reads a number, compares it against the limit they
//! wrote, and the two do not match even though nothing is wrong.
//!
//! The Go version got this wrong once: a joint's position was reported in
//! revolutions to somebody who had written the limit in radians, and at the
//! precision printed the two looked identical.

use drlabs_core::render::Units;
use drlabs_core::units::{Nm, Rev, RevPerSec, RevPerSec2};

use crate::joint::Mapping;

/// Renders motor-space quantities as joint-space text.
#[derive(Debug, Clone, Copy)]
pub struct JointUnits {
    pub mapping: Mapping,
}

impl JointUnits {
    pub fn new(mapping: Mapping) -> Self {
        Self { mapping }
    }
}

impl Units for JointUnits {
    /// A place, which carries the joint's zero offset.
    fn position(&self, value: Rev) -> String {
        format!("{:.4} rad", self.mapping.joint_position(value).value())
    }

    /// A distance, which does not.
    fn span(&self, value: Rev) -> String {
        format!("{:.4} rad", self.mapping.joint_span(value).value())
    }

    fn velocity(&self, value: RevPerSec) -> String {
        format!("{:.2} rad/s", self.mapping.joint_velocity(value).value())
    }

    fn accel(&self, value: RevPerSec2) -> String {
        format!("{:.2} rad/s²", self.mapping.joint_accel(value).value())
    }

    fn torque(&self, value: Nm) -> String {
        format!("{:.2} Nm", self.mapping.joint_torque(value).value())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::render::MotorUnits;

    fn reduced() -> JointUnits {
        JointUnits::new(Mapping {
            gear_ratio: 19.0,
            direction: 1.0,
            offset_rad: 0.0,
        })
    }

    /// The same axis, said two ways. Both are true; only one of them is the
    /// answer to "where is my joint".
    #[test]
    fn one_position_reads_differently_in_each_space() {
        let units = reduced();
        let motor = units.mapping.motor_position(drlabs_core::units::Rad(3.0));
        assert_eq!(MotorUnits.position(motor), "9.0718 rev");
        assert_eq!(units.position(motor), "3.0000 rad");
    }

    /// A margin is a width. Rendering it as a place would add the joint's zero
    /// offset and report a margin that sits near zero rather than a width.
    #[test]
    fn a_margin_is_rendered_as_a_width_not_as_a_place() {
        let units = JointUnits::new(Mapping {
            gear_ratio: 1.0,
            direction: 1.0,
            offset_rad: 4.5974, // the bench XL330's offset
        });

        // Zero at the shaft is a long way from the joint's zero…
        assert_eq!(units.position(Rev(0.0)), "-4.5974 rad");
        // …but a width of zero is still zero.
        assert_eq!(units.span(Rev(0.0)), "0.0000 rad");
    }

    /// Torque goes the other way through the gearbox, so a rotor's newton-metre
    /// is nineteen at the mechanism.
    #[test]
    fn torque_is_reported_at_the_mechanism() {
        assert_eq!(reduced().torque(Nm(1.0)), "19.00 Nm");
        assert_eq!(MotorUnits.torque(Nm(1.0)), "1.000 Nm");
    }

    #[test]
    fn velocity_and_acceleration_are_reported_at_the_output() {
        let units = reduced();
        // Nineteen shaft revolutions a second is one output revolution: 2π rad/s.
        assert_eq!(units.velocity(RevPerSec(19.0)), "6.28 rad/s");
        assert_eq!(units.accel(RevPerSec2(19.0)), "6.28 rad/s²");
    }

    /// A reversed joint reports the sign its operator expects, not the shaft's.
    #[test]
    fn a_reversed_joint_reports_the_sign_the_operator_wrote() {
        let units = JointUnits::new(Mapping {
            gear_ratio: 1.0,
            direction: -1.0,
            offset_rad: 0.0,
        });
        assert!(units.position(Rev(0.5)).starts_with('-'));
    }
}
