//! The one place motor space and joint space meet.
//!
//! Everything below `drlabs-robot` works in **motor space**: revolutions at the
//! shaft, torque at the rotor. Everything above it is authored in **joint
//! space**: radians at the output, torque at the mechanism. A gear ratio, a
//! sign and a zero offset separate them.
//!
//! This module is the only place that converts. Do not add a second one. The
//! same axis is legitimately `0.477 rev` and `3.000 rad` at the same instant,
//! so a number without its space is not a fact.

use drlabs_core::units::{Nm, Rad, RadPerSec, RadPerSec2, Rev, RevPerSec, RevPerSec2};
use drlabs_safety::Config as SafetyConfig;

use crate::config::{JointConfig, JointLimits};

/// How one joint's output relates to its motor's shaft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mapping {
    /// Motor revolutions per output revolution. 1.0 is a direct drive.
    pub gear_ratio: f64,
    /// +1 or -1: which way the output turns for a positive motor rotation.
    pub direction: f64,
    /// Where the joint's zero sits, in output radians.
    pub offset_rad: f64,
}

impl Mapping {
    pub fn of(joint: &JointConfig) -> Self {
        Self {
            gear_ratio: joint.gear_ratio,
            direction: joint.direction,
            offset_rad: joint.offset_rad,
        }
    }

    /// A place at the output, as a place at the shaft.
    pub fn motor_position(&self, joint: Rad) -> Rev {
        Rad((joint.value() + self.offset_rad) * self.direction).to_rev() * self.gear_ratio
    }

    /// And back.
    pub fn joint_position(&self, motor: Rev) -> Rad {
        Rad((motor / self.gear_ratio).to_rad().value() * self.direction - self.offset_rad)
    }

    /// A *distance* at the output, as a distance at the shaft.
    ///
    /// Separate from [`Mapping::motor_position`] because a distance does not
    /// carry the zero offset. Converting a margin as though it were a place
    /// reports a width near the joint's zero instead of a width.
    pub fn motor_span(&self, joint: Rad) -> Rev {
        joint.to_rev() * self.gear_ratio
    }

    pub fn joint_span(&self, motor: Rev) -> Rad {
        (motor / self.gear_ratio).to_rad()
    }

    pub fn motor_velocity(&self, joint: RadPerSec) -> RevPerSec {
        RadPerSec(joint.value() * self.direction).to_rev_per_sec() * self.gear_ratio
    }

    pub fn joint_velocity(&self, motor: RevPerSec) -> RadPerSec {
        (motor / self.gear_ratio).to_rad_per_sec() * self.direction
    }

    /// Acceleration has no sign to carry — a limit is a magnitude.
    pub fn motor_accel(&self, joint: RadPerSec2) -> RevPerSec2 {
        joint.to_rev_per_sec2() * self.gear_ratio
    }

    pub fn joint_accel(&self, motor: RevPerSec2) -> RadPerSec2 {
        (motor / self.gear_ratio).to_rad_per_sec2()
    }

    /// Torque runs the other way through the gearbox: the mechanism sees the
    /// ratio multiplied, the rotor sees it divided.
    pub fn motor_torque(&self, joint: Nm) -> Nm {
        joint / self.gear_ratio
    }

    pub fn joint_torque(&self, motor: Nm) -> Nm {
        motor * self.gear_ratio
    }

    /// Turns a joint's declared limits into the motor-space envelope the safety
    /// layer enforces.
    pub fn safety_config(&self, limits: &JointLimits, base: SafetyConfig) -> SafetyConfig {
        let mut lower = self.motor_position(Rad(limits.min_rad));
        let mut upper = self.motor_position(Rad(limits.max_rad));
        // A joint with direction -1 maps its lower limit to the higher motor
        // position. The envelope is a range, not a pair of named ends.
        if upper < lower {
            std::mem::swap(&mut lower, &mut upper);
        }

        let mut config = base;
        config.position_limit_enabled = true;
        config.position_min = lower;
        config.position_max = upper;
        config.max_torque = self.motor_torque(Nm(limits.max_torque_nm));
        config.max_velocity = self
            .motor_velocity(RadPerSec(limits.max_velocity_rad_s))
            .abs();
        if limits.max_accel_rad_s2 > 0.0 {
            config.accel_limit = self.motor_accel(RadPerSec2(limits.max_accel_rad_s2)).abs();
        }

        // A robot's joints are commanded by a program, not by a browser, so the
        // operator-facing deadman has no meaning here. What protects the
        // machine if the upper layer dies is the controller's own watchdog,
        // which the session sends every cycle.
        config.deadman_timeout = 0.0;
        config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn direct() -> Mapping {
        Mapping {
            gear_ratio: 1.0,
            direction: 1.0,
            offset_rad: 0.0,
        }
    }

    /// The bench's own numbers: a qdd100 through a 19:1 reduction.
    fn reduced() -> Mapping {
        Mapping {
            gear_ratio: 19.0,
            direction: 1.0,
            offset_rad: 0.0,
        }
    }

    #[test]
    fn a_direct_drive_is_just_a_change_of_units() {
        let mapping = direct();
        assert!((mapping.motor_position(Rad(PI)).value() - 0.5).abs() < 1e-12);
        assert!((mapping.joint_position(Rev(0.5)).value() - PI).abs() < 1e-12);
    }

    /// The sentence from the project's own notes: the same axis is legitimately
    /// 0.477 rev and 3.000 rad at the same instant.
    #[test]
    fn one_axis_has_two_true_positions_at_once() {
        let mapping = reduced();
        let joint = Rad(3.0);
        let motor = mapping.motor_position(joint);
        assert!((motor.value() - 9.071).abs() < 0.001, "{motor}");
        assert!((mapping.joint_position(motor).value() - 3.0).abs() < 1e-12);
    }

    #[test]
    fn every_conversion_round_trips() {
        let mapping = Mapping {
            gear_ratio: 19.0,
            direction: -1.0,
            offset_rad: 0.7,
        };
        for value in [-2.0, -0.5, 0.0, 0.25, 3.0] {
            let back = mapping.joint_position(mapping.motor_position(Rad(value)));
            assert!((back.value() - value).abs() < 1e-12, "{value} -> {back}");

            let back = mapping.joint_velocity(mapping.motor_velocity(RadPerSec(value)));
            assert!((back.value() - value).abs() < 1e-12);

            let back = mapping.joint_torque(mapping.motor_torque(Nm(value)));
            assert!((back.value() - value).abs() < 1e-12);
        }
    }

    /// The distinction the renderer exists for: a place carries the offset, a
    /// distance does not.
    #[test]
    fn a_distance_does_not_carry_the_zero_offset() {
        let mapping = Mapping {
            gear_ratio: 1.0,
            direction: 1.0,
            offset_rad: 1.0,
        };
        // Zero at the output is one radian away from the joint's zero.
        assert!((mapping.motor_position(Rad(0.0)).to_rad().value() - 1.0).abs() < 1e-12);
        // But a width of zero is a width of zero.
        assert_eq!(mapping.motor_span(Rad(0.0)), Rev(0.0));
    }

    /// Torque runs the other way through a gearbox. Getting this backwards on a
    /// 19:1 joint would command 361 times the intended torque.
    #[test]
    fn torque_runs_the_other_way_through_the_gearbox() {
        let mapping = reduced();
        // 19 Nm wanted at the mechanism is 1 Nm at the rotor.
        assert!((mapping.motor_torque(Nm(19.0)).value() - 1.0).abs() < 1e-12);
        assert!((mapping.joint_torque(Nm(1.0)).value() - 19.0).abs() < 1e-12);
    }

    /// A reversed joint maps its lower limit to the higher motor position. An
    /// envelope written as [min, max] in one space is not one in the other
    /// until it is put back in order.
    #[test]
    fn a_reversed_joint_still_produces_an_ordered_envelope() {
        let mapping = Mapping {
            gear_ratio: 1.0,
            direction: -1.0,
            offset_rad: 0.0,
        };
        let limits = JointLimits {
            min_rad: -1.0,
            max_rad: 2.0,
            max_torque_nm: 1.0,
            max_velocity_rad_s: 1.0,
            max_accel_rad_s2: 0.0,
        };
        let config = mapping.safety_config(&limits, SafetyConfig::default());
        assert!(
            config.position_min < config.position_max,
            "[{}, {}] is not a range",
            config.position_min,
            config.position_max
        );
        // -1 rad at the output is +1 rad at the shaft, so the envelope is
        // [-2 rad, +1 rad] in motor space.
        assert!((config.position_max.to_rad().value() - 1.0).abs() < 1e-12);
    }

    /// The joint's declared ceiling becomes the motor's, converted — which is
    /// the whole reason the safety layer can stay in motor space.
    #[test]
    fn the_declared_limits_become_the_motor_space_envelope() {
        let mapping = reduced();
        let limits = JointLimits {
            min_rad: -0.5,
            max_rad: 0.5,
            max_torque_nm: 19.0,
            max_velocity_rad_s: std::f64::consts::TAU,
            max_accel_rad_s2: 0.0,
        };
        let config = mapping.safety_config(&limits, SafetyConfig::default());

        // ±0.5 rad through 19:1 is ±1.512 rev at the shaft.
        assert!((config.position_max.value() - 1.512).abs() < 0.001);
        assert!((config.max_torque.value() - 1.0).abs() < 1e-9);
        // One output revolution per second is nineteen at the shaft.
        assert!((config.max_velocity.value() - 19.0).abs() < 1e-6);
    }

    /// The deadman is an operator's guard. A robot's joints are commanded by a
    /// program, and the guard that matters there is the controller's watchdog.
    #[test]
    fn a_joint_has_no_operator_deadman() {
        let limits = JointLimits {
            min_rad: -1.0,
            max_rad: 1.0,
            max_torque_nm: 1.0,
            max_velocity_rad_s: 1.0,
            max_accel_rad_s2: 0.0,
        };
        let base = SafetyConfig::default();
        assert!(base.deadman_timeout > 0.0, "a session has one");
        assert_eq!(direct().safety_config(&limits, base).deadman_timeout, 0.0);
    }
}
