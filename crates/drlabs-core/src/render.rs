//! Rendering a quantity as text, in the space the reader is working in.
//!
//! Types keep [`Rev`] and [`Rad`] apart at compile time, which is most of the
//! problem solved. This is the rest of it: a number that is *about* to be shown
//! to somebody has to be shown in the units they wrote their limits in.
//!
//! A single-axis session speaks motor space — revolutions at the shaft — and
//! that is what [`MotorUnits`] renders. A robot's joints are authored in joint
//! space, and `runtime/robot` supplies a renderer that converts on the way out.
//! The safety layer holds one of these and never converts anything itself, so
//! there is exactly one place where the two spaces meet.
//!
//! The Go version's bug is worth remembering. A message built in the safety
//! layer reported a joint's position in revolutions to somebody who had written
//! the limit in radians, and at the precision printed the two numbers looked
//! identical. Nothing was wrong with the arithmetic; the sentence was in the
//! wrong language.

use crate::units::{Nm, Rev, RevPerSec, RevPerSec2};

/// Renders motor-space quantities as text for a reader.
///
/// [`Units::position`] and [`Units::span`] are separate on purpose. A position
/// is a *place*, and converting one carries the joint's zero offset; a span is
/// a *distance*, and converting one must not. Rendering "0.05 rev of margin"
/// through a position renderer would add the offset and report a margin near
/// the joint's zero rather than a width.
pub trait Units: Send + Sync {
    /// A place on the axis.
    fn position(&self, value: Rev) -> String;
    /// A distance along it — a margin, a tolerance, a move size.
    fn span(&self, value: Rev) -> String;
    fn velocity(&self, value: RevPerSec) -> String;
    fn accel(&self, value: RevPerSec2) -> String;
    fn torque(&self, value: Nm) -> String;
}

/// Motor space: revolutions at the shaft, torque at the rotor.
///
/// What a session driving one controller directly speaks.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorUnits;

impl Units for MotorUnits {
    fn position(&self, value: Rev) -> String {
        format!("{:.4} rev", value.value())
    }

    fn span(&self, value: Rev) -> String {
        format!("{:.4} rev", value.value())
    }

    fn velocity(&self, value: RevPerSec) -> String {
        format!("{:.3} rev/s", value.value())
    }

    fn accel(&self, value: RevPerSec2) -> String {
        format!("{:.3} rev/s²", value.value())
    }

    fn torque(&self, value: Nm) -> String {
        format!("{:.3} Nm", value.value())
    }
}

/// The renderer to use, defaulting to motor space.
///
/// A `None` is not a missing renderer, it is a positive statement: this layer
/// is working in motor space, which is what a single-axis session does.
pub fn render(units: Option<&dyn Units>) -> &dyn Units {
    match units {
        Some(units) => units,
        None => &MotorUnits,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_space_renders_revolutions() {
        assert_eq!(MotorUnits.position(Rev(0.4771)), "0.4771 rev");
        assert_eq!(MotorUnits.velocity(RevPerSec(1.5)), "1.500 rev/s");
        assert_eq!(MotorUnits.torque(Nm(0.25)), "0.250 Nm");
    }

    #[test]
    fn no_renderer_means_motor_space() {
        assert_eq!(render(None).position(Rev(1.0)), "1.0000 rev");
    }

    /// In motor space a place and a distance render alike. They do not in joint
    /// space, which is why they are separate methods rather than one.
    #[test]
    fn a_place_and_a_distance_are_asked_for_separately() {
        let units = render(None);
        assert_eq!(units.position(Rev(0.05)), units.span(Rev(0.05)));
    }

    /// A renderer that converts, to show the distinction the trait exists for.
    #[test]
    fn a_converting_renderer_offsets_a_place_but_not_a_distance() {
        struct Joint {
            ratio: f64,
            offset_rad: f64,
        }
        impl Units for Joint {
            fn position(&self, value: Rev) -> String {
                let rad = value.value() * std::f64::consts::TAU / self.ratio - self.offset_rad;
                format!("{rad:.3} rad")
            }
            fn span(&self, value: Rev) -> String {
                // No offset: a width is not a place.
                let rad = value.value() * std::f64::consts::TAU / self.ratio;
                format!("{rad:.3} rad")
            }
            fn velocity(&self, value: RevPerSec) -> String {
                format!(
                    "{:.3} rad/s",
                    value.value() * std::f64::consts::TAU / self.ratio
                )
            }
            fn accel(&self, value: RevPerSec2) -> String {
                format!(
                    "{:.3} rad/s²",
                    value.value() * std::f64::consts::TAU / self.ratio
                )
            }
            fn torque(&self, value: Nm) -> String {
                format!("{:.3} Nm", value.value() * self.ratio)
            }
        }

        let joint = Joint {
            ratio: 1.0,
            offset_rad: 1.0,
        };
        // The same number is a different sentence depending on what it means.
        assert_eq!(joint.position(Rev(0.0)), "-1.000 rad");
        assert_eq!(joint.span(Rev(0.0)), "0.000 rad");
    }
}
