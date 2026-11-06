//! Host-side motion profiles.
//!
//! The runtime does not plan moves; a client does, and streams the samples
//! as setpoints. This lives in the core so a client needs no more than the
//! core to plan one. The arithmetic is unit-agnostic: the panel uses it in
//! radians at the joint, a motor-space caller in revolutions.
//!
//! Both CAN controller families can profile a move themselves — moteus from its
//! velocity and acceleration limits, RobStride from its CSP speed limit — and
//! when they can, letting them do it is better: the profile is regenerated at
//! the firmware's own rate rather than at whatever the host manages.
//!
//! This exists for the cases where that is not enough:
//!
//! * the controller has no profile generator in the mode being used, which is
//!   true of every operation/MIT-mode command;
//! * the move has to be repeatable and known in advance, so a test can compare
//!   what the axis did against what it was told;
//! * the host wants the velocity feedforward that comes with knowing where on
//!   the profile it is, which a bare position setpoint cannot express.

use crate::units::{Rev, RevPerSec, RevPerSec2};

/// One point on a profile.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    /// Seconds from the start.
    pub time: f64,
    pub position: Rev,
    pub velocity: RevPerSec,
    pub accel: RevPerSec2,
}

/// Constant acceleration, constant velocity, constant deceleration.
///
/// When the move is too short to reach the cruise speed the middle phase
/// vanishes and the profile becomes triangular, which is handled by computing
/// the cruise speed the distance actually allows rather than by special-casing
/// it later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trapezoidal {
    start: Rev,
    target: Rev,
    max_velocity: RevPerSec,
    accel: RevPerSec2,

    distance: f64,
    cruise_velocity: f64,
    accel_time: f64,
    cruise_time: f64,
    total_time: f64,
}

/// Why a profile could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ProfileError {
    #[error("the velocity limit must be positive")]
    VelocityNotPositive,
    #[error("the acceleration limit must be positive")]
    AccelNotPositive,
}

impl Trapezoidal {
    pub fn new(
        start: Rev,
        target: Rev,
        max_velocity: RevPerSec,
        accel: RevPerSec2,
    ) -> Result<Self, ProfileError> {
        if max_velocity.value() <= 0.0 {
            return Err(ProfileError::VelocityNotPositive);
        }
        if accel.value() <= 0.0 {
            return Err(ProfileError::AccelNotPositive);
        }

        let distance = (target - start).abs().value();
        let (cruise_velocity, accel_time, cruise_time) = if distance == 0.0 {
            (0.0, 0.0, 0.0)
        } else {
            let time_to_max = max_velocity.value() / accel.value();
            // The distance spent getting up to speed and back down again.
            let ramp_distance = max_velocity.value() * time_to_max;
            if ramp_distance > distance {
                // Too short to reach the cruise speed: a triangle.
                let peak = (accel.value() * distance).sqrt();
                (peak, peak / accel.value(), 0.0)
            } else {
                let cruise_distance = distance - ramp_distance;
                (
                    max_velocity.value(),
                    time_to_max,
                    cruise_distance / max_velocity.value(),
                )
            }
        };

        Ok(Self {
            start,
            target,
            max_velocity,
            accel,
            distance,
            cruise_velocity,
            accel_time,
            cruise_time,
            total_time: 2.0 * accel_time + cruise_time,
        })
    }

    pub fn start(&self) -> Rev {
        self.start
    }

    pub fn target(&self) -> Rev {
        self.target
    }

    pub fn max_velocity(&self) -> RevPerSec {
        self.max_velocity
    }

    pub fn accel(&self) -> RevPerSec2 {
        self.accel
    }

    /// The profile's duration, in seconds.
    pub fn total_time(&self) -> f64 {
        self.total_time
    }

    /// Whether this profile reaches its cruise speed.
    pub fn is_triangular(&self) -> bool {
        self.cruise_time == 0.0 && self.distance > 0.0
    }

    /// Where the axis should be `t` seconds after the start, and how fast.
    pub fn sample(&self, t: f64) -> Point {
        if t <= 0.0 {
            return Point {
                time: 0.0,
                position: self.start,
                ..Default::default()
            };
        }
        if t >= self.total_time {
            return Point {
                time: self.total_time,
                position: self.target,
                ..Default::default()
            };
        }

        let direction = if self.target < self.start { -1.0 } else { 1.0 };
        let accel = self.accel.value();

        let (distance, velocity, acceleration) = if t <= self.accel_time {
            (0.5 * accel * t * t, accel * t, accel)
        } else if t <= self.accel_time + self.cruise_time {
            let ramp = 0.5 * accel * self.accel_time * self.accel_time;
            let cruising = t - self.accel_time;
            (
                ramp + self.cruise_velocity * cruising,
                self.cruise_velocity,
                0.0,
            )
        } else {
            let ramp = 0.5 * accel * self.accel_time * self.accel_time;
            let cruised = ramp + self.cruise_velocity * self.cruise_time;
            let slowing = t - self.accel_time - self.cruise_time;
            (
                cruised + self.cruise_velocity * slowing - 0.5 * accel * slowing * slowing,
                self.cruise_velocity - accel * slowing,
                -accel,
            )
        };

        Point {
            time: t,
            position: Rev(self.start.value() + direction * distance),
            velocity: RevPerSec(direction * velocity),
            accel: RevPerSec2(direction * acceleration),
        }
    }

    /// The whole profile, sampled at a rate.
    pub fn generate(&self, sample_rate_hz: f64) -> Vec<Point> {
        if sample_rate_hz <= 0.0 {
            return Vec::new();
        }
        let step = 1.0 / sample_rate_hz;
        let count = (self.total_time / step).ceil() as usize;
        let mut points: Vec<Point> = (0..=count).map(|i| self.sample(i as f64 * step)).collect();
        // The last sample lands on or past the end; make it exactly the end, so
        // a caller replaying the profile finishes at the target.
        if let Some(last) = points.last_mut() {
            *last = self.sample(self.total_time);
        }
        points
    }
}

/// One axis's share of a coordinated move: where it is, where it goes, and
/// the most it may be asked to do on the way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Leg {
    pub start: Rev,
    pub target: Rev,
    pub max_velocity: RevPerSec,
    pub accel: RevPerSec2,
}

/// Plans several axes so that they leave together and arrive together.
///
/// Planned independently, each axis finishes when its own distance and limits
/// say, so a short move lands early and the joints take different paths
/// through the space between. Here the slowest axis keeps its natural profile
/// and every other axis is stretched in time to match it.
///
/// A profile stretched in time by a factor `s` is still a trapezoid: the same
/// shape with the velocity limit divided by `s` and the acceleration limit by
/// `s²`. That keeps every axis inside the limits it was given — time is only
/// ever added, never removed — and makes the arrival times equal by
/// construction rather than by tuning. An axis that does not move stays put.
///
/// The result is in the order of `legs`. Any leg with unusable limits fails
/// the whole plan, so a move is never started on half the machine.
pub fn synchronized(legs: &[Leg]) -> Result<Vec<Trapezoidal>, ProfileError> {
    let natural = legs
        .iter()
        .map(|leg| Trapezoidal::new(leg.start, leg.target, leg.max_velocity, leg.accel))
        .collect::<Result<Vec<_>, _>>()?;
    let total = natural
        .iter()
        .map(Trapezoidal::total_time)
        .fold(0.0, f64::max);

    legs.iter()
        .zip(&natural)
        .map(|(leg, profile)| {
            let own = profile.total_time();
            // The slowest axis, and any axis with nowhere to go, keep theirs.
            if own == 0.0 || own >= total {
                return Ok(*profile);
            }
            let stretch = total / own;
            Trapezoidal::new(
                leg.start,
                leg.target,
                RevPerSec(leg.max_velocity.value() / stretch),
                RevPerSec2(leg.accel.value() / (stretch * stretch)),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leg(start: f64, target: f64, velocity: f64, accel: f64) -> Leg {
        Leg {
            start: Rev(start),
            target: Rev(target),
            max_velocity: RevPerSec(velocity),
            accel: RevPerSec2(accel),
        }
    }

    /// The point of the exercise: a long move, a short one and a triangle
    /// finish at the same instant, each exactly on its own target.
    #[test]
    fn synchronized_axes_arrive_together() {
        let legs = [
            leg(0.0, 3.0, 1.0, 2.0),  // long: trapezoid, the slowest
            leg(0.0, 0.5, 1.0, 2.0),  // shorter: stretched
            leg(1.0, 0.98, 4.0, 1.0), // tiny and backwards: a triangle
        ];
        let profiles = synchronized(&legs).unwrap();
        let slowest = profiles[0].total_time();
        for (profile, leg) in profiles.iter().zip(&legs) {
            assert!(
                (profile.total_time() - slowest).abs() < 1e-9,
                "{} against {slowest}",
                profile.total_time()
            );
            assert_eq!(profile.sample(profile.total_time()).position, leg.target);
        }
    }

    /// Stretching must only ever slow an axis down. Sampled finely, no axis
    /// exceeds the velocity or acceleration it was allowed.
    #[test]
    fn synchronizing_never_exceeds_a_limit() {
        let legs = [
            leg(0.0, 3.0, 1.0, 2.0),
            leg(0.0, 0.5, 1.0, 2.0),
            leg(1.0, 0.98, 4.0, 1.0),
        ];
        let profiles = synchronized(&legs).unwrap();
        for (profile, leg) in profiles.iter().zip(&legs) {
            for point in profile.generate(2000.0) {
                assert!(point.velocity.value().abs() <= leg.max_velocity.value() + 1e-9);
                assert!(point.accel.value().abs() <= leg.accel.value() + 1e-9);
            }
        }
    }

    /// The slowest axis is not touched: it is the reference the rest are
    /// stretched to, and its profile is the one it would have had alone.
    #[test]
    fn the_slowest_axis_keeps_its_own_profile() {
        let legs = [leg(0.0, 3.0, 1.0, 2.0), leg(0.0, 0.5, 1.0, 2.0)];
        let profiles = synchronized(&legs).unwrap();
        let alone = Trapezoidal::new(Rev(0.0), Rev(3.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        assert_eq!(profiles[0], alone);
    }

    /// An axis that is already where it is told to go has no profile to
    /// stretch; it must not turn a zero time into a division by zero.
    #[test]
    fn an_axis_with_nowhere_to_go_stays_put() {
        let legs = [leg(0.0, 2.0, 1.0, 2.0), leg(0.7, 0.7, 1.0, 2.0)];
        let profiles = synchronized(&legs).unwrap();
        assert_eq!(profiles[1].total_time(), 0.0);
        assert_eq!(profiles[1].sample(0.5).position, Rev(0.7));
        assert!(profiles[0].total_time() > 0.0);
    }

    #[test]
    fn no_axes_is_an_empty_plan() {
        assert_eq!(synchronized(&[]).unwrap(), Vec::new());
    }

    /// One bad limit refuses the whole plan.
    #[test]
    fn one_unusable_limit_refuses_the_whole_move() {
        let legs = [leg(0.0, 1.0, 1.0, 2.0), leg(0.0, 1.0, 0.0, 2.0)];
        assert_eq!(synchronized(&legs), Err(ProfileError::VelocityNotPositive));
    }

    #[test]
    fn a_profile_starts_where_it_says_and_ends_where_it_was_told() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        assert_eq!(profile.sample(0.0).position, Rev(0.0));
        assert_eq!(profile.sample(profile.total_time()).position, Rev(1.0));
        // And past the end it stays there rather than continuing.
        assert_eq!(profile.sample(1000.0).position, Rev(1.0));
    }

    /// The three phases, and the shape that gives the profile its name.
    #[test]
    fn a_long_move_accelerates_cruises_and_decelerates() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(10.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        assert!(!profile.is_triangular());

        // Accelerating.
        assert!(profile.sample(0.1).accel.value() > 0.0);
        // Cruising: at the limit, with no acceleration.
        let cruising = profile.sample(profile.total_time() / 2.0);
        assert!((cruising.velocity.value() - 1.0).abs() < 1e-9);
        assert_eq!(cruising.accel, RevPerSec2(0.0));
        // Slowing.
        assert!(profile.sample(profile.total_time() - 0.1).accel.value() < 0.0);
    }

    /// A move too short to reach the cruise speed is a triangle, and it must
    /// still finish exactly at the target.
    #[test]
    fn a_short_move_becomes_a_triangle() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(0.01), RevPerSec(10.0), RevPerSec2(1.0)).unwrap();
        assert!(profile.is_triangular());
        assert!(
            profile.max_velocity().value()
                > profile.sample(profile.total_time() / 2.0).velocity.value(),
            "a triangle never reaches the ceiling"
        );
        assert!((profile.sample(profile.total_time()).position.value() - 0.01).abs() < 1e-12);
    }

    /// A move backwards is the same profile with the sign flipped, not a
    /// different one.
    #[test]
    fn a_move_backwards_mirrors_the_profile() {
        let forward =
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        let backward =
            Trapezoidal::new(Rev(0.0), Rev(-1.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();

        assert!((forward.total_time() - backward.total_time()).abs() < 1e-12);
        let t = forward.total_time() / 3.0;
        assert!(
            (forward.sample(t).position.value() + backward.sample(t).position.value()).abs()
                < 1e-12
        );
        assert!(backward.sample(t).velocity.value() < 0.0);
    }

    /// A move to where the axis already is takes no time and asks for no
    /// motion. Without this, a "go to the current position" command would
    /// divide by a zero distance.
    #[test]
    fn a_move_of_no_distance_is_already_finished() {
        let profile =
            Trapezoidal::new(Rev(0.5), Rev(0.5), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        assert_eq!(profile.total_time(), 0.0);
        assert_eq!(profile.sample(0.0).position, Rev(0.5));
        assert_eq!(profile.sample(1.0).position, Rev(0.5));
    }

    /// The position the profile reports has to be the integral of the velocity
    /// it reports, or the feedforward is describing a different move than the
    /// setpoint is.
    #[test]
    fn the_velocity_integrates_to_the_position() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(2.0), RevPerSec(1.5), RevPerSec2(3.0)).unwrap();

        let steps = 20_000;
        let dt = profile.total_time() / steps as f64;
        let mut integrated = 0.0;
        for i in 0..steps {
            // Midpoint, which is exact for the piecewise-linear velocity here.
            integrated += profile.sample((i as f64 + 0.5) * dt).velocity.value() * dt;
        }
        assert!(
            (integrated - 2.0).abs() < 1e-6,
            "integrated {integrated}, expected 2.0"
        );
    }

    /// Limits that leave no room to move are refused rather than producing a
    /// profile that divides by zero.
    #[test]
    fn a_profile_with_no_room_to_move_is_refused() {
        assert_eq!(
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(0.0), RevPerSec2(1.0)),
            Err(ProfileError::VelocityNotPositive)
        );
        assert_eq!(
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(0.0)),
            Err(ProfileError::AccelNotPositive)
        );
        assert_eq!(
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(-1.0)),
            Err(ProfileError::AccelNotPositive)
        );
    }

    #[test]
    fn generating_the_whole_profile_ends_at_the_target() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        let points = profile.generate(100.0);
        assert!(points.len() > 10);
        assert_eq!(points[0].position, Rev(0.0));
        assert_eq!(points.last().unwrap().position, Rev(1.0));
    }
}
