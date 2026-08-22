//! Host-side motion profiles.
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

use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};

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

#[cfg(test)]
mod tests {
    use super::*;

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
