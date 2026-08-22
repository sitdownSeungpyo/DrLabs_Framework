//! The A↔B shuttle: an unattended back-and-forth between two positions, for
//! burn-in, endurance runs and mechanism testing.
//!
//! The sequencer runs inside the control loop rather than in an application on
//! top of it. That is the whole point: its timing does not depend on a UI
//! staying responsive, and it stops the instant the axis is disarmed, faulted
//! or emergency-stopped, because it is evaluated by the same code that decides
//! whether to command anything at all.
//!
//! Unlike the Go version this is a state machine in its own right rather than a
//! set of fields on the session, so it can be tested by stepping it — which is
//! how the leg timeout and the cycle counting below are checked without a motor.

use std::time::Instant;

use drlabs_core::units::{Rev, RevPerSec};

/// Tolerances for calling a leg finished.
///
/// Both matter: position alone would count a fast pass through the target as an
/// arrival, and the axis would turn round while still travelling.
pub const TOLERANCE: Rev = Rev(0.0028); // about one degree
pub const STILLNESS: RevPerSec = RevPerSec(0.05);

/// Which end the shuttle is heading for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    A,
    B,
}

impl End {
    pub fn label(self) -> &'static str {
        match self {
            End::A => "A",
            End::B => "B",
        }
    }

    fn other(self) -> Self {
        match self {
            End::A => End::B,
            End::B => End::A,
        }
    }
}

/// What the sequencer is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Idle,
    /// The initial run to A, which is not counted as a cycle.
    Approach,
    Moving,
    Dwell,
}

impl Phase {
    pub fn name(self) -> &'static str {
        match self {
            Phase::Idle => "idle",
            Phase::Approach => "approach",
            Phase::Moving => "moving",
            Phase::Dwell => "dwell",
        }
    }
}

/// What one step of the sequencer decided.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// Not running; the caller keeps whatever setpoint it had.
    Idle,
    /// Command this position.
    Target(Rev),
    /// The requested cycles are done.
    Finished { reason: String, target: Rev },
    /// Something went wrong and the sequence has stopped.
    Failed(String),
}

/// The A↔B sequencer.
#[derive(Debug, Clone)]
pub struct Shuttle {
    pub running: bool,
    pub a: Rev,
    pub b: Rev,
    /// Seconds to hold at each end.
    pub dwell: f64,
    /// Zero runs until stopped.
    pub cycles: u32,
    /// Full A→B→A cycles finished.
    pub completed: u32,

    pub phase: Phase,
    pub target: End,
    /// Human readable progress.
    pub status: String,

    was_approach: bool,
    arrived_at: Option<Instant>,
    leg_started_at: Option<Instant>,
    legs: u32,
}

impl Default for Shuttle {
    fn default() -> Self {
        Self {
            running: false,
            a: Rev(-0.25), // -90°
            b: Rev(0.25),  // +90°
            dwell: 1.0,
            cycles: 0,
            completed: 0,
            phase: Phase::Idle,
            target: End::A,
            status: String::new(),
            was_approach: false,
            arrived_at: None,
            leg_started_at: None,
            legs: 0,
        }
    }
}

impl Shuttle {
    /// Sets the endpoints and timing without starting it.
    pub fn configure(&mut self, a: Rev, b: Rev, dwell: f64, cycles: u32) {
        self.a = a;
        self.b = b;
        self.dwell = dwell.clamp(0.0, 600.0);
        self.cycles = cycles;
    }

    /// Begins the sequence, running first to A.
    ///
    /// Returns an error when the endpoints are the same, because a shuttle
    /// between one position and itself never leaves the dwell phase and looks
    /// from the outside like an axis that has hung.
    pub fn start(&mut self, now: Instant) -> Result<(), &'static str> {
        if (self.b - self.a).abs().value() < 1e-4 {
            return Err("A and B are the same position");
        }
        self.running = true;
        self.completed = 0;
        self.legs = 0;
        self.target = End::A;
        self.phase = Phase::Approach;
        self.was_approach = false;
        self.arrived_at = None;
        self.leg_started_at = Some(now);
        self.status = "moving to A".into();
        Ok(())
    }

    pub fn stop(&mut self, reason: &str) {
        self.running = false;
        self.phase = Phase::Idle;
        self.arrived_at = None;
        self.leg_started_at = None;
        self.status = reason.to_string();
    }

    fn target_position(&self) -> Rev {
        match self.target {
            End::A => self.a,
            End::B => self.b,
        }
    }

    /// How long a leg may take before the sequence gives up.
    ///
    /// Generous, and derived from the move rather than fixed: a slow axis
    /// crossing a wide span legitimately takes a while, and a timeout that does
    /// not know the span would stop it every time.
    pub fn leg_timeout(&self, velocity_limit: RevPerSec) -> f64 {
        let span = (self.b - self.a).abs().value();
        let speed = velocity_limit.value().max(0.05);
        (span / speed * 4.0 + 5.0).clamp(5.0, 300.0)
    }

    /// Advances the sequence by one control cycle.
    ///
    /// `position` and `velocity` are the axis's own; `None` for either means it
    /// is not being reported, in which case the current target is held rather
    /// than the sequence being abandoned — one missing telemetry sample is a
    /// dropped cycle, not a fault.
    pub fn advance(
        &mut self,
        now: Instant,
        position: Option<Rev>,
        velocity: Option<RevPerSec>,
        velocity_limit: RevPerSec,
    ) -> Step {
        if !self.running {
            return Step::Idle;
        }
        let target = self.target_position();
        let label = self.target.label();

        let position = match position {
            Some(position) => position,
            None => return Step::Target(target),
        };
        let velocity = velocity.unwrap_or(RevPerSec(0.0));

        let arrived = (position - target).abs() <= TOLERANCE && velocity.abs() <= STILLNESS;

        match self.phase {
            Phase::Idle => Step::Idle,

            Phase::Approach | Phase::Moving => {
                self.status = format!("moving to {label}");
                if arrived {
                    self.was_approach = self.phase == Phase::Approach;
                    self.phase = Phase::Dwell;
                    self.arrived_at = Some(now);
                    return Step::Target(target);
                }
                let timeout = self.leg_timeout(velocity_limit);
                let elapsed = self
                    .leg_started_at
                    .map(|started| now.duration_since(started).as_secs_f64())
                    .unwrap_or(0.0);
                if elapsed > timeout {
                    let reason = format!("axis did not reach {label} within {timeout:.0} s");
                    self.stop(&reason);
                    return Step::Failed(reason);
                }
                Step::Target(target)
            }

            Phase::Dwell => {
                let waited = self
                    .arrived_at
                    .map(|arrived| now.duration_since(arrived).as_secs_f64())
                    .unwrap_or(0.0);
                let remaining = self.dwell - waited;
                self.status = format!("holding at {label} ({:.1} s)", remaining.max(0.0));
                if remaining > 0.0 {
                    return Step::Target(target);
                }

                // The first leg only brings the axis to A; counting starts once
                // the real A→B→A shuttle begins.
                if self.was_approach {
                    self.was_approach = false;
                } else {
                    self.legs += 1;
                    self.completed = self.legs / 2;
                    if self.cycles > 0 && self.target == End::A && self.completed >= self.cycles {
                        let reason = format!("{} cycle(s) finished", self.cycles);
                        self.stop(&reason);
                        return Step::Finished { reason, target };
                    }
                }

                self.target = self.target.other();
                self.phase = Phase::Moving;
                self.leg_started_at = Some(now);
                Step::Target(self.target_position())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn started() -> (Shuttle, Instant) {
        let mut shuttle = Shuttle::default();
        let now = Instant::now();
        shuttle.start(now).unwrap();
        (shuttle, now)
    }

    /// A shuttle between one position and itself never leaves the dwell phase,
    /// and looks from the outside like an axis that has hung.
    #[test]
    fn endpoints_that_are_the_same_are_refused() {
        let mut shuttle = Shuttle::default();
        shuttle.configure(Rev(0.1), Rev(0.1), 1.0, 0);
        assert!(shuttle.start(Instant::now()).is_err());
        assert!(!shuttle.running);
    }

    /// Arrival needs both position *and* stillness. Position alone would count
    /// a fast pass through the target, and the axis would turn round while
    /// still travelling.
    #[test]
    fn passing_through_the_target_at_speed_is_not_an_arrival() {
        let (mut shuttle, now) = started();

        // At the target, but still moving fast.
        shuttle.advance(now, Some(Rev(-0.25)), Some(RevPerSec(1.0)), RevPerSec(1.0));
        assert_eq!(shuttle.phase, Phase::Approach, "still travelling");

        // At the target and still.
        shuttle.advance(now, Some(Rev(-0.25)), Some(RevPerSec(0.0)), RevPerSec(1.0));
        assert_eq!(shuttle.phase, Phase::Dwell);
    }

    /// The dwell is held before the axis turns round.
    #[test]
    fn the_shuttle_dwells_before_reversing() {
        let (mut shuttle, now) = started();
        shuttle.advance(now, Some(Rev(-0.25)), Some(RevPerSec(0.0)), RevPerSec(1.0));

        // Half a second into a one-second dwell: still at A.
        let step = shuttle.advance(
            now + Duration::from_millis(500),
            Some(Rev(-0.25)),
            Some(RevPerSec(0.0)),
            RevPerSec(1.0),
        );
        assert_eq!(step, Step::Target(Rev(-0.25)));
        assert!(shuttle.status.contains("holding at A"));

        // Past it: heading for B.
        let step = shuttle.advance(
            now + Duration::from_millis(1100),
            Some(Rev(-0.25)),
            Some(RevPerSec(0.0)),
            RevPerSec(1.0),
        );
        assert_eq!(step, Step::Target(Rev(0.25)));
        assert_eq!(shuttle.target, End::B);
    }

    /// The initial run to A is not a cycle. Counting it would report one more
    /// cycle than the machine actually did, which for an endurance run is the
    /// number that matters.
    #[test]
    fn the_approach_to_a_is_not_counted_as_a_cycle() {
        let (mut shuttle, start) = started();
        let mut now = start;

        // Arrive at A and dwell out.
        shuttle.advance(now, Some(Rev(-0.25)), Some(RevPerSec(0.0)), RevPerSec(1.0));
        now += Duration::from_millis(1100);
        shuttle.advance(now, Some(Rev(-0.25)), Some(RevPerSec(0.0)), RevPerSec(1.0));
        assert_eq!(shuttle.completed, 0, "the approach is not a cycle");
        assert_eq!(shuttle.target, End::B);
    }

    /// One A→B→A is one cycle.
    #[test]
    fn a_full_round_trip_counts_as_one_cycle() {
        let (mut shuttle, start) = started();
        let mut now = start;
        let limit = RevPerSec(1.0);

        let arrive_and_dwell = |shuttle: &mut Shuttle, now: &mut Instant, at: Rev| {
            shuttle.advance(*now, Some(at), Some(RevPerSec(0.0)), limit);
            *now += Duration::from_millis(1100);
            shuttle.advance(*now, Some(at), Some(RevPerSec(0.0)), limit);
        };

        arrive_and_dwell(&mut shuttle, &mut now, Rev(-0.25)); // approach done
        arrive_and_dwell(&mut shuttle, &mut now, Rev(0.25)); // A→B
        assert_eq!(shuttle.completed, 0, "half a cycle");
        arrive_and_dwell(&mut shuttle, &mut now, Rev(-0.25)); // B→A
        assert_eq!(shuttle.completed, 1);
    }

    /// A requested number of cycles ends the run, at A.
    #[test]
    fn a_bounded_run_finishes_after_its_cycles() {
        let mut shuttle = Shuttle::default();
        shuttle.configure(Rev(-0.25), Rev(0.25), 0.0, 1);
        let mut now = Instant::now();
        shuttle.start(now).unwrap();
        let limit = RevPerSec(1.0);

        let leg = |shuttle: &mut Shuttle, now: &mut Instant, at: Rev| -> Step {
            shuttle.advance(*now, Some(at), Some(RevPerSec(0.0)), limit);
            *now += Duration::from_millis(10);
            shuttle.advance(*now, Some(at), Some(RevPerSec(0.0)), limit)
        };

        leg(&mut shuttle, &mut now, Rev(-0.25)); // approach
        leg(&mut shuttle, &mut now, Rev(0.25)); // A→B
        let step = leg(&mut shuttle, &mut now, Rev(-0.25)); // B→A, one cycle

        assert!(matches!(step, Step::Finished { .. }), "{step:?}");
        assert!(!shuttle.running);
        assert_eq!(shuttle.completed, 1);
    }

    /// An axis that never arrives stops the sequence rather than hanging in it
    /// for ever.
    #[test]
    fn a_leg_that_never_arrives_times_out() {
        let (mut shuttle, now) = started();
        let limit = RevPerSec(1.0);
        let timeout = shuttle.leg_timeout(limit);

        let step = shuttle.advance(
            now + Duration::from_secs_f64(timeout + 1.0),
            Some(Rev(0.0)), // nowhere near A
            Some(RevPerSec(0.0)),
            limit,
        );
        assert!(matches!(step, Step::Failed(_)), "{step:?}");
        assert!(!shuttle.running);
    }

    /// The timeout scales with the move: a slow axis crossing a wide span
    /// legitimately takes a while.
    #[test]
    fn the_leg_timeout_follows_the_span_and_the_speed() {
        let mut shuttle = Shuttle::default();
        shuttle.configure(Rev(-5.0), Rev(5.0), 0.0, 0);
        let slow = shuttle.leg_timeout(RevPerSec(0.1));
        let fast = shuttle.leg_timeout(RevPerSec(5.0));
        assert!(slow > fast, "{slow} should exceed {fast}");
        assert!(slow <= 300.0, "but it is still bounded");
    }

    /// One missing telemetry sample is a dropped cycle, not a reason to
    /// abandon an endurance run that has been going for hours.
    #[test]
    fn a_missing_position_holds_the_target_rather_than_stopping() {
        let (mut shuttle, now) = started();
        let step = shuttle.advance(now, None, None, RevPerSec(1.0));
        assert_eq!(step, Step::Target(Rev(-0.25)));
        assert!(shuttle.running);
    }

    #[test]
    fn a_stopped_shuttle_commands_nothing() {
        let mut shuttle = Shuttle::default();
        assert_eq!(
            shuttle.advance(Instant::now(), Some(Rev(0.0)), None, RevPerSec(1.0)),
            Step::Idle
        );
    }
}
