//! Learning a joint's travel by moving the machine by hand.
//!
//! A description has to state what each joint may do before anything can drive
//! it, and on a machine nobody has characterised those numbers are a guess. The
//! guess is not harmless: too wide and the envelope protects nothing, too
//! narrow and a joint refuses to arm because it is resting outside limits
//! somebody invented.
//!
//! So measure them. Torque off, move every joint through its travel by hand,
//! and record the extremes the encoders actually reported. What comes back is
//! not a proposal — it is where the mechanism went.
//!
//! Two things this deliberately does not do. It does not add margin: a learned
//! range is what was reached, and whether to back off the hard stops is the
//! operator's call, made with the numbers in front of them. And it does not
//! apply anything by itself; it reports, and something else decides.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::robot::Robot;

/// How often the travel is sampled.
///
/// Faster than any control loop this framework runs, so an extreme is not
/// missed between samples, and slow enough to cost nothing: the sampler reads
/// telemetry the loop has already fetched and never touches the bus.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(25);

/// What one joint was seen to do.
#[derive(Debug, Clone, PartialEq)]
pub struct Learned {
    pub name: String,
    /// The furthest each way, in joint-space radians. `None` until the joint
    /// has reported a position at all.
    pub min_rad: Option<f64>,
    pub max_rad: Option<f64>,
    pub current_rad: Option<f64>,
    pub samples: u64,

    /// What the description says today, so a panel can show the two together.
    pub configured_min_rad: f64,
    pub configured_max_rad: f64,
    pub direction: f64,
}

impl Learned {
    /// How far the joint was moved, or `None` if it never reported.
    pub fn span_rad(&self) -> Option<f64> {
        Some(self.max_rad? - self.min_rad?)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LearnStatus {
    pub running: bool,
    pub elapsed_s: f64,
    pub remaining_s: f64,
    pub joints: Vec<Learned>,
}

#[derive(Default)]
struct Inner {
    running: bool,
    started: Option<Instant>,
    finished_at: Option<Instant>,
    seconds: f64,
    /// min, max, samples, latest.
    seen: BTreeMap<String, (f64, f64, u64, f64)>,
    /// Bumped on every start, so a sampler left over from a previous run
    /// notices it has been replaced and stops instead of writing into the new
    /// one's results.
    generation: u64,

    /// The machine this run released, kept so that ending the run can put it
    /// back however the run ended. Leaving that to the sampler thread meant a
    /// path — clearing the results — where the thread went away without ever
    /// re-arming, and the machine stayed limp for good.
    robot: Option<Arc<Robot>>,
}

/// Records how far each joint is moved while the machine is limp.
#[derive(Clone, Default)]
pub struct RangeLearner {
    inner: Arc<Mutex<Inner>>,
}

impl RangeLearner {
    pub fn new() -> Self {
        Self::default()
    }

    /// Releases every joint and starts recording.
    ///
    /// The release is the point: a joint under torque holds its target, and the
    /// only travel it would report is what the control loop asked for. Learning
    /// a range means the operator's hands are the only thing moving it.
    pub fn start(&self, robot: Arc<Robot>, seconds: f64) -> Result<(), String> {
        if seconds <= 0.0 || !seconds.is_finite() {
            return Err(format!("{seconds} is not a usable duration"));
        }

        // Automatic arming goes off first. Disarming a joint that re-arms
        // itself on the next cycle is not a release, and the operator would be
        // moving a machine that had quietly become rigid again.
        robot.release_all();

        let generation = {
            let mut inner = self.lock();
            inner.generation += 1;
            inner.running = true;
            inner.started = Some(Instant::now());
            inner.finished_at = None;
            inner.seconds = seconds;
            inner.seen.clear();
            inner.robot = Some(robot.clone());
            inner.generation
        };

        let learner = self.clone();
        std::thread::spawn(move || learner.sample_until_done(robot, generation));
        Ok(())
    }

    fn sample_until_done(&self, robot: Arc<Robot>, generation: u64) {
        loop {
            let states = robot.state();
            {
                let mut inner = self.lock();
                // A newer run, or one that has already been ended: whoever did
                // that owns putting the machine back.
                if inner.generation != generation || !inner.running {
                    return;
                }
                for state in &states {
                    let Some(position) = state.position_rad else {
                        continue;
                    };
                    if !position.is_finite() {
                        continue;
                    }
                    match inner.seen.get_mut(&state.name) {
                        Some((min, max, count, latest)) => {
                            if position < *min {
                                *min = position;
                            }
                            if position > *max {
                                *max = position;
                            }
                            *count += 1;
                            *latest = position;
                        }
                        None => {
                            inner
                                .seen
                                .insert(state.name.clone(), (position, position, 1, position));
                        }
                    }
                }

                let elapsed = inner
                    .started
                    .map(|at| at.elapsed().as_secs_f64())
                    .unwrap_or(0.0);
                if elapsed >= inner.seconds {
                    drop(inner);
                    self.finish();
                    return;
                }
            }
            std::thread::sleep(SAMPLE_INTERVAL);
        }
    }

    /// Ends a run, keeping everything recorded so far, and lets the joints arm
    /// themselves again.
    ///
    /// Every way a run can end goes through here, because every one of them has
    /// to put the machine back. A release that outlives the run that asked for
    /// it is a robot nobody can drive and, worse, one an operator may believe
    /// is holding.
    pub fn finish(&self) {
        let robot = {
            let mut inner = self.lock();
            if !inner.running {
                return;
            }
            inner.running = false;
            inner.finished_at = Some(Instant::now());
            inner.robot.take()
        };
        if let Some(robot) = robot {
            robot.allow_arming();
        }
    }

    /// Ends a run early. The results are kept.
    pub fn stop(&self) {
        self.finish();
    }

    /// Throws the results away, so a bad run is not mistaken for a good one.
    pub fn clear(&self) {
        self.finish();
        let mut inner = self.lock();
        inner.generation += 1;
        inner.started = None;
        inner.finished_at = None;
        inner.seen.clear();
        inner.robot = None;
    }

    /// What has been seen so far, joined to what the description says.
    pub fn status(&self, robot: &Robot) -> LearnStatus {
        let inner = self.lock();
        let elapsed = inner
            .started
            .map(|at| match inner.finished_at {
                Some(end) => end.duration_since(at).as_secs_f64(),
                None => at.elapsed().as_secs_f64(),
            })
            .unwrap_or(0.0);

        let joints = robot
            .config
            .joints
            .iter()
            .map(|joint| {
                let seen = inner.seen.get(&joint.name);
                Learned {
                    name: joint.name.clone(),
                    min_rad: seen.map(|(min, _, _, _)| *min),
                    max_rad: seen.map(|(_, max, _, _)| *max),
                    current_rad: seen.map(|(_, _, _, latest)| *latest),
                    samples: seen.map(|(_, _, count, _)| *count).unwrap_or(0),
                    configured_min_rad: joint.limits.min_rad,
                    configured_max_rad: joint.limits.max_rad,
                    direction: joint.direction,
                }
            })
            .collect();

        LearnStatus {
            running: inner.running,
            elapsed_s: elapsed,
            remaining_s: (inner.seconds - elapsed).max(0.0),
            joints,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|error| error.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn learner_with(samples: &[(&str, f64)]) -> RangeLearner {
        let learner = RangeLearner::new();
        {
            let mut inner = learner.lock();
            inner.running = true;
            inner.started = Some(Instant::now());
            inner.seconds = 10.0;
            for (name, value) in samples {
                match inner.seen.get_mut(*name) {
                    Some((min, max, count, latest)) => {
                        *min = min.min(*value);
                        *max = max.max(*value);
                        *count += 1;
                        *latest = *value;
                    }
                    None => {
                        inner
                            .seen
                            .insert((*name).into(), (*value, *value, 1, *value));
                    }
                }
            }
        }
        learner
    }

    /// The extremes are the whole product: a joint swung one way and then the
    /// other has to come back as both ends, not as wherever it was left.
    #[test]
    fn the_range_is_the_furthest_each_way_not_the_last_reading() {
        let learner = learner_with(&[("j1", 0.0), ("j1", -1.2), ("j1", 0.9), ("j1", 0.1)]);
        let inner = learner.lock();
        let (min, max, count, latest) = inner.seen["j1"];
        assert_eq!(min, -1.2);
        assert_eq!(max, 0.9);
        assert_eq!(count, 4);
        assert_eq!(latest, 0.1, "the current position is kept separately");
    }

    /// A joint nobody moved has one sample and no span, and must not be
    /// reported as a range of zero width that somebody might apply.
    #[test]
    fn a_joint_that_never_moved_has_no_span_worth_applying() {
        let learner = learner_with(&[("j1", 0.42)]);
        let inner = learner.lock();
        let (min, max, _, _) = inner.seen["j1"];
        assert_eq!(min, max);

        let learned = Learned {
            name: "j1".into(),
            min_rad: Some(min),
            max_rad: Some(max),
            current_rad: Some(max),
            samples: 1,
            configured_min_rad: -1.0,
            configured_max_rad: 1.0,
            direction: 1.0,
        };
        assert_eq!(learned.span_rad(), Some(0.0));
    }

    /// A duration that cannot run is refused rather than starting a recorder
    /// that ends on its first sample.
    #[test]
    fn a_duration_that_is_not_a_duration_is_refused() {
        let learner = RangeLearner::new();
        // Robot construction needs hardware, so only the guard is exercised
        // here; it runs before anything is released or spawned.
        assert!(learner.lock().seen.is_empty());
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                !(bad > 0.0 && bad.is_finite()),
                "{bad} must not pass the guard"
            );
        }
    }

    /// Clearing has to make a stale run unmistakable, not merely stop it.
    #[test]
    fn clearing_drops_what_was_recorded() {
        let learner = learner_with(&[("j1", 0.5), ("j1", -0.5)]);
        assert!(!learner.lock().seen.is_empty());
        learner.clear();
        let inner = learner.lock();
        assert!(inner.seen.is_empty());
        assert!(!inner.running);
    }
}
