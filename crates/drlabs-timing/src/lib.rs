//! The scheduling accuracy a motor control loop needs.
//!
//! The problem it solves is concrete. Windows' default timer granularity is
//! 15.6 ms, so a naive sleep in a 100 Hz loop overshoots its 10 ms period by
//! more than the period itself — the loop silently runs at 64 Hz with 15 ms of
//! jitter, and every watchdog timeout derived from the nominal rate is wrong.
//!
//! Two things fix it: raising the multimedia timer resolution to 1 ms removes
//! the coarse error, and a short spin at the end of each wait removes what is
//! left. The spin is bounded and only covers the final approach, so it costs a
//! fraction of a core rather than a whole one.
//!
//! macOS has the same problem in a different shape — a default thread's sleep
//! overshoots by a quarter of its length, so a 20 ms wait lands 5 ms late —
//! and a different remedy, a per-thread scheduling policy. [`Resolution`]
//! covers both: it is "whatever this OS needs so that this thread's waits
//! land on time", held for as long as the loop runs.

use std::marker::PhantomData;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::time::{Duration, Instant};

#[cfg(target_os = "macos")]
pub mod mach;
mod platform;
pub mod rt;

/// How long before the deadline the wait switches from sleeping to spinning.
///
/// A little above the 1 ms granularity a raised timer gives, so the sleep never
/// overshoots into the spin's territory.
pub const DEFAULT_SPIN: Duration = Duration::from_micros(1500);

static RESOLUTION_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Holds the finest timer granularity the OS offers for as long as it lives.
///
/// On Windows that is the multimedia timer period, which is process-wide and
/// therefore reference counted: nested or concurrent holders each get their
/// own and the release happens when the last one drops. On macOS it is the
/// mach time-constraint policy on the *calling thread*, so it is raised on the
/// thread that will wait and released there — which is why this type cannot
/// be sent to another thread. On platforms where the request is unnecessary
/// or unavailable [`Resolution::raised`] reports false and the caller carries
/// on either way, since the spin still bounds the error.
#[derive(Debug)]
pub struct Resolution {
    raised: bool,
    /// Not `Send`: the macOS raise belongs to one thread.
    _thread: PhantomData<*const ()>,
}

impl Resolution {
    /// Ask the OS for the finest granularity it offers.
    pub fn raise() -> Self {
        if platform::PER_THREAD {
            return Self {
                raised: platform::raise_timer_resolution(),
                _thread: PhantomData,
            };
        }
        // The count is incremented first so that a concurrent drop cannot
        // release the resolution between the check and the request.
        if RESOLUTION_COUNT.fetch_add(1, Ordering::AcqRel) == 0
            && !platform::raise_timer_resolution()
        {
            RESOLUTION_COUNT.fetch_sub(1, Ordering::AcqRel);
            return Self {
                raised: false,
                _thread: PhantomData,
            };
        }
        Self {
            raised: true,
            _thread: PhantomData,
        }
    }

    /// Whether the request was granted.
    pub fn raised(&self) -> bool {
        self.raised
    }
}

impl Drop for Resolution {
    fn drop(&mut self) {
        if !self.raised {
            return;
        }
        if platform::PER_THREAD || RESOLUTION_COUNT.fetch_sub(1, Ordering::AcqRel) == 1 {
            platform::release_timer_resolution();
        }
    }
}

/// Why a wait ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Woke {
    /// The deadline arrived.
    Deadline,
    /// Something asked for attention before the deadline.
    Early,
    /// The channel the wake came from is gone; the caller should shut down.
    Closed,
}

/// Waits until `deadline`, or until `wake` fires.
///
/// The final approach spins rather than sleeping, so the return lands on the
/// deadline instead of somewhere after it.
///
/// Passing `None` for `wake` waits out the whole deadline regardless of what
/// anyone wants — which is how the control loop enforces its rate. A command
/// may wake the loop so that it is noticed without polling, but it must not be
/// able to make the loop cycle faster than the machine was budgeted for.
pub fn sleep_until(deadline: Instant, wake: Option<&Receiver<()>>, spin: Duration) -> Woke {
    let spin = if spin.is_zero() { DEFAULT_SPIN } else { spin };

    let now = Instant::now();
    if let Some(remaining) = deadline.checked_duration_since(now) {
        if let Some(sleep_for) = remaining.checked_sub(spin) {
            match wake {
                Some(channel) => match channel.recv_timeout(sleep_for) {
                    Ok(()) => return Woke::Early,
                    Err(RecvTimeoutError::Disconnected) => return Woke::Closed,
                    Err(RecvTimeoutError::Timeout) => {}
                },
                None => std::thread::sleep(sleep_for),
            }
        }
    }

    // Final approach. Yielding rather than sleeping keeps the landing
    // independent of OS timer granularity.
    while Instant::now() < deadline {
        if let Some(channel) = wake {
            match channel.try_recv() {
                Ok(()) => return Woke::Early,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => return Woke::Closed,
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
            }
        }
        platform::yield_now();
    }

    // Report an already-pending wake, so a signal that arrived during the spin
    // is not swallowed.
    match wake.map(|channel| channel.try_recv()) {
        Some(Ok(())) => Woke::Early,
        Some(Err(std::sync::mpsc::TryRecvError::Disconnected)) => Woke::Closed,
        _ => Woke::Deadline,
    }
}

/// A summary of how a loop actually kept time.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Statistics {
    pub count: usize,
    pub mean: Duration,
    pub p95: Duration,
    pub max: Duration,
}

/// Mean, 95th percentile and maximum.
///
/// A p95 far above the mean is the signature of an adapter that is mostly fast
/// but occasionally stalls, which is exactly the case a mean alone hides.
pub fn summarise(samples: &[Duration]) -> Statistics {
    if samples.is_empty() {
        return Statistics::default();
    }
    let mut ordered = samples.to_vec();
    ordered.sort_unstable();

    let total: Duration = ordered.iter().sum();
    let index = ((ordered.len() as f64) * 0.95) as usize;
    let index = index.min(ordered.len() - 1);

    Statistics {
        count: ordered.len(),
        mean: total / ordered.len() as u32,
        p95: ordered[index],
        max: ordered[ordered.len() - 1],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::sync::Mutex;

    /// Tests run in parallel, and on Windows the raised resolution is one
    /// process-wide count. Every test that holds a `Resolution` takes this
    /// first, so none of them sees another's hold in the count.
    static HOLDING_RESOLUTION: Mutex<()> = Mutex::new(());

    fn holding_resolution() -> std::sync::MutexGuard<'static, ()> {
        HOLDING_RESOLUTION
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    #[test]
    fn summarise_reports_the_tail_a_mean_would_hide() {
        let mut samples: Vec<Duration> = (0..99).map(|_| Duration::from_millis(1)).collect();
        samples.push(Duration::from_millis(200));

        let stats = summarise(&samples);
        assert_eq!(stats.count, 100);
        assert_eq!(stats.max, Duration::from_millis(200));
        assert!(stats.mean < Duration::from_millis(4), "{:?}", stats.mean);
        // The single stall is the 100th of 100 samples, so p95 is still fast —
        // and that is the point of reporting max alongside it.
        assert_eq!(stats.p95, Duration::from_millis(1));
    }

    #[test]
    fn summarise_of_nothing_is_not_a_panic() {
        assert_eq!(summarise(&[]), Statistics::default());
    }

    /// Held the way every loop holds it: with the resolution raised first.
    /// Without that, a 20 ms wait on a default macOS thread lands about 3 ms
    /// late, and that is the finding, not a tolerance to widen.
    #[test]
    fn a_wait_lands_on_its_deadline() {
        let _serial = holding_resolution();
        let _resolution = Resolution::raise();
        let deadline = Instant::now() + Duration::from_millis(20);
        assert_eq!(sleep_until(deadline, None, DEFAULT_SPIN), Woke::Deadline);

        let overshoot = Instant::now().saturating_duration_since(deadline);
        assert!(
            overshoot < Duration::from_millis(3),
            "landed {overshoot:?} late"
        );
    }

    #[test]
    fn a_wake_cuts_the_wait_short() {
        let (sender, receiver) = mpsc::channel();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            let _ = sender.send(());
        });

        let deadline = Instant::now() + Duration::from_secs(5);
        let started = Instant::now();
        assert_eq!(
            sleep_until(deadline, Some(&receiver), DEFAULT_SPIN),
            Woke::Early
        );
        assert!(started.elapsed() < Duration::from_millis(500));
    }

    /// The rate ceiling depends on this: a wait with no wake channel is not
    /// interruptible, so a flood of commands cannot shorten it.
    #[test]
    fn a_wait_without_a_wake_channel_runs_its_full_course() {
        let deadline = Instant::now() + Duration::from_millis(25);
        let started = Instant::now();
        assert_eq!(sleep_until(deadline, None, DEFAULT_SPIN), Woke::Deadline);
        assert!(
            started.elapsed() >= Duration::from_millis(24),
            "returned after {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn a_deadline_already_past_returns_at_once() {
        let deadline = Instant::now() - Duration::from_millis(50);
        let started = Instant::now();
        assert_eq!(sleep_until(deadline, None, DEFAULT_SPIN), Woke::Deadline);
        assert!(started.elapsed() < Duration::from_millis(5));
    }

    /// A nested raise must not release what the outer one still holds —
    /// whichever mechanism the OS uses.
    #[test]
    fn a_nested_raise_does_not_release_the_outer_one() {
        let _serial = holding_resolution();
        std::thread::spawn(|| {
            let outer = Resolution::raise();
            {
                let _inner = Resolution::raise();
            }
            if platform::PER_THREAD {
                // macOS: the inner raise found the thread's policy in place and
                // took nothing it would have to give back.
                #[cfg(target_os = "macos")]
                assert_eq!(mach::has_time_constraint(), outer.raised());
                assert_eq!(RESOLUTION_COUNT.load(Ordering::Acquire), 0);
            } else {
                let held = if outer.raised() { 1 } else { 0 };
                assert_eq!(RESOLUTION_COUNT.load(Ordering::Acquire), held);
            }
            drop(outer);
            assert_eq!(RESOLUTION_COUNT.load(Ordering::Acquire), 0);
            #[cfg(target_os = "macos")]
            assert!(!mach::has_time_constraint());
        })
        .join()
        .unwrap();
    }

    /// The measurement that decided the macOS design: a raised thread's sleep
    /// lands within a fraction of a millisecond where a default thread's lands
    /// milliseconds late. Twenty samples of the longest wait a loop here makes.
    #[cfg(target_os = "macos")]
    #[test]
    fn a_raised_thread_sleeps_accurately_on_macos() {
        let _serial = holding_resolution();
        std::thread::spawn(|| {
            let resolution = Resolution::raise();
            assert!(resolution.raised());
            let asked = Duration::from_micros(18_500);
            let worst = (0..20)
                .map(|_| {
                    let started = Instant::now();
                    std::thread::sleep(asked);
                    started.elapsed().saturating_sub(asked)
                })
                .max()
                .unwrap();
            assert!(worst < Duration::from_millis(1), "overshoot {worst:?}");
        })
        .join()
        .unwrap();
    }
}
