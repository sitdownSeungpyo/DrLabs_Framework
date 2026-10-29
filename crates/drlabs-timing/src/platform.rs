//! Where the OS has to be asked for something.
//!
//! Three answers to the same question, "make this thread's waits land on
//! time". Windows has a process-wide timer period to raise. macOS has a
//! per-thread scheduling policy, and nothing process-wide that helps. Linux
//! already wakes a sleeping thread when it asked to be woken.

/// Whether raising the resolution acts on the calling thread rather than the
/// whole process. When true, [`crate::Resolution`] is not reference counted:
/// each thread that waits raises its own, and drops it on the same thread.
pub use imp::PER_THREAD;

#[cfg(windows)]
mod imp {
    use windows_sys::Win32::Media::{timeBeginPeriod, timeEndPeriod};

    pub const PER_THREAD: bool = false;

    /// The granularity we ask for. 1 ms is the finest the multimedia timer API
    /// offers and is enough for a 500 Hz loop.
    const TIMER_PERIOD_MS: u32 = 1;

    /// Asks Windows for 1 ms timers.
    ///
    /// Without it, the scheduler's default 15.6 ms quantum makes any loop above
    /// about 60 Hz miss its period on most cycles. The call affects the whole
    /// process, which is why it is reference counted and released on shutdown.
    pub fn raise_timer_resolution() -> bool {
        // TIMERR_NOERROR
        unsafe { timeBeginPeriod(TIMER_PERIOD_MS) == 0 }
    }

    pub fn release_timer_resolution() {
        unsafe {
            timeEndPeriod(TIMER_PERIOD_MS);
        }
    }
}

#[cfg(target_os = "macos")]
mod imp {
    use crate::mach;

    pub const PER_THREAD: bool = true;

    /// Puts the calling thread under the mach time-constraint policy, which
    /// is what stops its sleeps overshooting by a quarter of their length
    /// (the numbers are in [`crate::mach`]).
    ///
    /// A thread that already holds the policy — because [`crate::rt::promote`]
    /// gave it one tuned to the loop's real period — is left exactly as it is,
    /// and this reports `false`: there was nothing to raise, so there will be
    /// nothing to release.
    pub fn raise_timer_resolution() -> bool {
        if mach::has_time_constraint() {
            return false;
        }
        mach::set_time_constraint(mach::DEFAULT_PERIOD)
    }

    pub fn release_timer_resolution() {
        mach::clear_time_constraint();
    }
}

#[cfg(not(any(windows, target_os = "macos")))]
mod imp {
    pub const PER_THREAD: bool = false;

    /// Linux already schedules with the granularity a control loop needs, so
    /// there is nothing to raise.
    pub fn raise_timer_resolution() -> bool {
        false
    }

    pub fn release_timer_resolution() {}
}

pub use imp::{raise_timer_resolution, release_timer_resolution};

/// Hands the rest of this time slice to the scheduler.
///
/// During the final spin that is cheaper than a busy loop and precise enough,
/// because the remaining wait is under a millisecond.
#[inline]
pub fn yield_now() {
    std::thread::yield_now();
}
