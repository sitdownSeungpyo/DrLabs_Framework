//! Where the OS has to be asked for something.

#[cfg(windows)]
mod imp {
    use windows_sys::Win32::Media::{timeBeginPeriod, timeEndPeriod};

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

#[cfg(not(windows))]
mod imp {
    /// Linux and macOS already schedule with the granularity a control loop
    /// needs, so there is nothing to raise.
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
