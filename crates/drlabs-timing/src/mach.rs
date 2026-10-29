//! The mach calls behind macOS scheduling.
//!
//! macOS has no `SCHED_FIFO` an ordinary process may use and no timer period
//! to raise. What it has is a per-thread *time-constraint policy*: a thread
//! declares how often it runs, how long it computes and how late it may be,
//! and the kernel schedules it ahead of everything timeshared and stops
//! coalescing its timers. It is what CoreAudio's render thread runs under,
//! and it is available without privilege.
//!
//! Measured on an Apple Silicon Mac (macOS 26.5), `std::thread::sleep` on a
//! default thread overshoots by about a quarter of the requested time —
//! 18.5 ms asked, 3.5 ms late on average, 8.7 ms at worst, and the same for
//! `recv_timeout` and `mach_wait_until`. Under this policy the same calls land
//! 15 to 50 µs late. Nothing else tried — a bigger spin, a different wait
//! primitive — closed that gap, which is why both [`crate::Resolution`] and
//! [`crate::rt::promote`] end up here.
//!
//! The declarations are written out rather than taken from `libc`, whose
//! mach bindings are deprecated in favour of a crate this workspace does not
//! otherwise need. They are the stable kernel ABI from `<mach/thread_policy.h>`
//! and `<mach/mach_time.h>`.

use std::time::Duration;

type KernReturn = i32;
type MachPort = u32;

const KERN_SUCCESS: KernReturn = 0;
const THREAD_STANDARD_POLICY: u32 = 1;
const THREAD_TIME_CONSTRAINT_POLICY: u32 = 2;

/// `thread_time_constraint_policy_data_t`. The three times are in mach
/// absolute-time units, not nanoseconds.
#[repr(C)]
#[derive(Default)]
struct TimeConstraintPolicy {
    period: u32,
    computation: u32,
    constraint: u32,
    preemptible: u32,
}

/// `THREAD_TIME_CONSTRAINT_POLICY_COUNT`: the struct, in 32-bit words.
const TIME_CONSTRAINT_POLICY_COUNT: u32 = 4;

#[repr(C)]
#[derive(Default)]
struct TimebaseInfo {
    numer: u32,
    denom: u32,
}

extern "C" {
    static mach_task_self_: MachPort;
    fn mach_thread_self() -> MachPort;
    fn mach_port_deallocate(task: MachPort, name: MachPort) -> KernReturn;
    fn mach_timebase_info(info: *mut TimebaseInfo) -> KernReturn;
    fn thread_policy_set(
        thread: MachPort,
        flavor: u32,
        policy_info: *mut u32,
        count: u32,
    ) -> KernReturn;
    fn thread_policy_get(
        thread: MachPort,
        flavor: u32,
        policy_info: *mut u32,
        count: *mut u32,
        get_default: *mut u32,
    ) -> KernReturn;
}

/// Runs `f` with a send right to the calling thread, and gives the right back.
///
/// `mach_thread_self` allocates a port right on every call; leaking one per
/// tick would run the task out of them in about a day.
fn with_current_thread<T>(f: impl FnOnce(MachPort) -> T) -> T {
    // SAFETY: plain mach calls on the calling thread's own port.
    unsafe {
        let thread = mach_thread_self();
        let result = f(thread);
        mach_port_deallocate(mach_task_self_, thread);
        result
    }
}

/// Nanoseconds to mach absolute-time units, saturating at `u32::MAX` — the
/// policy's fields are 32 bits, which is over a minute on every Mac made.
fn to_absolute(duration: Duration) -> u32 {
    let mut info = TimebaseInfo::default();
    // SAFETY: a valid pointer to an initialised struct.
    if unsafe { mach_timebase_info(&mut info) } != KERN_SUCCESS || info.numer == 0 {
        return u32::MAX;
    }
    let ns = duration.as_nanos();
    let units = ns * u128::from(info.denom) / u128::from(info.numer);
    units.min(u128::from(u32::MAX)) as u32
}

/// The period assumed when the caller does not know its own.
pub const DEFAULT_PERIOD: Duration = Duration::from_millis(1);

/// Puts the calling thread under the time-constraint policy for a loop that
/// runs every `period`.
///
/// A quarter of the period is declared as computation and half as the
/// deadline, the proportions Apple's own examples use; a thread that exceeds
/// its declaration is not killed, it is scheduled as if it had not asked. A
/// zero period means "unknown" and gets [`DEFAULT_PERIOD`].
pub fn set_time_constraint(period: Duration) -> bool {
    let period = if period.is_zero() {
        DEFAULT_PERIOD
    } else {
        period
    };
    let computation = (period / 4).max(Duration::from_micros(50));
    let constraint = (period / 2).max(computation);
    let mut policy = TimeConstraintPolicy {
        period: to_absolute(period),
        computation: to_absolute(computation),
        constraint: to_absolute(constraint),
        preemptible: 1,
    };
    with_current_thread(|thread| {
        // SAFETY: `policy` is a repr(C) struct of exactly COUNT words, alive
        // for the call.
        unsafe {
            thread_policy_set(
                thread,
                THREAD_TIME_CONSTRAINT_POLICY,
                &mut policy as *mut TimeConstraintPolicy as *mut u32,
                TIME_CONSTRAINT_POLICY_COUNT,
            ) == KERN_SUCCESS
        }
    })
}

/// Whether the calling thread currently holds a time-constraint policy, as
/// opposed to the default the kernel reports for a thread nobody has touched.
pub fn has_time_constraint() -> bool {
    let mut policy = TimeConstraintPolicy::default();
    let mut count = TIME_CONSTRAINT_POLICY_COUNT;
    let mut get_default: u32 = 0;
    with_current_thread(|thread| {
        // SAFETY: as above; `get_default` is in/out and reads back true when
        // the thread has no policy of its own.
        let result = unsafe {
            thread_policy_get(
                thread,
                THREAD_TIME_CONSTRAINT_POLICY,
                &mut policy as *mut TimeConstraintPolicy as *mut u32,
                &mut count,
                &mut get_default,
            )
        };
        result == KERN_SUCCESS && get_default == 0
    })
}

/// Returns the calling thread to ordinary timeshare scheduling.
pub fn clear_time_constraint() -> bool {
    with_current_thread(|thread| {
        // SAFETY: the standard policy takes no parameters; a null pointer
        // with a zero count is the documented way to ask for it.
        unsafe {
            thread_policy_set(thread, THREAD_STANDARD_POLICY, std::ptr::null_mut(), 0)
                == KERN_SUCCESS
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_thread_has_no_policy_and_one_can_be_given_and_taken_back() {
        std::thread::spawn(|| {
            assert!(!has_time_constraint());
            assert!(set_time_constraint(Duration::from_millis(2)));
            assert!(has_time_constraint());
            assert!(clear_time_constraint());
            assert!(!has_time_constraint());
        })
        .join()
        .unwrap();
    }

    #[test]
    fn absolute_time_is_monotonic_in_the_duration_and_saturates() {
        let one = to_absolute(Duration::from_millis(1));
        let two = to_absolute(Duration::from_millis(2));
        assert!(one > 0 && two > one, "{one} {two}");
        assert_eq!(to_absolute(Duration::from_secs(1 << 40)), u32::MAX);
    }
}
