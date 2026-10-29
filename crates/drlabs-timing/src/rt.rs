//! Asking the OS to treat a thread as real-time.
//!
//! Linux: `SCHED_FIFO` at the given priority, pinned to the given core, with
//! the process's memory locked so a page fault cannot stall a tick. All
//! three need privileges (`CAP_SYS_NICE`, a `memlock` limit) and each is
//! reported separately, so a daemon running unprivileged says exactly which
//! guarantee it is running without.
//!
//! macOS: the mach time-constraint policy, declared for the loop's actual
//! period. It needs no privilege and it is the only thing on that OS that
//! makes a wait land on time (see [`crate::mach`]). There is no numeric
//! priority — any non-zero request asks for the policy — no core pinning on
//! Apple Silicon, and no `mlockall`; the report says so.
//!
//! Windows: `THREAD_PRIORITY_TIME_CRITICAL`, which is the most the ordinary
//! scheduler offers, and a core affinity mask. Neither is a guarantee, and
//! the result says so.
//!
//! Everything here is best effort by design: a refused request is a line
//! in the log, not a reason to stop a machine that would otherwise run.

use std::time::Duration;

/// What the OS granted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Granted {
    /// A real-time scheduling class (SCHED_FIFO), the mach time-constraint
    /// policy, or on Windows the top ordinary priority.
    pub priority: bool,
    /// What `priority` means on this OS, for the log — `"SCHED_FIFO"`,
    /// `"time-constraint"`, `"TIME_CRITICAL"`. Empty when nothing was asked.
    pub policy: &'static str,
    /// The thread is pinned to the requested core.
    pub affinity: bool,
    /// The process's pages are locked in memory (Linux only).
    pub memory_locked: bool,
    /// Whether what was granted amounts to a guarantee. Only a real-time
    /// class on Linux counts.
    pub guaranteed: bool,
}

impl Granted {
    /// One line for a log.
    pub fn describe(&self, priority: u32, cpu: Option<usize>) -> String {
        let mut parts = Vec::new();
        if priority > 0 {
            let verdict = if self.priority { "granted" } else { "refused" };
            parts.push(if self.policy.is_empty() {
                format!("priority {priority}: {verdict}")
            } else {
                format!("priority {priority}: {verdict} ({})", self.policy)
            });
        }
        if let Some(cpu) = cpu {
            parts.push(if self.affinity {
                format!("cpu {cpu}: pinned")
            } else {
                format!("cpu {cpu}: refused")
            });
        }
        parts.push(if self.memory_locked {
            "memory locked".into()
        } else {
            "memory not locked".into()
        });
        parts.push(if self.guaranteed {
            "real-time".into()
        } else {
            "best effort, not real-time".into()
        });
        parts.join(", ")
    }
}

/// Promotes the calling thread. `priority` 0 asks for no scheduling change;
/// `cpu` `None` asks for no pinning. `period` is how often the thread will
/// run — macOS declares it to the scheduler; the others do not need it, and
/// `Duration::ZERO` is accepted everywhere as "unknown".
pub fn promote(priority: u32, cpu: Option<usize>, period: Duration) -> Granted {
    imp::promote(priority, cpu, period)
}

/// Locks the process's current and future pages in memory. Linux only;
/// elsewhere it reports `false` and does nothing.
pub fn lock_memory() -> bool {
    imp::lock_memory()
}

#[cfg(target_os = "linux")]
mod imp {
    use super::Granted;
    use std::time::Duration;

    pub fn promote(priority: u32, cpu: Option<usize>, _period: Duration) -> Granted {
        let mut granted = Granted::default();
        if priority > 0 {
            granted.policy = "SCHED_FIFO";
            let param = libc::sched_param {
                sched_priority: priority.clamp(1, 99) as libc::c_int,
            };
            // SAFETY: a valid pointer to an initialised sched_param, for the
            // calling thread.
            let result = unsafe {
                libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param)
            };
            granted.priority = result == 0;
            granted.guaranteed = granted.priority;
        }
        if let Some(cpu) = cpu {
            // SAFETY: cpu_set_t is plain data; CPU_ZERO/CPU_SET write inside it.
            unsafe {
                let mut set: libc::cpu_set_t = std::mem::zeroed();
                libc::CPU_ZERO(&mut set);
                libc::CPU_SET(cpu, &mut set);
                let result = libc::pthread_setaffinity_np(
                    libc::pthread_self(),
                    std::mem::size_of::<libc::cpu_set_t>(),
                    &set,
                );
                granted.affinity = result == 0;
            }
        }
        granted.memory_locked = lock_memory();
        granted
    }

    pub fn lock_memory() -> bool {
        // SAFETY: no pointers involved; the flags are the documented ones.
        unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) == 0 }
    }
}

#[cfg(target_os = "macos")]
mod imp {
    use super::Granted;
    use crate::mach;
    use std::time::Duration;

    pub fn promote(priority: u32, cpu: Option<usize>, period: Duration) -> Granted {
        let mut granted = Granted::default();
        if priority > 0 {
            granted.policy = "time-constraint";
            granted.priority = mach::set_time_constraint(period);
        }
        // Apple Silicon does not pin threads to cores: THREAD_AFFINITY_POLICY
        // returns KERN_NOT_SUPPORTED there, and on Intel Macs it was only ever
        // a hint. Not asked, so that the report is a plain refusal rather
        // than a false "pinned".
        let _ = cpu;
        // No mlockall on macOS.
        granted.memory_locked = false;
        // The policy is honoured by the scheduler but a thread that overruns
        // its declaration loses it for a while; soft, so not claimed.
        granted.guaranteed = false;
        granted
    }

    pub fn lock_memory() -> bool {
        false
    }
}

#[cfg(windows)]
mod imp {
    use super::Granted;
    use std::time::Duration;
    use windows_sys::Win32::System::Threading::{
        GetCurrentThread, SetThreadAffinityMask, SetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL,
    };

    pub fn promote(priority: u32, cpu: Option<usize>, _period: Duration) -> Granted {
        let mut granted = Granted::default();
        if priority > 0 {
            granted.policy = "TIME_CRITICAL";
            // SAFETY: the current thread's pseudo-handle is always valid.
            granted.priority =
                unsafe { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) }
                    != 0;
        }
        if let Some(cpu) = cpu {
            if cpu < usize::BITS as usize {
                // SAFETY: as above; the mask is a plain integer.
                granted.affinity =
                    unsafe { SetThreadAffinityMask(GetCurrentThread(), 1usize << cpu) } != 0;
            }
        }
        // Windows has no real-time scheduling class for an ordinary process.
        granted.guaranteed = false;
        granted
    }

    pub fn lock_memory() -> bool {
        false
    }
}

#[cfg(not(any(target_os = "linux", target_os = "macos", windows)))]
mod imp {
    use super::Granted;
    use std::time::Duration;

    pub fn promote(_priority: u32, _cpu: Option<usize>, _period: Duration) -> Granted {
        Granted::default()
    }

    pub fn lock_memory() -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Asking for nothing is granted nothing and is not an error.
    #[test]
    fn asking_for_nothing_changes_nothing() {
        let granted = promote(0, None, Duration::ZERO);
        assert!(!granted.priority);
        assert!(!granted.affinity);
        let line = granted.describe(0, None);
        assert!(line.contains("best effort"), "{line}");
    }

    /// The report never claims a guarantee it was not given.
    #[test]
    fn a_refusal_is_reported_as_one() {
        let granted = Granted {
            priority: false,
            policy: "SCHED_FIFO",
            affinity: true,
            memory_locked: false,
            guaranteed: false,
        };
        let line = granted.describe(80, Some(2));
        assert!(line.contains("priority 80: refused (SCHED_FIFO)"), "{line}");
        assert!(line.contains("cpu 2: pinned"), "{line}");
        assert!(line.contains("not real-time"), "{line}");
    }

    /// macOS grants the policy to an unprivileged process, pins nothing, and
    /// locks nothing — and says exactly that.
    #[cfg(target_os = "macos")]
    #[test]
    fn macos_grants_the_policy_and_refuses_the_rest_honestly() {
        std::thread::spawn(|| {
            let granted = promote(80, Some(2), Duration::from_millis(2));
            assert!(granted.priority);
            assert!(!granted.affinity);
            assert!(!granted.memory_locked);
            assert!(!granted.guaranteed);
            assert!(crate::mach::has_time_constraint());
            let line = granted.describe(80, Some(2));
            assert_eq!(
                line,
                "priority 80: granted (time-constraint), cpu 2: refused, memory not locked, \
                 best effort, not real-time"
            );
        })
        .join()
        .unwrap();
    }
}
