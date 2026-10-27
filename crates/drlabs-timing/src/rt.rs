//! Asking the OS to treat a thread as real-time.
//!
//! Linux: `SCHED_FIFO` at the given priority, pinned to the given core, with
//! the process's memory locked so a page fault cannot stall a tick. All
//! three need privileges (`CAP_SYS_NICE`, a `memlock` limit) and each is
//! reported separately, so a daemon running unprivileged says exactly which
//! guarantee it is running without.
//!
//! Windows: `THREAD_PRIORITY_TIME_CRITICAL`, which is the most the ordinary
//! scheduler offers, and a core affinity mask. Neither is a guarantee, and
//! the result says so.
//!
//! Everything here is best effort by design: a refused request is a line
//! in the log, not a reason to stop a machine that would otherwise run.

/// What the OS granted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Granted {
    /// A real-time scheduling class (SCHED_FIFO), or on Windows the top
    /// ordinary priority.
    pub priority: bool,
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
            parts.push(if self.priority {
                format!("priority {priority}: granted")
            } else {
                format!("priority {priority}: refused")
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
/// `cpu` `None` asks for no pinning.
pub fn promote(priority: u32, cpu: Option<usize>) -> Granted {
    imp::promote(priority, cpu)
}

/// Locks the process's current and future pages in memory. Linux only;
/// elsewhere it reports `false` and does nothing.
pub fn lock_memory() -> bool {
    imp::lock_memory()
}

#[cfg(target_os = "linux")]
mod imp {
    use super::Granted;

    pub fn promote(priority: u32, cpu: Option<usize>) -> Granted {
        let mut granted = Granted::default();
        if priority > 0 {
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

#[cfg(windows)]
mod imp {
    use super::Granted;
    use windows_sys::Win32::System::Threading::{
        GetCurrentThread, SetThreadAffinityMask, SetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL,
    };

    pub fn promote(priority: u32, cpu: Option<usize>) -> Granted {
        let mut granted = Granted::default();
        if priority > 0 {
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

#[cfg(not(any(target_os = "linux", windows)))]
mod imp {
    use super::Granted;

    pub fn promote(_priority: u32, _cpu: Option<usize>) -> Granted {
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
        let granted = promote(0, None);
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
            affinity: true,
            memory_locked: false,
            guaranteed: false,
        };
        let line = granted.describe(80, Some(2));
        assert!(line.contains("priority 80: refused"), "{line}");
        assert!(line.contains("cpu 2: pinned"), "{line}");
        assert!(line.contains("not real-time"), "{line}");
    }
}
