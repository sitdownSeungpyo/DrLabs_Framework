//! What a session knows, and the rules for changing it.
//!
//! The state lives behind one lock and the link behind another, deliberately.
//! A control cycle holds the link for a whole transaction — a CAN round trip is
//! milliseconds — and a status poll must not wait for that, or a UI refreshing
//! at 10 Hz turns into a UI that stutters at the control rate.

use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::command::{ControlMode, QueryProfile};
use drlabs_core::link::Identity;
use drlabs_core::render::Units;
use drlabs_core::telemetry::Telemetry;
use drlabs_core::units::{Nm, Rev, RevPerSec, RevPerSec2};
use drlabs_safety::{Config as SafetyConfig, Monitor, Preset};

use crate::record::{EventLog, Level, SampleRing};
use crate::shuttle::Shuttle;
use crate::trajectory::Trapezoidal;

/// Where the transport stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Connection {
    #[default]
    Disconnected,
    Connecting,
    Connected,
}

impl Connection {
    pub fn as_str(self) -> &'static str {
        match self {
            Connection::Disconnected => "disconnected",
            Connection::Connecting => "connecting",
            Connection::Connected => "connected",
        }
    }
}

/// The rate a loop may be asked for.
///
/// The ceiling is what a USB-CAN adapter can sustain in practice; the floor
/// keeps the watchdog arithmetic meaningful.
pub const MIN_RATE_HZ: u32 = 10;
pub const MAX_RATE_HZ: u32 = 500;
pub const DEFAULT_RATE_HZ: u32 = 100;

/// Query profile rotation.
///
/// The verify divisor is prime so verify and full cycles do not repeatedly
/// coincide — two wide queries landing on the same period is exactly the frame
/// that overruns.
pub const FULL_QUERY_DIVISOR: u64 = 10;
pub const VERIFY_QUERY_DIVISOR: u64 = 97;

/// How often the complete parameter set is re-sent even when nothing changed.
///
/// Command registers are sticky, so a controller that reset underneath us would
/// otherwise keep stale limits until the operator happened to touch something.
pub const PARAM_REFRESH: Duration = Duration::from_secs(2);

/// Keeps commands this far inside the controller's own limits.
///
/// Commanding exactly the bound leaves no room for overshoot, and the firmware
/// faults the moment position control starts at or beyond it.
pub const BOUNDS_INSET: Rev = Rev(0.05);

/// The control parameters the loop sends every cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Params {
    pub max_torque: Nm,
    pub velocity_limit: RevPerSec,
    pub accel_limit: RevPerSec2,
    pub kp_scale: f64,
    pub kd_scale: f64,
    pub ilimit_scale: f64,
    pub feedforward: Nm,

    /// Hands the torque limit back to the firmware.
    ///
    /// While it is set the host torque ceiling does not apply; the current,
    /// stall and runaway guards still do.
    pub defer_torque_to_controller: bool,

    /// Bypasses the controller's stored travel envelope for every command,
    /// leaving only the host envelope.
    pub ignore_position_bounds: bool,
}

impl Default for Params {
    /// Deliberately gentle: a third of a newton-metre and one revolution per
    /// second will move a small motor and will not break a machine.
    fn default() -> Self {
        Self {
            max_torque: Nm(0.30),
            velocity_limit: RevPerSec(1.0),
            accel_limit: RevPerSec2(5.0),
            kp_scale: 1.0,
            kd_scale: 1.0,
            ilimit_scale: 1.0,
            feedforward: Nm(0.0),
            defer_torque_to_controller: false,
            ignore_position_bounds: false,
        }
    }
}

/// What the operator is asking the axis to do.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Setpoint {
    pub position: Rev,
    pub velocity: RevPerSec,
    pub within_lower: Rev,
    pub within_upper: Rev,
}

/// The state of a host-side profiled move.
#[derive(Debug, Clone, Default)]
pub struct TrajectoryStatus {
    pub running: bool,
    pub start: Rev,
    pub target: Rev,
    /// The profile's own limits, which may be tighter than the session's
    /// parameters but never looser.
    pub max_velocity: RevPerSec,
    pub accel: RevPerSec2,
    pub elapsed: f64,
    pub total: f64,
    /// 0..1.
    pub progress: f64,
    /// Where the axis should be now, and how fast it should be going.
    pub setpoint: Rev,
    pub feedforward: RevPerSec,
    pub status: String,
}

/// A latched emergency stop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EStop {
    pub code: String,
    pub message: String,
    pub detail: String,
}

/// The loop's own bookkeeping.
#[derive(Debug, Default)]
pub struct LoopStats {
    pub cycles: u64,
    pub replies: u64,
    pub timeouts: u64,
    pub consecutive_timeouts: u32,

    /// Transaction times and measured intervals between cycles.
    pub durations: Vec<Duration>,
    pub periods: Vec<Duration>,
    pub rate_window: Vec<Instant>,

    pub last_cycle_at: Option<Instant>,
    pub rate_warned_at: Option<Instant>,
}

pub const STATS_WINDOW: usize = 400;
pub const RATE_WINDOW: usize = 200;

impl LoopStats {
    pub fn record(sample: &mut Vec<Duration>, value: Duration) {
        if sample.len() >= STATS_WINDOW {
            sample.remove(0);
        }
        sample.push(value);
    }
}

/// Everything a session knows.
pub struct State {
    pub connection: Connection,
    pub connect_error: String,
    pub link_description: Vec<(String, String)>,
    pub identity: Option<Identity>,

    pub safety: SafetyConfig,
    pub monitor: Monitor,
    pub preset: Option<Preset>,
    pub units: Option<Arc<dyn Units>>,

    pub armed: bool,
    pub arm_blockers: Vec<String>,
    pub arm_advisories: Vec<String>,
    pub estop: Option<EStop>,

    pub mode: ControlMode,
    pub rate_hz: u32,
    pub setpoint: Setpoint,
    pub params: Params,

    pub shuttle: Shuttle,
    pub trajectory: TrajectoryStatus,
    pub profile: Option<Trapezoidal>,
    pub trajectory_started_at: Option<Instant>,

    pub telemetry: Telemetry,
    pub latched_fault: Option<i32>,
    pub last_reply_at: Option<Instant>,

    pub require_ack: bool,
    pub auto_arm: bool,
    pub initial_mode: ControlMode,
    /// Throttles the "cannot arm" log, so a blocked axis explains itself once
    /// rather than at the control rate.
    pub auto_arm_reported_at: Option<Instant>,

    pub stats: LoopStats,
    pub samples: SampleRing,
    pub events: EventLog,

    pub params_dirty: bool,
    pub params_sent_at: Option<Instant>,
    /// How many stop frames still have to go out before anything else.
    pub stop_burst: u32,
    pub cycle_count: u64,
    pub pending_special: Option<drlabs_core::command::Command>,

    pub heartbeat_at: Instant,
    pub started_at: Instant,
}

impl State {
    pub fn log(&mut self, level: Level, message: &str, detail: &str) {
        self.events.append(level, message, detail);
    }

    /// The control period.
    pub fn period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.rate_hz as f64)
    }

    /// The bounds a target is actually held to: the host envelope, tightened by
    /// the controller's own stored limits when they are known and narrower.
    ///
    /// The inset matters. Commanding exactly the controller's bound leaves no
    /// room for overshoot, and the firmware faults the moment position control
    /// starts at or beyond it.
    pub fn effective_limits(&self) -> (Rev, Rev) {
        let mut lower = self.safety.position_min;
        let mut upper = self.safety.position_max;

        if self.params.ignore_position_bounds {
            return (lower, upper);
        }
        if let Some(identity) = &self.identity {
            if let Some((controller_lower, controller_upper)) = identity.bounds {
                let inset_lower = controller_lower + BOUNDS_INSET;
                let inset_upper = controller_upper - BOUNDS_INSET;
                if inset_lower > lower {
                    lower = inset_lower;
                }
                if inset_upper < upper {
                    upper = inset_upper;
                }
            }
        }
        if upper < lower {
            // The two envelopes do not overlap. Refusing all motion is the
            // honest answer; the operator has to widen one of them.
            upper = lower;
        }
        (lower, upper)
    }

    /// Constrains a position target to whatever bounds are in force.
    pub fn clamp_target(&self, value: Rev) -> (Rev, bool) {
        if !self.safety.position_limit_enabled {
            return (value, false);
        }
        let (lower, upper) = self.effective_limits();
        value.clamp_changed(lower, upper)
    }

    /// This cycle's query profile.
    pub fn profile_for(&self, full_command: bool) -> QueryProfile {
        let mut profile = if self.cycle_count % VERIFY_QUERY_DIVISOR == 0 {
            QueryProfile::Verify
        } else if self.cycle_count % FULL_QUERY_DIVISOR == 0 {
            QueryProfile::Full
        } else {
            QueryProfile::Fast
        };
        // A full parameter frame plus a wide query would approach the CAN-FD
        // payload limit; keep them on separate cycles.
        if full_command && profile != QueryProfile::Fast {
            profile = QueryProfile::Fast;
        }
        profile
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::link::BoardStatus;

    fn state() -> State {
        State {
            connection: Connection::Connected,
            connect_error: String::new(),
            link_description: Vec::new(),
            identity: None,
            safety: SafetyConfig::default(),
            monitor: Monitor::default(),
            preset: None,
            units: None,
            armed: false,
            arm_blockers: Vec::new(),
            arm_advisories: Vec::new(),
            estop: None,
            mode: ControlMode::Stop,
            rate_hz: DEFAULT_RATE_HZ,
            setpoint: Setpoint::default(),
            params: Params::default(),
            shuttle: Shuttle::default(),
            trajectory: TrajectoryStatus::default(),
            profile: None,
            trajectory_started_at: None,
            telemetry: Telemetry::default(),
            latched_fault: None,
            last_reply_at: None,
            require_ack: true,
            auto_arm: false,
            initial_mode: ControlMode::Position,
            auto_arm_reported_at: None,
            stats: LoopStats::default(),
            samples: SampleRing::new(100),
            events: EventLog::new(100),
            params_dirty: true,
            params_sent_at: None,
            stop_burst: 0,
            cycle_count: 0,
            pending_special: None,
            heartbeat_at: Instant::now(),
            started_at: Instant::now(),
        }
    }

    #[test]
    fn the_period_follows_the_rate() {
        let mut state = state();
        assert_eq!(state.period(), Duration::from_millis(10));
        state.rate_hz = 200;
        assert_eq!(state.period(), Duration::from_millis(5));
    }

    /// With no controller bounds known, the host envelope is the only one.
    #[test]
    fn the_host_envelope_stands_alone_when_nothing_else_is_known() {
        let state = state();
        assert_eq!(state.effective_limits(), (Rev(-0.5), Rev(0.5)));
    }

    /// The controller's own stored envelope tightens the host one, inset so a
    /// command never lands exactly on the bound the firmware faults at.
    #[test]
    fn the_controllers_own_bounds_tighten_the_envelope_with_an_inset() {
        let mut state = state();
        state.identity = Some(Identity {
            bounds: Some((Rev(-0.3), Rev(0.3))),
            status: BoardStatus::Supported,
            ..Identity::new("moteus")
        });

        let (lower, upper) = state.effective_limits();
        assert_eq!(lower, Rev(-0.25), "inset inside the controller's own -0.3");
        assert_eq!(upper, Rev(0.25));
    }

    /// A wider controller envelope does not loosen the operator's.
    #[test]
    fn a_wider_controller_envelope_does_not_widen_the_host_one() {
        let mut state = state();
        state.identity = Some(Identity {
            bounds: Some((Rev(-10.0), Rev(10.0))),
            ..Identity::new("moteus")
        });
        assert_eq!(state.effective_limits(), (Rev(-0.5), Rev(0.5)));
    }

    /// Asking to ignore the controller's bounds leaves the host envelope as the
    /// only one in force — which is the whole point of the flag.
    #[test]
    fn ignoring_the_controller_bounds_leaves_only_the_host_envelope() {
        let mut state = state();
        state.identity = Some(Identity {
            bounds: Some((Rev(-0.1), Rev(0.1))),
            ..Identity::new("moteus")
        });
        state.params.ignore_position_bounds = true;
        assert_eq!(state.effective_limits(), (Rev(-0.5), Rev(0.5)));
    }

    /// Envelopes that do not overlap allow no motion at all, rather than
    /// silently picking one. The operator has to widen one of them.
    #[test]
    fn envelopes_that_do_not_overlap_allow_no_motion() {
        let mut state = state();
        state.safety.position_min = Rev(1.0);
        state.safety.position_max = Rev(2.0);
        state.identity = Some(Identity {
            bounds: Some((Rev(-0.5), Rev(0.5))),
            ..Identity::new("moteus")
        });

        let (lower, upper) = state.effective_limits();
        assert_eq!(lower, upper, "no room to move");
    }

    /// The verify divisor is prime so the two wide queries do not repeatedly
    /// land on the same cycle. Two of them in one frame is the overrun.
    #[test]
    fn the_wide_queries_rarely_coincide() {
        let mut state = state();
        let mut both = 0;
        for cycle in 1..10_000u64 {
            state.cycle_count = cycle;
            if cycle % FULL_QUERY_DIVISOR == 0 && cycle % VERIFY_QUERY_DIVISOR == 0 {
                both += 1;
            }
        }
        // Once every 970 cycles, and verify wins when they do.
        assert!(both < 15, "{both} coincidences in 10,000 cycles");
        state.cycle_count = 970;
        assert_eq!(state.profile_for(false), QueryProfile::Verify);
    }

    /// A full parameter frame and a wide query would together approach the
    /// payload limit, so they never share a cycle.
    #[test]
    fn a_full_parameter_frame_never_shares_a_cycle_with_a_wide_query() {
        let mut state = state();
        state.cycle_count = VERIFY_QUERY_DIVISOR;
        assert_eq!(state.profile_for(false), QueryProfile::Verify);
        assert_eq!(state.profile_for(true), QueryProfile::Fast);
    }
}
