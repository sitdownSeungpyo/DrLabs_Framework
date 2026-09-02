//! The control loop: one command out, one telemetry reply in, on a schedule.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_core::command::{Command, ControlMode, Field, Mode, QueryProfile, TorqueLimit};
use drlabs_core::error::Error;
use drlabs_core::link::Link;
use drlabs_core::render::render;
use drlabs_core::units::{Nm, Rev, RevPerSec};
use drlabs_safety::Trip;
use drlabs_timing::{sleep_until, Resolution, Woke, DEFAULT_SPIN};

use crate::record::Level;
use crate::shuttle::Step;
use crate::state::{Connection, EStop, LoopStats, State, PARAM_REFRESH, RATE_WINDOW};

/// What the loop needs to run: the state, and the link it drives.
///
/// Two locks rather than one, because a cycle holds the link for a whole
/// transaction and a status poll must not wait behind it.
pub struct Shared {
    pub state: Mutex<State>,
    pub link: Mutex<Option<Box<dyn Link>>>,
    pub running: AtomicBool,
}

impl Shared {
    pub fn state(&self) -> std::sync::MutexGuard<'_, State> {
        self.state.lock().expect("session state poisoned")
    }
}

/// Runs until `running` goes false.
///
/// The only thread that touches the link.
pub fn run(shared: Arc<Shared>, wake: Receiver<()>) {
    // A coarse OS timer turns a 100 Hz loop into a 64 Hz one; ask for something
    // finer for as long as we are running.
    let _resolution = Resolution::raise();

    let mut period = shared.state().period();
    let mut next = Instant::now() + period;

    while shared.running.load(Ordering::Relaxed) {
        period = shared.state().period();
        let spin = DEFAULT_SPIN.min(period / 4);

        // Wait for the next slot. A command wakes this early so the loop
        // notices it without polling — but the slot itself does not move.
        //
        // It used to. A wake ran a cycle immediately, so a client that
        // commanded faster than the rate simply got a faster loop: measured on
        // this bench, a robot configured for 100 Hz ran its CAN joint at 801 Hz
        // and its serial joint at 345 Hz under a flood of poses from a slider
        // drag. The serial bus cannot carry 345 Hz, so it fell behind, the
        // servo's own watchdog latched, and that joint e-stopped — a machine
        // stopped by the rate of somebody's mouse.
        //
        // The rate in the description is what the buses were budgeted for. A
        // caller may ask for attention; it may not spend more of the bus than
        // the description allows.
        match sleep_until(next, Some(&wake), spin) {
            // Somebody asked for attention. Notice it — and then wait out the
            // rest of the slot anyway, with no wake channel, so it cannot be
            // cut short again.
            Woke::Early => {
                if !shared.running.load(Ordering::Relaxed) {
                    return;
                }
                sleep_until(next, None, spin);
            }
            Woke::Closed => return,
            Woke::Deadline => {}
        }
        if !shared.running.load(Ordering::Relaxed) {
            return;
        }

        next += period;
        // If we have fallen more than a period behind — a long pause, a laptop
        // resuming from sleep — resynchronise rather than trying to catch up
        // with a burst of back-to-back cycles.
        if next < Instant::now() - period {
            next = Instant::now() + period;
        }

        cycle(&shared, period);
    }
}

/// One control period: build a command, transact, evaluate.
pub fn cycle(shared: &Arc<Shared>, period: Duration) {
    let started = Instant::now();

    let (command, profile, was_full) = {
        let mut state = shared.state();
        if state.connection != Connection::Connected {
            return;
        }
        if let Some(last) = state.stats.last_cycle_at {
            let elapsed = started.duration_since(last);
            LoopStats::record(&mut state.stats.periods, elapsed);
        }
        state.stats.last_cycle_at = Some(started);

        let command = build_command(&mut state, period);
        let profile = state.profile_for(command.full);
        (command, profile, command.full)
    };

    // The link is held only for the transaction itself.
    let outcome = {
        let mut link = shared.link.lock().expect("link poisoned");
        match link.as_mut() {
            Some(link) => link.cycle(&command, profile),
            None => return,
        }
    };

    // A Transport error (or a link closed underneath the loop) is a link the
    // driver has declared unusable — not a dropped cycle. It must be closed,
    // not retried: the clearest case is the moteus full-deadline timeout,
    // where each retry holds the shared bus for the whole two-second budget
    // and freezes every other joint on the adapter with it. A plain Timeout
    // stays what it always was — one dropped cycle, counted and survived.
    let link_dead = matches!(&outcome, Err(Error::Transport { .. }) | Err(Error::Closed));

    let mut trip: Option<Trip> = None;
    let mut estop: Option<EStop> = None;
    let rate_warning: Option<(String, String)>;
    let already_stopped;

    {
        let mut state = shared.state();
        state.stats.cycles += 1;
        LoopStats::record(&mut state.stats.durations, started.elapsed());
        if state.stats.rate_window.len() >= RATE_WINDOW {
            state.stats.rate_window.pop_front();
        }
        state.stats.rate_window.push_back(started);

        match outcome {
            Err(error) => {
                estop = Some(EStop {
                    code: "link_error".into(),
                    message: "Transport error".into(),
                    detail: error.to_string(),
                });
            }
            Ok(None) => {
                state.stats.timeouts += 1;
                state.stats.consecutive_timeouts += 1;
                if state.stats.consecutive_timeouts >= state.safety.max_consecutive_timeouts
                    && state.estop.is_none()
                {
                    let watchdog = state.safety.watchdog_timeout(period.as_secs_f64()) * 1e3;
                    estop = Some(EStop {
                        code: "comm_loss".into(),
                        message: "No reply from the controller".into(),
                        detail: format!(
                            "{} consecutive cycles without a response. The controller's own \
                             watchdog will stop the motor after {watchdog:.0} ms.",
                            state.stats.consecutive_timeouts
                        ),
                    });
                }
            }
            Ok(Some(telemetry)) => {
                if was_full {
                    state.params_dirty = false;
                    state.params_sent_at = Some(started);
                }
                state.stats.consecutive_timeouts = 0;
                state.stats.replies += 1;
                state.last_reply_at = Some(Instant::now());
                state.telemetry.merge(&telemetry);

                let elapsed = started.duration_since(state.started_at);
                {
                    // Disjoint fields through one reborrow: no Telemetry
                    // clone (four heap strings) inside the state lock.
                    let state = &mut *state;
                    state.samples.append(elapsed, &state.telemetry);
                }

                if let Some(fault) = &state.telemetry.fault {
                    if fault.hard {
                        state.latched_fault = Some(fault.code);
                    }
                }
                if profile == QueryProfile::Verify {
                    check_active_limits(&mut state, &telemetry, period);
                }

                let armed = state.armed;
                let now = Instant::now();
                let state = &mut *state;
                trip = state.monitor.evaluate(&state.telemetry, &command, armed, now);
            }
        }

        // Deadman: the supervising client has to keep talking to us.
        if estop.is_none() && state.armed && state.safety.deadman_timeout > 0.0 {
            let idle = state.heartbeat_at.elapsed().as_secs_f64();
            if idle > state.safety.deadman_timeout {
                estop = Some(EStop {
                    code: "deadman".into(),
                    message: "Lost contact with the supervising client".into(),
                    detail: format!("No heartbeat for {idle:.1} s."),
                });
            }
        }

        try_auto_arm(&mut state);

        // Re-assert the full parameter set periodically, so a controller that
        // restarted does not keep running on defaults we never chose.
        if state.armed && !state.params_dirty {
            let stale = state
                .params_sent_at
                .map(|at| started.duration_since(at) > PARAM_REFRESH)
                .unwrap_or(true);
            if stale {
                state.params_dirty = true;
            }
        }

        state.cycle_count += 1;
        rate_warning = check_achieved_rate(&mut state, started, period);
        already_stopped = state.estop.is_some();
    }

    if let Some((message, detail)) = rate_warning {
        let mut state = shared.state();
        state.log(Level::Warn, &message, &detail);
    }
    if !already_stopped {
        if let Some(trip) = trip {
            latch_estop(
                shared,
                EStop {
                    code: trip.code.to_string(),
                    message: trip.message,
                    detail: trip.detail,
                },
            );
        } else if let Some(estop) = estop {
            latch_estop(shared, estop);
        }
    }

    if link_dead {
        // Close and drop the link, and say so. The motor itself is stopped by
        // its own watchdog, which every cycle's command carries; what is being
        // protected here is the rest of the bus, and the operator's ten
        // seconds. Reconnecting is the operator's move — a link that died
        // mid-cycle is not one to quietly reopen.
        if let Some(mut link) = shared.link.lock().expect("link poisoned").take() {
            let _ = link.close();
        }
        let mut state = shared.state();
        state.armed = false;
        state.connection = Connection::Disconnected;
        state.connect_error = "the link failed mid-cycle and was closed".into();
        state.shuttle.stop("link error");
        state.trajectory.running = false;
        state.log(
            Level::Error,
            "Link torn down",
            "A transport failure ended the session; reconnect to continue.",
        );
    }
}

/// Latches an emergency stop and starts a burst of stop frames.
pub fn latch_estop(shared: &Arc<Shared>, reason: EStop) {
    let mut state = shared.state();
    if state.estop.is_some() {
        return;
    }
    state.log(Level::Error, &reason.message, &reason.detail);
    state.estop = Some(reason);
    state.armed = false;
    // Several stop frames rather than one: a single frame can be the one that
    // is lost, and this is the frame that must not be.
    state.stop_burst = 5;
    state.shuttle.stop("emergency stop");
    state.trajectory.running = false;
    state.profile = None;
}

/// Translates the state machine into one command.
fn build_command(state: &mut State, period: Duration) -> Command {
    // A pending one-shot — re-referencing the output, say — takes this cycle
    // and is then forgotten.
    if let Some(special) = state.pending_special.take() {
        return special;
    }

    if state.estop.is_some() || state.stop_burst > 0 {
        state.stop_burst = state.stop_burst.saturating_sub(1);
        return Command::new(Mode::Stop);
    }
    if !state.armed {
        // Disarmed: watch the axis, do not drive it.
        return Command::new(Mode::Query);
    }

    let mode = match state.mode {
        ControlMode::Stop => Mode::Stop,
        ControlMode::Query => Mode::Query,
        ControlMode::Brake => Mode::Brake,
        ControlMode::Hold => Mode::Hold,
        ControlMode::Position => Mode::Position,
        ControlMode::Velocity => Mode::Velocity,
        ControlMode::Torque => Mode::Torque,
        ControlMode::StayWithin => Mode::StayWithin {
            lower: state.clamp_target(state.setpoint.within_lower).0,
            upper: state.clamp_target(state.setpoint.within_upper).0,
        },
        ControlMode::SetOutput => Mode::SetOutput {
            position: state.setpoint.position,
        },
    };

    let velocity_limit = state
        .params
        .velocity_limit
        .min_of(state.safety.max_velocity);

    let mut command = Command {
        watchdog: Field::Set(state.safety.watchdog_timeout(period.as_secs_f64())),
        velocity_limit: Field::Set(velocity_limit),
        accel_limit: Field::Set(state.params.accel_limit),
        kp_scale: Field::Set(state.params.kp_scale),
        kd_scale: Field::Set(state.params.kd_scale),
        ilimit_scale: Field::Set(state.params.ilimit_scale),
        feedforward_torque: Field::Set(state.params.feedforward),
        ignore_position_bounds: state.params.ignore_position_bounds,
        full: state.params_dirty,
        max_torque: if state.params.defer_torque_to_controller {
            TorqueLimit::Controller
        } else {
            TorqueLimit::Nm(state.params.max_torque.min_of(state.safety.max_torque))
        },
        ..Command::new(mode)
    };

    match state.mode {
        ControlMode::Stop | ControlMode::Brake | ControlMode::Hold | ControlMode::Query => {
            // The mode is the whole command.
        }
        ControlMode::StayWithin => {
            // The bounds are the point of this mode, so they go out every cycle
            // rather than only when the parameter set is refreshed.
            command.full = true;
        }
        ControlMode::SetOutput => {}
        ControlMode::Position => {
            // A host-profiled move and the shuttle both own the setpoint; only
            // one of them can be running.
            let mut feedforward = RevPerSec(0.0);
            let now = Instant::now();

            if state.trajectory.running {
                if let Some((position, velocity)) = advance_trajectory(state, now) {
                    state.setpoint.position = position;
                    feedforward = velocity;
                }
            } else if state.shuttle.running {
                let position = state.telemetry.position;
                let velocity = state.telemetry.velocity;
                let step = state
                    .shuttle
                    .advance(now, position, velocity, velocity_limit);
                match step {
                    Step::Target(target) | Step::Finished { target, .. } => {
                        state.setpoint.position = target;
                    }
                    Step::Failed(reason) => {
                        state.log(Level::Warn, "Shuttle stopped", &reason);
                    }
                    Step::Idle => {}
                }
            }

            let (target, clamped) = state.clamp_target(state.setpoint.position);
            if clamped {
                state.setpoint.position = target;
            }
            command.position = Field::Set(target);
            // The profile knows how fast the axis should be going, and saying
            // so is the difference between the controller chasing a moving
            // target and tracking one. Without a profile there is nothing to
            // feed forward.
            command.velocity = Field::Set(feedforward);

            // Tell the safety monitor whether motion is expected, so holding
            // against a load is not mistaken for a stall.
            if let Some(position) = state.telemetry.position {
                let threshold = Rev(0.02_f64.max(state.safety.stall_velocity.value()));
                if (target - position).abs() > threshold {
                    command.velocity_target = velocity_limit;
                }
            }
        }
        ControlMode::Velocity => {
            let (velocity, clamped) = state.safety.clamp_velocity(state.setpoint.velocity);
            if clamped {
                state.setpoint.velocity = velocity;
            }
            command.velocity = Field::Set(velocity);
            command.velocity_target = velocity;
        }
        ControlMode::Torque => {
            command.velocity_target = RevPerSec(0.0);
        }
    }

    command
}

/// Advances a host-side profiled move, returning where the axis should be and
/// how fast.
fn advance_trajectory(state: &mut State, now: Instant) -> Option<(Rev, RevPerSec)> {
    let profile = state.profile?;
    let started = state.trajectory_started_at?;
    let elapsed = now.duration_since(started).as_secs_f64();
    let point = profile.sample(elapsed);

    state.trajectory.elapsed = elapsed;
    state.trajectory.setpoint = point.position;
    state.trajectory.feedforward = point.velocity;
    state.trajectory.progress = if profile.total_time() > 0.0 {
        (elapsed / profile.total_time()).clamp(0.0, 1.0)
    } else {
        1.0
    };

    if elapsed >= profile.total_time() {
        state.trajectory.running = false;
        state.trajectory.progress = 1.0;
        state.trajectory.status = "finished".into();
        state.profile = None;
        state.log(
            Level::Ok,
            "Trajectory finished",
            &format!("{:.2} s", profile.total_time()),
        );
        return Some((profile.target(), RevPerSec(0.0)));
    }
    Some((point.position, point.velocity))
}

/// Compares the controller's live command registers with what this session
/// believes it commanded.
///
/// A mismatch is not a theoretical worry. Command registers are sticky and
/// survive things the host does not see — a brownout that resets the
/// controller, a dropped frame, another program on the same bus. When they
/// disagree, the parameter frame is re-sent immediately rather than waiting for
/// the next periodic refresh.
fn check_active_limits(
    state: &mut State,
    telemetry: &drlabs_core::telemetry::Telemetry,
    period: Duration,
) {
    if !state.armed {
        return;
    }
    if !matches!(
        state.mode,
        ControlMode::Position | ControlMode::Velocity | ControlMode::StayWithin
    ) {
        return;
    }

    let mut checks: Vec<(&str, f64, Option<f64>)> = vec![
        (
            "velocity limit",
            state
                .params
                .velocity_limit
                .min_of(state.safety.max_velocity)
                .value(),
            telemetry.active.velocity_limit.map(|v| v.value()),
        ),
        (
            "acceleration limit",
            state.params.accel_limit.value(),
            telemetry.active.accel_limit.map(|v| v.value()),
        ),
        (
            "watchdog timeout",
            state.safety.watchdog_timeout(period.as_secs_f64()),
            telemetry.active.watchdog,
        ),
    ];
    if !state.params.defer_torque_to_controller {
        checks.push((
            "max torque",
            state
                .params
                .max_torque
                .min_of(state.safety.max_torque)
                .value(),
            telemetry.active.max_torque.map(|v| v.value()),
        ));
    }

    for (name, expected, actual) in checks {
        let actual = match actual {
            Some(actual) => actual,
            None => continue,
        };
        let tolerance = 0.02_f64.max(expected.abs() * 0.05);
        if (actual - expected).abs() > tolerance {
            state.params_dirty = true;
            state.log(
                Level::Warn,
                &format!("Controller limit mismatch ({name})"),
                &format!(
                    "expected {expected:.3}, controller reports {actual:.3} — resending parameters"
                ),
            );
            return;
        }
    }
}

/// Tells the operator when the requested rate is not being achieved, instead of
/// quietly running slower than the status claims.
fn check_achieved_rate(
    state: &mut State,
    now: Instant,
    period: Duration,
) -> Option<(String, String)> {
    if state.cycle_count % 200 != 0 || state.stats.rate_window.len() <= 50 {
        return None;
    }
    let window = &state.stats.rate_window;
    let (Some(first), Some(last)) = (window.front(), window.back()) else {
        return None;
    };
    let span = last.duration_since(*first).as_secs_f64();
    if span <= 0.0 {
        return None;
    }
    let achieved = (window.len() - 1) as f64 / span;
    if achieved >= state.rate_hz as f64 * 0.85 {
        return None;
    }
    if let Some(warned) = state.stats.rate_warned_at {
        if now.duration_since(warned) <= Duration::from_secs(30) {
            return None;
        }
    }
    state.stats.rate_warned_at = Some(now);
    Some((
        format!(
            "Control loop is running at {achieved:.0} Hz, not {} Hz",
            state.rate_hz
        ),
        format!(
            "The transport cannot complete a transaction every {:.1} ms. Lower the control rate.",
            period.as_secs_f64() * 1e3
        ),
    ))
}

/// Energises the axis when it is configured to run unattended and nothing is
/// standing in the way.
///
/// Called every cycle rather than once at connect, because the reasons an axis
/// cannot arm are usually transient — a supply still settling, a controller
/// still reporting the fault it powered up with — and a servo that gave up on
/// the first attempt would need a human to notice, which is the thing this role
/// exists to avoid.
///
/// The one condition it will not wait out is a latched emergency stop. That has
/// to be cleared deliberately, by whatever decided to raise it.
fn try_auto_arm(state: &mut State) {
    if !state.auto_arm
        || state.armed
        || state.estop.is_some()
        || state.connection != Connection::Connected
    {
        return;
    }
    // Nothing has been heard from the controller yet; there is nothing to check
    // preflight against.
    if state.telemetry.mode.is_none() && state.telemetry.position.is_none() {
        return;
    }

    let preflight = state
        .monitor
        .preflight(state.identity.as_ref(), &state.telemetry, true);

    if !preflight.may_arm() {
        // Say why, but once a second rather than at the control rate.
        let due = state
            .auto_arm_reported_at
            .map(|at| at.elapsed() > Duration::from_secs(1))
            .unwrap_or(true);
        if due {
            state.auto_arm_reported_at = Some(Instant::now());
            let detail = preflight.blocking.join(" ");
            state.log(Level::Warn, "Waiting to arm", &detail);
        }
        return;
    }

    state.monitor.reset();
    state.armed = true;
    state.heartbeat_at = Instant::now();
    state.params_dirty = true;
    state.auto_arm_reported_at = None;
    state.mode = state.initial_mode;

    // Adopt the position the axis is already at, so energising is not itself a
    // move. This is the whole difference between "torque on, holding" and
    // "torque on, and now travelling to wherever the last setpoint was".
    if let Some(position) = state.telemetry.position {
        match state.mode {
            ControlMode::Position => state.setpoint.position = position,
            ControlMode::StayWithin => {
                let half = Rev(0.1_f64.max(
                    (state.setpoint.within_upper - state.setpoint.within_lower).value() / 2.0,
                ));
                state.setpoint.within_lower = position - half;
                state.setpoint.within_upper = position + half;
            }
            _ => {}
        }
    }
    if state.mode == ControlMode::Velocity {
        state.setpoint.velocity = RevPerSec(0.0);
    }

    let torque = state.params.max_torque.min_of(state.safety.max_torque);
    let detail = {
        let units = render(state.units.as_deref());
        format!(
            "holding {} at {} · torque limit {}",
            state.mode.as_str(),
            units.position(state.setpoint.position),
            units.torque(torque)
        )
    };
    state.log(Level::Ok, "Armed automatically", &detail);
}

/// Smallest of two, so a session limit never exceeds the safety ceiling.
trait MinOf {
    fn min_of(self, other: Self) -> Self;
}

impl MinOf for RevPerSec {
    fn min_of(self, other: Self) -> Self {
        if self < other {
            self
        } else {
            other
        }
    }
}

impl MinOf for Nm {
    fn min_of(self, other: Self) -> Self {
        if self < other {
            self
        } else {
            other
        }
    }
}
