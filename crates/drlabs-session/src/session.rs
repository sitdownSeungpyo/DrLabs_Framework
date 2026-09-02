//! One controller, driven by one control loop.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Instant;

use drlabs_core::command::{Command, ControlMode, Mode};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::Link;
use drlabs_core::render::{render, Units};
use drlabs_core::telemetry::Telemetry;
use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};
use drlabs_safety::{Config as SafetyConfig, Monitor, Preflight, Preset};

use crate::control_loop::{self, Shared};
use crate::record::{EventLog, Level, SampleRing};
use crate::shuttle::Shuttle;
use crate::state::{
    Connection, EStop, LoopStats, Params, Setpoint, State, TrajectoryStatus, DEFAULT_RATE_HZ,
    MAX_RATE_HZ, MIN_RATE_HZ,
};
use crate::trajectory::Trapezoidal;

/// How a session is set up.
pub struct Options {
    /// The starting envelope.
    pub safety: SafetyConfig,

    /// Renders the positions, speeds and torques this session puts in its log.
    ///
    /// `None` means motor space — revolutions at the shaft, torque at the rotor
    /// — which is what somebody driving one controller directly is thinking in.
    /// A robot supplies a joint renderer instead, so a limit written in radians
    /// is not reported back in revolutions.
    pub units: Option<Arc<dyn Units>>,

    pub rate_hz: u32,
    /// How many telemetry samples to retain (about two minutes at 100 Hz).
    pub sample_buffer: usize,
    pub event_buffer: usize,

    /// Makes [`Session::arm`] refuse until the caller confirms it has shown the
    /// operator the preflight advisories.
    ///
    /// On by default: an arm that cannot be refused is not a checklist.
    /// [`Options::auto_arm`] turns it off implicitly — there is nobody to
    /// acknowledge.
    pub require_acknowledgement: bool,

    /// Energises the motor as soon as it can and holds it there, waiting for
    /// commands.
    ///
    /// This is the servo role rather than the operator role: the framework is
    /// the low level, something above it decides what the machine does, and an
    /// axis that needs a human to press Arm first cannot serve that. With it
    /// set, the loop arms the moment preflight is clear and re-arms if it later
    /// becomes clear — a supply that was still settling at start-up should not
    /// leave the axis dead until someone notices.
    ///
    /// It never re-arms through a latched emergency stop. A stop that undid
    /// itself would be worse than no stop at all.
    pub auto_arm: bool,

    /// What an auto-armed session commands once armed. Position holds where the
    /// axis already was: torque on, standing still, waiting.
    pub initial_mode: ControlMode,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            safety: SafetyConfig::default(),
            units: None,
            rate_hz: DEFAULT_RATE_HZ,
            sample_buffer: 12_000,
            event_buffer: 500,
            require_acknowledgement: true,
            auto_arm: false,
            initial_mode: ControlMode::Position,
        }
    }
}

/// A snapshot of everything a UI needs.
#[derive(Debug, Clone)]
pub struct Status {
    pub connection: Connection,
    pub connect_error: String,
    pub link: Vec<(String, String)>,
    pub identity: Option<drlabs_core::link::Identity>,

    pub armed: bool,
    pub estop: Option<EStop>,
    pub mode: ControlMode,
    pub rate_hz: u32,
    pub achieved_rate_hz: f64,

    pub setpoint: Setpoint,
    pub params: Params,
    pub safety: SafetyConfig,
    pub preset: Option<Preset>,
    pub telemetry: Telemetry,
    pub warnings: Vec<String>,

    pub shuttle: Shuttle,
    pub trajectory: TrajectoryStatus,

    pub cycles: u64,
    pub replies: u64,
    pub timeouts: u64,
    pub jitter: drlabs_timing::Statistics,
    /// Time spent inside `link.cycle()` — the wire transaction alone,
    /// as opposed to `jitter`, which measures the loop's cadence. A
    /// full-query spike shows up here first.
    pub transaction: drlabs_timing::Statistics,
}

/// Drives exactly one controller.
pub struct Session {
    shared: Arc<Shared>,
    wake: SyncSender<()>,
    loop_handle: Mutex<Option<JoinHandle<()>>>,
}

impl Session {
    /// Creates a disconnected session.
    pub fn new(options: Options) -> Arc<Self> {
        let mut safety = options.safety;
        let notes = safety.normalise(None);

        let mut monitor = Monitor::new(safety);
        if let Some(units) = &options.units {
            monitor = monitor.with_units(Box::new(SharedUnits(units.clone())));
        }

        let mut events = EventLog::new(options.event_buffer);
        for note in &notes {
            events.append(Level::Info, "Safety configuration adjusted", note);
        }

        let state = State {
            connection: Connection::Disconnected,
            connect_error: String::new(),
            link_description: Vec::new(),
            identity: None,
            safety,
            monitor,
            preset: None,
            units: options.units.clone(),
            armed: false,
            arm_blockers: Vec::new(),
            arm_advisories: Vec::new(),
            estop: None,
            mode: ControlMode::Stop,
            rate_hz: options.rate_hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ),
            setpoint: Setpoint::default(),
            params: Params::default(),
            shuttle: Shuttle::default(),
            trajectory: TrajectoryStatus::default(),
            profile: None,
            trajectory_started_at: None,
            telemetry: Telemetry::default(),
            latched_fault: None,
            last_reply_at: None,
            // Auto-arm turns acknowledgement off implicitly: there is nobody to
            // acknowledge.
            require_ack: options.require_acknowledgement && !options.auto_arm,
            auto_arm: options.auto_arm,
            initial_mode: options.initial_mode,
            auto_arm_reported_at: None,
            stats: LoopStats::default(),
            samples: SampleRing::new(options.sample_buffer),
            events,
            params_dirty: true,
            params_sent_at: None,
            stop_burst: 0,
            cycle_count: 0,
            pending_special: None,
            heartbeat_at: Instant::now(),
            started_at: Instant::now(),
        };

        let (wake, wake_rx) = sync_channel(1);
        let shared = Arc::new(Shared {
            state: Mutex::new(state),
            link: Mutex::new(None),
            running: AtomicBool::new(true),
        });

        let loop_shared = shared.clone();
        let handle = std::thread::Builder::new()
            .name("drlabs-session".into())
            .spawn(move || control_loop::run(loop_shared, wake_rx))
            .expect("the control loop thread could not be started");

        Arc::new(Self {
            shared,
            wake,
            loop_handle: Mutex::new(Some(handle)),
        })
    }

    /// Nudges the loop so it notices a change without waiting out the period.
    ///
    /// It does *not* make the loop run early. See the comment in
    /// [`control_loop::run`]: a wake that ran a cycle turned a slider drag into
    /// an eight-fold overrun of the configured rate, and stopped a machine.
    fn signal(&self) {
        let _ = self.wake.try_send(());
    }

    /// Takes a link that is already open and starts driving it.
    pub fn connect(&self, mut link: Box<dyn Link>, allow_unknown_board: bool) -> Result<()> {
        {
            let mut state = self.shared.state();
            state.connection = Connection::Connecting;
            state.connect_error.clear();
        }

        // Open before anything else. This is where a driver takes control of
        // what the last session left behind — a moteus still driving, a
        // Dynamixel with a latched bus watchdog — and where it tunes the
        // transport for a control loop. Skipping it is not a small omission:
        // an unopened Dynamixel keeps its port's default ten-millisecond read
        // timeout, and a loop that reads twice a cycle runs at 44 Hz however
        // it is configured. Measured on this bench, exactly that way.
        if let Err(error) = link.open() {
            let mut state = self.shared.state();
            state.connection = Connection::Disconnected;
            state.connect_error = error.to_string();
            return Err(error);
        }

        let description = link.describe();
        let identity = match link.identify() {
            Ok(identity) => identity,
            Err(error) => {
                let mut state = self.shared.state();
                state.connection = Connection::Disconnected;
                state.connect_error = error.to_string();
                return Err(error);
            }
        };

        if let Some(identity) = &identity {
            if identity.status == drlabs_core::link::BoardStatus::Rejected && !allow_unknown_board {
                let mut state = self.shared.state();
                state.connection = Connection::Disconnected;
                let message = format!(
                    "board {:?} is outside the range this build supports",
                    identity.board
                );
                state.connect_error = message.clone();
                return Err(Error::Refused(message));
            }
        }

        let envelope = link.envelope();

        {
            let mut state = self.shared.state();

            if let Some(envelope) = &envelope {
                // The hardware's own ratings tighten the declared envelope…
                let notes = state.safety.normalise(Some(envelope));
                for note in notes {
                    state.log(Level::Info, "Safety configuration adjusted", &note);
                }
                // …except the voltage band, which is widened when it would
                // otherwise exclude the motor's rated supply.
                let notes = state.safety.fit_voltage_to(envelope);
                for note in notes {
                    state.log(Level::Info, "Safety configuration adjusted", &note);
                }
                let safety = state.safety;
                state.monitor.config = safety;
            }

            state.link_description = description;
            state.identity = identity;
            state.connection = Connection::Connected;
            state.armed = false;
            state.estop = None;
            state.mode = ControlMode::Stop;
            state.params_dirty = true;
            state.telemetry = Telemetry::default();
            state.stats = LoopStats::default();
            state.monitor.reset();
            state.log(Level::Ok, "Connected", "");
        }

        *self.shared.link.lock().expect("link poisoned") = Some(link);
        self.signal();
        Ok(())
    }

    /// Stops the motor and releases the link.
    pub fn disconnect(&self) -> Result<()> {
        {
            let mut state = self.shared.state();
            state.armed = false;
            state.connection = Connection::Disconnected;
            state.shuttle.stop("disconnected");
            state.trajectory.running = false;
            state.profile = None;
            state.log(Level::Info, "Disconnected", "");
        }
        let mut link = self.shared.link.lock().expect("link poisoned");
        if let Some(mut link) = link.take() {
            link.close()?;
        }
        Ok(())
    }

    /// What stands between the axis and being armed.
    pub fn preflight(&self) -> Preflight {
        let state = self.shared.state();
        let connected = state.connection == Connection::Connected;
        state
            .monitor
            .preflight(state.identity.as_ref(), &state.telemetry, connected)
    }

    /// Energises the axis.
    ///
    /// `acknowledged` says the caller has shown the operator the preflight
    /// advisories. Without it, and with acknowledgement required, this refuses —
    /// an arm that cannot be refused is not a checklist.
    pub fn arm(&self, acknowledged: bool) -> Result<()> {
        let preflight = self.preflight();
        let mut state = self.shared.state();

        if state.estop.is_some() {
            return Err(Error::Refused(
                "clear the emergency stop before arming".into(),
            ));
        }
        state.arm_blockers = preflight.blocking.clone();
        state.arm_advisories = preflight.advisory.clone();

        if !preflight.may_arm() {
            return Err(Error::Refused(preflight.blocking.join(" ")));
        }
        if state.require_ack && !acknowledged && !preflight.advisory.is_empty() {
            return Err(Error::Refused(
                "the preflight advisories have to be acknowledged first".into(),
            ));
        }

        state.monitor.reset();
        state.armed = true;
        state.heartbeat_at = Instant::now();
        state.params_dirty = true;
        state.mode = state.initial_mode;

        // Adopt the position the axis is already at, so energising is not
        // itself a move.
        if let Some(position) = state.telemetry.position {
            if state.mode == ControlMode::Position {
                state.setpoint.position = position;
            }
        }
        if state.mode == ControlMode::Velocity {
            state.setpoint.velocity = RevPerSec(0.0);
        }

        let detail = {
            let units = render(state.units.as_deref());
            format!("holding at {}", units.position(state.setpoint.position))
        };
        state.log(Level::Ok, "Armed", &detail);
        drop(state);
        self.signal();
        Ok(())
    }

    /// Releases the axis without latching anything.
    /// Turns automatic arming on or off while the session is running.
    ///
    /// A robot's joints arm themselves, because there is nobody at the axis to
    /// press a button. That is right for driving and wrong for anything that
    /// needs the machine to *stay* released: disarming a joint that arms itself
    /// again on the next cycle is not releasing it, and an operator who was
    /// told the machine is limp would be leaning on a live one.
    pub fn set_auto_arm(&self, enabled: bool) {
        let mut state = self.shared.state();
        if state.auto_arm == enabled {
            return;
        }
        state.auto_arm = enabled;
        state.auto_arm_reported_at = None;
        drop(state);
        self.signal();
    }

    pub fn auto_arm(&self) -> bool {
        self.shared.state().auto_arm
    }

    pub fn disarm(&self, reason: &str) {
        let mut state = self.shared.state();
        if !state.armed {
            return;
        }
        state.armed = false;
        state.stop_burst = 3;
        state.shuttle.stop("disarmed");
        state.trajectory.running = false;
        state.profile = None;
        state.log(Level::Info, "Disarmed", reason);
        drop(state);
        self.signal();
    }

    /// Latches an emergency stop.
    pub fn estop(&self, code: &str, message: &str, detail: &str) {
        control_loop::latch_estop(
            &self.shared,
            EStop {
                code: code.to_string(),
                message: message.to_string(),
                detail: detail.to_string(),
            },
        );
        self.signal();
    }

    /// Clears a latched emergency stop.
    ///
    /// Deliberately explicit: a stop that undid itself would be worse than no
    /// stop at all. The axis comes back disarmed, so somebody has to arm it
    /// again as well.
    pub fn reset_estop(&self) -> Result<()> {
        let mut state = self.shared.state();
        if state.estop.is_none() {
            return Ok(());
        }
        if let Some(fault) = state.latched_fault {
            if let Some(current) = &state.telemetry.fault {
                if current.hard && current.code == fault {
                    return Err(Error::Refused(format!(
                        "the controller still reports fault {}: {}",
                        current.code, current.name
                    )));
                }
            }
        }
        state.estop = None;
        state.latched_fault = None;
        state.armed = false;
        state.monitor.reset();
        state.log(Level::Ok, "Emergency stop cleared", "the axis is disarmed");
        drop(state);
        self.signal();
        Ok(())
    }

    /// Whether a mode can be commanded right now.
    pub fn can_command(&self, mode: ControlMode) -> Result<()> {
        let state = self.shared.state();
        if state.connection != Connection::Connected {
            return Err(Error::Closed);
        }
        if state.estop.is_some() {
            return Err(Error::Refused("emergency stop is latched".into()));
        }
        if !state.armed && !matches!(mode, ControlMode::Stop | ControlMode::Query) {
            return Err(Error::Refused("the axis is not armed".into()));
        }
        Ok(())
    }

    pub fn set_mode(&self, mode: ControlMode) -> Result<()> {
        self.can_command(mode)?;
        let mut state = self.shared.state();
        if state.mode == mode {
            return Ok(());
        }
        // Changing mode abandons whatever was sequencing the setpoint.
        state.shuttle.stop("the mode changed");
        state.trajectory.running = false;
        state.profile = None;

        state.mode = mode;
        state.params_dirty = true;
        if mode == ControlMode::Position {
            if let Some(position) = state.telemetry.position {
                state.setpoint.position = position;
            }
        }
        if mode == ControlMode::Velocity {
            state.setpoint.velocity = RevPerSec(0.0);
        }
        state.log(Level::Info, "Mode changed", mode.as_str());
        drop(state);
        self.signal();
        Ok(())
    }

    /// Sets the position target, and reports what it was clamped to.
    pub fn set_position(&self, target: Rev) -> Rev {
        let mut state = self.shared.state();
        let (clamped, _) = state.clamp_target(target);
        state.setpoint.position = clamped;
        // A manual target ends a sequence that was driving the setpoint.
        state.shuttle.stop("a position was commanded");
        state.trajectory.running = false;
        state.profile = None;
        drop(state);
        self.signal();
        clamped
    }

    pub fn set_velocity(&self, target: RevPerSec) -> RevPerSec {
        let mut state = self.shared.state();
        let (clamped, _) = state.safety.clamp_velocity(target);
        state.setpoint.velocity = clamped;
        drop(state);
        self.signal();
        clamped
    }

    pub fn set_within(&self, lower: Rev, upper: Rev) -> (Rev, Rev) {
        let mut state = self.shared.state();
        let lower = state.clamp_target(lower).0;
        let upper = state.clamp_target(upper).0;
        state.setpoint.within_lower = lower;
        state.setpoint.within_upper = upper;
        drop(state);
        self.signal();
        (lower, upper)
    }

    pub fn set_params(&self, params: Params) -> Params {
        let mut state = self.shared.state();
        state.params = params;
        // A parameter change has to reach the controller now, not at the next
        // periodic refresh.
        state.params_dirty = true;
        let params = state.params;
        drop(state);
        self.signal();
        params
    }

    pub fn params(&self) -> Params {
        self.shared.state().params
    }

    /// Replaces the safety envelope, and reports any adjustments.
    pub fn set_safety(&self, config: SafetyConfig) -> (SafetyConfig, Vec<String>) {
        let mut state = self.shared.state();
        let mut config = config;
        let notes = config.normalise(None);
        state.safety = config;
        state.monitor.config = config;
        state.params_dirty = true;
        for note in &notes {
            state.log(Level::Info, "Safety configuration adjusted", note);
        }
        drop(state);
        self.signal();
        (config, notes)
    }

    pub fn safety(&self) -> SafetyConfig {
        self.shared.state().safety
    }

    pub fn set_preset(&self, preset: Preset) {
        let mut state = self.shared.state();
        let mut config = state.safety;
        preset.apply(&mut config);
        config.normalise(None);
        state.safety = config;
        state.monitor.config = config;
        state.preset = Some(preset);
        state.log(Level::Info, "Safety preset applied", preset.label());
    }

    /// Changes the control rate, and reports what it was clamped to.
    pub fn set_rate(&self, hz: u32) -> u32 {
        let mut state = self.shared.state();
        let clamped = hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ);
        state.rate_hz = clamped;
        // The watchdog is a multiple of the period, so it moves with the rate.
        state.params_dirty = true;
        state.log(
            Level::Info,
            "Control rate changed",
            &format!("{clamped} Hz"),
        );
        drop(state);
        self.signal();
        clamped
    }

    /// Re-references the output so the axis's current place is called
    /// `position`.
    ///
    /// Refused while armed: the axis would jump to wherever the old setpoint
    /// now points, which is a move nobody asked for.
    pub fn zero_output(&self, position: Rev) -> Result<()> {
        let mut state = self.shared.state();
        if state.connection != Connection::Connected {
            return Err(Error::Closed);
        }
        if state.armed {
            return Err(Error::Refused(
                "disarm before re-referencing the output".into(),
            ));
        }
        state.pending_special = Some(Command::new(Mode::SetOutput { position }));
        state.setpoint.position = position;
        let detail = {
            let units = render(state.units.as_deref());
            format!("the axis is now at {}", units.position(position))
        };
        state.log(Level::Ok, "Output re-referenced", &detail);
        drop(state);
        self.signal();
        Ok(())
    }

    /// Tells the deadman the supervising client is still there.
    pub fn heartbeat(&self) {
        self.shared.state().heartbeat_at = Instant::now();
    }

    /// Begins a host-profiled move from where the axis actually is.
    ///
    /// It starts from the measured position rather than the last setpoint:
    /// starting from a stale setpoint would put a step at the beginning of a
    /// profile whose whole purpose is to have none.
    pub fn start_trajectory(
        &self,
        target: Rev,
        max_velocity: RevPerSec,
        accel: RevPerSec2,
    ) -> Result<()> {
        self.set_mode(ControlMode::Position)?;
        let mut state = self.shared.state();

        if !state.armed {
            return Err(Error::Refused("arm the axis first".into()));
        }
        let start = state
            .telemetry
            .position
            .ok_or_else(|| Error::Refused("the axis position is not being reported".into()))?;

        // The shuttle and a profiled move both own the position setpoint;
        // running them together would have them fight over it every cycle.
        state.shuttle.stop("a trajectory was started");

        let (clamped, _) = state.clamp_target(target);
        // The profile may not exceed the operator's envelope, whatever was
        // asked for here.
        let velocity_ceiling = if state.params.velocity_limit < state.safety.max_velocity {
            state.params.velocity_limit
        } else {
            state.safety.max_velocity
        };
        let max_velocity = max_velocity.clamp_to(RevPerSec(0.0), velocity_ceiling);
        let accel_ceiling = RevPerSec2(state.safety.accel_limit.value().max(0.01));
        let accel = accel.clamp_to(RevPerSec2(0.0), accel_ceiling);

        let profile = Trapezoidal::new(start, clamped, max_velocity, accel)
            .map_err(|error| Error::Refused(error.to_string()))?;

        state.trajectory = TrajectoryStatus {
            running: true,
            start,
            target: clamped,
            max_velocity,
            accel,
            total: profile.total_time(),
            setpoint: start,
            status: "running".into(),
            ..Default::default()
        };
        state.profile = Some(profile);
        state.trajectory_started_at = Some(Instant::now());

        let detail = {
            let units = render(state.units.as_deref());
            format!(
                "{} → {} · {} · {} · {:.2} s",
                units.position(start),
                units.position(clamped),
                units.velocity(max_velocity),
                units.accel(accel),
                profile.total_time()
            )
        };
        state.log(Level::Ok, "Trajectory started", &detail);
        drop(state);
        self.signal();
        Ok(())
    }

    /// Sets the shuttle's endpoints and timing without starting it.
    pub fn configure_shuttle(&self, a: Rev, b: Rev, dwell: f64, cycles: u32) -> Shuttle {
        let mut state = self.shared.state();
        let a = state.clamp_target(a).0;
        let b = state.clamp_target(b).0;
        state.shuttle.configure(a, b, dwell, cycles);
        state.shuttle.clone()
    }

    pub fn start_shuttle(&self) -> Result<()> {
        self.set_mode(ControlMode::Position)?;
        let mut state = self.shared.state();
        if !state.armed {
            return Err(Error::Refused("arm the axis first".into()));
        }
        // A profiled move and the shuttle both own the setpoint.
        state.trajectory.running = false;
        state.profile = None;

        state
            .shuttle
            .start(Instant::now())
            .map_err(|reason| Error::Refused(reason.into()))?;
        let detail = {
            let units = render(state.units.as_deref());
            format!(
                "{} ↔ {} · {:.1} s dwell",
                units.position(state.shuttle.a),
                units.position(state.shuttle.b),
                state.shuttle.dwell
            )
        };
        state.log(Level::Ok, "Shuttle started", &detail);
        drop(state);
        self.signal();
        Ok(())
    }

    pub fn stop_shuttle(&self, reason: &str) {
        let mut state = self.shared.state();
        if !state.shuttle.running {
            return;
        }
        state.shuttle.stop(reason);
        state.log(Level::Info, "Shuttle stopped", reason);
    }

    pub fn shuttle(&self) -> Shuttle {
        self.shared.state().shuttle.clone()
    }

    /// Everything a UI needs, in one lock.
    pub fn status(&self) -> Status {
        let state = self.shared.state();
        let achieved = achieved_rate(&state);

        Status {
            connection: state.connection,
            connect_error: state.connect_error.clone(),
            link: state.link_description.clone(),
            identity: state.identity.clone(),
            armed: state.armed,
            estop: state.estop.clone(),
            mode: state.mode,
            rate_hz: state.rate_hz,
            achieved_rate_hz: achieved,
            setpoint: state.setpoint,
            params: state.params,
            safety: state.safety,
            preset: state.preset,
            telemetry: state.telemetry.clone(),
            warnings: state.monitor.warnings.clone(),
            shuttle: state.shuttle.clone(),
            trajectory: state.trajectory.clone(),
            cycles: state.stats.cycles,
            replies: state.stats.replies,
            timeouts: state.stats.timeouts,
            // Collected out of the ring: `summarise` wants a slice, and this
            // runs at status-poll rate, not loop rate.
            jitter: drlabs_timing::summarise(&Vec::from_iter(state.stats.periods.iter().copied())),
            transaction: drlabs_timing::summarise(&Vec::from_iter(
                state.stats.durations.iter().copied(),
            )),
        }
    }

    /// The events newer than `seq`.
    pub fn events(&self, seq: u64) -> Vec<crate::record::Event> {
        self.shared.state().events.since(seq)
    }

    /// The telemetry samples newer than `seq`, at most `limit` of them.
    pub fn samples(&self, seq: u64, limit: usize) -> Vec<crate::record::Sample> {
        self.shared.state().samples.since(seq, limit)
    }

    /// Stops the loop and releases the link. Safe to call more than once.
    pub fn shutdown(&self) {
        self.shared.running.store(false, Ordering::Relaxed);
        self.signal();
        if let Some(handle) = self.loop_handle.lock().expect("handle poisoned").take() {
            let _ = handle.join();
        }
        let _ = self.disconnect();
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        // A dropped session must not leave a motor energised, whatever the
        // caller forgot.
        self.shutdown();
    }
}

/// The rate the loop is actually achieving, measured rather than assumed.
fn achieved_rate(state: &State) -> f64 {
    let window = &state.stats.rate_window;
    if window.len() < 2 {
        return 0.0;
    }
    let (Some(first), Some(last)) = (window.front(), window.back()) else {
        return 0.0;
    };
    let span = last.duration_since(*first).as_secs_f64();
    if span <= 0.0 {
        return 0.0;
    }
    (window.len() - 1) as f64 / span
}

/// Lets an `Arc<dyn Units>` be handed to something that wants a `Box<dyn Units>`.
struct SharedUnits(Arc<dyn Units>);

impl Units for SharedUnits {
    fn position(&self, value: Rev) -> String {
        self.0.position(value)
    }
    fn span(&self, value: Rev) -> String {
        self.0.span(value)
    }
    fn velocity(&self, value: RevPerSec) -> String {
        self.0.velocity(value)
    }
    fn accel(&self, value: RevPerSec2) -> String {
        self.0.accel(value)
    }
    fn torque(&self, value: drlabs_core::units::Nm) -> String {
        self.0.torque(value)
    }
}
