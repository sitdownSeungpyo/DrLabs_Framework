//! The bus thread.
//!
//! One tick, in order:
//!
//! 1. take the latest [`LowCmd`] snapshot and note how old it is;
//! 2. act on its robot-wide flags, once per command sequence number;
//! 3. for every axis, decide through the gate whether it is driven, and
//!    build its motor-space command — clamped to the safety envelope; an
//!    axis that has just left driving gets a short burst of stops, and an
//!    axis that is off and was asked to re-reference gets that once;
//! 4. one call into the bus, the only blocking I/O on the thread;
//! 5. for every axis, merge the reply, run the guard, convert to joint
//!    space, and latch the robot-wide stop if anything tripped;
//! 6. publish the bus's state snapshot.
//!
//! The command built in step 3 reflects what was known at the end of the
//! previous tick, so a trip takes effect one period later. That is the same
//! latency the session had, and it is bounded by the period rather than by
//! anything a client does.
//!
//! Nothing here allocates after construction. The command and reply arrays
//! are built once; the records are `Copy`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::bus::{AxisCmd, AxisReply, AxisStatus, Bus};
use drlabs_core::command::{Command, Field, Mode, QueryProfile, TorqueLimit};
use drlabs_core::command::ControlMode;
use drlabs_core::lowlevel::{flags, mode, LowCmd, MotorCmd, MotorState, MAX_AXES};
use drlabs_core::units::{Nm, Rad, RadPerSec, Rev, RevPerSec};
use drlabs_safety::{Config as SafetyConfig, Guard, TripCode};
use drlabs_timing::{sleep_until, Resolution};

use crate::exchange::{Reader, Stamped, Writer};
use crate::gate::{trip_allows_hold, AxisGate, Estop, EstopPolicy, TimeoutPolicy};
use crate::mapping::Mapping;

/// What one axis on the bus is.
#[derive(Debug, Clone, Copy)]
pub struct AxisSpec {
    /// Its index in [`LowCmd::cmd`] and [`BusState::state`].
    pub joint: usize,
    pub mapping: Mapping,
    /// The motor-space envelope, already through the mapping.
    pub safety: SafetyConfig,
    /// What the axis does when the robot is stopped while it is driven.
    pub on_estop: EstopPolicy,
    /// The following-error ceiling, motor revolutions; `None` is off.
    pub following_error: Option<f64>,
}

/// How long a following error must persist before it stops the robot.
const FOLLOWING_ERROR_TIME: Duration =
    Duration::from_millis((crate::config::FOLLOWING_ERROR_TIME_S * 1000.0) as u64);

/// An axis an [`EstopPolicy`] is keeping powered through a stop.
#[derive(Debug, Clone, Copy)]
struct EstopHold {
    /// Where it comes to rest, motor space: the setpoint's stopping point
    /// when the stop began, so it decelerates straight to it.
    target: Rev,
    /// `stop_then_release`: let go once at rest.
    release_when_still: bool,
    since: Instant,
}

/// How long `stop_then_release` waits for the setpoint to come to rest
/// before releasing anyway.
const STOP_THEN_RELEASE_LIMIT: Duration = Duration::from_secs(2);

/// How the thread runs.
#[derive(Debug, Clone)]
pub struct WorkerConfig {
    pub name: String,
    pub period: Duration,
    /// A command older than this is stale; see [`TimeoutPolicy`].
    pub cmd_timeout: Duration,
    pub on_timeout: TimeoutPolicy,
    /// Consecutive ticks that start a whole period late before the robot is
    /// stopped. Zero disables the trip.
    pub deadline_miss_trip: u32,
    /// Every this many ticks, the wide query goes out instead of the fast one.
    pub full_every: u32,
    /// Every this many ticks, the complete parameter set is re-sent.
    pub resend_every: u32,
    /// Real-time priority to ask the OS for: Linux SCHED_FIFO 1..99; on
    /// macOS any non-zero value asks for the time-constraint policy at this
    /// bus's period and the number is otherwise ignored; Windows
    /// TIME_CRITICAL. Zero asks for none.
    pub priority: u32,
    /// The core to pin the thread to.
    pub cpu: Option<usize>,
}

/// One bus's share of the robot state, published every tick.
///
/// `state` is indexed by joint, like [`LowCmd::cmd`]; only this bus's joints
/// are written. The daemon merges the buses' snapshots into one
/// [`drlabs_core::LowState`].
#[derive(Debug, Clone, Copy)]
pub struct BusState {
    pub tick: u64,
    /// Nanoseconds since the daemon's epoch at the end of the tick.
    pub t_ns: u64,
    /// Ticks that started a whole period late, since start.
    pub deadline_miss: u32,
    /// The worst lateness seen, ns.
    pub late_max_ns: u32,
    /// The command sequence number this tick acted on.
    pub cmd_seq: u32,
    pub state: [MotorState; MAX_AXES],
}

impl BusState {
    pub const EMPTY: Self = Self {
        tick: 0,
        t_ns: 0,
        deadline_miss: 0,
        late_max_ns: 0,
        cmd_seq: 0,
        state: [MotorState::EMPTY; MAX_AXES],
    };
}

/// Stop frames sent after an axis leaves driving, before it is merely
/// observed. One would do for a controller that acts on the first; a
/// dropped frame on a noisy bus should not leave a motor energised.
const STOP_BURST: u32 = 3;

struct Axis {
    spec: AxisSpec,
    guard: Guard,
    gate: AxisGate,
    /// The last readings, merged across query profiles.
    known: AxisReply,
    silent_run: u32,
    /// What the guard should expect the axis to be doing.
    velocity_target: RevPerSec,
    /// The client's mode in the previous tick, for the edges that matter:
    /// a change into `ZERO`.
    last_mode: u8,
    /// Stops still to send after the axis left driving.
    stop_ticks: u32,
    /// Where a hold latched, for a bus that cannot hold natively.
    hold_at: Option<Rev>,
    /// A re-reference waiting for the axis to be stopped, in motor space.
    zero_pending: Option<Rev>,
    /// Whether the bus holds zero velocity natively.
    can_hold: bool,
    /// The acceleration limit's state: where the setpoint actually sent is.
    shaper: Shaper,
    /// Set while the stop policy keeps this axis powered.
    estop_hold: Option<EstopHold>,
    /// This axis's own guard or silence raised a stop since the last reset.
    /// It is the one misbehaving, so it is never the one kept powered.
    self_tripped: bool,
    /// Since when the axis has been further from its setpoint than its
    /// following-error ceiling, without a break.
    following_since: Option<Instant>,
    /// This axis raised a following-error stop. It is not misbehaving the
    /// way a runaway is — it is being overpowered — so it follows its stop
    /// policy like the others, but a hold keeps it where it actually is
    /// rather than where its setpoint had got to.
    hold_where_it_is: bool,
}

impl Axis {
    /// Whether the axis is powered under the runtime's command: driven by a
    /// client, or held by the stop policy. The guard and the comm-loss count
    /// watch both.
    fn energised(&self) -> bool {
        self.gate.driving || self.estop_hold.is_some()
    }

    /// Where the setpoint would come to rest decelerating at the limit from
    /// where it is now, inside the envelope; else where the axis is.
    fn stopping_point(&self) -> Option<Rev> {
        let safety = &self.spec.safety;
        let accel = safety.accel_limit.value();
        // A position setpoint brakes from where it is at its own velocity.
        // A velocity command has no setpoint; the hold's shaper will start
        // from the measured position and velocity, so predict from those.
        let (from, v) = match self.shaper.shaping {
            Shaping::Off => return self.known.position.filter(|p| p.value().is_finite()),
            Shaping::Position => (self.shaper.position, self.shaper.velocity),
            Shaping::Velocity => (
                self.known.position?.value(),
                self.known.velocity.map(|v| v.value()).unwrap_or(0.0),
            ),
        };
        let travel = if accel > 0.0 { v * v.abs() / (2.0 * accel) } else { 0.0 };
        let point = from + travel;
        point
            .is_finite()
            .then(|| safety.clamp_position(Rev(point)).0)
    }

    /// Lets go of a held axis: the stop burst, then off.
    fn release_hold(&mut self) {
        if self.estop_hold.take().is_some() {
            self.stop_ticks = STOP_BURST;
            self.shaper.reset();
        }
    }
}

/// What the shaper is shaping: nothing, a position setpoint, or a velocity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shaping {
    Off,
    Position,
    Velocity,
}

/// The envelope's acceleration limit, applied to the setpoint here rather
/// than left to the controller.
///
/// A client's position target goes through a second-order limiter: the
/// setpoint moves toward it at no more than the velocity limit, its velocity
/// changes by no more than `accel · period` a tick, and it brakes in time to
/// stop on the target. A target that already moves within the limits — a
/// streamed trapezoid — passes through unchanged; a step, a jump or a first
/// command far from where the axis is becomes a ramp. A velocity command is
/// slewed the same way. Units are motor space, like the envelope.
///
/// Why here: the Dynamixel bus writes Profile Acceleration 0 because an
/// XL330 on firmware 53 runs away with any other value while the goal is
/// streamed (docs/DYNAMIXEL.md), so the servo enforces nothing. Doing it on
/// the setpoint makes every bus behave the same whatever its firmware.
///
/// The limit is a magnitude for braking too: a target that stops dead
/// while the setpoint is moving is overshot by `v² / 2a` before the setpoint
/// comes back, inside the position envelope.
#[derive(Debug, Clone, Copy)]
struct Shaper {
    shaping: Shaping,
    /// The setpoint sent last tick, rev.
    position: f64,
    /// Its velocity, rev/s.
    velocity: f64,
    /// The client's target last tick, rev — its change is the target's own
    /// velocity.
    last_target: f64,
}

impl Shaper {
    const OFF: Self = Self {
        shaping: Shaping::Off,
        position: 0.0,
        velocity: 0.0,
        last_target: 0.0,
    };

    fn reset(&mut self) {
        self.shaping = Shaping::Off;
    }

    /// Rewrites `command`'s position or velocity setpoint to obey
    /// `safety.accel_limit`. `client_mode` is the client's mode: a HOLD that
    /// became a position command holds where it latched and is not shaped.
    /// `known` starts the setpoint where the axis is, so the first command
    /// after enabling is ramped from there.
    fn apply(
        &mut self,
        command: &mut Command,
        client_mode: u8,
        safety: &SafetyConfig,
        dt: f64,
        known: &AxisReply,
    ) {
        let accel = safety.accel_limit.value();
        if !(accel > 0.0 && accel.is_finite() && dt > 0.0) {
            self.reset();
            return;
        }
        let vmax = match safety.max_velocity.value() {
            v if v > 0.0 && v.is_finite() => v,
            _ => f64::INFINITY,
        };
        let dv = accel * dt;
        let measured_velocity = known
            .velocity
            .map(|v| v.value())
            .filter(|v| v.is_finite())
            .unwrap_or(0.0)
            .clamp(-vmax, vmax);

        match command.mode {
            Mode::Position if client_mode != mode::HOLD => {
                let Some(target) = command.position.get() else {
                    self.reset();
                    return;
                };
                let mut target = target.value();
                if !target.is_finite() {
                    // Nothing upstream rejects a NaN or an infinity in the
                    // client's record, and `f64::clamp` panics on a NaN
                    // bound: on this thread that is a bus left energised
                    // with nobody watching. A target that is not a number
                    // holds the setpoint where it is.
                    target = if self.shaping == Shaping::Position {
                        self.position
                    } else {
                        match known.position.map(|p| p.value()) {
                            Some(p) if p.is_finite() => p,
                            _ => {
                                // Nowhere known to hold: do not drive it.
                                self.reset();
                                *command = Command::new(Mode::Stop);
                                return;
                            }
                        }
                    };
                }
                if self.shaping != Shaping::Position {
                    let from = known
                        .position
                        .map(|p| p.value())
                        .filter(|p| p.is_finite())
                        .unwrap_or(target);
                    *self = Self {
                        shaping: Shaping::Position,
                        position: from,
                        velocity: measured_velocity,
                        last_target: target,
                    };
                }
                // The target's own velocity, so a moving target is followed
                // rather than chased: with the setpoint on it, `ahead` is 0.
                let target_velocity = ((target - self.last_target) / dt).clamp(-vmax, vmax);
                self.last_target = target;
                let ahead = target - (self.position + target_velocity * dt);
                // The fastest the setpoint may move and still stop on the
                // target, braking by `dv` a tick: from k·dv the distance to
                // rest is dv·dt·k(k+1)/2. The continuous √(2a·e) overshoots
                // by a fraction of a tick's travel.
                let braking =
                    dv * ((0.25 + 2.0 * ahead.abs() / (dv * dt)).sqrt() - 0.5);
                let wanted = (target_velocity + (ahead / dt).clamp(-braking, braking))
                    .clamp(-vmax, vmax);
                self.velocity = wanted.clamp(self.velocity - dv, self.velocity + dv);
                self.position += self.velocity * dt;
                let (clamped, at_edge) = safety.clamp_position(Rev(self.position));
                if at_edge {
                    self.position = clamped.value();
                    self.velocity = 0.0;
                }
                command.position = Field::Set(Rev(self.position));
                command.velocity = Field::Set(RevPerSec(self.velocity));
                command.velocity_target = RevPerSec(self.velocity);
            }
            Mode::Velocity => {
                // A velocity that is not a number is a request to stop.
                let wanted = command
                    .velocity
                    .get()
                    .map(|v| v.value())
                    .filter(|v| v.is_finite())
                    .unwrap_or(0.0);
                if self.shaping != Shaping::Velocity {
                    *self = Self {
                        shaping: Shaping::Velocity,
                        velocity: measured_velocity,
                        ..Self::OFF
                    };
                }
                self.velocity = wanted
                    .clamp(-vmax, vmax)
                    .clamp(self.velocity - dv, self.velocity + dv);
                command.velocity = Field::Set(RevPerSec(self.velocity));
                command.velocity_target = RevPerSec(self.velocity);
            }
            _ => self.reset(),
        }
    }
}

pub struct BusWorker {
    config: WorkerConfig,
    bus: Box<dyn Bus>,
    axes: Vec<Axis>,
    cmds: Vec<AxisCmd>,
    replies: Vec<AxisReply>,
    cmd_rx: Reader<Stamped<LowCmd>>,
    state_tx: Writer<BusState>,
    estop: Estop,
    running: Arc<AtomicBool>,
    state: BusState,
    tick: u64,
    last_cmd_seq: u32,
    reset_requested: bool,
    /// The sequence number of the command that cleared the last stop. A
    /// held axis keeps holding until a later one arrives.
    reset_seq: u32,
    /// The estop's clear count this thread has acted on.
    seen_generation: u32,
    late_run: u32,
    epoch: Instant,
}

impl BusWorker {
    /// Builds the thread's state. Every allocation the thread will ever make
    /// happens here.
    pub fn new(
        config: WorkerConfig,
        bus: Box<dyn Bus>,
        specs: &[AxisSpec],
        cmd_rx: Reader<Stamped<LowCmd>>,
        state_tx: Writer<BusState>,
        estop: Estop,
        running: Arc<AtomicBool>,
    ) -> Self {
        assert_eq!(bus.axes(), specs.len(), "one spec per axis on the bus");
        let axes = specs
            .iter()
            .enumerate()
            .map(|(index, spec)| Axis {
                spec: *spec,
                guard: Guard::new(spec.safety),
                gate: AxisGate::default(),
                known: AxisReply::SILENT,
                silent_run: 0,
                velocity_target: RevPerSec(0.0),
                last_mode: mode::OFF,
                stop_ticks: 0,
                hold_at: None,
                zero_pending: None,
                can_hold: bus.capabilities(index).modes.contains(&ControlMode::Hold),
                shaper: Shaper::OFF,
                estop_hold: None,
                self_tripped: false,
                following_since: None,
                hold_where_it_is: false,
            })
            .collect();
        Self {
            cmds: vec![AxisCmd::default(); specs.len()],
            replies: vec![AxisReply::SILENT; specs.len()],
            config,
            bus,
            axes,
            cmd_rx,
            state_tx,
            estop,
            running,
            state: BusState::EMPTY,
            tick: 0,
            last_cmd_seq: 0,
            reset_requested: false,
            reset_seq: 0,
            seen_generation: 0,
            late_run: 0,
            epoch: Instant::now(),
        }
    }

    /// Opens the bus and runs ticks on the period until `running` goes
    /// false, then closes it. Every bus thread given the same `epoch` wakes
    /// on the same grid.
    pub fn run(mut self, epoch: Instant) {
        self.epoch = epoch;
        match self.bus.open() {
            Ok(identities) => {
                for (axis, identity) in self.axes.iter().zip(identities) {
                    match identity {
                        Some(identity) => eprintln!(
                            "[{}] joint {}: {}{}{}",
                            self.config.name,
                            axis.spec.joint,
                            identity.board,
                            if identity.firmware.is_empty() {
                                String::new()
                            } else {
                                format!(" fw {}", identity.firmware)
                            },
                            if identity.simulated {
                                " (simulated)"
                            } else {
                                ""
                            }
                        ),
                        None => eprintln!(
                            "[{}] joint {}: reports nothing identifying",
                            self.config.name, axis.spec.joint
                        ),
                    }
                }
            }
            Err(error) => {
                eprintln!("[{}] open failed: {error}", self.config.name);
                self.estop.latch(TripCode::Transport);
                return;
            }
        }
        // The driver's own ratings, now that open has identified each servo:
        // they tighten the declared envelope, and widen a voltage band nobody
        // declared so it contains the motor's rated supply. Without this a 5 V
        // or 12 V servo was judged against the default 11–42 V band and
        // tripped under-voltage on its first drive. Same as the session does
        // on connect; allocation here is at open, not on the tick.
        for (index, axis) in self.axes.iter_mut().enumerate() {
            if let Some(envelope) = self.bus.envelope(index) {
                let mut notes = axis.spec.safety.normalise(Some(&envelope));
                notes.extend(axis.spec.safety.fit_voltage_to(&envelope));
                axis.guard = Guard::new(axis.spec.safety);
                for note in notes {
                    eprintln!("[{}] axis {}: {note}", self.config.name, axis.spec.joint);
                }
            }
        }
        let period = self.config.period;
        let granted = drlabs_timing::rt::promote(self.config.priority, self.config.cpu, period);
        eprintln!(
            "[{}] scheduling: {}",
            self.config.name,
            granted.describe(self.config.priority, self.config.cpu)
        );
        let _resolution = Resolution::raise();
        let spin = drlabs_timing::DEFAULT_SPIN.min(period / 4);

        let mut next = epoch;
        let now = Instant::now();
        while next <= now {
            next += period;
        }

        while self.running.load(Ordering::Relaxed) {
            sleep_until(next, None, spin);
            let now = Instant::now();
            let late = now.saturating_duration_since(next);
            self.note_lateness(late);
            next += period;
            if next + period < now {
                // Fallen more than a period behind — a long pause, a laptop
                // resuming from sleep. Resynchronise rather than catching up
                // with a burst of back-to-back ticks.
                next = now + period;
            }
            self.tick(now);
        }

        if let Err(error) = self.bus.close() {
            eprintln!("[{}] close failed: {error}", self.config.name);
        }
    }

    fn note_lateness(&mut self, late: Duration) {
        let late_ns = late.as_nanos().min(u32::MAX as u128) as u32;
        self.state.late_max_ns = self.state.late_max_ns.max(late_ns);
        if late > self.config.period {
            self.state.deadline_miss = self.state.deadline_miss.saturating_add(1);
            self.late_run += 1;
            if self.config.deadline_miss_trip > 0 && self.late_run >= self.config.deadline_miss_trip
            {
                self.estop.latch(TripCode::Deadline);
            }
        } else {
            self.late_run = 0;
        }
    }

    /// One tick. Public so a test can drive it with its own clock.
    pub fn tick(&mut self, now: Instant) {
        self.tick += 1;
        let (stamped, _) = self.cmd_rx.latest();
        let cmd = stamped.value;
        let never = cmd.seq == 0;
        let stale = never || now.saturating_duration_since(stamped.at) > self.config.cmd_timeout;

        // Robot-wide flags act once per command, not once per tick.
        if cmd.seq != self.last_cmd_seq {
            self.last_cmd_seq = cmd.seq;
            if cmd.flags & flags::QUICK_STOP != 0 {
                self.estop.latch(TripCode::Requested);
            }
            if cmd.flags & flags::FAULT_RESET != 0 {
                self.reset_requested = true;
            }
        }
        if self.reset_requested {
            self.reset_requested = false;
            self.estop.clear_by(cmd.seq);
        }
        // Whichever thread cleared it, every thread resets its own axes and
        // learns which command did it. The latch is read before the
        // generation: `clear_by` moves the generation first, so seeing the
        // latch open guarantees seeing the reset behind it.
        let stopped = self.estop.is_latched();
        let (generation, reset_seq) = self.estop.generation();
        if generation != self.seen_generation {
            self.seen_generation = generation;
            self.reset_seq = reset_seq;
            for axis in &mut self.axes {
                axis.gate.release();
                axis.guard.reset();
                axis.silent_run = 0;
                axis.self_tripped = false;
                axis.following_since = None;
                axis.hold_where_it_is = false;
            }
        }
        let trip = if stopped { self.estop.code() } else { None };
        let dt = self.config.period.as_secs_f64();

        let profile =
            if self.config.full_every > 0 && self.tick % self.config.full_every as u64 == 0 {
                QueryProfile::Full
            } else {
                QueryProfile::Fast
            };
        let resend =
            self.config.resend_every > 0 && self.tick % self.config.resend_every as u64 == 0;

        for (slot, axis) in self.cmds.iter_mut().zip(self.axes.iter_mut()) {
            let mut motor = if !never && axis.spec.joint < cmd.n as usize {
                cmd.cmd[axis.spec.joint]
            } else {
                MotorCmd::OFF
            };

            // The edges are the client's own mode, before the stale rewrite:
            // a keep-alive that repeats the record is not a new request.
            if motor.mode != axis.last_mode {
                axis.last_mode = motor.mode;
                if motor.mode == mode::ZERO {
                    axis.zero_pending =
                        Some(axis.spec.mapping.motor_position(Rad(motor.q as f64)));
                }
            }

            if stale {
                match self.config.on_timeout {
                    TimeoutPolicy::Release => motor.mode = mode::OFF,
                    TimeoutPolicy::Hold => {
                        // A position is kept; a velocity or a torque has
                        // nothing to keep, so it becomes a hold.
                        motor.dq = 0.0;
                        motor.tau = 0.0;
                        if motor.mode == mode::VELOCITY || motor.mode == mode::TORQUE {
                            motor.mode = mode::HOLD;
                        }
                    }
                }
            }
            if motor.mode == mode::HOLD {
                // Latched once, where the axis was when the hold began, so a
                // position servo holds a place rather than chasing a reading.
                if axis.hold_at.is_none() {
                    axis.hold_at = axis.known.position;
                }
            } else {
                axis.hold_at = None;
            }

            // The stop policy. An axis being driven when the stop came, under
            // a policy that keeps power and a stop that allows it, is brought
            // to rest and held instead of released.
            if stopped {
                axis.zero_pending = None;
                if axis.estop_hold.is_none() {
                    if axis.gate.driving
                        && axis.spec.on_estop != EstopPolicy::Release
                        && trip.is_some_and(trip_allows_hold)
                        && axis.silent_run == 0
                        && !axis.self_tripped
                    {
                        let target = if axis.hold_where_it_is {
                            // Its setpoint ran ahead of it; hold the place
                            // it reached, starting the shaper from there.
                            axis.shaper.reset();
                            axis.known
                                .position
                                .filter(|p| p.value().is_finite())
                                .map(|p| axis.spec.safety.clamp_position(p).0)
                        } else {
                            axis.stopping_point()
                        };
                        if let Some(target) = target {
                            axis.estop_hold = Some(EstopHold {
                                target,
                                release_when_still: axis.spec.on_estop
                                    == EstopPolicy::StopThenRelease,
                                since: now,
                            });
                        }
                    }
                } else if !trip.is_some_and(trip_allows_hold) {
                    axis.release_hold();
                }
            } else if axis.estop_hold.is_some()
                && stale
                && self.config.on_timeout == TimeoutPolicy::Release
            {
                // Reset, and nobody is talking: the timeout policy decides,
                // as it would for an axis the client was driving.
                axis.release_hold();
            } else if axis.estop_hold.is_some() && cmd.seq != self.reset_seq {
                // The client's first command after the reset decides: drive
                // carries on from the held place, anything else lets go.
                if mode::drives(motor.mode) {
                    axis.estop_hold = None;
                } else {
                    axis.release_hold();
                }
            }
            if axis.estop_hold.is_some() {
                // A held axis that stops answering is let go: when it comes
                // back it is not re-energised by a hold nobody can see.
                let limit = axis.spec.safety.max_consecutive_timeouts;
                if limit > 0 && axis.silent_run >= limit {
                    axis.release_hold();
                }
            }
            let holding = axis.estop_hold.is_some();

            // A held axis waits for the client's next word, so the record
            // that carried the reset does not arm it.
            let want = mode::drives(motor.mode) && !holding;
            let was_driving = axis.gate.driving;
            let drive = axis
                .gate
                .decide(want, &axis.known, &mut axis.guard, stopped);
            if was_driving && !drive {
                axis.stop_ticks = if holding { 0 } else { STOP_BURST };
            }

            if (!drive || stopped) && !holding {
                // Driving again later starts from wherever the axis is then.
                axis.shaper.reset();
            }
            let command = if let Some(hold) = axis.estop_hold {
                let held = MotorCmd {
                    mode: mode::DRIVE,
                    q: axis.spec.mapping.joint_position(hold.target).value() as f32,
                    ..MotorCmd::OFF
                };
                let mut command =
                    drive_command(&axis.spec, &held, self.config.period, resend, None);
                axis.shaper
                    .apply(&mut command, mode::DRIVE, &axis.spec.safety, dt, &axis.known);
                if hold.release_when_still {
                    let at_rest = match axis.shaper.shaping {
                        Shaping::Position => {
                            axis.shaper.velocity.abs() <= 1e-6
                                && (axis.shaper.position - hold.target.value()).abs() <= 1e-5
                        }
                        // Nothing shapes the stop, so the target is already
                        // the command: as still as this axis will be made.
                        _ => true,
                    };
                    if at_rest
                        || now.saturating_duration_since(hold.since) >= STOP_THEN_RELEASE_LIMIT
                    {
                        // This tick still holds; the burst starts on the next.
                        axis.release_hold();
                    }
                }
                command
            } else if stopped {
                // A stop cancels what was pending; the client asks again.
                axis.zero_pending = None;
                Command::new(Mode::Stop)
            } else if drive {
                let hold_at = if axis.can_hold { None } else { axis.hold_at };
                let mut command =
                    drive_command(&axis.spec, &motor, self.config.period, resend, hold_at);
                axis.shaper
                    .apply(&mut command, motor.mode, &axis.spec.safety, dt, &axis.known);
                command
            } else if axis.stop_ticks > 0 {
                axis.stop_ticks -= 1;
                Command::new(Mode::Stop)
            } else if let Some(position) = axis.zero_pending.take() {
                Command::new(Mode::SetOutput { position })
            } else {
                Command::new(Mode::Query)
            };
            axis.velocity_target = command.velocity_target;
            *slot = AxisCmd { command, profile };
        }

        if self.bus.cycle(&self.cmds, &mut self.replies).is_err() {
            self.estop.latch(TripCode::Transport);
            self.replies.fill(AxisReply::SILENT);
            // A link that failed cannot carry a hold, and must not resume
            // one on its own when it comes back.
            for axis in &mut self.axes {
                axis.release_hold();
            }
        }

        for ((axis, reply), sent) in self
            .axes
            .iter_mut()
            .zip(self.replies.iter())
            .zip(self.cmds.iter())
        {
            axis.known.merge(reply);
            if reply.status.answered() {
                axis.silent_run = 0;
            } else {
                axis.silent_run = axis.silent_run.saturating_add(1);
                let limit = axis.spec.safety.max_consecutive_timeouts;
                if axis.energised() && limit > 0 && axis.silent_run >= limit {
                    self.estop.latch(TripCode::CommLoss);
                    axis.self_tripped = true;
                    axis.release_hold();
                }
            }
            if let Some(code) =
                axis.guard
                    .evaluate(&axis.known, axis.velocity_target, axis.energised(), now)
            {
                self.estop.latch(code);
                // The axis whose own guard tripped is the one misbehaving —
                // running away, out of its travel, overheating — whatever
                // the code: holding it would keep feeding the fault. Others
                // may hold.
                axis.self_tripped = true;
                axis.release_hold();
            }

            // Following error: a client-driven position setpoint the axis
            // has stayed away from. Only while the client drives it — a hold
            // the stop policy is keeping is not let go for sagging, which
            // would only let it fall further.
            // A tick it did not answer is comm loss's to judge, not this:
            // the merged position is then an old one.
            let error = match (axis.spec.following_error, sent.command.mode) {
                (Some(limit), Mode::Position)
                    if axis.gate.driving && !stopped && reply.status.answered() =>
                {
                    sent.command
                        .position
                        .get()
                        .zip(axis.known.position)
                        .map(|(setpoint, at)| (setpoint.value() - at.value()).abs())
                        .filter(|e| e.is_finite())
                        .map(|e| e > limit)
                }
                _ => None,
            };
            match error {
                Some(true) => {
                    let since = *axis.following_since.get_or_insert(now);
                    if now.saturating_duration_since(since) >= FOLLOWING_ERROR_TIME {
                        self.estop.latch(TripCode::FollowingError);
                        // Overpowered, not misbehaving: its stop policy
                        // decides, and a hold keeps the place it reached.
                        axis.hold_where_it_is = true;
                    }
                }
                _ => axis.following_since = None,
            }
            self.state.state[axis.spec.joint] = motor_state(axis, self.estop.is_latched());
        }

        self.state.tick = self.tick;
        self.state.t_ns = now.saturating_duration_since(self.epoch).as_nanos() as u64;
        self.state.cmd_seq = cmd.seq;
        self.state_tx.publish(self.state);
    }

    /// The gate state of every axis, in spec order. For tests.
    pub fn driving(&self) -> Vec<bool> {
        self.axes.iter().map(|axis| axis.gate.driving).collect()
    }
}

/// The motor-space command for a driven axis.
///
/// What each field of the client's command becomes, by mode:
///
/// | mode | motor-space command |
/// |---|---|
/// | `DRIVE` | `position` = `q`, `velocity` = `dq` (feed-forward), `feedforward_torque` = `tau` |
/// | `VELOCITY` | `velocity` = `dq`, no position target, `feedforward_torque` = `tau`; `stop_position` = the envelope edge ahead |
/// | `TORQUE` | `feedforward_torque` = `tau`, no position or velocity opinion |
/// | `HOLD` | `Hold` (zero velocity); on a bus without it, `position` = where the hold began, `velocity` 0 |
/// | `STAY_WITHIN` | `StayWithin` at the envelope's bounds, `feedforward_torque` = `tau` |
/// | `kp`, `kd` | not applied in this version: the controller's own gains stand |
///
/// Every value goes through the mapping and is clamped to the envelope. The
/// torque ceiling, velocity limit, acceleration limit and watchdog come from
/// the safety envelope, every tick, whatever the mode.
fn drive_command(
    spec: &AxisSpec,
    motor: &MotorCmd,
    period: Duration,
    resend: bool,
    hold_at: Option<Rev>,
) -> Command {
    let mapping = &spec.mapping;
    let safety = &spec.safety;

    let (position, _) = safety.clamp_position(mapping.motor_position(Rad(motor.q as f64)));
    let (velocity, _) = safety.clamp_velocity(mapping.motor_velocity(RadPerSec(motor.dq as f64)));
    let feedforward = mapping
        .motor_torque(Nm(motor.tau as f64))
        .clamp_to(-safety.max_torque, safety.max_torque);

    let limits = Command {
        max_torque: TorqueLimit::Nm(safety.max_torque),
        watchdog: Field::Set(safety.watchdog_timeout(period.as_secs_f64())),
        velocity_limit: Field::Set(safety.max_velocity),
        accel_limit: Field::Set(safety.accel_limit),
        full: resend,
        ..Command::default()
    };

    match motor.mode {
        mode::VELOCITY => Command {
            mode: Mode::Velocity,
            velocity: Field::Set(velocity),
            feedforward_torque: Field::Set(feedforward),
            // Run to the edge of the travel and no further, on a controller
            // that honours a stop position; the guard's envelope trip is the
            // layer behind it.
            stop_position: Field::Set(if velocity.value() < 0.0 {
                safety.position_min
            } else {
                safety.position_max
            }),
            velocity_target: velocity,
            ..limits
        },
        mode::TORQUE => Command {
            mode: Mode::Torque,
            feedforward_torque: Field::Set(feedforward),
            velocity_target: RevPerSec(0.0),
            ..limits
        },
        mode::HOLD => match hold_at {
            Some(at) => Command {
                mode: Mode::Position,
                position: Field::Set(safety.clamp_position(at).0),
                velocity: Field::Set(RevPerSec(0.0)),
                feedforward_torque: Field::Set(Nm(0.0)),
                velocity_target: RevPerSec(0.0),
                ..limits
            },
            None => Command {
                mode: Mode::Hold,
                velocity_target: RevPerSec(0.0),
                ..limits
            },
        },
        mode::STAY_WITHIN => Command {
            mode: Mode::StayWithin {
                lower: safety.position_min,
                upper: safety.position_max,
            },
            feedforward_torque: Field::Set(feedforward),
            // The bounds are the point of this mode, so they go out every
            // cycle rather than only on a parameter refresh.
            full: true,
            velocity_target: RevPerSec(0.0),
            ..limits
        },
        _ => Command {
            mode: Mode::Position,
            position: Field::Set(position),
            velocity: Field::Set(velocity),
            feedforward_torque: Field::Set(feedforward),
            velocity_target: velocity,
            ..limits
        },
    }
}

fn f32_or_nan(value: Option<f64>) -> f32 {
    value.map(|v| v as f32).unwrap_or(f32::NAN)
}

/// The joint-space record for one axis.
fn motor_state(axis: &Axis, stopped: bool) -> MotorState {
    let mapping = &axis.spec.mapping;
    let known = &axis.known;
    // A held axis says so before anything else, stop or no stop: it is
    // powered, and "tripped" reads as limp to whoever is standing next to
    // it. The robot-wide stop is in the record's own `estop` field.
    let status = if axis.estop_hold.is_some() {
        AxisStatus::Held
    } else if stopped {
        AxisStatus::Tripped
    } else if !known.status.answered() {
        AxisStatus::Silent
    } else if !axis.gate.driving {
        AxisStatus::Off
    } else {
        known.status
    };
    MotorState {
        q: f32_or_nan(known.position.map(|p| mapping.joint_position(p).value())),
        dq: f32_or_nan(known.velocity.map(|v| mapping.joint_velocity(v).value())),
        tau: f32_or_nan(known.torque.map(|t| mapping.joint_torque(t).value())),
        current: f32_or_nan(known.current.map(|c| c.value())),
        voltage: f32_or_nan(known.voltage.map(|v| v.value())),
        temp: f32_or_nan(known.hottest().map(|t| t.value())),
        fault: known
            .fault
            .map(|f| f.code.clamp(0, u16::MAX as i32) as u16)
            .unwrap_or(0),
        status: status as u8,
        blocked: axis.gate.blocked as u8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exchange::triple;
    use drlabs_core::units::{RadPerSec2, Rev, RevPerSec2};
    use drlabs_safety::Blocked;
    use drlabs_sim::SimBus;

    fn spec(joint: usize) -> AxisSpec {
        let mapping = Mapping {
            gear_ratio: 1.0,
            direction: 1.0,
            offset_rad: 0.0,
        };
        let limits = crate::config::JointLimits {
            min_rad: -1.5,
            max_rad: 1.5,
            max_torque_nm: 1.0,
            max_velocity_rad_s: 6.0,
            max_accel_rad_s2: 30.0,
            bus_voltage_min: None,
            bus_voltage_max: None,
            max_following_error_rad: None,
        };
        AxisSpec {
            joint,
            mapping,
            safety: mapping.safety_config(&limits, SafetyConfig::default()),
            on_estop: EstopPolicy::Release,
            following_error: None,
        }
    }

    fn worker(
        axes: usize,
        on_timeout: TimeoutPolicy,
    ) -> (BusWorker, Writer<Stamped<LowCmd>>, Reader<BusState>, Estop) {
        let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
        let (state_tx, state_rx) = triple(BusState::EMPTY);
        let estop = Estop::new();
        let specs: Vec<AxisSpec> = (0..axes).map(spec).collect();
        let mut bus = SimBus::new(axes).fixed_step(0.002);
        bus.open().unwrap();
        let worker = BusWorker::new(
            WorkerConfig {
                name: "test".into(),
                period: Duration::from_millis(2),
                cmd_timeout: Duration::from_millis(100),
                on_timeout,
                deadline_miss_trip: 5,
                full_every: 10,
                resend_every: 50,
                priority: 0,
                cpu: None,
            },
            Box::new(bus),
            &specs,
            cmd_rx,
            state_tx,
            estop.clone(),
            Arc::new(AtomicBool::new(true)),
        );
        (worker, cmd_tx, state_rx, estop)
    }

    fn drive_all(n: usize, q: f32, seq: u32) -> LowCmd {
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = seq;
        cmd.n = n as u16;
        for motor in cmd.cmd.iter_mut().take(n) {
            motor.mode = mode::DRIVE;
            motor.q = q;
        }
        cmd
    }

    /// Nothing moves until a client asks, and then every axis goes where it
    /// was sent.
    #[test]
    fn axes_are_off_until_commanded_and_then_follow() {
        let (mut worker, mut cmd_tx, mut state_rx, _) = worker(3, TimeoutPolicy::Hold);
        let start = Instant::now();

        worker.tick(start);
        let (state, _) = state_rx.latest();
        assert!(state.state[..3]
            .iter()
            .all(|s| s.status == AxisStatus::Off as u8));
        assert_eq!(worker.driving(), vec![false; 3]);

        cmd_tx.publish(Stamped {
            value: drive_all(3, 0.5, 1),
            at: start,
        });
        for i in 1..1500u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        let (state, _) = state_rx.latest();
        assert_eq!(worker.driving(), vec![true; 3]);
        for s in &state.state[..3] {
            assert!((s.q - 0.5).abs() < 0.05, "{s:?}");
            assert_eq!(s.status, AxisStatus::Ok as u8);
            assert_eq!(s.blocked, 0);
        }
        assert_eq!(state.cmd_seq, 1);
    }

    /// A command that stops arriving: with `release`, the axes go off; with
    /// `hold`, they stay where they were sent.
    #[test]
    fn a_stale_command_is_handled_by_the_policy() {
        for (policy, expect_driving) in
            [(TimeoutPolicy::Release, false), (TimeoutPolicy::Hold, true)]
        {
            let (mut worker, mut cmd_tx, _, _) = worker(2, policy);
            let start = Instant::now();
            cmd_tx.publish(Stamped {
                value: drive_all(2, 0.1, 1),
                at: start,
            });
            // Fresh for the first 100 ms …
            for i in 1..40u64 {
                worker.tick(start + Duration::from_millis(2 * i));
            }
            assert_eq!(worker.driving(), vec![true; 2]);
            // … and stale after.
            worker.tick(start + Duration::from_millis(200));
            assert_eq!(worker.driving(), vec![expect_driving; 2], "{policy:?}");
        }
    }

    /// A client's e-stop stops every axis, and only a client's reset
    /// releases it — to off, not back to driving.
    #[test]
    fn an_estop_flag_latches_and_a_reset_releases_to_off() {
        let (mut worker, mut cmd_tx, mut state_rx, estop) = worker(2, TimeoutPolicy::Hold);
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: drive_all(2, 0.0, 1),
            at: start,
        });
        worker.tick(start);
        worker.tick(start + Duration::from_millis(2));
        assert_eq!(worker.driving(), vec![true; 2]);

        let mut stop = drive_all(2, 0.0, 2);
        stop.flags = flags::QUICK_STOP;
        cmd_tx.publish(Stamped {
            value: stop,
            at: start,
        });
        worker.tick(start + Duration::from_millis(4));
        assert_eq!(estop.code(), Some(TripCode::Requested));
        assert_eq!(worker.driving(), vec![false; 2]);
        let (state, _) = state_rx.latest();
        assert_eq!(state.state[0].status, AxisStatus::Tripped as u8);
        assert_eq!(state.state[0].blocked, Blocked::Estop as u8);

        // Still asking to drive, with the stop still latched: no.
        cmd_tx.publish(Stamped {
            value: drive_all(2, 0.0, 3),
            at: start,
        });
        worker.tick(start + Duration::from_millis(6));
        assert_eq!(worker.driving(), vec![false; 2]);

        // Reset, then ask again: the axes come back through the preflight.
        let mut reset = drive_all(2, 0.0, 4);
        reset.flags = flags::FAULT_RESET;
        cmd_tx.publish(Stamped {
            value: reset,
            at: start,
        });
        worker.tick(start + Duration::from_millis(8));
        assert_eq!(estop.code(), None);
        worker.tick(start + Duration::from_millis(10));
        assert_eq!(worker.driving(), vec![true; 2]);
    }

    /// A joint is an index into the client's record. A bus that carries
    /// joints 3 and 7 reads and writes exactly those.
    #[test]
    fn a_bus_touches_only_its_own_joints() {
        let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
        let (state_tx, mut state_rx) = triple(BusState::EMPTY);
        let mut cmd_tx = cmd_tx;
        let mut bus = SimBus::new(2).fixed_step(0.002);
        bus.open().unwrap();
        let mut worker = BusWorker::new(
            WorkerConfig {
                name: "b".into(),
                period: Duration::from_millis(2),
                cmd_timeout: Duration::from_secs(1),
                on_timeout: TimeoutPolicy::Hold,
                deadline_miss_trip: 0,
                full_every: 0,
                resend_every: 0,
                priority: 0,
                cpu: None,
            },
            Box::new(bus),
            &[spec(3), spec(7)],
            cmd_rx,
            state_tx,
            Estop::new(),
            Arc::new(AtomicBool::new(true)),
        );
        let start = Instant::now();
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = 1;
        cmd.n = 8;
        cmd.cmd[7].mode = mode::DRIVE;
        cmd.cmd[7].q = 0.3;
        cmd_tx.publish(Stamped {
            value: cmd,
            at: start,
        });
        for i in 0..1000u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        let (state, _) = state_rx.latest();
        assert_eq!(state.state[3].status, AxisStatus::Off as u8);
        assert_eq!(state.state[7].status, AxisStatus::Ok as u8);
        assert!((state.state[7].q - 0.3).abs() < 0.05);
        assert!(
            state.state[0].q.is_nan(),
            "a joint not on this bus is untouched"
        );
    }

    /// The envelope is applied in motor space, every tick, whatever the
    /// client sends.
    #[test]
    fn a_target_outside_the_envelope_is_clamped_to_it() {
        let s = spec(0);
        let motor = MotorCmd {
            mode: mode::DRIVE,
            q: 9.0,
            dq: 100.0,
            tau: 50.0,
            ..MotorCmd::OFF
        };
        let command = drive_command(&s, &motor, Duration::from_millis(2), false, None);
        let max = s.safety.position_max;
        assert_eq!(command.position.get(), Some(max));
        assert_eq!(command.velocity.get(), Some(s.safety.max_velocity));
        assert_eq!(command.feedforward_torque.get(), Some(s.safety.max_torque));
        assert_eq!(command.max_torque, TorqueLimit::Nm(s.safety.max_torque));
        assert!(max < Rev(0.3), "±1.5 rad is ±0.24 rev");
        let _ = RadPerSec2(0.0);
    }

    /// Runs a shaper over `targets`, one per tick, and returns the setpoints.
    fn shape(safety: &SafetyConfig, dt: f64, targets: &[f64]) -> Vec<(f64, f64)> {
        let mut shaper = Shaper::OFF;
        let known = AxisReply {
            position: Some(Rev(targets[0])),
            velocity: Some(RevPerSec(0.0)),
            ..AxisReply::SILENT
        };
        targets
            .iter()
            .map(|&t| {
                let mut command = Command {
                    mode: Mode::Position,
                    position: Field::Set(Rev(t)),
                    velocity: Field::Set(RevPerSec(0.0)),
                    ..Command::default()
                };
                shaper.apply(&mut command, mode::DRIVE, safety, dt, &known);
                (
                    command.position.get().unwrap().value(),
                    command.velocity.get().unwrap().value(),
                )
            })
            .collect()
    }

    /// A step in the target becomes a ramp: the setpoint's velocity never
    /// changes by more than the limit allows in a tick, never exceeds the
    /// velocity limit, and the setpoint settles on the target without
    /// passing it.
    #[test]
    fn a_step_is_ramped_within_the_acceleration_limit() {
        let safety = spec(0).safety;
        let (a, vmax, dt) = (safety.accel_limit.value(), safety.max_velocity.value(), 0.002);
        let target = 0.2; // rev, inside the ±0.2387 rev envelope
        let mut targets = vec![0.0];
        targets.extend(std::iter::repeat(target).take(1000));
        let out = shape(&safety, dt, &targets);

        let mut last_v = 0.0;
        for (i, &(p, v)) in out.iter().enumerate() {
            assert!((v - last_v).abs() <= a * dt + 1e-9, "tick {i}: dv {}", v - last_v);
            assert!(v.abs() <= vmax + 1e-9, "tick {i}: v {v}");
            // A velocity off the dv grid lands within a few µrev of the
            // target; one Dynamixel count is 244 µrev.
            assert!(p <= target + 1e-5, "tick {i}: overshoot to {p}");
            last_v = v;
        }
        let (p, v) = *out.last().unwrap();
        assert!((p - target).abs() < 1e-6 && v.abs() < 1e-6, "settled at {p} moving {v}");
        // And it did not jump: the first move is one tick of the limit.
        assert!(out[1].0 - out[0].0 <= a * dt * dt + 1e-9);
    }

    /// A target that is already a trapezoid within the limits is passed
    /// through: the shaper adds no lag to a client that plans its moves.
    #[test]
    fn a_feasible_trapezoid_passes_through() {
        use drlabs_core::trajectory::Trapezoidal;
        let safety = spec(0).safety;
        let dt = 0.002;
        let profile = Trapezoidal::new(
            Rev(-0.2),
            Rev(0.2),
            RevPerSec(safety.max_velocity.value() * 0.9),
            RevPerSec2(safety.accel_limit.value() * 0.9),
        )
        .unwrap();
        let ticks = (profile.total_time() / dt) as usize + 50;
        let targets: Vec<f64> = (0..ticks)
            .map(|i| profile.sample(i as f64 * dt).position.value())
            .collect();
        let out = shape(&safety, dt, &targets);
        for (i, (&t, &(p, _))) in targets.iter().zip(&out).enumerate() {
            assert!((p - t).abs() < 1e-6, "tick {i}: setpoint {p} target {t}");
        }
    }

    /// No limit, no shaping: the client's target goes out as sent.
    #[test]
    fn a_zero_acceleration_limit_shapes_nothing() {
        let mut safety = spec(0).safety;
        safety.accel_limit = RevPerSec2(0.0);
        let out = shape(&safety, 0.002, &[0.0, 0.2, 0.2]);
        assert_eq!(out[1].0, 0.2);
    }

    /// Through the worker: a client that jumps the target sends the bus a
    /// ramp, and the first command after enabling starts where the axis is.
    #[test]
    fn the_worker_ramps_a_jump_it_is_sent() {
        let (mut worker, mut cmd_tx, _, _, log) =
            recording(1, true, Rev(0.0), TimeoutPolicy::Hold);
        let safety = spec(0).safety;
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
            at: start,
        });
        for i in 0..600u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        let positions: Vec<f64> = log
            .lock()
            .unwrap()
            .iter()
            .filter_map(|cmds| cmds[0].position.get().map(|p| p.value()))
            .collect();
        assert!(positions.len() > 100, "{} position commands", positions.len());
        assert!(positions[0].abs() < 1e-3, "first setpoint {} is not a jump", positions[0]);
        let a = safety.accel_limit.value();
        let dt = 0.002;
        for w in positions.windows(3) {
            let accel = (w[2] - 2.0 * w[1] + w[0]) / (dt * dt);
            assert!(accel.abs() <= a * 1.0001, "{accel} rev/s² over {a}");
        }
        let target = Mapping {
            gear_ratio: 1.0,
            direction: 1.0,
            offset_rad: 0.0,
        }
        .motor_position(Rad(1.2))
        .value();
        assert!((positions.last().unwrap() - target).abs() < 1e-6);
    }

    /// A NaN or an infinity from a client must not panic the bus thread: a
    /// position that is not a number holds the setpoint, a velocity that is
    /// not a number brakes to zero. `f64::clamp` panics on a NaN bound.
    #[test]
    fn a_non_finite_command_holds_instead_of_panicking() {
        let mut safety = spec(0).safety;
        let dt = 0.002;
        let known = AxisReply {
            position: Some(Rev(0.1)),
            velocity: Some(RevPerSec(0.0)),
            ..AxisReply::SILENT
        };
        for limited in [true, false] {
            safety.position_limit_enabled = limited;
            let mut shaper = Shaper::OFF;
            for t in [0.15, f64::NAN, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
                let mut command = Command {
                    mode: Mode::Position,
                    position: Field::Set(Rev(t)),
                    velocity: Field::Set(RevPerSec(0.0)),
                    ..Command::default()
                };
                shaper.apply(&mut command, mode::DRIVE, &safety, dt, &known);
                let p = command.position.get().unwrap().value();
                assert!(p.is_finite() && (p - 0.1).abs() < 0.01, "setpoint {p} after {t}");
            }
            // First command not a number, nothing shaped yet: holds where
            // the axis is.
            let mut shaper = Shaper::OFF;
            let mut command = Command {
                mode: Mode::Position,
                position: Field::Set(Rev(f64::NAN)),
                ..Command::default()
            };
            shaper.apply(&mut command, mode::DRIVE, &safety, dt, &known);
            assert_eq!(command.position.get(), Some(Rev(0.1)));
        }

        let mut shaper = Shaper::OFF;
        for v in [0.3, f64::NAN, f64::NAN, f64::INFINITY, f64::NAN] {
            let mut command = Command {
                mode: Mode::Velocity,
                velocity: Field::Set(RevPerSec(v)),
                ..Command::default()
            };
            shaper.apply(&mut command, mode::VELOCITY, &safety, dt, &known);
            assert!(command.velocity.get().unwrap().value().is_finite());
        }

        // And with no position to hold at all, the axis is stopped.
        let mut shaper = Shaper::OFF;
        let mut command = Command {
            mode: Mode::Position,
            position: Field::Set(Rev(f64::NAN)),
            ..Command::default()
        };
        shaper.apply(&mut command, mode::DRIVE, &safety, dt, &AxisReply::SILENT);
        assert_eq!(command.mode, Mode::Stop);
    }

    /// A velocity command is slewed, not stepped.
    #[test]
    fn a_velocity_step_is_slewed() {
        let safety = spec(0).safety;
        let dt = 0.002;
        let mut shaper = Shaper::OFF;
        let known = AxisReply {
            velocity: Some(RevPerSec(0.0)),
            ..AxisReply::SILENT
        };
        let mut last = 0.0;
        for _ in 0..500 {
            let mut command = Command {
                mode: Mode::Velocity,
                velocity: Field::Set(RevPerSec(0.5)),
                ..Command::default()
            };
            shaper.apply(&mut command, mode::VELOCITY, &safety, dt, &known);
            let v = command.velocity.get().unwrap().value();
            assert!((v - last).abs() <= safety.accel_limit.value() * dt + 1e-9);
            last = v;
        }
        assert!((last - 0.5).abs() < 1e-9);
    }

    /// Drives one axis toward 1.2 rad from rest and stops it mid-move with
    /// `code`; returns the worker, its command writer, the estop, the log,
    /// the clock and the index of the first logged tick after the stop.
    fn stopped_mid_move(
        on_estop: EstopPolicy,
        code: TripCode,
    ) -> (BusWorker, Writer<Stamped<LowCmd>>, Estop, Log, Instant, usize) {
        let (mut worker, mut cmd_tx, _, estop, log) =
            recording_with(1, false, Rev(0.0), TimeoutPolicy::Hold, on_estop);
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
            at: start,
        });
        // Long enough to be moving fast, not long enough to arrive.
        for i in 0..60u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert!(worker.axes[0].shaper.velocity.abs() > 0.05, "moving when stopped");
        estop.latch(code);
        let first = log.lock().unwrap().len();
        (worker, cmd_tx, estop, log, start + Duration::from_millis(120), first)
    }

    fn positions(log: &Log, from: usize) -> Vec<Option<f64>> {
        log.lock().unwrap()[from..]
            .iter()
            .map(|cmds| match cmds[0].mode {
                Mode::Position => cmds[0].position.get().map(|p| p.value()),
                _ => None,
            })
            .collect()
    }

    /// `release`, the default, is what it always was: torque off on the
    /// first tick of the stop.
    #[test]
    fn release_lets_go_at_once() {
        let (mut worker, _, _, log, t, first) =
            stopped_mid_move(EstopPolicy::Release, TripCode::Requested);
        worker.tick(t);
        assert_eq!(log.lock().unwrap()[first][0].mode, Mode::Stop);
    }

    /// `hold` decelerates within the acceleration limit to the stopping
    /// point and stays there, powered, without passing it.
    #[test]
    fn hold_brakes_within_the_limit_and_keeps_the_place() {
        let (mut worker, _, _, log, t, first) =
            stopped_mid_move(EstopPolicy::Hold, TripCode::Requested);
        let target = worker.axes[0].stopping_point().unwrap().value();
        for i in 0..400u64 {
            worker.tick(t + Duration::from_millis(2 * i));
        }
        let held: Vec<f64> = positions(&log, first)
            .into_iter()
            .map(|p| p.expect("every tick of a hold is a position command"))
            .collect();
        let a = spec(0).safety.accel_limit.value();
        let dt = 0.002;
        for w in held.windows(3) {
            let accel = (w[2] - 2.0 * w[1] + w[0]) / (dt * dt);
            assert!(accel.abs() <= a * 1.0001, "{accel} over {a}");
        }
        assert!(held.iter().all(|&p| p <= target + 1e-5), "passed the stopping point");
        assert!((held.last().unwrap() - target).abs() < 1e-5);
        assert!(worker.axes[0].energised());
    }

    /// A fault that holding makes worse releases even under `hold`.
    #[test]
    fn hold_releases_on_a_motor_fault() {
        let (mut worker, _, _, log, t, first) =
            stopped_mid_move(EstopPolicy::Hold, TripCode::OverCurrent);
        worker.tick(t);
        assert_eq!(log.lock().unwrap()[first][0].mode, Mode::Stop);
        assert!(!worker.axes[0].energised());
    }

    /// `stop_then_release` comes to rest, then lets go with the stop burst.
    #[test]
    fn stop_then_release_rests_then_lets_go() {
        let (mut worker, _, _, log, t, first) =
            stopped_mid_move(EstopPolicy::StopThenRelease, TripCode::Requested);
        for i in 0..400u64 {
            worker.tick(t + Duration::from_millis(2 * i));
        }
        let modes: Vec<Mode> = log.lock().unwrap()[first..].iter().map(|c| c[0].mode).collect();
        let braking = modes.iter().take_while(|m| **m == Mode::Position).count();
        assert!(braking > 10, "it braked for {braking} ticks");
        assert!(modes[braking..].iter().all(|m| *m == Mode::Stop), "{:?}", &modes[braking..]);
        assert!(!worker.axes[0].energised());
    }

    /// After a reset a held axis keeps its place until the client's next
    /// command: drive carries on without letting go, off releases.
    #[test]
    fn a_reset_keeps_the_hold_until_the_client_speaks() {
        for (next_mode, expect_release) in [(mode::DRIVE, false), (mode::OFF, true)] {
            let (mut worker, mut cmd_tx, estop, log, t, _) =
                stopped_mid_move(EstopPolicy::Hold, TripCode::Requested);
            for i in 0..300u64 {
                worker.tick(t + Duration::from_millis(2 * i));
            }
            let held_at = worker.axes[0].estop_hold.unwrap().target.value();

            // The reset, carrying the client's old drive record.
            let mut reset = one(mode::DRIVE, 1.2, 0.0, 0.0, 2);
            reset.flags = flags::FAULT_RESET;
            let t = t + Duration::from_millis(600);
            cmd_tx.publish(Stamped { value: reset, at: t });
            let from = log.lock().unwrap().len();
            for i in 0..50u64 {
                worker.tick(t + Duration::from_millis(2 * i));
            }
            assert_eq!(estop.code(), None);
            for p in positions(&log, from) {
                let p = p.expect("still a position command after the reset");
                assert!((p - held_at).abs() < 1e-5, "held, not driven to the old target: {p}");
            }

            // The client's next command.
            let t = t + Duration::from_millis(100);
            cmd_tx.publish(Stamped {
                value: one(next_mode, 1.2, 0.0, 0.0, 3),
                at: t,
            });
            let from = log.lock().unwrap().len();
            for i in 0..5u64 {
                worker.tick(t + Duration::from_millis(2 * i));
            }
            let modes: Vec<Mode> =
                log.lock().unwrap()[from..].iter().map(|c| c[0].mode).collect();
            if expect_release {
                assert_eq!(modes[0], Mode::Stop, "{modes:?}");
                assert!(!worker.axes[0].energised());
            } else {
                assert!(modes.iter().all(|m| *m == Mode::Position), "{modes:?}");
                assert!(worker.axes[0].gate.driving);
            }
        }
    }

    /// Two bus threads share one stop. A reset that one of them clears is a
    /// reset for both: neither drives a held axis on the record that
    /// carried it.
    #[test]
    fn a_reset_is_the_same_reset_on_every_bus() {
        let estop = Estop::new();
        let mut sender = crate::exchange::Broadcast::new();
        let mut workers = Vec::new();
        let mut logs = Vec::new();
        for joint in 0..2 {
            let log: Log = Arc::new(std::sync::Mutex::new(Vec::new()));
            let mut bus = Recording {
                inner: SimBus::new(1).fixed_step(0.002),
                log: Arc::clone(&log),
                holds: false,
            };
            bus.open().unwrap();
            let (state_tx, _) = triple(BusState::EMPTY);
            workers.push(BusWorker::new(
                WorkerConfig {
                    name: format!("b{joint}"),
                    period: Duration::from_millis(2),
                    cmd_timeout: Duration::from_millis(100),
                    on_timeout: TimeoutPolicy::Hold,
                    deadline_miss_trip: 0,
                    full_every: 10,
                    resend_every: 50,
                    priority: 0,
                    cpu: None,
                },
                Box::new(bus),
                &[AxisSpec {
                    on_estop: EstopPolicy::Hold,
                    ..spec(joint)
                }],
                sender.subscribe(Stamped::now(LowCmd::EMPTY)),
                state_tx,
                estop.clone(),
                Arc::new(AtomicBool::new(true)),
            ));
            logs.push(log);
        }
        let drive = |seq: u32, flags_: u32| {
            let mut cmd = LowCmd::EMPTY;
            cmd.seq = seq;
            cmd.n = 2;
            cmd.flags = flags_;
            for motor in cmd.cmd.iter_mut().take(2) {
                motor.mode = mode::DRIVE;
                motor.q = 1.2;
            }
            cmd
        };
        let start = Instant::now();
        let mut t = start;
        let step = |workers: &mut Vec<BusWorker>, t: &mut Instant, n: u64| {
            for _ in 0..n {
                *t += Duration::from_millis(2);
                for w in workers.iter_mut() {
                    w.tick(*t);
                }
            }
        };
        sender.publish(Stamped { value: drive(1, 0), at: t });
        step(&mut workers, &mut t, 60);
        sender.publish(Stamped { value: drive(2, flags::QUICK_STOP), at: t });
        step(&mut workers, &mut t, 300);
        let held: Vec<f64> = workers
            .iter()
            .map(|w| w.axes[0].estop_hold.expect("both held").target.value())
            .collect();

        // The reset rides on the old drive record; worker 0 runs first and
        // clears it, worker 1 must still see it as this reset.
        sender.publish(Stamped {
            value: drive(3, flags::FAULT_RESET),
            at: t,
        });
        let from: Vec<usize> = logs.iter().map(|l| l.lock().unwrap().len()).collect();
        step(&mut workers, &mut t, 50);
        assert_eq!(estop.code(), None);
        for (k, log) in logs.iter().enumerate() {
            for p in positions(log, from[k]) {
                let p = p.expect("a position command");
                assert!((p - held[k]).abs() < 1e-5, "bus {k} moved off its hold to {p}");
            }
            assert_eq!(workers[k].reset_seq, 3, "bus {k} knows which command reset it");
        }
    }

    /// The axis whose own guard raised the stop is not held, whatever the
    /// code; the others are.
    #[test]
    fn the_axis_that_tripped_is_not_the_one_held() {
        let (mut worker, mut cmd_tx, _, estop, log) =
            recording_with(2, false, Rev(0.0), TimeoutPolicy::Hold, EstopPolicy::Hold);
        let start = Instant::now();
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = 1;
        cmd.n = 2;
        for motor in cmd.cmd.iter_mut().take(2) {
            motor.mode = mode::DRIVE;
            motor.q = 1.2;
        }
        cmd_tx.publish(Stamped { value: cmd, at: start });
        for i in 0..60u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        // Axis 0 is the culprit, as its guard would have said.
        worker.axes[0].self_tripped = true;
        estop.latch(TripCode::VelocityRunaway);
        let from = log.lock().unwrap().len();
        worker.tick(start + Duration::from_millis(120));
        let sent = log.lock().unwrap()[from].clone();
        assert_eq!(sent[0].mode, Mode::Stop, "the culprit is released");
        assert_eq!(sent[1].mode, Mode::Position, "the other holds");
    }

    /// After a reset, a client that has gone quiet is handled by the
    /// timeout policy: `release` lets a held axis go.
    #[test]
    fn a_dead_client_after_a_reset_gets_the_timeout_policy() {
        let (mut worker, _, _, log, t, _) = {
            let (mut worker, mut cmd_tx, _, estop, log) = recording_with(
                1,
                false,
                Rev(0.0),
                TimeoutPolicy::Release,
                EstopPolicy::Hold,
            );
            let start = Instant::now();
            cmd_tx.publish(Stamped {
                value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
                at: start,
            });
            for i in 0..40u64 {
                worker.tick(start + Duration::from_millis(2 * i));
            }
            estop.latch(TripCode::Requested);
            worker.tick(start + Duration::from_millis(80));
            assert!(worker.axes[0].estop_hold.is_some());
            let mut reset = one(mode::DRIVE, 1.2, 0.0, 0.0, 2);
            reset.flags = flags::FAULT_RESET;
            let t = start + Duration::from_millis(82);
            cmd_tx.publish(Stamped { value: reset, at: t });
            worker.tick(t);
            (worker, cmd_tx, estop, log, t, 0usize)
        };
        assert!(worker.axes[0].estop_hold.is_some(), "held through the reset");
        // Nothing more from the client: stale after 100 ms.
        let from = log.lock().unwrap().len();
        for i in 1..80u64 {
            worker.tick(t + Duration::from_millis(2 * i));
        }
        assert!(worker.axes[0].estop_hold.is_none(), "released once stale");
        assert!(log.lock().unwrap()[from..].iter().any(|c| c[0].mode == Mode::Stop));
    }

    /// A bus that fails while axes are held lets them go: a link that comes
    /// back must not pick the hold up again on its own.
    #[test]
    fn a_failed_bus_releases_its_holds() {
        let (mut worker, _, estop, _, t, _) =
            stopped_mid_move(EstopPolicy::Hold, TripCode::Requested);
        worker.tick(t);
        assert!(worker.axes[0].estop_hold.is_some());
        // Swap in a bus that fails every cycle.
        struct Dead;
        impl Bus for Dead {
            fn kind(&self) -> &'static str {
                "dead"
            }
            fn axes(&self) -> usize {
                1
            }
            fn open(&mut self) -> drlabs_core::Result<Vec<Option<drlabs_core::Identity>>> {
                Ok(vec![None])
            }
            fn close(&mut self) -> drlabs_core::Result<()> {
                Ok(())
            }
            fn cycle(&mut self, _: &[AxisCmd], _: &mut [AxisReply]) -> drlabs_core::Result<()> {
                Err(drlabs_core::Error::Config("gone".into()))
            }
            fn capabilities(&self, _: usize) -> drlabs_core::Capabilities {
                drlabs_core::Capabilities::default()
            }
        }
        worker.bus = Box::new(Dead);
        worker.tick(t + Duration::from_millis(2));
        assert!(worker.axes[0].estop_hold.is_none());
        assert_eq!(estop.code(), Some(TripCode::Requested), "the first reason is kept");
    }

    /// The record says a held axis is held — not tripped, not off — during
    /// the stop and after the reset.
    #[test]
    fn a_held_axis_reports_held() {
        let (mut worker, mut cmd_tx, estop, _, t, _) =
            stopped_mid_move(EstopPolicy::Hold, TripCode::Requested);
        worker.tick(t);
        assert_eq!(motor_state(&worker.axes[0], true).status, AxisStatus::Held as u8);
        let mut reset = one(mode::DRIVE, 1.2, 0.0, 0.0, 2);
        reset.flags = flags::FAULT_RESET;
        cmd_tx.publish(Stamped { value: reset, at: t });
        worker.tick(t + Duration::from_millis(2));
        assert_eq!(estop.code(), None);
        assert_eq!(motor_state(&worker.axes[0], false).status, AxisStatus::Held as u8);
    }

    /// A bus whose axis answers but never moves: a joint overpowered by its
    /// load, or held under a torque ceiling below it.
    struct Stuck;
    impl Bus for Stuck {
        fn kind(&self) -> &'static str {
            "stuck"
        }
        fn axes(&self) -> usize {
            1
        }
        fn open(&mut self) -> drlabs_core::Result<Vec<Option<drlabs_core::Identity>>> {
            Ok(vec![None])
        }
        fn close(&mut self) -> drlabs_core::Result<()> {
            Ok(())
        }
        fn cycle(&mut self, _: &[AxisCmd], out: &mut [AxisReply]) -> drlabs_core::Result<()> {
            out[0] = AxisReply {
                status: AxisStatus::Ok,
                mode: Some(1),
                position: Some(Rev(0.0)),
                velocity: Some(RevPerSec(0.0)),
                voltage: Some(drlabs_core::units::Volts(24.0)),
                board_temperature: Some(drlabs_core::units::Celsius(30.0)),
                ..AxisReply::SILENT
            };
            Ok(())
        }
        fn capabilities(&self, _: usize) -> drlabs_core::Capabilities {
            drlabs_core::Capabilities::default()
        }
    }

    fn stuck_worker(following_error: Option<f64>) -> (BusWorker, Writer<Stamped<LowCmd>>, Estop) {
        let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
        let (state_tx, _) = triple(BusState::EMPTY);
        let estop = Estop::new();
        let worker = BusWorker::new(
            WorkerConfig {
                name: "stuck".into(),
                period: Duration::from_millis(2),
                cmd_timeout: Duration::from_secs(10),
                on_timeout: TimeoutPolicy::Hold,
                deadline_miss_trip: 0,
                full_every: 10,
                resend_every: 50,
                priority: 0,
                cpu: None,
            },
            Box::new(Stuck),
            &[AxisSpec {
                following_error,
                ..spec(0)
            }],
            cmd_rx,
            state_tx,
            estop.clone(),
            Arc::new(AtomicBool::new(true)),
        );
        (worker, cmd_tx, estop)
    }

    /// An axis that answers healthy but does not follow its setpoint stops
    /// the robot once the error has lasted long enough — and not before.
    #[test]
    fn a_joint_that_does_not_follow_trips() {
        let (mut worker, mut cmd_tx, estop) = stuck_worker(Some(0.05));
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
            at: start,
        });
        let mut tripped_at = None;
        for i in 0..1000u64 {
            let t = start + Duration::from_millis(2 * i);
            worker.tick(t);
            if estop.code().is_some() {
                tripped_at = Some(t);
                break;
            }
        }
        let tripped_at = tripped_at.expect("it trips");
        assert_eq!(estop.code(), Some(TripCode::FollowingError));
        // The setpoint ramps away from 0; the error passes 0.05 rev after a
        // while, and must then persist for the full time.
        assert!(
            tripped_at.duration_since(start) >= FOLLOWING_ERROR_TIME,
            "tripped after {:?}",
            tripped_at.duration_since(start)
        );
        assert!(worker.axes[0].hold_where_it_is);
        assert!(!worker.axes[0].self_tripped, "overpowered, not misbehaving");
    }

    /// Under `hold`, the joint that could not follow is held where it got
    /// to — not released to fall, and not driven on toward where its
    /// setpoint had run.
    #[test]
    fn a_joint_that_could_not_follow_is_held_where_it_is() {
        let (mut worker, mut cmd_tx, estop) = stuck_worker(Some(0.05));
        worker.axes[0].spec.on_estop = EstopPolicy::Hold;
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
            at: start,
        });
        let mut t = start;
        for i in 0..1000u64 {
            t = start + Duration::from_millis(2 * i);
            worker.tick(t);
            if estop.code().is_some() {
                break;
            }
        }
        assert_eq!(estop.code(), Some(TripCode::FollowingError));
        for i in 1..300u64 {
            worker.tick(t + Duration::from_millis(2 * i));
        }
        let hold = worker.axes[0].estop_hold.expect("held, not released");
        assert!(hold.target.value().abs() < 1e-9, "held where it is (0), at {:?}", hold.target);
        assert!(worker.axes[0].shaper.position.abs() < 1e-3, "setpoint back at the axis");
    }

    /// Off means off, and a short excursion does not trip.
    #[test]
    fn following_error_off_or_brief_does_not_trip() {
        let (mut worker, mut cmd_tx, estop) = stuck_worker(None);
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
            at: start,
        });
        for i in 0..1000u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert_eq!(estop.code(), None);

        // A joint that freezes for a little under the limit and then
        // catches up is not a trip; one that stays frozen is.
        for (frozen_ms, expect_trip) in [(200u64, false), (600, true)] {
            let frozen = Arc::new(AtomicBool::new(false));
            let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
            let mut cmd_tx = cmd_tx;
            let (state_tx, _) = triple(BusState::EMPTY);
            let estop = Estop::new();
            let mut worker = BusWorker::new(
                WorkerConfig {
                    name: "freezes".into(),
                    period: Duration::from_millis(2),
                    cmd_timeout: Duration::from_secs(10),
                    on_timeout: TimeoutPolicy::Hold,
                    deadline_miss_trip: 0,
                    full_every: 10,
                    resend_every: 50,
                    priority: 0,
                    cpu: None,
                },
                Box::new(Freezes {
                    at: 0.0,
                    frozen: Arc::clone(&frozen),
                }),
                &[AxisSpec {
                    following_error: Some(0.02),
                    ..spec(0)
                }],
                cmd_rx,
                state_tx,
                estop.clone(),
                Arc::new(AtomicBool::new(true)),
            );
            cmd_tx.publish(Stamped {
                value: one(mode::DRIVE, 1.2, 0.0, 0.0, 1),
                at: start,
            });
            let mut t = start;
            for i in 0..600u64 {
                // Moving from the start; frozen from 100 ms for `frozen_ms`.
                let ms = 2 * i;
                frozen.store(ms >= 100 && ms < 100 + frozen_ms, Ordering::Relaxed);
                t = start + Duration::from_millis(ms);
                worker.tick(t);
            }
            let _ = t;
            assert_eq!(
                estop.code().is_some(),
                expect_trip,
                "frozen {frozen_ms} ms: {:?}",
                estop.code()
            );
        }
    }

    /// Follows its command exactly, except while `frozen`.
    struct Freezes {
        at: f64,
        frozen: Arc<AtomicBool>,
    }
    impl Bus for Freezes {
        fn kind(&self) -> &'static str {
            "freezes"
        }
        fn axes(&self) -> usize {
            1
        }
        fn open(&mut self) -> drlabs_core::Result<Vec<Option<drlabs_core::Identity>>> {
            Ok(vec![None])
        }
        fn close(&mut self) -> drlabs_core::Result<()> {
            Ok(())
        }
        fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> drlabs_core::Result<()> {
            if !self.frozen.load(Ordering::Relaxed) {
                if let Some(p) = cmds[0].command.position.get() {
                    self.at = p.value();
                }
            }
            out[0] = AxisReply {
                status: AxisStatus::Ok,
                mode: Some(1),
                position: Some(Rev(self.at)),
                velocity: Some(RevPerSec(0.0)),
                voltage: Some(drlabs_core::units::Volts(24.0)),
                board_temperature: Some(drlabs_core::units::Celsius(30.0)),
                ..AxisReply::SILENT
            };
            Ok(())
        }
        fn capabilities(&self, _: usize) -> drlabs_core::Capabilities {
            drlabs_core::Capabilities::default()
        }
    }

    /// A bus that remembers what it was told, so a test can see the modes
    /// the worker chose rather than only where the axes ended up.
    struct Recording {
        inner: SimBus,
        log: Arc<std::sync::Mutex<Vec<Vec<Command>>>>,
        holds: bool,
    }

    impl Bus for Recording {
        fn kind(&self) -> &'static str {
            "recording"
        }
        fn axes(&self) -> usize {
            self.inner.axes()
        }
        fn open(&mut self) -> drlabs_core::Result<Vec<Option<drlabs_core::Identity>>> {
            self.inner.open()
        }
        fn close(&mut self) -> drlabs_core::Result<()> {
            self.inner.close()
        }
        fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> drlabs_core::Result<()> {
            self.log
                .lock()
                .unwrap()
                .push(cmds.iter().map(|c| c.command).collect());
            self.inner.cycle(cmds, out)
        }
        fn capabilities(&self, axis: usize) -> drlabs_core::Capabilities {
            let mut capabilities = self.inner.capabilities(axis);
            if !self.holds {
                capabilities.modes.retain(|m| *m != ControlMode::Hold);
            }
            capabilities
        }
    }

    type Log = Arc<std::sync::Mutex<Vec<Vec<Command>>>>;

    fn recording(
        axes: usize,
        holds: bool,
        at: Rev,
        on_timeout: TimeoutPolicy,
    ) -> (BusWorker, Writer<Stamped<LowCmd>>, Reader<BusState>, Estop, Log) {
        recording_with(axes, holds, at, on_timeout, EstopPolicy::Release)
    }

    fn recording_with(
        axes: usize,
        holds: bool,
        at: Rev,
        on_timeout: TimeoutPolicy,
        on_estop: EstopPolicy,
    ) -> (BusWorker, Writer<Stamped<LowCmd>>, Reader<BusState>, Estop, Log) {
        let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
        let (state_tx, state_rx) = triple(BusState::EMPTY);
        let estop = Estop::new();
        let specs: Vec<AxisSpec> = (0..axes)
            .map(|joint| AxisSpec {
                on_estop,
                ..spec(joint)
            })
            .collect();
        let log: Log = Arc::new(std::sync::Mutex::new(Vec::new()));
        let mut inner = SimBus::new(axes).fixed_step(0.002);
        for axis in 0..axes {
            inner = inner.at(axis, at);
        }
        let mut bus = Recording {
            inner,
            log: Arc::clone(&log),
            holds,
        };
        bus.open().unwrap();
        let worker = BusWorker::new(
            WorkerConfig {
                name: "rec".into(),
                period: Duration::from_millis(2),
                cmd_timeout: Duration::from_millis(100),
                on_timeout,
                deadline_miss_trip: 0,
                full_every: 10,
                resend_every: 50,
                priority: 0,
                cpu: None,
            },
            Box::new(bus),
            &specs,
            cmd_rx,
            state_tx,
            estop.clone(),
            Arc::new(AtomicBool::new(true)),
        );
        (worker, cmd_tx, state_rx, estop, log)
    }

    fn one(mode: u8, q: f32, dq: f32, tau: f32, seq: u32) -> LowCmd {
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = seq;
        cmd.n = 1;
        cmd.cmd[0] = MotorCmd {
            mode,
            q,
            dq,
            tau,
            ..MotorCmd::OFF
        };
        cmd
    }

    fn last_mode(log: &Log) -> Mode {
        log.lock().unwrap().last().unwrap()[0].mode
    }

    /// A velocity command runs the axis without a position target, and a
    /// torque command pushes it. Neither needs `q`.
    #[test]
    fn velocity_and_torque_modes_move_the_axis_without_a_target() {
        let (mut worker, mut cmd_tx, mut state_rx, estop, log) =
            recording(1, true, Rev(0.0), TimeoutPolicy::Hold);
        let start = Instant::now();
        for i in 0..250u64 {
            // Kept fresh: a command older than the timeout becomes a hold.
            let now = start + Duration::from_millis(2 * i);
            cmd_tx.publish(Stamped {
                value: one(mode::VELOCITY, 0.0, 1.0, 0.0, 1),
                at: now,
            });
            worker.tick(now);
        }
        assert_eq!(last_mode(&log), Mode::Velocity);
        let sent = log.lock().unwrap().last().unwrap()[0];
        assert_eq!(sent.position.get(), None, "no position target in velocity mode");
        assert_eq!(
            sent.stop_position.get(),
            Some(spec(0).safety.position_max),
            "the stop is the envelope edge ahead"
        );
        let (state, _) = state_rx.latest();
        assert_eq!(state.state[0].status, AxisStatus::Ok as u8);
        assert!((state.state[0].dq - 1.0).abs() < 0.1, "{:?}", state.state[0]);
        assert!(state.state[0].q > 0.3, "{:?}", state.state[0]);

        // Torque: a push, briefly, so the sim axis does not reach the
        // envelope.
        cmd_tx.publish(Stamped {
            value: one(mode::TORQUE, 0.0, 0.0, -0.05, 2),
            at: start + Duration::from_millis(500),
        });
        for i in 250..270u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert_eq!(last_mode(&log), Mode::Torque);
        let sent = log.lock().unwrap().last().unwrap()[0];
        assert_eq!(sent.position.get(), None);
        assert_eq!(sent.velocity.get(), None);
        assert!((sent.feedforward_torque.get().unwrap().value() + 0.05).abs() < 1e-9);
        assert_eq!(estop.code(), None);
    }

    /// A hold is the controller's own zero-velocity mode where it has one,
    /// and a position at the place the hold began where it does not.
    #[test]
    fn a_hold_is_native_or_a_latched_position() {
        for holds in [true, false] {
            let (mut worker, mut cmd_tx, mut state_rx, _, log) =
                recording(1, holds, Rev(0.1), TimeoutPolicy::Hold);
            let start = Instant::now();
            cmd_tx.publish(Stamped {
                value: one(mode::HOLD, 0.0, 0.0, 0.0, 1),
                at: start,
            });
            for i in 0..50u64 {
                worker.tick(start + Duration::from_millis(2 * i));
            }
            let sent = log.lock().unwrap().last().unwrap()[0];
            if holds {
                assert_eq!(sent.mode, Mode::Hold);
            } else {
                assert_eq!(sent.mode, Mode::Position);
                assert!((sent.position.get().unwrap().value() - 0.1).abs() < 1e-3);
                assert_eq!(sent.velocity.get(), Some(RevPerSec(0.0)));
            }
            let (state, _) = state_rx.latest();
            assert_eq!(state.state[0].status, AxisStatus::Ok as u8, "holds={holds}");
            assert!((state.state[0].q - Rev(0.1).to_rad().value() as f32).abs() < 0.05);
        }
    }

    /// With the `hold` policy a stale velocity or torque command has
    /// nothing to keep, so it becomes a hold; a stale position keeps its
    /// target.
    #[test]
    fn a_stale_velocity_command_becomes_a_hold() {
        let (mut worker, mut cmd_tx, _, _, log) =
            recording(1, true, Rev(0.0), TimeoutPolicy::Hold);
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::VELOCITY, 0.0, 0.5, 0.0, 1),
            at: start,
        });
        for i in 0..10u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert_eq!(last_mode(&log), Mode::Velocity);
        worker.tick(start + Duration::from_millis(200));
        assert_eq!(last_mode(&log), Mode::Hold);
        assert_eq!(worker.driving(), vec![true]);
    }

    /// Stay-within carries the envelope as its bounds, every cycle.
    #[test]
    fn stay_within_uses_the_envelope_as_its_bounds() {
        let s = spec(0);
        let motor = MotorCmd {
            mode: mode::STAY_WITHIN,
            tau: 0.1,
            ..MotorCmd::OFF
        };
        let command = drive_command(&s, &motor, Duration::from_millis(2), false, None);
        assert_eq!(
            command.mode,
            Mode::StayWithin {
                lower: s.safety.position_min,
                upper: s.safety.position_max
            }
        );
        assert!(command.full, "the bounds go out every cycle");
        assert!((command.feedforward_torque.get().unwrap().value() - 0.1).abs() < 1e-6);
    }

    /// Leaving driving sends a burst of stops before the axis is merely
    /// observed: a released motor must be told so, not left to its watchdog.
    #[test]
    fn a_release_sends_stops_then_queries() {
        let (mut worker, mut cmd_tx, _, _, log) =
            recording(1, true, Rev(0.0), TimeoutPolicy::Hold);
        let start = Instant::now();
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 0.0, 0.0, 0.0, 1),
            at: start,
        });
        for i in 0..5u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert_eq!(last_mode(&log), Mode::Position);
        cmd_tx.publish(Stamped {
            value: one(mode::OFF, 0.0, 0.0, 0.0, 2),
            at: start,
        });
        let before = log.lock().unwrap().len();
        for i in 5..15u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        let modes: Vec<Mode> = log.lock().unwrap()[before..]
            .iter()
            .map(|c| c[0].mode)
            .collect();
        assert_eq!(&modes[..STOP_BURST as usize], &[Mode::Stop; 3]);
        assert!(modes[STOP_BURST as usize..]
            .iter()
            .all(|m| *m == Mode::Query), "{modes:?}");
    }

    /// A re-reference happens once, on the change of mode, and not again
    /// for a keep-alive that repeats the record; and an axis that was
    /// driving is stopped before it is re-referenced.
    #[test]
    fn a_zero_request_re_references_once_after_the_axis_is_stopped() {
        let (mut worker, mut cmd_tx, mut state_rx, _, log) =
            recording(1, true, Rev(0.3), TimeoutPolicy::Hold);
        let start = Instant::now();
        let set_outputs = |log: &Log| {
            log.lock()
                .unwrap()
                .iter()
                .filter(|c| matches!(c[0].mode, Mode::SetOutput { .. }))
                .count()
        };

        // Off, then ZERO: the very next cycle re-references.
        worker.tick(start);
        let (state, _) = state_rx.latest();
        assert!((state.state[0].q - Rev(0.3).to_rad().value() as f32).abs() < 1e-3);
        cmd_tx.publish(Stamped {
            value: one(mode::ZERO, 0.0, 0.0, 0.0, 1),
            at: start,
        });
        worker.tick(start + Duration::from_millis(2));
        assert_eq!(last_mode(&log), Mode::SetOutput { position: Rev(0.0) });
        assert_eq!(set_outputs(&log), 1);
        let (state, _) = state_rx.latest();
        assert!(state.state[0].q.abs() < 1e-3, "{:?}", state.state[0]);
        assert_eq!(state.state[0].status, AxisStatus::Off as u8);

        // The same record again, with a new sequence number: nothing.
        for seq in 2..6u32 {
            cmd_tx.publish(Stamped {
                value: one(mode::ZERO, 0.0, 0.0, 0.0, seq),
                at: start,
            });
            worker.tick(start + Duration::from_millis(2 * seq as u64));
        }
        assert_eq!(set_outputs(&log), 1, "a keep-alive is not a new request");
        assert_eq!(last_mode(&log), Mode::Query);

        // Drive, then ZERO at 0.5 rad: three stops first, then the
        // re-reference, then queries.
        cmd_tx.publish(Stamped {
            value: one(mode::DRIVE, 0.0, 0.0, 0.0, 6),
            at: start,
        });
        for i in 6..12u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        assert_eq!(worker.driving(), vec![true]);
        cmd_tx.publish(Stamped {
            value: one(mode::ZERO, 0.5, 0.0, 0.0, 7),
            at: start,
        });
        let before = log.lock().unwrap().len();
        for i in 12..20u64 {
            worker.tick(start + Duration::from_millis(2 * i));
        }
        let modes: Vec<Mode> = log.lock().unwrap()[before..]
            .iter()
            .map(|c| c[0].mode)
            .collect();
        assert_eq!(&modes[..3], &[Mode::Stop; 3]);
        assert_eq!(
            modes[3],
            Mode::SetOutput {
                position: Rad(0.5).to_rev()
            }
        );
        assert!(modes[4..].iter().all(|m| *m == Mode::Query), "{modes:?}");
        assert_eq!(set_outputs(&log), 2);
    }
}
