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
use crate::gate::{AxisGate, Estop, TimeoutPolicy};
use crate::mapping::Mapping;

/// What one axis on the bus is.
#[derive(Debug, Clone, Copy)]
pub struct AxisSpec {
    /// Its index in [`LowCmd::cmd`] and [`BusState::state`].
    pub joint: usize,
    pub mapping: Mapping,
    /// The motor-space envelope, already through the mapping.
    pub safety: SafetyConfig,
}

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
            late_run: 0,
            epoch: Instant::now(),
        }
    }

    /// Opens the bus and runs ticks on the period until `running` goes
    /// false, then closes it. Every bus thread given the same `epoch` wakes
    /// on the same grid.
    pub fn run(mut self, epoch: Instant) {
        self.epoch = epoch;
        if let Err(error) = self.bus.open() {
            eprintln!("[{}] open failed: {error}", self.config.name);
            self.estop.latch(TripCode::Transport);
            return;
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
            if self.estop.is_latched() {
                self.estop.clear();
                for axis in &mut self.axes {
                    axis.gate.release();
                    axis.guard.reset();
                    axis.silent_run = 0;
                }
            }
        }
        let stopped = self.estop.is_latched();

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

            let want = mode::drives(motor.mode);
            let was_driving = axis.gate.driving;
            let drive = axis
                .gate
                .decide(want, &axis.known, &mut axis.guard, stopped);
            if was_driving && !drive {
                axis.stop_ticks = STOP_BURST;
            }

            let command = if stopped {
                // A stop cancels what was pending; the client asks again.
                axis.zero_pending = None;
                Command::new(Mode::Stop)
            } else if drive {
                let hold_at = if axis.can_hold { None } else { axis.hold_at };
                drive_command(&axis.spec, &motor, self.config.period, resend, hold_at)
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
            for reply in &mut self.replies {
                *reply = AxisReply::SILENT;
            }
        }

        for (axis, reply) in self.axes.iter_mut().zip(self.replies.iter()) {
            axis.known.merge(reply);
            if reply.status.answered() {
                axis.silent_run = 0;
            } else {
                axis.silent_run = axis.silent_run.saturating_add(1);
                let limit = axis.spec.safety.max_consecutive_timeouts;
                if axis.gate.driving && limit > 0 && axis.silent_run >= limit {
                    self.estop.latch(TripCode::CommLoss);
                }
            }
            if let Some(code) =
                axis.guard
                    .evaluate(&axis.known, axis.velocity_target, axis.gate.driving, now)
            {
                self.estop.latch(code);
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
    let status = if stopped {
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
    use drlabs_core::units::{RadPerSec2, Rev};
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
        };
        AxisSpec {
            joint,
            mapping,
            safety: mapping.safety_config(&limits, SafetyConfig::default()),
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
        let (cmd_tx, cmd_rx) = triple(Stamped::now(LowCmd::EMPTY));
        let (state_tx, state_rx) = triple(BusState::EMPTY);
        let estop = Estop::new();
        let specs: Vec<AxisSpec> = (0..axes).map(spec).collect();
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
