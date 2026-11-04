//! What the runtime does when driven, tested against a fake bus.
//!
//! These are the session's loop tests, carried over: the claims a control
//! loop has to keep whichever crate owns the tick. The first one is the one
//! that matters most. It reproduces a defect found on the bench, where a
//! flood of commands from a slider drag made a 100 Hz robot run its CAN
//! joint at 801 Hz until the other joint's watchdog latched and e-stopped
//! it. A machine stopped by the rate of somebody's mouse.
//!
//! The runtime cannot have that defect by construction — the bus thread
//! owns the tick and a client only ever replaces a snapshot — so the test
//! here asserts the property rather than the fix: a client at 2 kHz, a bus
//! at the configured rate.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_core::bus::{AxisCmd, AxisReply, AxisStatus, Bus};
use drlabs_core::command::{Command, ControlMode, Mode};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{Capabilities, Envelope, Identity};
use drlabs_core::lowlevel::{flags, mode, LowCmd, MotorCmd};
use drlabs_core::units::{Amps, Celsius, Rad, Rev, RevPerSec, Volts};
use drlabs_rt::{Config, Daemon, Handle};
use drlabs_safety::TripCode;

/// A bus that always answers, instantly, follows the commanded position
/// exactly, and counts how often it was asked.
#[derive(Clone, Default)]
struct FakeBus {
    cycles: Arc<AtomicU64>,
    opens: Arc<AtomicU64>,
    closes: Arc<AtomicU64>,
    /// The commands of the last cycle.
    last: Arc<Mutex<Vec<Command>>>,
    positions: Arc<Mutex<Vec<Rev>>>,
    /// When set, every cycle fails as a transport would.
    dead: Arc<AtomicBool>,
    axes: usize,
}

impl FakeBus {
    fn new(axes: usize) -> Self {
        Self {
            positions: Arc::new(Mutex::new(vec![Rev(0.0); axes])),
            axes,
            ..Default::default()
        }
    }

    fn count(&self) -> u64 {
        self.cycles.load(Ordering::Relaxed)
    }

    fn last_command(&self, axis: usize) -> Option<Command> {
        self.last.lock().unwrap().get(axis).copied()
    }
}

impl Bus for FakeBus {
    fn kind(&self) -> &'static str {
        "fake"
    }
    fn axes(&self) -> usize {
        self.axes
    }
    fn open(&mut self) -> Result<Vec<Option<Identity>>> {
        self.opens.fetch_add(1, Ordering::Relaxed);
        Ok(vec![None; self.axes])
    }
    fn close(&mut self) -> Result<()> {
        self.closes.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
    fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> Result<()> {
        self.cycles.fetch_add(1, Ordering::Relaxed);
        if self.dead.load(Ordering::Relaxed) {
            return Err(Error::transport(
                "fake transaction",
                std::io::Error::new(std::io::ErrorKind::TimedOut, "no reply"),
            ));
        }
        *self.last.lock().unwrap() = cmds.iter().map(|c| c.command).collect();
        let mut positions = self.positions.lock().unwrap();
        for ((cmd, slot), position) in cmds.iter().zip(out.iter_mut()).zip(positions.iter_mut()) {
            if let Some(target) = cmd.command.position.get() {
                *position = target;
            }
            *slot = AxisReply {
                status: AxisStatus::Ok,
                mode: Some(10),
                position: Some(*position),
                velocity: Some(RevPerSec(0.0)),
                current: Some(Amps(0.1)),
                voltage: Some(Volts(24.0)),
                board_temperature: Some(Celsius(30.0)),
                ..AxisReply::SILENT
            };
        }
        Ok(())
    }
    fn capabilities(&self, _axis: usize) -> Capabilities {
        Capabilities {
            modes: vec![
                ControlMode::Stop,
                ControlMode::Query,
                ControlMode::Position,
                ControlMode::Velocity,
            ],
            max_rate_hz: 1000,
            has_watchdog: true,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }
    fn envelope(&self, _axis: usize) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(10.0)),
            voltage_max: Some(Volts(44.0)),
            voltage_rated: Some(Volts(24.0)),
            velocity_max: Some(RevPerSec(100.0)),
            peak_current: Some(Amps(100.0)),
        })
    }
}

/// One joint, ±0.5 rev of travel, on one bus at `rate_hz`.
fn config(rate_hz: u32, on_timeout: &str) -> Config {
    Config::parse(&format!(
        r#"{{"robot":"fake","rate_hz":{rate_hz},
            "buses":[{{"name":"b","transport":"sim"}}],
            "api":{{"cmd_timeout_ms":100,"on_cmd_timeout":"{on_timeout}"}},
            "joints":[{{"name":"j","bus":"b","driver":"sim","id":1,"gear_ratio":1,"direction":1,
              "limits":{{"min_rad":-3.14159265,"max_rad":3.14159265,"max_torque_nm":1,
                         "max_velocity_rad_s":12,"max_accel_rad_s2":60}}}}]}}"#
    ))
    .expect("a valid description")
}

fn started(rate_hz: u32, on_timeout: &str) -> (Handle, FakeBus) {
    let bus = FakeBus::new(1);
    let factory = {
        let bus = bus.clone();
        move |_: &drlabs_rt::BusConfig, _: &[&drlabs_rt::JointConfig]| -> std::result::Result<Box<dyn Bus>, String> {
            Ok(Box::new(bus.clone()))
        }
    };
    let handle = Daemon::start(&config(rate_hz, on_timeout), &factory).expect("the fake bus starts");
    (handle, bus)
}

fn drive(q_rad: f64, seq: u32) -> LowCmd {
    let mut cmd = LowCmd::EMPTY;
    cmd.seq = seq;
    cmd.n = 1;
    cmd.cmd[0] = MotorCmd {
        mode: mode::DRIVE,
        q: q_rad as f32,
        ..MotorCmd::OFF
    };
    cmd
}

/// Waits until the one joint is being driven, or says why not.
fn wait_driving(handle: &mut Handle, budget: Duration) {
    let deadline = Instant::now() + budget;
    loop {
        let state = handle.state();
        if state.state[0].status == AxisStatus::Ok as u8 {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "the joint did not energise: status {} blocked {} estop {}",
            state.state[0].status,
            state.state[0].blocked,
            state.estop_code
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The defect, as a property: a client commanding as fast as it can must
/// not make the bus cycle faster than the rate the machine was budgeted
/// for. The bus is a shared resource and the description says how much of
/// it each joint may spend.
#[test]
fn a_flood_of_commands_cannot_outrun_the_configured_rate() {
    let rate = 50;
    let (mut handle, bus) = started(rate, "hold");
    handle.send(drive(0.1, 1));
    wait_driving(&mut handle, Duration::from_secs(2));

    let before = bus.count();
    let started = Instant::now();
    let window = Duration::from_millis(600);

    // Hammer it exactly the way a slider drag does.
    let mut commanded = 0u32;
    while started.elapsed() < window {
        commanded += 1;
        handle.send(drive(0.1, commanded));
        commanded += 1;
        handle.send(drive(-0.1, commanded));
    }
    let cycles = bus.count() - before;
    let elapsed = started.elapsed().as_secs_f64();
    let achieved = cycles as f64 / elapsed;

    assert!(
        commanded > 1000,
        "the test did not actually flood it: {commanded} commands"
    );
    assert!(
        achieved <= rate as f64 * 1.35,
        "the bus ran at {achieved:.0} Hz under a flood, with a {rate} Hz ceiling \
         ({cycles} cycles in {elapsed:.2} s from {commanded} commands)"
    );
    handle.stop();
}

/// And it does keep the rate it was asked for, rather than defending it by
/// running slowly.
#[test]
fn the_bus_holds_the_rate_it_was_given() {
    let rate = 100;
    let (handle, bus) = started(rate, "hold");
    std::thread::sleep(Duration::from_millis(100));

    let before = bus.count();
    let started = Instant::now();
    std::thread::sleep(Duration::from_millis(600));
    let elapsed = started.elapsed().as_secs_f64();
    let achieved = (bus.count() - before) as f64 / elapsed;

    assert!(
        achieved > rate as f64 * 0.8,
        "expected about {rate} Hz, achieved {achieved:.0} Hz"
    );
    assert!(
        achieved < rate as f64 * 1.2,
        "expected about {rate} Hz, achieved {achieved:.0} Hz"
    );
    handle.stop();
}

/// A command still takes effect promptly: the next tick picks it up, and the
/// state record says so within a period or two.
#[test]
fn a_command_is_acted_on_within_a_period() {
    let (mut handle, bus) = started(50, "hold"); // 20 ms period
    handle.send(drive(0.0, 1));
    wait_driving(&mut handle, Duration::from_secs(2));

    handle.send(drive(0.3, 2));
    std::thread::sleep(Duration::from_millis(60));

    let sent = bus.last_command(0).expect("a command reached the bus");
    let target = sent.position.get().expect("a position was commanded");
    assert!(
        (target - Rad(0.3).to_rev()).abs() < Rev(1e-6),
        "the bus was not given the commanded position: {target:?}"
    );
    let state = handle.state();
    assert!((state.state[0].q - 0.3).abs() < 1e-4, "{:?}", state.state[0]);
    assert_eq!(state.cmd_seq, 2);
    handle.stop();
}

/// Nothing is driven until a client asks; once asked, the axis is driven
/// to where it was told, not to a stale setpoint.
#[test]
fn an_axis_is_off_until_asked_and_then_driven() {
    let (mut handle, bus) = started(100, "hold");
    std::thread::sleep(Duration::from_millis(100));
    let state = handle.state();
    assert_eq!(state.state[0].status, AxisStatus::Off as u8);
    assert_eq!(bus.last_command(0).unwrap().mode, Mode::Query);

    handle.send(drive(0.25, 1));
    wait_driving(&mut handle, Duration::from_secs(2));
    assert_eq!(bus.last_command(0).unwrap().mode, Mode::Position);
    handle.stop();
}

/// A quick stop latches, turns every axis off, and is not undone by the
/// loop carrying on or by a client still asking to drive. Clearing it is a
/// deliberate act, and it clears to off.
#[test]
fn a_quick_stop_latches_until_it_is_reset() {
    let (mut handle, bus) = started(100, "hold");
    handle.send(drive(0.0, 1));
    wait_driving(&mut handle, Duration::from_secs(2));

    let mut stop = drive(0.0, 2);
    stop.flags = flags::QUICK_STOP;
    handle.send(stop);
    std::thread::sleep(Duration::from_millis(50));

    let state = handle.state();
    assert_eq!(state.estop, 1);
    assert_eq!(state.estop_code, TripCode::Requested as u16);
    assert_eq!(state.state[0].status, AxisStatus::Tripped as u8);
    assert_eq!(bus.last_command(0).unwrap().mode, Mode::Stop);

    // Still asking to drive, with the stop latched: no.
    for seq in 3..8 {
        handle.send(drive(0.0, seq));
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(handle.state().estop, 1, "a latched stop must not undo itself");
    assert_eq!(bus.last_command(0).unwrap().mode, Mode::Stop);

    // Reset: the latch clears and the axis is off, not driving.
    let mut reset = LowCmd::EMPTY;
    reset.seq = 8;
    reset.n = 1;
    reset.flags = flags::FAULT_RESET;
    handle.send(reset);
    std::thread::sleep(Duration::from_millis(100));
    let state = handle.state();
    assert_eq!(state.estop, 0);
    assert_eq!(state.state[0].status, AxisStatus::Off as u8);
    assert_eq!(bus.last_command(0).unwrap().mode, Mode::Query);
    handle.stop();
}

/// The safety envelope clamps a command before it reaches the bus.
#[test]
fn a_target_outside_the_envelope_is_clamped_before_it_is_sent() {
    let (mut handle, bus) = started(100, "hold");
    handle.send(drive(0.0, 1));
    wait_driving(&mut handle, Duration::from_secs(2));

    handle.send(drive(50.0, 2));
    std::thread::sleep(Duration::from_millis(50));
    let sent = bus.last_command(0).unwrap().position.get().unwrap();
    assert!(
        (sent - Rev(0.5)).abs() < Rev(1e-6),
        "±π rad is ±0.5 rev at the shaft; got {sent:?}"
    );
    handle.stop();
}

/// A client that stops sending: with `release`, the axis goes off; with
/// `hold`, it stays where it was sent.
#[test]
fn a_stale_command_is_handled_by_the_policy() {
    for (policy, expect) in [("release", Mode::Query), ("hold", Mode::Position)] {
        let (mut handle, bus) = started(100, policy);
        handle.send(drive(0.1, 1));
        wait_driving(&mut handle, Duration::from_secs(2));
        // The command is 100 ms old after this; the stop burst after a
        // release is three ticks, long gone by then.
        std::thread::sleep(Duration::from_millis(250));
        assert_eq!(bus.last_command(0).unwrap().mode, expect, "{policy}");
        handle.stop();
    }
}

/// A transport error mid-cycle is a bus the driver has declared dead. The
/// runtime latches the stop and names the transport, rather than carrying
/// on as if the axes were being driven.
#[test]
fn a_transport_error_latches_a_transport_stop() {
    let (mut handle, bus) = started(100, "hold");
    handle.send(drive(0.0, 1));
    wait_driving(&mut handle, Duration::from_secs(2));

    bus.dead.store(true, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(50));
    let state = handle.state();
    assert_eq!(state.estop, 1);
    assert_eq!(state.estop_code, TripCode::Transport as u16);
    assert_eq!(state.state[0].status, AxisStatus::Tripped as u8);
    handle.stop();
}

/// Starting opens the bus exactly once, and stopping closes it exactly
/// once — on the bus thread, after its last tick.
#[test]
fn starting_opens_the_bus_once_and_stopping_closes_it_once() {
    let (handle, bus) = started(100, "hold");
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(bus.opens.load(Ordering::Relaxed), 1);
    assert_eq!(bus.closes.load(Ordering::Relaxed), 0);
    handle.stop();
    assert_eq!(bus.closes.load(Ordering::Relaxed), 1);
}
