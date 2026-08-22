//! What the control loop does when driven, tested against a fake controller.
//!
//! The first test here is the one that matters most: it reproduces a defect
//! found on the bench, where a flood of commands from a slider drag made a
//! 100 Hz robot run its CAN joint at 801 Hz until the other joint's watchdog
//! latched and e-stopped it. A machine stopped by the rate of somebody's mouse.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::command::{Command, ControlMode, QueryProfile};
use drlabs_core::error::Result;
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::Telemetry;
use drlabs_core::units::{Amps, Celsius, Rev, RevPerSec, Volts};
use drlabs_session::{Options, Session};

/// A controller that always answers, instantly, and counts how often it was
/// asked.
#[derive(Clone, Default)]
struct FakeController {
    cycles: Arc<AtomicU64>,
    position: Arc<std::sync::Mutex<Rev>>,
}

impl FakeController {
    fn count(&self) -> u64 {
        self.cycles.load(Ordering::Relaxed)
    }
}

impl Link for FakeController {
    fn kind(&self) -> &'static str {
        "fake"
    }

    fn describe(&self) -> Vec<(String, String)> {
        vec![("driver".into(), "fake".into())]
    }

    fn open(&mut self) -> Result<()> {
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        Ok(())
    }

    fn identify(&mut self) -> Result<Option<Identity>> {
        Ok(Some(Identity {
            board: "fake controller".into(),
            status: BoardStatus::Supported,
            ..Identity::new("fake")
        }))
    }

    fn cycle(&mut self, command: &Command, _profile: QueryProfile) -> Result<Option<Telemetry>> {
        self.cycles.fetch_add(1, Ordering::Relaxed);
        // Follow the commanded position exactly, so the axis never looks stuck.
        if let Some(target) = command.position.get() {
            *self.position.lock().unwrap() = target;
        }
        Ok(Some(Telemetry {
            mode: Some(10),
            mode_name: "Position".into(),
            position: Some(*self.position.lock().unwrap()),
            velocity: Some(RevPerSec(0.0)),
            q_current: Some(Amps(0.1)),
            voltage: Some(Volts(24.0)),
            board_temperature: Some(Celsius(30.0)),
            ..Default::default()
        }))
    }

    fn capabilities(&self) -> Capabilities {
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

    fn envelope(&self) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(10.0)),
            voltage_max: Some(Volts(44.0)),
            voltage_rated: Some(Volts(24.0)),
            velocity_max: Some(RevPerSec(100.0)),
            peak_current: Some(Amps(100.0)),
        })
    }
}

fn connected(rate_hz: u32) -> (Arc<Session>, FakeController) {
    let controller = FakeController::default();
    let session = Session::new(Options {
        rate_hz,
        auto_arm: true,
        require_acknowledgement: false,
        ..Default::default()
    });
    session
        .connect(Box::new(controller.clone()), false)
        .expect("the fake controller connects");
    (session, controller)
}

/// The defect, reproduced.
///
/// A client commanding as fast as it can must not make the loop cycle faster
/// than the rate the machine was budgeted for. The bus is a shared resource and
/// the description is what says how much of it each joint may spend.
#[test]
fn a_flood_of_commands_cannot_outrun_the_configured_rate() {
    let rate = 50;
    let (session, controller) = connected(rate);
    std::thread::sleep(Duration::from_millis(100)); // let it arm

    let before = controller.count();
    let started = Instant::now();
    let window = Duration::from_millis(600);

    // Hammer it exactly the way a slider drag does.
    let mut commanded = 0u64;
    while started.elapsed() < window {
        session.set_position(Rev(0.1));
        session.set_position(Rev(-0.1));
        commanded += 2;
    }
    let cycles = controller.count() - before;
    let elapsed = started.elapsed().as_secs_f64();
    let achieved = cycles as f64 / elapsed;

    assert!(
        commanded > 1000,
        "the test did not actually flood it: {commanded} commands"
    );
    assert!(
        achieved <= rate as f64 * 1.35,
        "the loop ran at {achieved:.0} Hz under a flood, with a {rate} Hz ceiling \
         ({cycles} cycles in {elapsed:.2} s from {commanded} commands)"
    );
}

/// And it does keep the rate it was asked for, rather than defending it by
/// running slowly.
#[test]
fn the_loop_holds_the_rate_it_was_given() {
    let rate = 100;
    let (_session, controller) = connected(rate);
    std::thread::sleep(Duration::from_millis(100));

    let before = controller.count();
    let started = Instant::now();
    std::thread::sleep(Duration::from_millis(600));
    let elapsed = started.elapsed().as_secs_f64();
    let achieved = (controller.count() - before) as f64 / elapsed;

    assert!(
        achieved > rate as f64 * 0.8,
        "expected about {rate} Hz, achieved {achieved:.0} Hz"
    );
    assert!(
        achieved < rate as f64 * 1.2,
        "expected about {rate} Hz, achieved {achieved:.0} Hz"
    );
}

/// A command still takes effect promptly — the point of the wake is that the
/// loop notices without polling, and giving that up would trade one problem for
/// another.
#[test]
fn a_command_is_acted_on_within_a_period() {
    let (session, _controller) = connected(50); // 20 ms period
    std::thread::sleep(Duration::from_millis(100));

    session.set_position(Rev(0.3));
    std::thread::sleep(Duration::from_millis(60));

    let status = session.status();
    assert!(
        (status.telemetry.position.unwrap() - Rev(0.3)).abs() < Rev(1e-6),
        "the axis did not reach the commanded position: {:?}",
        status.telemetry.position
    );
}

/// A session set up as a servo energises itself once preflight is clear, and
/// holds the position the axis was already at rather than travelling to a stale
/// setpoint.
#[test]
fn an_auto_arming_session_energises_and_holds_where_it_is() {
    let (session, _controller) = connected(100);
    std::thread::sleep(Duration::from_millis(150));

    let status = session.status();
    assert!(status.armed, "the session should have armed itself");
    assert_eq!(status.mode, ControlMode::Position);
    assert_eq!(status.setpoint.position, Rev(0.0));
}

/// An emergency stop latches, disarms, and is not undone by the loop carrying
/// on. Clearing it is a deliberate act.
#[test]
fn an_emergency_stop_latches_until_it_is_cleared() {
    let (session, _controller) = connected(100);
    std::thread::sleep(Duration::from_millis(150));
    assert!(session.status().armed);

    session.estop("test", "Stopped by the test", "");
    std::thread::sleep(Duration::from_millis(100));

    let status = session.status();
    assert!(!status.armed);
    assert!(status.estop.is_some());
    // Auto-arm must not re-arm through it.
    std::thread::sleep(Duration::from_millis(150));
    assert!(
        !session.status().armed,
        "a latched stop must not undo itself"
    );

    session.reset_estop().expect("nothing is faulted");
    assert!(session.status().estop.is_none());
}

/// The safety envelope clamps a command before it reaches the controller.
#[test]
fn a_target_outside_the_envelope_is_clamped_before_it_is_sent() {
    let (session, _controller) = connected(100);
    std::thread::sleep(Duration::from_millis(150));

    let clamped = session.set_position(Rev(50.0));
    assert_eq!(clamped, Rev(0.5), "the default envelope is ±0.5 rev");

    std::thread::sleep(Duration::from_millis(60));
    let status = session.status();
    assert!((status.telemetry.position.unwrap() - Rev(0.5)).abs() < Rev(1e-6));
}

/// A host-profiled move arrives at its target, and reports itself finished
/// rather than leaving a UI showing a move that is still running.
#[test]
fn a_trajectory_runs_to_its_target_and_finishes() {
    let (session, _controller) = connected(200);
    std::thread::sleep(Duration::from_millis(150));

    session
        .start_trajectory(
            Rev(0.3),
            RevPerSec(2.0),
            drlabs_core::units::RevPerSec2(10.0),
        )
        .expect("a trajectory inside the envelope starts");

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && session.status().trajectory.running {
        std::thread::sleep(Duration::from_millis(20));
    }

    let status = session.status();
    assert!(!status.trajectory.running, "the trajectory never finished");
    assert!((status.setpoint.position - Rev(0.3)).abs() < Rev(1e-6));
}

/// The status reports the rate it is actually achieving, measured rather than
/// echoed back from the configuration.
#[test]
fn the_status_reports_a_measured_rate() {
    let (session, _controller) = connected(100);
    std::thread::sleep(Duration::from_millis(500));

    let status = session.status();
    assert!(
        status.achieved_rate_hz > 50.0 && status.achieved_rate_hz < 150.0,
        "achieved rate looks wrong: {:.1} Hz",
        status.achieved_rate_hz
    );
    assert!(status.cycles > 20);
    assert_eq!(status.timeouts, 0);
}
