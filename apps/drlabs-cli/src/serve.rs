//! The servo role: energised, waiting, executing what it is told.
//!
//! This is the framework's own framing of itself. It is the low level;
//! something above it decides what the machine does. Its job is to be ready and
//! to carry out commands — inside an envelope nothing above it can exceed.
//!
//! The protocol is newline-delimited JSON, in both directions. One object per
//! line, no framing, no handshake: it is readable from a terminal with `nc`,
//! and any language with a JSON encoder can drive it in ten lines.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_core::command::ControlMode;
use drlabs_core::units::{Nm, Rev, RevPerSec, RevPerSec2};
use drlabs_safety::Config as SafetyConfig;
use drlabs_session::{Options, Params, Session};
use serde::Deserialize;

use crate::flags::{Flags, Kind};
use crate::probe::link_for;
use crate::{Target, COMMON_FLAGS};

/// One command from a client.
///
/// Every field is optional because every command uses a different handful, and
/// a client should not have to send nulls for the twenty it does not want.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct ClientCommand {
    cmd: String,
    value: Option<f64>,

    target: Option<f64>,
    max_velocity: Option<f64>,
    accel: Option<f64>,

    a: Option<f64>,
    b: Option<f64>,
    dwell: Option<f64>,
    cycles: Option<u32>,

    max_torque: Option<f64>,
    velocity_limit: Option<f64>,
    accel_limit: Option<f64>,
    kp_scale: Option<f64>,
    kd_scale: Option<f64>,
    feedforward: Option<f64>,

    mode: Option<String>,
}

const SERVE_FLAGS: &[(&str, Kind)] = &[
    ("addr", Kind::Value),
    ("rate", Kind::Value),
    ("state-rate", Kind::Value),
    ("max-torque", Kind::Value),
    ("velocity-limit", Kind::Value),
    ("accel-limit", Kind::Value),
    ("position-min", Kind::Value),
    ("position-max", Kind::Value),
    ("command-timeout", Kind::Value),
    ("deadman", Kind::Value),
    ("no-auto-arm", Kind::Switch),
    ("allow-unknown-board", Kind::Switch),
];

/// Whether open-ended motion is currently standing, and when it was last asked
/// for.
struct Motion {
    commanded: bool,
    at: Instant,
}

pub fn serve(args: &[String]) -> Result<(), String> {
    let known: Vec<(&str, Kind)> = COMMON_FLAGS.iter().chain(SERVE_FLAGS).copied().collect();
    let flags = Flags::parse(args, &known).map_err(|error| error.to_string())?;
    let target = Target::from(&flags)?;

    let addr = flags.string("addr", "127.0.0.1:7777");
    let rate: u32 = flags.number("rate", 200)?;
    let state_rate: u32 = flags.number("state-rate", 50)?;
    let max_torque: f64 = flags.number("max-torque", 0.3)?;
    let velocity_limit: f64 = flags.number("velocity-limit", 1.0)?;
    let accel_limit: f64 = flags.number("accel-limit", 5.0)?;
    let command_timeout: f64 = flags.number("command-timeout", 1.0)?;
    let auto_arm = !flags.switch("no-auto-arm");
    let allow_unknown = flags.switch("allow-unknown-board");

    let safety = SafetyConfig {
        max_torque: Nm(max_torque),
        max_velocity: RevPerSec(velocity_limit),
        accel_limit: RevPerSec2(accel_limit),
        position_min: Rev(flags.number("position-min", -0.5)?),
        position_max: Rev(flags.number("position-max", 0.5)?),
        deadman_timeout: flags.number("deadman", 0.0)?,
        ..Default::default()
    };

    let link = link_for(&target)?;
    let session = Session::new(Options {
        safety,
        rate_hz: rate,
        auto_arm,
        require_acknowledgement: false,
        initial_mode: ControlMode::Position,
        ..Default::default()
    });
    session
        .connect(link, allow_unknown)
        .map_err(|error| error.to_string())?;
    session.set_params(Params {
        max_torque: Nm(max_torque),
        velocity_limit: RevPerSec(velocity_limit),
        accel_limit: RevPerSec2(accel_limit),
        ..Default::default()
    });

    let listener = TcpListener::bind(&addr).map_err(|error| {
        session.shutdown();
        format!("cannot listen on {addr}: {error}")
    })?;
    let bound = listener
        .local_addr()
        .map(|addr| addr.to_string())
        .unwrap_or(addr);

    println!("drlabs servo on {bound}");
    println!(
        "  driver {} · id {}{} · {rate} Hz",
        target.driver,
        target.id,
        if target.port.is_empty() {
            String::new()
        } else {
            format!(" · {}", target.port)
        }
    );
    if auto_arm {
        println!("  energising and waiting for commands");
    } else {
        println!("  auto-arm disabled — send {{\"cmd\":\"arm\"}} to energise");
    }
    println!("  Ctrl+C to stop");

    let motion = Arc::new(Mutex::new(Motion {
        commanded: false,
        at: Instant::now(),
    }));
    let running = Arc::new(AtomicBool::new(true));

    // Open-ended motion reverts to a position hold when the caller goes quiet.
    //
    // A position command is safe to leave standing: the axis arrives and holds.
    // A velocity or torque command is not — it runs until something stops it,
    // so a caller that crashes mid-move would leave the machine turning. This
    // is the controller's own watchdog, one level up.
    if command_timeout > 0.0 {
        let session = session.clone();
        let motion = motion.clone();
        let running = running.clone();
        std::thread::spawn(move || {
            while running.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(100));
                let idle = {
                    let mut motion = motion.lock().expect("motion poisoned");
                    let idle =
                        motion.commanded && motion.at.elapsed().as_secs_f64() > command_timeout;
                    if idle {
                        motion.commanded = false;
                    }
                    idle
                };
                if idle {
                    let _ = session.set_mode(ControlMode::Position);
                    if let Some(position) = session.status().telemetry.position {
                        session.set_position(position);
                    }
                }
            }
        });
    }

    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let session = session.clone();
        let motion = motion.clone();
        std::thread::spawn(move || {
            let _ = handle(stream, session, motion, state_rate);
        });
    }
    Ok(())
}

/// One client, for as long as it stays connected.
fn handle(
    stream: TcpStream,
    session: Arc<Session>,
    motion: Arc<Mutex<Motion>>,
    state_rate: u32,
) -> std::io::Result<()> {
    let reader = BufReader::new(stream.try_clone()?);
    let writer = Arc::new(Mutex::new(stream));

    // Unsolicited state, so a client does not have to poll for it.
    let streaming = Arc::new(AtomicBool::new(true));
    if state_rate > 0 {
        let session = session.clone();
        let writer = writer.clone();
        let streaming = streaming.clone();
        std::thread::spawn(move || {
            let period = Duration::from_secs_f64(1.0 / state_rate as f64);
            while streaming.load(Ordering::Relaxed) {
                let line = state_line(&session);
                let mut writer = writer.lock().expect("writer poisoned");
                if writeln!(writer, "{line}").is_err() {
                    break;
                }
                drop(writer);
                std::thread::sleep(period);
            }
        });
    }

    for line in reader.lines() {
        let Ok(line) = line else { break };
        if line.trim().is_empty() {
            continue;
        }
        // The client's heartbeat is its commands. A supervisor that is still
        // sending is still there.
        session.heartbeat();

        let reply = match serde_json::from_str::<ClientCommand>(&line) {
            Ok(command) => execute(&session, &motion, &command),
            Err(error) => refusal(&format!("{error}")),
        };
        let mut writer = writer.lock().expect("writer poisoned");
        writeln!(writer, "{reply}")?;
    }

    streaming.store(false, Ordering::Relaxed);
    Ok(())
}

fn ok() -> String {
    "{\"ok\":true}".into()
}

fn refusal(reason: &str) -> String {
    format!(
        "{{\"ok\":false,\"error\":{}}}",
        serde_json::to_string(reason).unwrap_or_else(|_| "\"refused\"".into())
    )
}

/// Marks that open-ended motion is standing, so the watchdog above knows to
/// wind it back if the caller goes quiet.
fn note_motion(motion: &Mutex<Motion>, open_ended: bool) {
    let mut motion = motion.lock().expect("motion poisoned");
    motion.commanded = open_ended;
    motion.at = Instant::now();
}

fn execute(session: &Arc<Session>, motion: &Mutex<Motion>, command: &ClientCommand) -> String {
    let need = |value: Option<f64>, what: &str| -> Result<f64, String> {
        value.ok_or_else(|| format!("{:?} needs a {what}", command.cmd))
    };

    let result: Result<String, String> = (|| match command.cmd.as_str() {
        "position" => {
            let value = need(command.value.or(command.target), "value")?;
            session
                .set_mode(ControlMode::Position)
                .map_err(|error| error.to_string())?;
            let clamped = session.set_position(Rev(value));
            note_motion(motion, false);
            Ok(format!("{{\"ok\":true,\"position\":{}}}", clamped.value()))
        }
        "velocity" => {
            let value = need(command.value, "value")?;
            session
                .set_mode(ControlMode::Velocity)
                .map_err(|error| error.to_string())?;
            let clamped = session.set_velocity(RevPerSec(value));
            note_motion(motion, true);
            Ok(format!("{{\"ok\":true,\"velocity\":{}}}", clamped.value()))
        }
        "torque" => {
            let value = need(command.value.or(command.feedforward), "value")?;
            session
                .set_mode(ControlMode::Torque)
                .map_err(|error| error.to_string())?;
            let mut params = session.params();
            params.feedforward = Nm(value);
            session.set_params(params);
            note_motion(motion, true);
            Ok(ok())
        }
        "mode" => {
            let name = command
                .mode
                .as_deref()
                .ok_or("\"mode\" needs a mode name")?;
            let mode = ControlMode::parse(name).ok_or_else(|| format!("unknown mode {name:?}"))?;
            session.set_mode(mode).map_err(|error| error.to_string())?;
            note_motion(
                motion,
                matches!(mode, ControlMode::Velocity | ControlMode::Torque),
            );
            Ok(ok())
        }
        "stop" => {
            session.disarm("a client asked it to stop");
            note_motion(motion, false);
            Ok(ok())
        }
        "hold" => {
            session
                .set_mode(ControlMode::Hold)
                .map_err(|error| error.to_string())?;
            note_motion(motion, false);
            Ok(ok())
        }
        "trajectory" => {
            let target = need(command.target.or(command.value), "target")?;
            let params = session.params();
            let max_velocity = command
                .max_velocity
                .map(RevPerSec)
                .unwrap_or(params.velocity_limit);
            let accel = command.accel.map(RevPerSec2).unwrap_or(params.accel_limit);
            session
                .start_trajectory(Rev(target), max_velocity, accel)
                .map_err(|error| error.to_string())?;
            note_motion(motion, false);
            Ok(ok())
        }
        "trajectory_stop" => {
            session
                .set_mode(ControlMode::Position)
                .map_err(|error| error.to_string())?;
            if let Some(position) = session.status().telemetry.position {
                session.set_position(position);
            }
            Ok(ok())
        }
        "shuttle" => {
            let a = need(command.a, "an \"a\"")?;
            let b = need(command.b, "a \"b\"")?;
            session.configure_shuttle(
                Rev(a),
                Rev(b),
                command.dwell.unwrap_or(1.0),
                command.cycles.unwrap_or(0),
            );
            session.start_shuttle().map_err(|error| error.to_string())?;
            note_motion(motion, false);
            Ok(ok())
        }
        "shuttle_stop" => {
            session.stop_shuttle("a client asked it to stop");
            Ok(ok())
        }
        "params" => {
            let mut params = session.params();
            if let Some(value) = command.max_torque {
                params.max_torque = Nm(value);
            }
            if let Some(value) = command.velocity_limit {
                params.velocity_limit = RevPerSec(value);
            }
            if let Some(value) = command.accel_limit {
                params.accel_limit = RevPerSec2(value);
            }
            if let Some(value) = command.kp_scale {
                params.kp_scale = value;
            }
            if let Some(value) = command.kd_scale {
                params.kd_scale = value;
            }
            if let Some(value) = command.feedforward {
                params.feedforward = Nm(value);
            }
            session.set_params(params);
            Ok(ok())
        }
        "arm" => session
            .arm(true)
            .map(|()| ok())
            .map_err(|error| error.to_string()),
        "disarm" => {
            session.disarm("a client asked it to disarm");
            Ok(ok())
        }
        "estop" => {
            session.estop("client", "A client raised an emergency stop", "");
            Ok(ok())
        }
        "reset" => session
            .reset_estop()
            .map(|()| ok())
            .map_err(|error| error.to_string()),
        "zero" => {
            let value = command.value.unwrap_or(0.0);
            session
                .zero_output(Rev(value))
                .map(|()| ok())
                .map_err(|error| error.to_string())
        }
        "state" => Ok(state_line(session)),
        "ping" => Ok(ok()),
        other => Err(format!("unknown command {other:?}")),
    })();

    match result {
        Ok(reply) => reply,
        Err(reason) => refusal(&reason),
    }
}

/// One line of state, as a client wants it.
fn state_line(session: &Arc<Session>) -> String {
    let status = session.status();
    let telemetry = &status.telemetry;

    let number = |value: Option<f64>| match value {
        // null, not zero. A driver that did not read a torque reports nothing.
        Some(value) if value.is_finite() => format!("{value}"),
        _ => "null".into(),
    };

    format!(
        "{{\"state\":true,\"armed\":{},\"estop\":{},\"estop_reason\":{},\"mode\":{},\
          \"position\":{},\"velocity\":{},\"torque\":{},\"current\":{},\"voltage\":{},\
          \"temperature\":{},\"target\":{},\"rate_hz\":{},\"fault\":{},\"cycles\":{}}}",
        status.armed,
        status.estop.is_some(),
        serde_json::to_string(
            &status
                .estop
                .as_ref()
                .map(|estop| estop.message.clone())
                .unwrap_or_default()
        )
        .unwrap_or_else(|_| "\"\"".into()),
        serde_json::to_string(status.mode.as_str()).unwrap_or_else(|_| "\"\"".into()),
        number(telemetry.position.map(|v| v.value())),
        number(telemetry.velocity.map(|v| v.value())),
        number(telemetry.torque.map(|v| v.value())),
        number(telemetry.q_current.map(|v| v.value())),
        number(telemetry.voltage.map(|v| v.value())),
        number(telemetry.board_temperature.map(|v| v.value())),
        status.setpoint.position.value(),
        status.achieved_rate_hz,
        serde_json::to_string(
            &telemetry
                .fault
                .as_ref()
                .map(|fault| fault.name.clone())
                .unwrap_or_default()
        )
        .unwrap_or_else(|_| "\"\"".into()),
        status.cycles
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Result<ClientCommand, String> {
        serde_json::from_str(text).map_err(|error| error.to_string())
    }

    /// A client sends the handful of fields its command needs, not nulls for
    /// the twenty it does not.
    #[test]
    fn a_command_needs_only_its_own_fields() {
        let command = parse(r#"{"cmd":"position","value":0.25}"#).unwrap();
        assert_eq!(command.cmd, "position");
        assert_eq!(command.value, Some(0.25));
        assert_eq!(command.max_velocity, None);
    }

    /// A field nobody defined is a typo, and a typo that is ignored is a
    /// command that silently did something else.
    #[test]
    fn an_unknown_field_is_refused_rather_than_ignored() {
        let error = parse(r#"{"cmd":"position","postion":0.25}"#).unwrap_err();
        assert!(error.contains("postion"), "{error}");
    }

    #[test]
    fn a_shuttle_carries_its_endpoints_and_timing() {
        let command =
            parse(r#"{"cmd":"shuttle","a":-0.25,"b":0.25,"dwell":0.5,"cycles":10}"#).unwrap();
        assert_eq!(command.a, Some(-0.25));
        assert_eq!(command.cycles, Some(10));
    }

    /// The reply carries the machine's own words. A refusal is an answer, and
    /// mangling it on the way out would be the bug.
    #[test]
    fn a_refusal_carries_its_reason_as_valid_json() {
        let reply = refusal(r#"joint "j1" is not armed"#);
        let parsed: serde_json::Value = serde_json::from_str(&reply).expect("valid JSON");
        assert_eq!(parsed["ok"], false);
        assert_eq!(parsed["error"], r#"joint "j1" is not armed"#);
    }

    /// Whether a command leaves motion standing decides whether the watchdog
    /// above has to wind it back. Position arrives and holds; velocity and
    /// torque run until something stops them.
    #[test]
    fn only_open_ended_commands_are_marked_as_standing_motion() {
        let motion = Mutex::new(Motion {
            commanded: false,
            at: Instant::now(),
        });

        note_motion(&motion, false);
        assert!(!motion.lock().unwrap().commanded);

        note_motion(&motion, true);
        assert!(motion.lock().unwrap().commanded);
    }
}
