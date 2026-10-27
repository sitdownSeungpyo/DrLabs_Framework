//! `drlabs rt`: the real-time runtime.
//!
//! ```text
//! drlabs rt [-check] [-seconds N] [-drive] [-report N] <robot.json>
//! ```
//!
//! With `-check` the description is validated and the plan printed, and
//! nothing is opened. Otherwise one thread per bus runs at the configured
//! rate, and a line a second says how it is keeping time. `-drive` sends a
//! slow sine to every joint from inside the process, which is the whole
//! command path with no client attached; it is how the loop is measured on
//! a simulated machine.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_api::{describe::describe_json, UdpServer};
use drlabs_core::lowlevel::{mode, LowCmd, MotorCmd};
use drlabs_core::AxisStatus;
use drlabs_rt::{Config, Daemon};

use crate::flags::{Flags, Kind};

const FLAGS: &[(&str, Kind)] = &[
    ("check", Kind::Switch),
    ("drive", Kind::Switch),
    ("seconds", Kind::Value),
    ("report", Kind::Value),
];

pub fn rt(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, FLAGS).map_err(|error| error.to_string())?;
    let path = flags
        .positional
        .first()
        .ok_or("usage: drlabs rt [-check] [-seconds N] [-drive] <robot.json>")?
        .clone();
    let seconds: u64 = flags.number("seconds", 0u64)?;
    let report_every: u64 = flags.number("report", 1u64)?;

    let config = Config::load(Path::new(&path)).map_err(|error| error.to_string())?;
    print_plan(&config);
    if flags.switch("check") {
        return Ok(());
    }

    let handle = Arc::new(Mutex::new(Daemon::start(&config, &factory)?));
    let state_period = Duration::from_secs_f64(config.api.state_div.max(1) as f64 / config.rate_hz as f64);
    let server = UdpServer::start(
        &config.api.udp,
        Arc::clone(&handle),
        describe_json(&config),
        state_period,
    )
    .map_err(|error| format!("cannot bind the UDP endpoint {}: {error}", config.api.udp))?;
    println!(
        "LowCmd/LowState on udp://{} (state every {:.1} ms)",
        server.local_addr(),
        state_period.as_secs_f64() * 1e3
    );
    let started = Instant::now();
    let mut next_report = started + Duration::from_secs(report_every.max(1));
    let mut last_tick = 0u64;
    let mut last_report = started;
    let mut seq = 0u32;
    let drive = flags.switch("drive");

    // Mid-travel for every joint, so a drive request passes its preflight
    // wherever the simulated axis happens to start.
    let centres: Vec<f32> = config
        .joints
        .iter()
        .map(|joint| ((joint.limits.min_rad + joint.limits.max_rad) * 0.5) as f32)
        .collect();

    loop {
        let now = Instant::now();
        if seconds > 0 && now.duration_since(started) >= Duration::from_secs(seconds) {
            break;
        }
        if drive {
            let t = now.duration_since(started).as_secs_f32();
            let mut cmd = LowCmd::EMPTY;
            seq = seq.wrapping_add(1).max(1);
            cmd.seq = seq;
            cmd.n = config.joints.len() as u16;
            for (i, motor) in cmd.cmd.iter_mut().enumerate().take(config.joints.len()) {
                *motor = MotorCmd {
                    mode: mode::DRIVE,
                    q: centres[i] + 0.2 * (t * std::f32::consts::TAU * 0.5).sin(),
                    ..MotorCmd::OFF
                };
            }
            handle.lock().unwrap().send(cmd);
        }
        if now >= next_report {
            let (state, late_max_ns) = {
                let mut handle = handle.lock().unwrap();
                (handle.state(), handle.late_max_ns())
            };
            let elapsed = now.duration_since(last_report).as_secs_f64();
            let rate = (state.tick - last_tick) as f64 / elapsed;
            last_tick = state.tick;
            last_report = now;
            next_report += Duration::from_secs(report_every.max(1));

            let mut ok = 0;
            let mut off = 0;
            let mut silent = 0;
            let mut other = 0;
            for s in state.active() {
                match AxisStatus::from_u8(s.status) {
                    Some(AxisStatus::Ok) | Some(AxisStatus::Alert) => ok += 1,
                    Some(AxisStatus::Off) => off += 1,
                    Some(AxisStatus::Silent) => silent += 1,
                    _ => other += 1,
                }
            }
            let first = state.active().first().copied();
            println!(
                "t={:>5.0}s  {:>6.1} Hz  tick {:>8}  miss {:>4}  late_max {:>6.2} ms  estop {:<2} axes ok {ok} off {off} silent {silent} other {other}{}",
                now.duration_since(started).as_secs_f64(),
                rate,
                state.tick,
                state.deadline_miss,
                late_max_ns as f64 / 1e6,
                if state.estop != 0 {
                    state.estop_code.to_string()
                } else {
                    "-".into()
                },
                first
                    .map(|s| format!("  j0 q={:+.3} blocked={}", s.q, s.blocked))
                    .unwrap_or_default()
            );
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    server.stop();
    let handle = Arc::try_unwrap(handle)
        .map_err(|_| "the UDP server still holds the runtime".to_string())?
        .into_inner()
        .map_err(|_| "the runtime handle was poisoned".to_string())?;
    let mut handle = handle;
    let state = handle.state();
    println!(
        "stopping after {} ticks, {} deadline misses, late_max {:.2} ms{}",
        state.tick,
        state.deadline_miss,
        handle.late_max_ns() as f64 / 1e6,
        if state.estop != 0 {
            format!(", e-stop latched (code {})", state.estop_code)
        } else {
            String::new()
        }
    );
    handle.stop();
    Ok(())
}

fn factory(
    bus: &drlabs_rt::BusConfig,
    joints: &[&drlabs_rt::JointConfig],
) -> Result<Box<dyn drlabs_core::Bus>, String> {
    match bus.transport.as_str() {
        "sim" => drlabs_rt::daemon::sim_factory(bus, joints),
        "serial" => {
            if let Some(joint) = joints.iter().find(|joint| joint.driver != "dynamixel") {
                return Err(format!(
                    "bus {:?}: joint {:?} is a {:?}; a serial bus carries dynamixel joints only",
                    bus.name, joint.name, joint.driver
                ));
            }
            if bus.port.is_empty() {
                return Err(format!("bus {:?}: a serial bus needs a port", bus.name));
            }
            let baud = if bus.baud == 0 { 1_000_000 } else { bus.baud };
            let port = drlabs_transport::serial::open(&bus.port, baud)
                .map_err(|error| format!("bus {:?}: open {}: {error}", bus.name, bus.port))?;
            let shared = drlabs_transport::serial::SharedPort::new(Box::new(port));
            let mut servos = Vec::with_capacity(joints.len());
            for joint in joints {
                let profile = match joint.options.get("profile") {
                    Some(name) => Some(
                        drlabs_dynamixel::profile::profile_named(name)
                            .ok_or_else(|| format!("joint {:?}: unknown profile {name:?}", joint.name))?,
                    ),
                    None => None,
                };
                servos.push((joint.id as u8, profile));
            }
            let period = Duration::from_secs_f64(1.0 / bus.rate_hz.max(1) as f64);
            Ok(Box::new(
                drlabs_dynamixel::DynamixelBus::new(shared, &servos)
                    .with_port_name(&bus.port, baud)
                    .with_period(period),
            ))
        }
        other => Err(format!(
            "bus {:?}: transport {other:?} is not carried by the runtime yet",
            bus.name
        )),
    }
}

fn print_plan(config: &Config) {
    println!(
        "{}: {} joints on {} bus(es) at {} Hz, cmd timeout {} ms ({}), deadline trip after {} misses",
        config.robot,
        config.joints.len(),
        config.bus_names().len(),
        config.rate_hz,
        config.api.cmd_timeout_ms,
        config.api.on_cmd_timeout,
        config.rt.deadline_miss_trip
    );
    for name in config.bus_names() {
        let transport = config
            .bus(&name)
            .map(|bus| bus.transport.clone())
            .unwrap_or_else(|| "(implicit)".into());
        let members: Vec<&str> = config
            .joints
            .iter()
            .filter(|joint| joint.bus == name)
            .map(|joint| joint.name.as_str())
            .collect();
        println!(
            "  bus {name:?} {transport} @ {} Hz: {} joints: {}",
            config.rate_for_bus(&name),
            members.len(),
            members.join(" ")
        );
    }
}
