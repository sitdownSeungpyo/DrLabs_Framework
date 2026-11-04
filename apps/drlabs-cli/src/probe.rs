//! The single-axis subcommands: what is plugged in, what it is, and driving it.

use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::link::Link;
use drlabs_core::trajectory::Trapezoidal;
use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};
use drlabs_dynamixel::DynamixelLink;
use drlabs_moteus::link::MoteusLink;
use drlabs_odrive::OdriveLink;
use drlabs_robstride::RobstrideLink;
use drlabs_rt::Config;
use drlabs_transport::can::SharedBus;
use drlabs_transport::serial::SharedPort;

use crate::flags::{Flags, Kind};
use crate::machine::{self, Commander};
use crate::{Target, COMMON_FLAGS};

/// Lists what is plugged in, adapters first.
pub fn ports() -> Result<(), String> {
    let ports = drlabs_transport::serial::list().map_err(|error| error.to_string())?;
    if ports.is_empty() {
        println!("No serial ports found.");
        return Ok(());
    }
    println!("{:<24}{:<10}DESCRIPTION", "PORT", "KIND");
    for port in ports {
        let kind = if port.is_can_adapter { "CAN" } else { "serial" };
        println!("{:<24}{:<10}{}", port.path, kind, port.description);
    }
    Ok(())
}

/// Lists the drivers this binary carries.
///
/// A list rather than a registry, because in a single binary the answer is
/// fixed at build time: what is compiled in is what this executable can drive.
pub fn drivers() -> Result<(), String> {
    println!("{:<12}HARDWARE", "DRIVER");
    for (name, description) in [
        ("moteus", "mjbots moteus brushless controller over CAN-FD"),
        (
            "dynamixel",
            "ROBOTIS Dynamixel servo over Protocol 2.0 serial",
        ),
        ("robstride", "RobStride RS00-RS06 actuator over CAN 2.0B"),
        (
            "odrive",
            "ODrive over CANSimple (protocol only, never bench tested)",
        ),
    ] {
        println!("{name:<12}{description}");
    }
    Ok(())
}

/// Builds a link for one axis from the command line.
pub fn link_for(target: &Target) -> Result<Box<dyn Link>, String> {
    match target.driver.as_str() {
        // A motor that is not there, so the whole stack — state machine,
        // control loop, safety monitor — runs with nothing plugged in.
        "sim" | "moteus-sim" | "robstride-sim" => {
            let physics = drlabs_sim::Physics {
                inertia: parse_option(target, "inertia", 0.006)?,
                friction: parse_option(target, "friction", 0.02)?,
                torque_constant: parse_option(target, "torque_constant", 0.06)?,
                ..Default::default()
            };
            if physics.inertia <= 0.0 || physics.torque_constant <= 0.0 {
                return Err("inertia and torque_constant must be positive".into());
            }
            let kind = match target.driver.as_str() {
                "moteus-sim" => "moteus-sim",
                "robstride-sim" => "robstride-sim",
                _ => "sim",
            };
            Ok(Box::new(drlabs_sim::SimLink::with_physics(
                target.id, kind, physics,
            )))
        }
        "moteus" | "robstride" | "odrive" => {
            let port = match target.port.as_str() {
                "" => auto_detect()?,
                port => port.to_string(),
            };
            let device =
                drlabs_transport::fdcanusb::open(&port).map_err(|error| error.to_string())?;
            let bus = Arc::new(SharedBus::new(Box::new(device)));

            Ok(match target.driver.as_str() {
                "moteus" => Box::new(MoteusLink::new(bus, target.id)),
                "odrive" => Box::new(OdriveLink::new(bus, target.id)),
                _ => {
                    let name = target.options.get("model").ok_or(
                        "robstride needs -opt model=RS01; the scaling differs by up to a factor \
                         of twenty across the family",
                    )?;
                    let model = drlabs_robstride::lookup_model(name)
                        .ok_or_else(|| format!("unknown robstride model {name:?}"))?;
                    Box::new(RobstrideLink::new(bus, target.id, model))
                }
            })
        }
        "dynamixel" => {
            if target.port.is_empty() {
                return Err("dynamixel needs -port".into());
            }
            let port = drlabs_transport::serial::open(&target.port, target.baud)
                .map_err(|error| error.to_string())?;
            let shared = SharedPort::new(Box::new(port));
            let profile = match target.options.get("profile") {
                Some(name) => Some(
                    drlabs_dynamixel::profile::profile_named(name)
                        .ok_or_else(|| format!("unknown dynamixel profile {name:?}"))?,
                ),
                None => None,
            };
            Ok(Box::new(
                DynamixelLink::new(shared, target.id, profile)
                    .with_port_name(&target.port, target.baud),
            ))
        }
        other => Err(format!("unknown driver {other:?}")),
    }
}

/// A numeric driver option, or its default.
fn parse_option(target: &Target, name: &str, default: f64) -> Result<f64, String> {
    match target.options.get(name) {
        Some(text) => text
            .parse()
            .map_err(|_| format!("option {name}={text:?} is not a number")),
        None => Ok(default),
    }
}

/// Picks the most likely CAN adapter when none was named.
///
/// Refusing when nothing identified itself is better than opening whatever
/// happens to be first — that could be a modem, a GPS, or another machine's
/// console.
pub fn auto_detect() -> Result<String, String> {
    let ports = drlabs_transport::serial::list().map_err(|error| error.to_string())?;
    match ports.first() {
        Some(port) if port.is_can_adapter => Ok(port.path.clone()),
        Some(_) => {
            let names: Vec<&str> = ports.iter().map(|port| port.path.as_str()).collect();
            Err(format!(
                "no CAN adapter recognised among {names:?}; name the port with -port"
            ))
        }
        None => Err("no serial ports found; plug in an adapter or name the port with -port".into()),
    }
}

/// Connects, identifies, prints what came back, and disconnects.
pub fn probe(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, COMMON_FLAGS).map_err(|error| error.to_string())?;
    let target = Target::from(&flags)?;

    let mut link = link_for(&target)?;
    link.open().map_err(|error| error.to_string())?;

    println!("{:<14}{}", "DRIVER", link.kind());
    for (key, value) in link.describe() {
        println!("{key:<14}{value}");
    }

    match link.identify() {
        Ok(Some(identity)) => {
            println!("{:<14}{}", "BOARD", identity.board);
            println!("{:<14}{}", "STATUS", identity.status.as_str());
            if !identity.firmware.is_empty() {
                println!("{:<14}{}", "FIRMWARE", identity.firmware);
            }
            if !identity.serial.is_empty() {
                println!("{:<14}{}", "SERIAL", identity.serial);
            }
            if let Some((lower, upper)) = identity.bounds {
                println!(
                    "{:<14}{:.4} … {:.4} rad",
                    "TRAVEL",
                    lower.to_rad().value(),
                    upper.to_rad().value()
                );
            }
        }
        Ok(None) => println!("{:<14}reports nothing identifying", "BOARD"),
        Err(error) => println!("{:<14}could not be read: {error}", "BOARD"),
    }

    if let Some(envelope) = link.envelope() {
        if let (Some(min), Some(max)) = (envelope.voltage_min, envelope.voltage_max) {
            println!("{:<14}{} … {}", "SUPPLY", min, max);
        }
    }

    // One cycle, so the axis is described rather than only named.
    let command = drlabs_core::command::Command::new(drlabs_core::command::Mode::Query);
    match link.cycle(&command, drlabs_core::command::QueryProfile::Full) {
        Ok(Some(telemetry)) => {
            println!();
            print_telemetry(&telemetry);
        }
        Ok(None) => println!("\nNo telemetry: the controller did not answer."),
        Err(error) => println!("\nNo telemetry: {error}"),
    }

    link.close().map_err(|error| error.to_string())
}

fn print_telemetry(telemetry: &drlabs_core::telemetry::Telemetry) {
    let show = |label: &str, value: Option<String>| {
        println!("{label:<14}{}", value.unwrap_or_else(|| "—".into()));
    };
    show(
        "MODE",
        Some(telemetry.mode_name.clone()).filter(|s| !s.is_empty()),
    );
    show("POSITION", telemetry.position.map(|v| format!("{v:.4}")));
    show("VELOCITY", telemetry.velocity.map(|v| format!("{v:.3}")));
    show("TORQUE", telemetry.torque.map(|v| format!("{v:.3}")));
    show("CURRENT", telemetry.q_current.map(|v| format!("{v:.2}")));
    show("VOLTAGE", telemetry.voltage.map(|v| format!("{v:.1}")));
    show(
        "BOARD TEMP",
        telemetry.board_temperature.map(|v| format!("{v:.0}")),
    );
    if let Some(fault) = &telemetry.fault {
        show("FAULT", Some(format!("{} ({})", fault.name, fault.code)));
        if !fault.detail.is_empty() {
            println!("              {}", fault.detail);
        }
    }
}

const RUN_FLAGS: &[(&str, Kind)] = &[
    ("rate", Kind::Value),
    ("seconds", Kind::Value),
    ("position", Kind::Value),
    ("shuttle", Kind::Value),
    ("min", Kind::Value),
    ("max", Kind::Value),
    ("torque", Kind::Value),
    ("velocity", Kind::Value),
    ("accel", Kind::Value),
];

/// How often this client puts out a setpoint. Independent of the bus rate:
/// the runtime ticks at its own rate and takes the latest.
const CLIENT_RATE_HZ: f64 = 100.0;

/// Drives one axis through the runtime.
///
/// The axis becomes a one-joint machine: a description built from the
/// flags, the runtime started in this process, and this function its only
/// client — planning the move, streaming setpoints, reading the state
/// record back. Nothing here can do to the motor what a client of
/// `drlabs rt` could not.
pub fn run(args: &[String]) -> Result<(), String> {
    let known: Vec<(&str, Kind)> = COMMON_FLAGS.iter().chain(RUN_FLAGS).copied().collect();
    let flags = Flags::parse(args, &known).map_err(|error| error.to_string())?;
    let target = Target::from(&flags)?;

    let rate: u32 = flags.number("rate", 100)?;
    let seconds: f64 = flags.number("seconds", 0.0)?;
    let position: Option<f64> = match flags.string("position", "") {
        text if text.is_empty() => None,
        text => Some(
            text.parse()
                .map_err(|_| format!("-position {text:?} is not a number"))?,
        ),
    };
    let shuttle = flags.string("shuttle", "");

    // The travel envelope is the operator's, not the driver's, so it has to
    // be sayable on the command line, in radians at the shaft. A default of
    // ±π (half a turn) is right for a first power-up and wrong for an axis
    // that is parked anywhere else — and a motor that cannot energise
    // because of a default nobody chose is the sort of thing people work
    // around by disabling the check.
    let min: f64 = flags.number("min", -std::f64::consts::PI)?;
    let max: f64 = flags.number("max", std::f64::consts::PI)?;
    let torque: f64 = flags.number("torque", 0.3)?;
    let velocity: f64 = flags.number("velocity", 6.0)?;
    let accel: f64 = flags.number("accel", 10.0)?;

    let (transport, driver) = match target.driver.as_str() {
        "sim" | "moteus-sim" | "robstride-sim" => ("sim", target.driver.as_str()),
        "moteus" => ("fdcanusb", "moteus"),
        "dynamixel" => ("serial", "dynamixel"),
        "robstride" | "odrive" => {
            return Err(format!(
                "{} has no bus driver yet; `drlabs probe` still identifies it",
                target.driver
            ))
        }
        other => return Err(format!("unknown driver {other:?}")),
    };
    let description = serde_json::json!({
        "robot": "run",
        "rate_hz": rate,
        "buses": [{
            "name": "bus",
            "transport": transport,
            "port": target.port,
            "baud": target.baud,
        }],
        "joints": [{
            "name": "axis",
            "bus": "bus",
            "driver": driver,
            "id": target.id,
            "options": target.options,
            "gear_ratio": 1.0,
            "direction": 1,
            "offset_rad": 0.0,
            "limits": {
                "min_rad": min,
                "max_rad": max,
                "max_torque_nm": torque,
                "max_velocity_rad_s": velocity,
                "max_accel_rad_s2": accel,
            },
        }],
    });
    let config = Config::parse(&description.to_string()).map_err(|error| error.to_string())?;

    // What this run will do, decided before anything is opened.
    let plan = if !shuttle.is_empty() {
        let parts: Vec<&str> = shuttle.split(',').collect();
        if parts.len() != 3 {
            return Err("-shuttle takes A,B,dwell — three comma-separated numbers".into());
        }
        let parse = |text: &str| -> Result<f64, String> {
            text.trim()
                .parse()
                .map_err(|_| format!("-shuttle {text:?} is not a number"))
        };
        Plan::Shuttle {
            a: parse(parts[0])?.clamp(min, max),
            b: parse(parts[1])?.clamp(min, max),
            dwell: parse(parts[2])?.max(0.0),
        }
    } else if let Some(target) = position {
        Plan::MoveTo(target.clamp(min, max))
    } else {
        Plan::Hold
    };

    let mut handle = machine::start(&config)?;
    let mut commander = Commander::new(1);
    let first = match machine::wait_for_telemetry(&mut handle, &config, Duration::from_secs(5)) {
        Ok(state) => state,
        Err(error) => {
            machine::shutdown(handle, &mut commander);
            return Err(error);
        }
    };
    let start_q = first.state[0].q as f64;

    // Energise where the axis is, and only then go anywhere.
    commander.drive(0, start_q, 0.0);
    if let Err(error) =
        machine::wait_until_driving(&mut handle, &config, &mut commander, Duration::from_secs(5))
    {
        machine::shutdown(handle, &mut commander);
        return Err(format!("the axis {error}"));
    }
    println!(
        "operation enabled at {start_q:+.4} rad · {} · bus {} Hz · {}",
        target.driver,
        rate,
        match plan {
            Plan::Hold => "holding".to_string(),
            Plan::MoveTo(q) => format!("moving to {q:+.4} rad"),
            Plan::Shuttle { a, b, dwell } =>
                format!("shuttling {a:+.4} ↔ {b:+.4} rad, dwell {dwell} s"),
        }
    );

    let mut mover = Mover::new(start_q, velocity, accel);
    let mut shuttle_next = match plan {
        Plan::Shuttle { a, .. } => {
            mover.start(a);
            Some(false)
        }
        Plan::MoveTo(q) => {
            mover.start(q);
            None
        }
        Plan::Hold => None,
    };
    let mut dwell_until: Option<Instant> = None;

    let started = Instant::now();
    let mut timing = machine::Timing::new(&first);
    let mut next_report = started + Duration::from_millis(200);
    let period = Duration::from_secs_f64(1.0 / CLIENT_RATE_HZ);
    let mut next = Instant::now();
    let outcome = loop {
        next += period;
        std::thread::sleep(next.saturating_duration_since(Instant::now()));
        let now = Instant::now();

        let (q, dq) = mover.sample(now);
        commander.drive(0, q, dq);
        commander.send(&mut handle, 0);

        if let Plan::Shuttle { a, b, dwell } = plan {
            if mover.finished(now) {
                match dwell_until {
                    None => dwell_until = Some(now + Duration::from_secs_f64(dwell)),
                    Some(until) if now >= until => {
                        let to_b = !shuttle_next.unwrap_or(false);
                        mover.start(if to_b { b } else { a });
                        shuttle_next = Some(to_b);
                        dwell_until = None;
                    }
                    Some(_) => {}
                }
            }
        }

        let state = handle.state();
        if now >= next_report {
            next_report += Duration::from_millis(200);
            let motor = state.state[0];
            let rate = timing.rate(&state);
            print!(
                "\r{:>8.4} rad  {:>7.3} rad/s  {:>6.2} A  {:>5.1} V  {:>5.0} Hz  miss {:>3}  late {:>5.2} ms  {:<8}{}   ",
                motor.q,
                motor.dq,
                motor.current,
                motor.voltage,
                rate,
                state.deadline_miss,
                handle.late_max_ns() as f64 / 1e6,
                machine::status_name(motor.status),
                if motor.blocked != 0 {
                    format!(" ({})", machine::blocked_name(motor.blocked))
                } else {
                    String::new()
                }
            );
            use std::io::Write;
            let _ = std::io::stdout().flush();
        }
        if state.estop != 0 {
            println!("\n\nEmergency stop: {}", machine::trip_name(state.estop_code));
            break Err(format!("stopped: {}", machine::trip_name(state.estop_code)));
        }
        if seconds > 0.0 && started.elapsed().as_secs_f64() >= seconds {
            println!();
            break Ok(());
        }
    };

    machine::shutdown(handle, &mut commander);
    outcome
}

#[derive(Clone, Copy)]
enum Plan {
    Hold,
    MoveTo(f64),
    Shuttle { a: f64, b: f64, dwell: f64 },
}

/// One trapezoid at a time, in radians, from wherever the last one ended.
struct Mover {
    at: f64,
    max_velocity: f64,
    accel: f64,
    profile: Option<(Trapezoidal, Instant)>,
}

impl Mover {
    fn new(at: f64, max_velocity: f64, accel: f64) -> Self {
        Self {
            at,
            max_velocity,
            accel,
            profile: None,
        }
    }

    fn start(&mut self, target: f64) {
        // The trapezoid is unit-agnostic arithmetic; it is used here in
        // radians, as the panel does.
        if let Ok(profile) = Trapezoidal::new(
            Rev(self.at),
            Rev(target),
            RevPerSec(self.max_velocity),
            RevPerSec2(self.accel),
        ) {
            self.profile = Some((profile, Instant::now()));
        }
    }

    /// Position and velocity feed-forward now.
    fn sample(&mut self, now: Instant) -> (f64, f64) {
        match &self.profile {
            Some((profile, started)) => {
                let point = profile.sample(now.duration_since(*started).as_secs_f64());
                self.at = point.position.value();
                (self.at, point.velocity.value())
            }
            None => (self.at, 0.0),
        }
    }

    fn finished(&self, now: Instant) -> bool {
        match &self.profile {
            Some((profile, started)) => {
                now.duration_since(*started).as_secs_f64() >= profile.total_time()
            }
            None => true,
        }
    }
}
