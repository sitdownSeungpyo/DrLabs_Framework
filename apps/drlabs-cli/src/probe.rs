//! The single-axis subcommands: what is plugged in, what it is, and driving it.

use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::command::ControlMode;
use drlabs_core::link::Link;
use drlabs_core::units::Rev;
use drlabs_dynamixel::DynamixelLink;
use drlabs_moteus::link::MoteusLink;
use drlabs_odrive::OdriveLink;
use drlabs_robstride::RobstrideLink;
use drlabs_session::{Options, Session};
use drlabs_transport::can::SharedBus;
use drlabs_transport::serial::SharedPort;

use crate::flags::{Flags, Kind};
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

/// Picks the most likely CAN adapter when none was named.
///
/// Refusing when nothing identified itself is better than opening whatever
/// happens to be first — that could be a modem, a GPS, or another machine's
/// console.
fn auto_detect() -> Result<String, String> {
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
                    "{:<14}{:.4} … {:.4} rev",
                    "TRAVEL",
                    lower.value(),
                    upper.value()
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
];

/// Runs a control session on one axis.
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

    let mut link = link_for(&target)?;
    link.open().map_err(|error| error.to_string())?;

    let session = Session::new(Options {
        rate_hz: rate,
        auto_arm: true,
        require_acknowledgement: false,
        ..Default::default()
    });
    session
        .connect(link, true)
        .map_err(|error| error.to_string())?;

    // Give the loop long enough to arm before anything is commanded.
    wait_until_armed(&session, Duration::from_secs(5))?;

    if !shuttle.is_empty() {
        let parts: Vec<&str> = shuttle.split(',').collect();
        if parts.len() != 3 {
            return Err("-shuttle takes A,B,dwell — three comma-separated numbers".into());
        }
        let parse = |text: &str| -> Result<f64, String> {
            text.trim()
                .parse()
                .map_err(|_| format!("-shuttle {text:?} is not a number"))
        };
        session.configure_shuttle(
            Rev(parse(parts[0])?),
            Rev(parse(parts[1])?),
            parse(parts[2])?,
            0,
        );
        session.start_shuttle().map_err(|error| error.to_string())?;
    } else if let Some(target) = position {
        session
            .set_mode(ControlMode::Position)
            .map_err(|e| e.to_string())?;
        session.set_position(Rev(target));
    }

    let started = Instant::now();
    let mut last_seq = 0u64;
    loop {
        std::thread::sleep(Duration::from_millis(200));
        session.heartbeat();

        for event in session.events(last_seq) {
            last_seq = event.seq;
            println!(
                "[{}] {} — {}",
                event.level.as_str(),
                event.message,
                event.detail
            );
        }

        let status = session.status();
        print!(
            "\r{:>8.4} rev  {:>7.3} rev/s  {:>6.2} A  {:>5.1} V  {:>4.0} Hz  {}   ",
            status
                .telemetry
                .position
                .map(|v| v.value())
                .unwrap_or(f64::NAN),
            status
                .telemetry
                .velocity
                .map(|v| v.value())
                .unwrap_or(f64::NAN),
            status
                .telemetry
                .q_current
                .map(|v| v.value())
                .unwrap_or(f64::NAN),
            status
                .telemetry
                .voltage
                .map(|v| v.value())
                .unwrap_or(f64::NAN),
            status.achieved_rate_hz,
            if status.armed { "armed" } else { "idle " }
        );
        use std::io::Write;
        let _ = std::io::stdout().flush();

        if let Some(estop) = &status.estop {
            println!("\n\nEmergency stop: {} — {}", estop.message, estop.detail);
            break;
        }
        if seconds > 0.0 && started.elapsed().as_secs_f64() >= seconds {
            println!();
            break;
        }
    }

    session.shutdown();
    Ok(())
}

fn wait_until_armed(session: &Arc<Session>, budget: Duration) -> Result<(), String> {
    let deadline = Instant::now() + budget;
    loop {
        let status = session.status();
        if status.armed {
            println!(
                "armed · {} · {:.0} Hz",
                status
                    .identity
                    .as_ref()
                    .map(|identity| identity.board.clone())
                    .unwrap_or_else(|| "unidentified".into()),
                status.achieved_rate_hz
            );
            return Ok(());
        }
        if Instant::now() >= deadline {
            let preflight = session.preflight();
            return Err(format!(
                "the axis did not energise: {}",
                preflight.blocking.join(" ")
            ));
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}
