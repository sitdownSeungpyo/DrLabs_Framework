//! Bringing a whole machine up from its description.
//!
//! This is the subcommand the single-binary idea is about: a robot is a JSON
//! file and an executable, and running this holds every joint under control
//! until somebody stops it.

use std::time::{Duration, Instant};

use drlabs_robot::{Config, Robot};

use crate::flags::{Flags, Kind};

const BRINGUP_FLAGS: &[(&str, Kind)] = &[("check", Kind::Switch), ("seconds", Kind::Value)];

pub fn bringup(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, BRINGUP_FLAGS).map_err(|error| error.to_string())?;
    let path = match flags.positional.first() {
        Some(path) => path.clone(),
        None => return Err("usage: drlabs bringup [flags] <robot.json>".into()),
    };
    let seconds: f64 = flags.number("seconds", 0.0)?;

    let config = Config::load(std::path::Path::new(&path)).map_err(|error| error.to_string())?;
    print_plan(&config);

    if flags.switch("check") {
        println!("\nDescription is valid. Nothing was opened.");
        return Ok(());
    }

    let joint_count = config.joints.len();
    let rate = config.rate_hz;
    let name = config.robot.clone();

    let mut robot = Robot::new(config).map_err(|error| error.to_string())?;
    println!("\nBringing up {joint_count} joints…");

    robot.bring_up().map_err(|error| error.to_string())?;

    // Let the loops run long enough to have a rate worth reporting; the
    // measurement is over a window, and there is no window yet.
    std::thread::sleep(Duration::from_millis(500));
    print_state(&robot);

    println!("\nrobot {name:?} live · {joint_count} joints · {rate} Hz");
    if seconds > 0.0 {
        println!("  holding for {seconds:.0} s");
    } else {
        println!("  Ctrl+C to stop");
    }

    let started = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(500));
        robot.heartbeat();

        // A joint that stopped itself is the whole machine's problem: the rest
        // of it will happily hold poses that assume the stopped one is holding
        // too.
        let stopped: Vec<String> = robot
            .state()
            .iter()
            .filter(|joint| joint.estop)
            .map(|joint| format!("{} ({})", joint.name, joint.estop_reason))
            .collect();
        if !stopped.is_empty() {
            println!("\nEmergency stop: {}", stopped.join(", "));
            print_state(&robot);
            break;
        }

        if seconds > 0.0 && started.elapsed().as_secs_f64() >= seconds {
            print_state(&robot);
            break;
        }
    }

    println!("\nStopping…");
    robot.shutdown();
    Ok(())
}

/// Says what the description asks for before anything is opened.
fn print_plan(config: &Config) {
    println!("{:<10}{}", "robot", config.robot);
    println!("{:<10}{} Hz", "rate", config.rate_hz);
    println!();

    println!(
        "{:<12}{:<12}{:<8}{:<5}{:<9}{:<5}{:<20}TORQUE",
        "JOINT", "DRIVER", "BUS", "ID", "GEAR", "DIR", "TRAVEL (rad)"
    );
    for joint in &config.joints {
        let gear = format!("{:.4}:1", joint.gear_ratio);
        let direction = format!("{:+.0}", joint.direction);
        let travel = format!("{:.3} … {:.3}", joint.limits.min_rad, joint.limits.max_rad);
        println!(
            "{:<12}{:<12}{:<8}{:<5}{gear:<9}{direction:<5}{travel:<20}{:.2} Nm",
            joint.name, joint.driver, joint.bus, joint.id, joint.limits.max_torque_nm
        );
    }

    if config.buses.is_empty() {
        return;
    }
    println!();
    println!(
        "{:<8}{:<12}{:<10}{:<10}LOAD",
        "BUS", "TRANSPORT", "PORT", "RATE"
    );
    let load = config.bus_load();
    for bus in &config.buses {
        let total = load
            .iter()
            .find(|(name, _)| name == &bus.name)
            .map(|(_, total)| *total)
            .unwrap_or(0);
        let port = if bus.port.is_empty() {
            "auto"
        } else {
            &bus.port
        };
        let rate = format!("{} Hz", config.rate_for_bus(&bus.name));
        let load = format!("{total} tx/s");
        println!(
            "{:<8}{:<12}{port:<10}{rate:<10}{load}",
            bus.name, bus.transport
        );
    }

    // The load line is the one worth reading: every joint on a bus costs a
    // transaction per cycle, so what matters is joints × rate. A number the
    // adapter cannot carry is better seen here than diagnosed later through a
    // machine that will not keep time.
    for (bus, total) in &load {
        if *total > 400 {
            println!(
                "\nnote: bus {bus:?} is asked for {total} transactions a second. A 1 Mbaud serial \
                 chain manages a few hundred; give that bus its own lower rate_hz if the loop \
                 cannot keep up."
            );
        }
    }
}

fn print_state(robot: &Robot) {
    println!();
    println!(
        "{:<12}{:<8}{:<12}{:<12}{:<10}{:<8}NOTE",
        "JOINT", "ARMED", "POSITION", "TARGET", "TORQUE", "RATE"
    );
    for joint in robot.state() {
        let position = joint
            .position_rad
            .map(|value| format!("{value:+.4} rad"))
            .unwrap_or_else(|| "—".into());
        let torque = joint
            .torque_nm
            .map(|value| format!("{value:+.3} Nm"))
            .unwrap_or_else(|| "—".into());
        let note = if joint.estop {
            joint.estop_reason.clone()
        } else if !joint.blocked.is_empty() {
            joint.blocked.join(" ")
        } else {
            joint.fault_name.clone()
        };
        let target = format!("{:+.4} rad", joint.target_rad);
        let rate = format!("{:.0} Hz", joint.rate_hz);
        println!(
            "{:<12}{:<8}{position:<12}{target:<12}{torque:<10}{rate:<8}{note}",
            joint.name,
            if joint.armed { "yes" } else { "no" }
        );
    }
}
