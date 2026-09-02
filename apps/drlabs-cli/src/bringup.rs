//! Bringing a whole machine up from its description.
//!
//! This is the subcommand the single-binary idea is about: a robot is a JSON
//! file and an executable, and running this holds every joint under control
//! until somebody stops it.

use std::time::{Duration, Instant};

use drlabs_robot::{Config, Robot};

use crate::flags::{Flags, Kind};

const BRINGUP_FLAGS: &[(&str, Kind)] = &[
    ("check", Kind::Switch),
    ("seconds", Kind::Value),
    ("sweep", Kind::Value),
    ("flood", Kind::Switch),
];

pub fn bringup(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, BRINGUP_FLAGS).map_err(|error| error.to_string())?;
    let path = match flags.positional.first() {
        Some(path) => path.clone(),
        None => return Err("usage: drlabs bringup [flags] <robot.json>".into()),
    };
    let seconds: f64 = flags.number("seconds", 0.0)?;
    // A fraction of each joint's own travel, so one number suits a machine
    // whose joints do not share an envelope.
    let sweep: f64 = flags.number("sweep", 0.0)?;
    let flood = flags.switch("flood");

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

    // Bringing a machine up is switching it on, not telling it what to do.
    //
    // Unless this run was asked for a sweep or a flood, every joint goes quiet
    // once it is energised: torque on, holding the position it was already in,
    // and no further target from here. Whatever drives the robot afterwards —
    // the panel, a program — is then the only thing that decides where it
    // goes, and `bringup` cannot be the reason a joint moved.
    if sweep <= 0.0 && !flood {
        robot
            .hold_where_they_are()
            .map_err(|error| error.to_string())?;
    }

    // Let the loops run long enough to have a rate worth reporting; the
    // measurement is over a window, and there is no window yet.
    std::thread::sleep(Duration::from_millis(500));
    print_state(&robot);

    println!("\nrobot {name:?} live · {joint_count} joints · {rate} Hz");
    if sweep <= 0.0 && !flood {
        println!("  energised and holding · no position commands are being sent");
    }
    if seconds > 0.0 {
        println!("  holding for {seconds:.0} s");
    } else {
        println!("  Ctrl+C to stop");
    }

    // A coordinated sweep drives every joint at once, which is the only way to
    // find out whether they interfere. On this bench that is exactly what the
    // Go version's two worst defects needed: one joint's commands starving the
    // other's bus, and a pose that reached some joints and not others.
    let sweep_targets: Vec<(String, f64, f64)> = if sweep > 0.0 {
        robot
            .config
            .joints
            .iter()
            .map(|joint| {
                let centre = (joint.limits.min_rad + joint.limits.max_rad) / 2.0;
                let half = (joint.limits.max_rad - joint.limits.min_rad) / 2.0;
                (joint.name.clone(), centre, half * sweep.clamp(0.0, 1.0))
            })
            .collect()
    } else {
        Vec::new()
    };
    if !sweep_targets.is_empty() {
        println!(
            "
Sweeping {} joints…",
            sweep_targets.len()
        );
        if flood {
            println!("  commanding as fast as this thread can, to test the rate ceiling");
        }
    }

    let started = Instant::now();
    loop {
        if sweep_targets.is_empty() {
            std::thread::sleep(Duration::from_millis(500));
        } else {
            // Either one pose per 20 ms, or as fast as the thread manages —
            // which is what a slider drag looks like from the loop's side.
            let deadline = Instant::now() + Duration::from_millis(if flood { 500 } else { 20 });
            loop {
                let phase = started.elapsed().as_secs_f64() * std::f64::consts::TAU / 4.0;
                let pose: std::collections::BTreeMap<String, drlabs_core::units::Rad> =
                    sweep_targets
                        .iter()
                        .map(|(name, centre, amplitude)| {
                            (
                                name.clone(),
                                drlabs_core::units::Rad(centre + amplitude * phase.sin()),
                            )
                        })
                        .collect();
                if let Err(error) = robot.set_positions(&pose) {
                    println!(
                        "
pose refused: {error}"
                    );
                    break;
                }
                if !flood || Instant::now() >= deadline {
                    break;
                }
            }
            if !flood {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
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
        if sweep_targets.is_empty() {
            continue;
        }
        // While sweeping, report often enough to watch both joints move.
        if started.elapsed().as_millis() % 1000 < 25 {
            let state = robot.state();
            let line: Vec<String> = state
                .iter()
                .map(|joint| {
                    format!(
                        "{} {:+.3} rad @ {:.0} Hz",
                        joint.name,
                        joint.position_rad.unwrap_or(f64::NAN),
                        joint.rate_hz
                    )
                })
                .collect();
            println!("  {}", line.join("   "));
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

    // The load line is the one worth reading. A CAN joint costs a transaction
    // per cycle, so a CAN bus carries joints × rate; a Dynamixel chain is
    // cycled as a whole and costs two a period however many servos are on it.
    // A number the adapter cannot carry is better seen here than diagnosed
    // later through a machine that will not keep time.
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
