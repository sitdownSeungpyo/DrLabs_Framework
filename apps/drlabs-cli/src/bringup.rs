//! Bringing a whole machine up from its description.
//!
//! This is the subcommand the single-binary idea is about: a robot is a JSON
//! file and an executable, and running this holds every joint under control
//! until somebody stops it. It is a client of the runtime it starts: the
//! buses belong to `drlabs-rt`, and this process only says where the joints
//! should be.

use std::time::{Duration, Instant};

use drlabs_rt::Config;

use crate::flags::{Flags, Kind};
use crate::machine::{self, Commander};

const BRINGUP_FLAGS: &[(&str, Kind)] = &[
    ("check", Kind::Switch),
    ("seconds", Kind::Value),
    ("sweep", Kind::Value),
    ("flood", Kind::Switch),
];

/// How often the command record is re-sent while nothing changes, so the
/// runtime never sees it go stale (`api.cmd_timeout_ms`, 100 by default).
const KEEPALIVE: Duration = Duration::from_millis(20);

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
    let name = config.robot.clone();
    println!("\nBringing up {joint_count} joints…");

    let mut handle = machine::start(&config)?;
    let mut commander = Commander::new(joint_count);

    // Every joint has to see at least one telemetry cycle before its
    // preflight can pass, and on a shared chain those cycles are taken in
    // turn — so the window grows with the machine.
    let budget = Duration::from_millis(5_000 + 500 * joint_count as u64);
    let first = match machine::wait_for_telemetry(&mut handle, &config, budget) {
        Ok(state) => state,
        Err(error) => {
            machine::shutdown(handle, &mut commander);
            return Err(error);
        }
    };

    // Bringing a machine up is switching it on, not telling it what to do:
    // every joint is energised where it already is, and no further target
    // comes from here unless a sweep was asked for. Whatever drives the
    // robot afterwards is then the only thing that decides where it goes.
    //
    // All or nothing. A humanoid with one dead shoulder is more dangerous
    // than one that never came up: the rest of the machine will happily
    // hold poses that assume the missing joint is holding too. So a joint
    // that cannot be energised takes the whole robot down, torque off, and
    // says which one and why.
    for index in 0..joint_count {
        commander.drive(index, first.state[index].q as f64, 0.0);
    }
    if let Err(error) = machine::wait_until_driving(&mut handle, &config, &mut commander, budget)
    {
        let state = handle.state();
        println!();
        print!("{}", machine::state_table(&config, &state));
        machine::shutdown(handle, &mut commander);
        return Err(format!("joints {error}"));
    }

    let state = handle.state();
    println!();
    print!("{}", machine::state_table(&config, &state));
    println!("\nrobot {name:?} live · {joint_count} joints · {} Hz", config.rate_hz);
    if sweep <= 0.0 && !flood {
        println!("  energised and holding · no position commands are being sent");
    }
    if seconds > 0.0 {
        println!("  holding for {seconds:.0} s");
    } else {
        println!("  Ctrl+C to stop");
    }

    // A coordinated sweep drives every joint at once, which is the only way
    // to find out whether they interfere: one joint's commands starving the
    // other's bus, or a pose that reaches some joints and not others.
    let sweep_targets: Vec<(usize, f64, f64)> = if sweep > 0.0 {
        config
            .joints
            .iter()
            .enumerate()
            .map(|(index, joint)| {
                let centre = (joint.limits.min_rad + joint.limits.max_rad) / 2.0;
                let half = (joint.limits.max_rad - joint.limits.min_rad) / 2.0;
                (index, centre, half * sweep.clamp(0.0, 1.0))
            })
            .collect()
    } else {
        Vec::new()
    };
    if !sweep_targets.is_empty() {
        println!("\nSweeping {} joints…", sweep_targets.len());
        if flood {
            println!("  commanding as fast as this thread can; the bus keeps its own rate");
        }
    }

    let started = Instant::now();
    let mut timing = machine::Timing::new(&state);
    let mut next_report = started + Duration::from_secs(1);
    let mut sends = 0u64;
    let outcome = loop {
        if sweep_targets.is_empty() {
            std::thread::sleep(KEEPALIVE);
            commander.send(&mut handle, 0);
            sends += 1;
        } else {
            // Either one pose per 20 ms, or as fast as the thread manages —
            // which is what a slider drag looks like from the runtime's side.
            let deadline = Instant::now() + if flood { Duration::from_millis(500) } else { KEEPALIVE };
            loop {
                let phase = started.elapsed().as_secs_f64() * std::f64::consts::TAU / 4.0;
                for (index, centre, amplitude) in &sweep_targets {
                    commander.drive(*index, centre + amplitude * phase.sin(), 0.0);
                }
                commander.send(&mut handle, 0);
                sends += 1;
                if !flood || Instant::now() >= deadline {
                    break;
                }
            }
            if !flood {
                std::thread::sleep(KEEPALIVE);
            }
        }

        let state = handle.state();
        if state.estop != 0 {
            println!(
                "\nEmergency stop: {}",
                machine::trip_name(state.estop_code)
            );
            print!("{}", machine::state_table(&config, &state));
            break Err(format!("stopped: {}", machine::trip_name(state.estop_code)));
        }
        if seconds > 0.0 && started.elapsed().as_secs_f64() >= seconds {
            print!("{}", machine::state_table(&config, &state));
            break Ok(());
        }
        let now = Instant::now();
        if now >= next_report {
            next_report += Duration::from_secs(1);
            let rate = timing.rate(&state);
            let elapsed = started.elapsed().as_secs_f64();
            let mut line = format!(
                "t={:>4.0}s  bus {:>6.1} Hz  client {:>6.0}/s  miss {:>3}  late_max {:>5.2} ms",
                elapsed,
                rate,
                sends as f64 / elapsed,
                state.deadline_miss,
                handle.late_max_ns() as f64 / 1e6
            );
            if !sweep_targets.is_empty() {
                for (index, joint) in config.joints.iter().enumerate().take(4) {
                    line.push_str(&format!("  {} {:+.3}", joint.name, state.state[index].q));
                }
                if joint_count > 4 {
                    line.push_str(&format!("  … {} more", joint_count - 4));
                }
            }
            println!("{line}");
        }
    };

    println!("\nStopping…");
    machine::shutdown(handle, &mut commander);
    outcome
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

    // The load line is the one worth reading. A CAN joint costs a
    // transaction per cycle, so a CAN bus carries joints × rate; a Dynamixel
    // chain is cycled as a whole and costs two a period however many servos
    // are on it. A number the adapter cannot carry is better seen here than
    // diagnosed later through a machine that will not keep time.
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
