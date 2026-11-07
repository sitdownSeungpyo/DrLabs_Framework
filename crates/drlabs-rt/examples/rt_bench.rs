//! Bench: the runtime on a real Dynamixel chain, without DDS.
//!
//!   rt_bench <robot.json> [seconds=N] [drive] [amplitude=RAD]
//!                         [shuttle=RAD] [dwell=S] [vel=RAD_S] [acc=RAD_S2]
//!                         [monitor=PORT] [log=MS]
//!
//! The same tick path as `drlabs rt` — `Daemon`, `BusWorker`, `DynamixelBus`
//! — with the report loop of `apps/drlabs-cli/src/rt.rs` and no transport
//! out of the process. Exists so the loop can be measured on a machine that
//! cannot build CycloneDDS.
//!
//! * no motion flag: every axis stays OFF, the chain is read every tick.
//! * `drive`: every joint follows a 0.5 Hz sine of `amplitude` radians
//!   around where it was when the run started.
//! * `shuttle=RAD`: every joint moves between start−RAD and start+RAD on a
//!   time-synchronised trapezoid (`drlabs_core::trajectory::synchronized`,
//!   the planner `drlabs run -shuttle` uses), streamed at the bus rate with
//!   velocity feed-forward, resting `dwell` seconds at each end. Velocity
//!   and acceleration default to the joint's declared limits; `vel`/`acc`
//!   lower them for every joint.
//! * `seconds=0` (the default) runs until Enter is pressed on stdin, which
//!   stops through the same release path as a timed run. Ctrl+C does not:
//!   it kills the process with torque on and the bus watchdog armed.
//! * `monitor=PORT`: serves a live page on 127.0.0.1:PORT — one row per
//!   joint with position, setpoint, error, velocity, supply and temperature,
//!   and a strip chart — from a JSON snapshot at `/state`.
//!
//! The tracking error reported is setpoint minus the latest reported
//! position, sampled at the client's rate; it carries one bus period of
//! lag, which at 1 rad/s and 400 Hz is 0.0025 rad.
//!
//! The one-joint-at-a-time pattern lives in its own example, `wave`.

mod common;

use std::path::Path;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use common::{dump_axes, factory, serve, AxisSnap, Snapshot};
use drlabs_core::lowlevel::{mode, LowCmd, MotorCmd};
use drlabs_core::trajectory::{synchronized, Leg, Trapezoidal};
use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};
use drlabs_core::AxisStatus;
use drlabs_rt::{Config, Daemon};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(path) = args.first() else {
        eprintln!("usage: rt_bench <robot.json> [seconds=N] [drive] [amplitude=RAD] [shuttle=RAD] [dwell=S] [vel=RAD_S] [acc=RAD_S2] [monitor=PORT] [log=MS]");
        std::process::exit(2);
    };
    let mut seconds = 0u64;
    let mut drive = false;
    let mut amplitude = 0.2f32;
    let mut shuttle: Option<f64> = None;
    let mut dwell = 0.25f64;
    let mut vel_cap: Option<f64> = None;
    let mut acc_cap: Option<f64> = None;
    let mut monitor: Option<u16> = None;
    let mut log_ms = 0u64;
    for token in &args[1..] {
        if let Some(value) = token.strip_prefix("seconds=") {
            seconds = value.parse().expect("seconds=N");
        } else if token == "drive" {
            drive = true;
        } else if let Some(value) = token.strip_prefix("amplitude=") {
            amplitude = value.parse().expect("amplitude=RAD");
        } else if let Some(value) = token.strip_prefix("shuttle=") {
            shuttle = Some(value.parse().expect("shuttle=RAD"));
        } else if let Some(value) = token.strip_prefix("dwell=") {
            dwell = value.parse().expect("dwell=S");
        } else if let Some(value) = token.strip_prefix("vel=") {
            vel_cap = Some(value.parse().expect("vel=RAD_S"));
        } else if let Some(value) = token.strip_prefix("acc=") {
            acc_cap = Some(value.parse().expect("acc=RAD_S2"));
        } else if let Some(value) = token.strip_prefix("monitor=") {
            monitor = Some(value.parse().expect("monitor=PORT"));
        } else if let Some(value) = token.strip_prefix("log=") {
            log_ms = value.parse().expect("log=MS");
        } else {
            eprintln!("unknown argument {token:?}");
            std::process::exit(2);
        }
    }

    let config = match Config::load(Path::new(path)) {
        Ok(config) => config,
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(1);
        }
    };
    let n = config.joints.len();
    let names: Vec<String> = config.joints.iter().map(|j| j.name.clone()).collect();
    println!(
        "{}: {} joints at {} Hz, deadline trip after {} misses, drive={drive} shuttle={shuttle:?}",
        config.robot, n, config.rate_hz, config.rt.deadline_miss_trip
    );

    // Enter on stdin ends the run through the release path below; a timed
    // run (`seconds=N`) ends the same way. Ctrl+C would skip it.
    let stop = common::stop_on_enter(seconds);
    if seconds == 0 {
        println!("running until Enter is pressed");
    }

    let snapshot = Arc::new(Mutex::new(Snapshot {
        plan: if let Some(amp) = shuttle {
            format!("shuttle ±{amp} rad, dwell {dwell} s")
        } else if drive {
            format!("sine ±{amplitude} rad 0.5 Hz")
        } else {
            "observe".into()
        },
        axes: names
            .iter()
            .map(|name| AxisSnap {
                name: name.clone(),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    }));
    if let Some(port) = monitor {
        let shared = Arc::clone(&snapshot);
        std::thread::spawn(move || serve(port, shared, "rt_bench"));
        println!("monitor: http://127.0.0.1:{port}/");
    }

    let mut handle = match Daemon::start(&config, &factory) {
        Ok(handle) => handle,
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(1);
        }
    };

    // Client period: the bus rate, like a streaming client would.
    let period = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let started = Instant::now();
    let mut next_report = started + Duration::from_secs(1);
    let mut next_minute = started + Duration::from_secs(60);
    let mut last_tick = 0u64;
    let mut last_report = started;
    let mut last_miss = 0u32;
    let mut seq = 0u32;
    let mut centres: Option<Vec<f32>> = None;
    let mut drive_started = started;
    let mut tripped_seen = false;
    let mut next_log = started;
    let mut next_send = started;

    // Shuttle state.
    let mut legs: Vec<Trapezoidal> = Vec::new();
    let mut leg_started = started;
    let mut leg_count = 0u64;
    let mut going_up = false;
    let mut dwell_until: Option<Instant> = None;
    let mut q_cmd = vec![f32::NAN; n];
    let mut err_max = vec![0f32; n];
    let mut err_max_minute = vec![0f32; n];
    let mut err_sum_minute = vec![0f64; n];
    let mut err_samples_minute = 0u64;

    let limits: Vec<(f64, f64)> = config
        .joints
        .iter()
        .map(|j| {
            let v = vel_cap.map_or(j.limits.max_velocity_rad_s, |c| c.min(j.limits.max_velocity_rad_s));
            let a = if j.limits.max_accel_rad_s2 > 0.0 { j.limits.max_accel_rad_s2 } else { 4.0 };
            let a = acc_cap.map_or(a, |c| c.min(a));
            (v, a)
        })
        .collect();

    loop {
        let now = Instant::now();
        if seconds > 0 && now.duration_since(started) >= Duration::from_secs(seconds) {
            break;
        }
        if stop.load(Ordering::Relaxed) {
            println!("stop requested");
            break;
        }
        let moving = drive || shuttle.is_some();
        if moving && centres.is_none() {
            let state = handle.state();
            let ready = state.active().iter().all(|s| {
                AxisStatus::from_u8(s.status) == Some(AxisStatus::Off) && s.q.is_finite()
            });
            if ready {
                let c: Vec<f32> = state.active().iter().map(|s| s.q).collect();
                println!("centres {:?}", c);
                if let Some(amp) = shuttle {
                    // First leg: from where each joint is to its lower end.
                    match plan(&c, &limits, |centre| centre - amp as f32, &c) {
                        Ok(planned) => legs = planned,
                        Err(error) => {
                            eprintln!("{error}");
                            break;
                        }
                    }
                    leg_started = now;
                    going_up = false;
                }
                // The sine's phase starts here, at zero, so its first
                // setpoint is where the joint already is.
                drive_started = now;
                centres = Some(c);
            }
        }

        if let Some(c) = &centres {
            if now >= next_send {
                next_send += period;
                if next_send < now {
                    next_send = now + period;
                }
                let t = now.duration_since(drive_started).as_secs_f32();
                let mut cmd = LowCmd::EMPTY;
                seq = seq.wrapping_add(1).max(1);
                cmd.seq = seq;
                cmd.n = n as u16;
                if let Some(amp) = shuttle {
                    let elapsed = now.duration_since(leg_started).as_secs_f64();
                    let finished = legs.iter().all(|l| elapsed >= l.total_time());
                    if finished {
                        match dwell_until {
                            None => dwell_until = Some(now + Duration::from_secs_f64(dwell)),
                            Some(until) if now >= until => {
                                leg_count += 1;
                                going_up = !going_up;
                                let from: Vec<f32> =
                                    legs.iter().map(|l| l.target().value() as f32).collect();
                                let sign = if going_up { 1.0 } else { -1.0 };
                                match plan(c, &limits, |centre| centre + sign * amp as f32, &from) {
                                    Ok(planned) => legs = planned,
                                    Err(error) => {
                                        eprintln!("{error}");
                                        break;
                                    }
                                }
                                leg_started = now;
                                dwell_until = None;
                            }
                            Some(_) => {}
                        }
                    }
                    let elapsed = now.duration_since(leg_started).as_secs_f64();
                    for (i, motor) in cmd.cmd.iter_mut().enumerate().take(n) {
                        let point = legs[i].sample(elapsed);
                        q_cmd[i] = point.position.value() as f32;
                        *motor = MotorCmd {
                            mode: mode::DRIVE,
                            q: q_cmd[i],
                            dq: point.velocity.value() as f32,
                            ..MotorCmd::OFF
                        };
                    }
                } else {
                    for (i, motor) in cmd.cmd.iter_mut().enumerate().take(n) {
                        q_cmd[i] = c[i] + amplitude * (t * std::f32::consts::TAU * 0.5).sin();
                        *motor = MotorCmd {
                            mode: mode::DRIVE,
                            q: q_cmd[i],
                            ..MotorCmd::OFF
                        };
                    }
                }
                handle.send(cmd);

                // Tracking error against the latest report.
                let state = handle.state();
                if state.estop == 0 {
                    err_samples_minute += 1;
                    for (i, s) in state.active().iter().enumerate() {
                        if s.q.is_finite() && AxisStatus::from_u8(s.status) == Some(AxisStatus::Ok) {
                            let e = (s.q - q_cmd[i]).abs();
                            err_max[i] = err_max[i].max(e);
                            err_max_minute[i] = err_max_minute[i].max(e);
                            err_sum_minute[i] += e as f64;
                        }
                    }
                }
            }
        }

        if !tripped_seen {
            let state = handle.state();
            if state.estop != 0 {
                tripped_seen = true;
                println!(
                    "e-stop code {} at t={:.3} s, tick {}:",
                    state.estop_code,
                    now.duration_since(started).as_secs_f64(),
                    state.tick
                );
                dump_axes(&state, &q_cmd);
            }
        }
        if log_ms > 0 && now >= next_log {
            next_log += Duration::from_millis(log_ms);
            let state = handle.state();
            let mut row = format!(
                "log t={:.3} tick {} estop {}",
                now.duration_since(started).as_secs_f64(),
                state.tick,
                state.estop_code
            );
            for (i, s) in state.active().iter().enumerate() {
                row.push_str(&format!("  {:+.3}/{:+.3}", s.q, q_cmd[i]));
            }
            println!("{row}");
        }
        if now >= next_report {
            let state = handle.state();
            let late_max_ns = handle.late_max_ns();
            let elapsed = now.duration_since(last_report).as_secs_f64();
            let rate = (state.tick - last_tick) as f64 / elapsed;
            last_tick = state.tick;
            last_report = now;
            next_report += Duration::from_secs(1);

            let (mut ok, mut off, mut silent, mut other) = (0, 0, 0, 0);
            for s in state.active() {
                match AxisStatus::from_u8(s.status) {
                    Some(AxisStatus::Ok) | Some(AxisStatus::Alert) => ok += 1,
                    Some(AxisStatus::Off) => off += 1,
                    Some(AxisStatus::Silent) => silent += 1,
                    _ => other += 1,
                }
            }
            let worst = err_max_minute
                .iter()
                .cloned()
                .fold(0f32, f32::max);
            println!(
                "t={:>5.0}s  {:>6.1} Hz  tick {:>8}  miss {:>4}  late_max {:>6.2} ms  estop {:<2} axes ok {ok} off {off} silent {silent} other {other}  legs {leg_count}  err_max(min) {worst:.4} rad",
                now.duration_since(started).as_secs_f64(),
                rate,
                state.tick,
                state.deadline_miss,
                late_max_ns as f64 / 1e6,
                if state.estop != 0 { state.estop_code.to_string() } else { "-".into() },
            );

            // Monitor snapshot.
            if let Ok(mut snap) = snapshot.lock() {
                snap.t = now.duration_since(started).as_secs_f64();
                snap.tick = state.tick;
                snap.rate = rate;
                snap.miss = state.deadline_miss;
                snap.late_max_ms = late_max_ns as f64 / 1e6;
                snap.estop = state.estop_code;
                snap.legs = leg_count;
                for (i, s) in state.active().iter().enumerate() {
                    let a = &mut snap.axes[i];
                    a.q = s.q;
                    a.q_cmd = q_cmd[i];
                    a.dq = s.dq;
                    a.err_max = err_max[i];
                    a.err_max_minute = err_max_minute[i];
                    a.voltage = s.voltage;
                    a.temp = s.temp;
                    a.status = s.status;
                    a.blocked = s.blocked;
                    if let (Some(c), Some(amp)) = (&centres, shuttle) {
                        a.lo = c[i] - amp as f32;
                        a.hi = c[i] + amp as f32;
                    } else if let Some(c) = &centres {
                        a.lo = c[i] - amplitude;
                        a.hi = c[i] + amplitude;
                    }
                }
            }
        }
        if now >= next_minute {
            next_minute += Duration::from_secs(60);
            let state = handle.state();
            let miss_delta = state.deadline_miss - last_miss;
            last_miss = state.deadline_miss;
            let samples = err_samples_minute.max(1) as f64;
            let mut line = format!(
                "minute {:>3}: miss +{miss_delta}  legs {leg_count}  per-axis err max/mean (rad):",
                now.duration_since(started).as_secs() / 60
            );
            for i in 0..n {
                line.push_str(&format!(
                    " {}={:.3}/{:.3}",
                    names[i],
                    err_max_minute[i],
                    err_sum_minute[i] / samples
                ));
                err_max_minute[i] = 0.0;
                err_sum_minute[i] = 0.0;
            }
            err_samples_minute = 0;
            println!("{line}");
        }
        // Monitor refresh between reports, cheap: positions only.
        if monitor.is_some() && now.duration_since(started).as_millis() % 100 < 3 {
            let state = handle.state();
            if let Ok(mut snap) = snapshot.lock() {
                for (i, s) in state.active().iter().enumerate() {
                    snap.axes[i].q = s.q;
                    snap.axes[i].q_cmd = q_cmd[i];
                    snap.axes[i].dq = s.dq;
                    snap.axes[i].status = s.status;
                }
                snap.tick = state.tick;
                snap.estop = state.estop_code;
            }
        }
        if centres.is_some() {
            let until = next_send.min(next_report);
            let gap = until.saturating_duration_since(Instant::now());
            if gap > Duration::from_micros(300) {
                std::thread::sleep(gap - Duration::from_micros(200));
            }
        } else {
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    let state = handle.state();
    dump_axes(&state, &q_cmd);
    println!(
        "stopping after {} ticks, {} deadline misses, late_max {:.2} ms, {} legs{}",
        state.tick,
        state.deadline_miss,
        handle.late_max_ns() as f64 / 1e6,
        leg_count,
        if state.estop != 0 {
            format!(", e-stop latched (code {})", state.estop_code)
        } else {
            String::new()
        }
    );
    let mut line = String::from("tracking error max over the run (rad):");
    for i in 0..n {
        line.push_str(&format!(" {}={:.3}", names[i], err_max[i]));
    }
    println!("{line}");
    // Release before the threads are joined, as `drlabs run` does.
    let mut cmd = LowCmd::EMPTY;
    cmd.n = n as u16;
    cmd.seq = seq.wrapping_add(1).max(1);
    handle.send(cmd);
    std::thread::sleep(Duration::from_millis(50));
    handle.stop();
}

/// Time-synchronised trapezoids for every joint from `from` to
/// `target(centre)`, within each joint's limits.
fn plan(
    centres: &[f32],
    limits: &[(f64, f64)],
    target: impl Fn(f32) -> f32,
    from: &[f32],
) -> Result<Vec<Trapezoidal>, String> {
    let legs: Vec<Leg> = centres
        .iter()
        .enumerate()
        .map(|(i, &centre)| Leg {
            start: Rev(from[i] as f64),
            target: Rev(target(centre) as f64),
            max_velocity: RevPerSec(limits[i].0),
            accel: RevPerSec2(limits[i].1),
        })
        .collect();
    // An error goes back to the caller, which is past torque-on and must
    // leave through its release path.
    synchronized(&legs).map_err(|error| format!("cannot plan the move: {error}"))
}
