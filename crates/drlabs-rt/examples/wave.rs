//! Wave: joints in a row turn one after another, out and back, forever.
//!
//!   wave <robot.json> [angle=RAD] [dwell=S] [vel=RAD_S] [acc=RAD_S2]
//!                     [seconds=N] [monitor=PORT]
//!
//! The first joint turns `angle` radians (default −π/2; the sign is the
//! direction) on its own trapezoid. The instant that profile's planned time
//! ends, the next joint starts the same turn, through the last joint. After
//! `dwell` seconds (default 0.5) the wave runs back, last joint first, and
//! every joint returns to where it started. Then it repeats.
//!
//! Each joint gets `drlabs_core::trajectory::Trapezoidal` within its declared
//! limits; `vel`/`acc` only lower them. The next joint's start is chained on
//! the previous profile's planned end, not on the tick that noticed it, so
//! one bus period of lag does not accumulate along the row. Joints that are
//! not moving hold their position in DRIVE, so every tick commands every
//! axis. The client streams q and dq feed-forward at the bus rate.
//!
//! `seconds=0` (the default) runs until Enter is pressed, then releases the
//! chain the same way a timed run does. Ctrl+C skips that: the process dies
//! with torque on and the bus watchdog armed. Stopping between passes (during
//! `dwell` after the return) leaves the row where it started; stopping
//! mid-wave leaves it half turned, and the next run takes that as its start.
//!
//! Same runtime path as `rt_bench` (`Daemon`, `BusWorker`, `DynamixelBus`),
//! no DDS. `monitor=PORT` serves the same live page on 127.0.0.1:PORT.

mod common;

use std::path::Path;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use common::{dump_axes, factory, serve, AxisSnap, Snapshot};
use drlabs_core::lowlevel::{mode, LowCmd, MotorCmd};
use drlabs_core::trajectory::Trapezoidal;
use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};
use drlabs_core::AxisStatus;
use drlabs_rt::{Config, Daemon};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(path) = args.first() else {
        eprintln!("usage: wave <robot.json> [angle=RAD] [dwell=S] [vel=RAD_S] [acc=RAD_S2] [seconds=N] [monitor=PORT]");
        std::process::exit(2);
    };
    let mut angle = -std::f64::consts::FRAC_PI_2;
    let mut dwell = 0.5f64;
    let mut vel_cap: Option<f64> = None;
    let mut acc_cap: Option<f64> = None;
    let mut seconds = 0u64;
    let mut monitor: Option<u16> = None;
    for token in &args[1..] {
        if let Some(value) = token.strip_prefix("angle=") {
            angle = value.parse().expect("angle=RAD");
        } else if let Some(value) = token.strip_prefix("dwell=") {
            dwell = value.parse().expect("dwell=S");
        } else if let Some(value) = token.strip_prefix("vel=") {
            vel_cap = Some(value.parse().expect("vel=RAD_S"));
        } else if let Some(value) = token.strip_prefix("acc=") {
            acc_cap = Some(value.parse().expect("acc=RAD_S2"));
        } else if let Some(value) = token.strip_prefix("seconds=") {
            seconds = value.parse().expect("seconds=N");
        } else if let Some(value) = token.strip_prefix("monitor=") {
            monitor = Some(value.parse().expect("monitor=PORT"));
        } else {
            eprintln!("unknown argument {token:?}");
            std::process::exit(2);
        }
    }
    if angle == 0.0 || !angle.is_finite() {
        eprintln!("angle must be a non-zero number of radians");
        std::process::exit(2);
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
    let limits: Vec<(f64, f64)> = config
        .joints
        .iter()
        .map(|j| {
            let v = vel_cap.map_or(j.limits.max_velocity_rad_s, |c| c.min(j.limits.max_velocity_rad_s));
            let a = if j.limits.max_accel_rad_s2 > 0.0 { j.limits.max_accel_rad_s2 } else { 4.0 };
            (v, acc_cap.map_or(a, |c| c.min(a)))
        })
        .collect();
    // Planned before the daemon starts, so a limit that cannot be planned
    // stops the run here, before any torque.
    let leg_time = match leg(0.0, angle as f32, limits[0]) {
        Ok(p) => p.total_time(),
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(1);
        }
    };
    println!(
        "{}: {} joints at {} Hz, angle {angle:+.4} rad, one turn {leg_time:.2} s (joint 1), dwell {dwell} s",
        config.robot, n, config.rate_hz
    );

    let snapshot = Arc::new(Mutex::new(Snapshot {
        plan: format!("wave {angle:+.4} rad/joint, {n} joints, dwell {dwell} s"),
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
        std::thread::spawn(move || serve(port, shared, "wave"));
        println!("monitor: http://127.0.0.1:{port}/");
    }

    let stop = common::stop_on_enter(seconds);
    if seconds == 0 {
        println!("running until Enter is pressed (not Ctrl+C)");
    }

    let mut handle = match Daemon::start(&config, &factory) {
        Ok(handle) => handle,
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(1);
        }
    };

    let period = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let started = Instant::now();
    let mut next_send = started;
    let mut next_report = started + Duration::from_secs(1);
    let mut last_report = started;
    let mut next_refresh = started;
    let mut last_tick = 0u64;
    let mut seq = 0u32;
    let mut tripped_seen = false;

    // Where each joint started, and what each idle joint holds now.
    let mut centres: Vec<f32> = Vec::new();
    let mut hold: Vec<f32> = Vec::new();
    // The one joint moving now, its profile and when that profile began.
    let mut idx = 0usize;
    let mut profile: Option<Trapezoidal> = None;
    let mut profile_started = started;
    let mut forward = true;
    let mut dwell_until: Option<Instant> = None;
    let mut turns = 0u64;
    let mut passes = 0u64;

    let mut q_cmd = vec![f32::NAN; n];
    let mut err_max = vec![0f32; n];

    loop {
        let now = Instant::now();
        if seconds > 0 && now.duration_since(started) >= Duration::from_secs(seconds) {
            break;
        }
        if stop.load(Ordering::Relaxed) {
            println!("stop requested");
            break;
        }

        // Start once every axis is up, reporting, and still OFF.
        if centres.is_empty() {
            let state = handle.state();
            let ready = state.active().iter().all(|s| {
                AxisStatus::from_u8(s.status) == Some(AxisStatus::Off) && s.q.is_finite()
            });
            if ready {
                centres = state.active().iter().map(|s| s.q).collect();
                println!("start positions {:?}", centres);
                hold = centres.clone();
                match leg(hold[0], centres[0] + angle as f32, limits[0]) {
                    Ok(p) => profile = Some(p),
                    Err(error) => {
                        eprintln!("{error}");
                        break;
                    }
                }
                profile_started = now;
            } else {
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
        }

        if now >= next_send {
            next_send += period;
            if next_send < now {
                next_send = now + period;
            }
            let target_of = |i: usize, forward: bool| {
                if forward {
                    centres[i] + angle as f32
                } else {
                    centres[i]
                }
            };

            if let Some(p) = profile {
                if now.duration_since(profile_started).as_secs_f64() >= p.total_time() {
                    hold[idx] = p.target().value() as f32;
                    turns += 1;
                    let next = if forward {
                        (idx + 1 < n).then_some(idx + 1)
                    } else {
                        idx.checked_sub(1)
                    };
                    match next {
                        Some(next) => {
                            match leg(hold[next], target_of(next, forward), limits[next]) {
                                Ok(next_profile) => profile = Some(next_profile),
                                Err(error) => {
                                    eprintln!("{error}");
                                    break;
                                }
                            }
                            idx = next;
                            profile_started += Duration::from_secs_f64(p.total_time());
                        }
                        None => {
                            profile = None;
                            passes += 1;
                            dwell_until = Some(now + Duration::from_secs_f64(dwell));
                        }
                    }
                }
            } else if dwell_until.is_some_and(|until| now >= until) {
                forward = !forward;
                idx = if forward { 0 } else { n - 1 };
                match leg(hold[idx], target_of(idx, forward), limits[idx]) {
                    Ok(p) => profile = Some(p),
                    Err(error) => {
                        eprintln!("{error}");
                        break;
                    }
                }
                profile_started = now;
                dwell_until = None;
            }

            let elapsed = now.duration_since(profile_started).as_secs_f64();
            let mut cmd = LowCmd::EMPTY;
            seq = seq.wrapping_add(1).max(1);
            cmd.seq = seq;
            cmd.n = n as u16;
            for (i, motor) in cmd.cmd.iter_mut().enumerate().take(n) {
                let mut dq = 0.0f32;
                q_cmd[i] = hold[i];
                if let (true, Some(p)) = (i == idx, profile) {
                    let point = p.sample(elapsed);
                    q_cmd[i] = point.position.value() as f32;
                    dq = point.velocity.value() as f32;
                }
                *motor = MotorCmd {
                    mode: mode::DRIVE,
                    q: q_cmd[i],
                    dq,
                    ..MotorCmd::OFF
                };
            }
            handle.send(cmd);

            let state = handle.state();
            if state.estop == 0 {
                for (i, s) in state.active().iter().enumerate() {
                    if s.q.is_finite() && AxisStatus::from_u8(s.status) == Some(AxisStatus::Ok) {
                        err_max[i] = err_max[i].max((s.q - q_cmd[i]).abs());
                    }
                }
            } else if !tripped_seen {
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

        if now >= next_report {
            next_report += Duration::from_secs(1);
            let state = handle.state();
            let dt = now.duration_since(last_report).as_secs_f64();
            let rate = (state.tick - last_tick) as f64 / dt;
            last_tick = state.tick;
            last_report = now;
            let late_max_ms = handle.late_max_ns() as f64 / 1e6;
            let moving = if profile.is_some() { names[idx].as_str() } else { "-" };
            println!(
                "t={:5.0}s {:6.1} Hz  miss {:4}  late_max {:5.2} ms  estop {}  pass {} turns {} moving {}  err_max {:.3} rad",
                now.duration_since(started).as_secs_f64(),
                rate,
                state.deadline_miss,
                late_max_ms,
                if state.estop != 0 { state.estop_code.to_string() } else { "-".into() },
                passes,
                turns,
                moving,
                err_max.iter().copied().fold(0.0, f32::max),
            );
            if let Ok(mut snap) = snapshot.lock() {
                snap.t = now.duration_since(started).as_secs_f64();
                snap.tick = state.tick;
                snap.rate = rate;
                snap.miss = state.deadline_miss;
                snap.late_max_ms = late_max_ms;
                snap.estop = state.estop_code;
                snap.legs = turns;
                for (i, s) in state.active().iter().enumerate() {
                    let a = &mut snap.axes[i];
                    a.q = s.q;
                    a.q_cmd = q_cmd[i];
                    a.dq = s.dq;
                    a.err_max = err_max[i];
                    a.err_max_minute = err_max[i];
                    a.voltage = s.voltage;
                    a.temp = s.temp;
                    a.status = s.status;
                    a.blocked = s.blocked;
                    a.lo = centres[i].min(centres[i] + angle as f32);
                    a.hi = centres[i].max(centres[i] + angle as f32);
                }
            }
        } else if monitor.is_some() && now >= next_refresh {
            // Keep the page's bars moving between the once-a-second reports.
            next_refresh = now + Duration::from_millis(100);
            let Ok(mut snap) = snapshot.try_lock() else { continue };
            let state = handle.state();
            snap.tick = state.tick;
            snap.estop = state.estop_code;
            snap.legs = turns;
            for (i, s) in state.active().iter().enumerate() {
                snap.axes[i].q = s.q;
                snap.axes[i].q_cmd = q_cmd[i];
                snap.axes[i].dq = s.dq;
                snap.axes[i].status = s.status;
            }
        }

        let gap = next_send.min(next_report).saturating_duration_since(Instant::now());
        if gap > Duration::from_micros(300) {
            std::thread::sleep(gap - Duration::from_micros(200));
        }
    }

    let state = handle.state();
    dump_axes(&state, &q_cmd);
    println!(
        "stopping after {} ticks, {} deadline misses, late_max {:.2} ms, {} passes, {} turns{}",
        state.tick,
        state.deadline_miss,
        handle.late_max_ns() as f64 / 1e6,
        passes,
        turns,
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

/// One joint's own trapezoid from `from` to `to` within `(velocity, accel)`.
/// An error is returned, not exited on: the caller is past torque-on and
/// must leave through its release path.
fn leg(from: f32, to: f32, limit: (f64, f64)) -> Result<Trapezoidal, String> {
    Trapezoidal::new(Rev(from as f64), Rev(to as f64), RevPerSec(limit.0), RevPerSec2(limit.1))
        .map_err(|error| format!("cannot plan the turn: {error}"))
}
