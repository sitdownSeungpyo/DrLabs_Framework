//! The bench panel: a browser, some sliders, and a live machine.
//!
//! The panel is a client of the runtime. `drlabs rt` owns the buses; this
//! process owns a browser session, talks LowCmd/LowState to the runtime
//! over UDP, and serves the same HTTP API it always has — so the Python
//! tools in `tools/` that drive it did not have to change.
//!
//! What moved here from the runtime, because the runtime no longer does it:
//! profiled moves (a trapezoid per joint, streamed as setpoints), holds, the
//! teach-mode recorder, and the targets the sliders last set.
//!
//! The assets are compiled in rather than read from disk, so the panel
//! travels with the binary.

mod http;

use std::collections::BTreeMap;
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_api::{Client, Description};
use drlabs_core::units::{Rev, RevPerSec, RevPerSec2};
use drlabs_core::AxisStatus;
use drlabs_rt::Config;
use drlabs_safety::{Blocked, TripCode};
use drlabs_session::trajectory::Trapezoidal;

use crate::flags::{Flags, Kind};
use http::{parse_learn, parse_limits, parse_move, parse_pose, quote, Request, Response};

const INDEX: &str = include_str!("static/index.html");
const APP_JS: &str = include_str!("static/app.js");
const STYLE: &str = include_str!("static/style.css");

const PANEL_FLAGS: &[(&str, Kind)] = &[
    ("addr", Kind::Value),
    ("rt", Kind::Value),
    ("check", Kind::Switch),
];

/// How often a running move puts out its next setpoint.
const MOVE_RATE_HZ: f64 = 100.0;

pub fn panel(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, PANEL_FLAGS).map_err(|error| error.to_string())?;
    // Loopback by default. This server is a bench tool: it has no
    // authentication, and a machine that can be driven by anything that can
    // reach a port should not be reachable from a network by accident.
    let addr = flags.string("addr", "127.0.0.1:8080");
    let runtime = flags.string("rt", "127.0.0.1:7700");
    // The description is optional: it is only needed to write edited limits
    // back. Everything else comes from the runtime.
    let path = flags.positional.first().map(std::path::PathBuf::from);
    if let Some(path) = &path {
        Config::load(path).map_err(|error| error.to_string())?;
    }
    if flags.switch("check") {
        println!("Description is valid. Nothing was opened.");
        return Ok(());
    }

    let client = Client::connect(runtime.as_str(), Duration::from_millis(50))
        .map_err(|error| format!("cannot open a socket to {runtime}: {error}"))?;
    let description = client
        .describe(Duration::from_secs(3))
        .map_err(|error| format!("{error}: is `drlabs rt` running?"))?;
    client.set_axes(description.joints.len());
    println!(
        "{}: {} joints at {} Hz, via {}",
        description.robot,
        description.joints.len(),
        description.rate_hz,
        client.server()
    );

    let listener = TcpListener::bind(&addr).map_err(|error| {
        // A port already in use is the commonest way this fails.
        format!("cannot listen on {addr}: {error}")
    })?;
    let bound = listener
        .local_addr()
        .map(|addr| addr.to_string())
        .unwrap_or(addr);
    println!("\npanel on http://{bound}");
    println!("  Ctrl+C to stop\n");

    let ctx = Arc::new(Panel {
        client,
        targets: Mutex::new(vec![f64::NAN; description.joints.len()]),
        mover: Mutex::new(None),
        learner: Mutex::new(Learner::new(description.joints.len())),
        description,
        path,
    });
    spawn_mover(Arc::clone(&ctx));
    spawn_learner(Arc::clone(&ctx));
    http::serve(listener, move |request| handle(&ctx, request));
    Ok(())
}

/// What a request handler needs.
struct Panel {
    client: Client,
    description: Description,
    /// The last target each joint was given, by index. NaN until it has one.
    targets: Mutex<Vec<f64>>,
    mover: Mutex<Option<Move>>,
    learner: Mutex<Learner>,
    /// The description this panel was started with, if any. Edited limits
    /// go back to exactly this file and nowhere else.
    path: Option<std::path::PathBuf>,
}

/// A profiled move in progress: one trapezoid per joint, all started at
/// once. The trapezoid is unit-agnostic arithmetic; it is used here in
/// radians.
struct Move {
    profiles: Vec<(usize, Trapezoidal)>,
    started: Instant,
    total: f64,
}

impl Move {
    fn target_of(&self, index: usize) -> Option<f64> {
        self.profiles
            .iter()
            .find(|(i, _)| *i == index)
            .map(|(_, profile)| profile.target().value())
    }

    fn progress(&self) -> f64 {
        if self.total <= 0.0 {
            1.0
        } else {
            (self.started.elapsed().as_secs_f64() / self.total).min(1.0)
        }
    }
}

/// Streams the running move's setpoints, and finishes it.
fn spawn_mover(ctx: Arc<Panel>) {
    std::thread::Builder::new()
        .name("drlabs-panel-move".into())
        .spawn(move || {
            let period = Duration::from_secs_f64(1.0 / MOVE_RATE_HZ);
            loop {
                std::thread::sleep(period);
                let mut mover = ctx.mover.lock().unwrap();
                let Some(current) = mover.as_ref() else {
                    continue;
                };
                let t = current.started.elapsed().as_secs_f64();
                let done = t >= current.total;
                let mut targets = ctx.targets.lock().unwrap();
                for (index, profile) in &current.profiles {
                    let q = profile.sample(t).position.value();
                    ctx.client.drive(*index, q as f32);
                    targets[*index] = q;
                }
                drop(targets);
                let _ = ctx.client.send(0);
                if done {
                    *mover = None;
                }
            }
        })
        .expect("a move thread");
}

/// Teach mode: the joints are released and their travel recorded.
struct Learner {
    running: bool,
    started: Option<Instant>,
    seconds: f64,
    min: Vec<f64>,
    max: Vec<f64>,
    samples: Vec<u64>,
}

impl Learner {
    fn new(n: usize) -> Self {
        Self {
            running: false,
            started: None,
            seconds: 0.0,
            min: vec![f64::NAN; n],
            max: vec![f64::NAN; n],
            samples: vec![0; n],
        }
    }

    fn clear(&mut self) {
        for value in &mut self.min {
            *value = f64::NAN;
        }
        for value in &mut self.max {
            *value = f64::NAN;
        }
        for value in &mut self.samples {
            *value = 0;
        }
        self.started = None;
    }
}

fn spawn_learner(ctx: Arc<Panel>) {
    std::thread::Builder::new()
        .name("drlabs-panel-learn".into())
        .spawn(move || loop {
            std::thread::sleep(Duration::from_millis(25));
            let mut learner = ctx.learner.lock().unwrap();
            if !learner.running {
                continue;
            }
            if let Some(started) = learner.started {
                if started.elapsed().as_secs_f64() >= learner.seconds {
                    learner.running = false;
                    continue;
                }
            }
            let Some((state, _)) = ctx.client.latest() else {
                continue;
            };
            for (index, motor) in state.active().iter().enumerate() {
                if index >= learner.min.len() || !motor.q.is_finite() {
                    continue;
                }
                let q = motor.q as f64;
                learner.min[index] = if learner.min[index].is_nan() {
                    q
                } else {
                    learner.min[index].min(q)
                };
                learner.max[index] = if learner.max[index].is_nan() {
                    q
                } else {
                    learner.max[index].max(q)
                };
                learner.samples[index] += 1;
            }
        })
        .expect("a learn thread");
}

impl Panel {
    fn stopped_reason(&self) -> Option<String> {
        let (state, _) = self.client.latest()?;
        if state.estop == 0 {
            return None;
        }
        Some(format!(
            "e-stop latched ({})",
            TripCode::from_u8(state.estop_code as u8)
                .map(|code| code.code())
                .unwrap_or("unknown")
        ))
    }

    /// Commands joints straight to targets, cancelling any move on them.
    fn pose(&self, targets: &BTreeMap<String, f64>) -> Result<(), String> {
        if let Some(reason) = self.stopped_reason() {
            return Err(reason);
        }
        let mut resolved = Vec::with_capacity(targets.len());
        for (name, &value) in targets {
            let joint = self
                .description
                .joint(name)
                .ok_or_else(|| format!("no joint named {name:?}"))?;
            resolved.push((joint.index, value.clamp(joint.min_rad, joint.max_rad)));
        }
        let mut mover = self.mover.lock().unwrap();
        if let Some(current) = mover.as_mut() {
            current
                .profiles
                .retain(|(index, _)| !resolved.iter().any(|(i, _)| i == index));
            if current.profiles.is_empty() {
                *mover = None;
            }
        }
        let mut last = self.targets.lock().unwrap();
        for (index, q) in &resolved {
            self.client.drive(*index, *q as f32);
            last[*index] = *q;
        }
        drop(last);
        drop(mover);
        self.client.send(0).map_err(|error| error.to_string())
    }

    /// Starts a profiled move from where each joint is now.
    fn start_move(
        &self,
        targets: &BTreeMap<String, f64>,
        max_velocity: f64,
        accel: f64,
    ) -> Result<(), String> {
        if let Some(reason) = self.stopped_reason() {
            return Err(reason);
        }
        let (state, _) = self
            .client
            .latest()
            .ok_or("no state from the runtime yet")?;
        let mut profiles = Vec::with_capacity(targets.len());
        for (name, &value) in targets {
            let joint = self
                .description
                .joint(name)
                .ok_or_else(|| format!("no joint named {name:?}"))?;
            let start = state
                .state
                .get(joint.index)
                .map(|m| m.q as f64)
                .filter(|q| q.is_finite())
                .ok_or_else(|| format!("{name:?} reports no position to move from"))?;
            let target = value.clamp(joint.min_rad, joint.max_rad);
            let velocity = max_velocity.min(joint.max_velocity_rad_s);
            let acceleration = if joint.max_accel_rad_s2 > 0.0 {
                accel.min(joint.max_accel_rad_s2)
            } else {
                accel
            };
            let profile = Trapezoidal::new(
                Rev(start),
                Rev(target),
                RevPerSec(velocity),
                RevPerSec2(acceleration),
            )
            .map_err(|error| format!("{name:?}: {error}"))?;
            profiles.push((joint.index, profile));
        }
        let total = profiles
            .iter()
            .map(|(_, p)| p.total_time())
            .fold(0.0, f64::max);
        *self.mover.lock().unwrap() = Some(Move {
            profiles,
            started: Instant::now(),
            total,
        });
        Ok(())
    }

    fn hold_all(&self) -> Result<(), String> {
        let (state, _) = self
            .client
            .latest()
            .ok_or("no state from the runtime yet")?;
        let mut targets = BTreeMap::new();
        for joint in &self.description.joints {
            if let Some(motor) = state.state.get(joint.index) {
                if motor.q.is_finite() {
                    targets.insert(joint.name.clone(), motor.q as f64);
                }
            }
        }
        self.pose(&targets)
    }

    fn release_all(&self) -> Result<(), String> {
        *self.mover.lock().unwrap() = None;
        self.client.release_all();
        self.client.send(0).map_err(|error| error.to_string())
    }
}

fn handle(ctx: &Panel, request: &Request) -> Response {
    match (request.method.as_str(), request.path.as_str()) {
        ("GET", "/") | ("GET", "/index.html") => Response::html(INDEX),
        ("GET", "/app.js") => Response::javascript(APP_JS),
        ("GET", "/style.css") => Response::css(STYLE),
        ("GET", "/api/state") => Response::json(state_json(ctx)),

        ("POST", "/api/pose") => match parse_pose(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(targets) => match ctx.pose(&targets) {
                Ok(()) => Response::json("{\"ok\":true}".into()),
                Err(reason) => Response::refused(&reason),
            },
        },

        // A profiled move: the sliders set a target directly, this plans one.
        ("POST", "/api/move") => match parse_move(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(request) => {
                let targets = match &request.pose {
                    Some(pose) => match ctx.description.poses.get(pose) {
                        Some(targets) => targets.clone(),
                        None => return Response::refused(&format!("no pose named {pose:?}")),
                    },
                    None => request.targets.clone(),
                };
                match ctx.start_move(&targets, request.max_velocity, request.accel) {
                    Ok(()) => Response::json("{\"ok\":true}".into()),
                    Err(reason) => Response::refused(&reason),
                }
            }
        },

        // Teach mode. Releasing the joints is the point rather than a side
        // effect: a joint under torque holds its target, so the only travel it
        // would report is the one the loop asked for.
        ("POST", "/api/learn/start") => match parse_learn(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(seconds) => {
                if let Err(reason) = ctx.release_all() {
                    return Response::refused(&reason);
                }
                let mut learner = ctx.learner.lock().unwrap();
                learner.clear();
                learner.running = true;
                learner.started = Some(Instant::now());
                learner.seconds = seconds;
                Response::json("{\"ok\":true}".into())
            }
        },
        ("POST", "/api/learn/stop") => {
            ctx.learner.lock().unwrap().running = false;
            Response::json("{\"ok\":true}".into())
        }
        ("POST", "/api/learn/clear") => {
            let mut learner = ctx.learner.lock().unwrap();
            learner.running = false;
            learner.clear();
            Response::json("{\"ok\":true}".into())
        }
        ("GET", "/api/learn") => Response::json(learn_json(ctx)),

        // Edited limits go back to the description. Nothing here changes the
        // running machine: the runtime built its envelope when it came up.
        ("POST", "/api/limits") => match (&ctx.path, parse_limits(&request.body)) {
            (_, Err(reason)) => Response::refused(&reason),
            (None, Ok(_)) => Response::refused(
                "the panel was started without a description file, so there is nowhere to \
                 write edited limits",
            ),
            (Some(path), Ok(edits)) => match Config::write_limits(path, &edits) {
                Ok(_) => Response::json(format!(
                    "{{\"ok\":true,\"written\":{},\"restart_required\":true}}",
                    quote(&path.display().to_string())
                )),
                Err(error) => Response::refused(&error.0),
            },
        },

        ("POST", "/api/estop") => {
            *ctx.mover.lock().unwrap() = None;
            match ctx.client.estop() {
                Ok(()) => Response::json("{\"ok\":true}".into()),
                Err(error) => Response::refused(&error.to_string()),
            }
        }
        ("POST", "/api/reset") => {
            *ctx.mover.lock().unwrap() = None;
            for target in ctx.targets.lock().unwrap().iter_mut() {
                *target = f64::NAN;
            }
            match ctx.client.reset_estop() {
                Ok(()) => Response::json("{\"ok\":true}".into()),
                Err(error) => Response::refused(&error.to_string()),
            }
        }
        ("POST", "/api/hold") => match ctx.hold_all() {
            Ok(()) => Response::json("{\"ok\":true}".into()),
            Err(reason) => Response::refused(&reason),
        },
        ("POST", "/api/stop") => match ctx.release_all() {
            Ok(()) => Response::json("{\"ok\":true}".into()),
            Err(reason) => Response::refused(&reason),
        },

        _ => Response::not_found(),
    }
}

fn number(value: f64) -> String {
    if value.is_finite() {
        format!("{value}")
    } else {
        "null".into()
    }
}

/// The whole machine, as the panel wants it.
///
/// Hand-written rather than derived, because the panel's shape and the
/// runtime's are not the same thing and pinning one to the other means every
/// internal rename is a UI change.
fn state_json(ctx: &Panel) -> String {
    let latest = ctx.client.latest();
    let connected = latest
        .as_ref()
        .map(|(_, age)| *age < Duration::from_secs(1))
        .unwrap_or(false);
    let state = latest.map(|(state, _)| state);
    let targets = ctx.targets.lock().unwrap().clone();
    let (moving_to, progress): (Vec<Option<f64>>, f64) = {
        let mover = ctx.mover.lock().unwrap();
        match mover.as_ref() {
            Some(current) => (
                (0..ctx.description.joints.len())
                    .map(|index| current.target_of(index))
                    .collect(),
                current.progress(),
            ),
            None => (vec![None; ctx.description.joints.len()], 0.0),
        }
    };
    let estop_reason = state
        .as_ref()
        .filter(|state| state.estop != 0)
        .and_then(|state| TripCode::from_u8(state.estop_code as u8))
        .map(|code| code.code().to_string())
        .unwrap_or_default();

    let mut joints = Vec::with_capacity(ctx.description.joints.len());
    for joint in &ctx.description.joints {
        let motor = state
            .as_ref()
            .and_then(|state| (joint.index < state.n as usize).then(|| state.state[joint.index]));
        let status = motor.and_then(|m| AxisStatus::from_u8(m.status));
        let armed = matches!(
            status,
            Some(AxisStatus::Ok | AxisStatus::Alert | AxisStatus::Refused)
        );
        let position = motor.map(|m| m.q as f64).unwrap_or(f64::NAN);
        // The target is what the panel last asked for, or where the joint
        // is: a tool waiting for "position near target" must not see null.
        let target = targets
            .get(joint.index)
            .copied()
            .filter(|t| t.is_finite())
            .unwrap_or(position);
        let blocked = motor
            .and_then(|m| match m.blocked {
                0 => None,
                b => Some(quote(blocked_name(b))),
            })
            .map(|s| vec![s])
            .unwrap_or_default();
        let fault_name = match motor.map(|m| m.fault).unwrap_or(0) {
            0 => String::new(),
            code => format!("fault 0x{code:x}"),
        };

        joints.push(format!(
            "{{\"name\":{},\"driver\":{},\"bus\":{},\"id\":{},\
              \"min_rad\":{},\"max_rad\":{},\
              \"armed\":{},\"estop\":{},\"estop_reason\":{},\"mode\":{},\
              \"position_rad\":{},\"target_rad\":{},\"torque_nm\":{},\
              \"velocity_rad_s\":{},\"current_a\":{},\"temperature_c\":{},\
              \"voltage_v\":{},\"rate_hz\":{},\"fault_name\":{},\
              \"moving_to_rad\":{},\"progress\":{},\"blocked\":[{}]}}",
            quote(&joint.name),
            quote(&joint.driver),
            quote(&joint.bus),
            joint.id,
            joint.min_rad,
            joint.max_rad,
            armed,
            !estop_reason.is_empty(),
            quote(&estop_reason),
            quote(status.map(|s| s.as_str()).unwrap_or("silent")),
            number(position),
            number(target),
            number(motor.map(|m| m.tau as f64).unwrap_or(f64::NAN)),
            number(motor.map(|m| m.dq as f64).unwrap_or(f64::NAN)),
            number(motor.map(|m| m.current as f64).unwrap_or(f64::NAN)),
            number(motor.map(|m| m.temp as f64).unwrap_or(f64::NAN)),
            number(motor.map(|m| m.voltage as f64).unwrap_or(f64::NAN)),
            ctx.description.rate_hz,
            quote(&fault_name),
            number(moving_to[joint.index].unwrap_or(f64::NAN)),
            number(if moving_to[joint.index].is_some() {
                progress
            } else {
                0.0
            }),
            blocked.join(",")
        ));
    }

    let poses: Vec<String> = ctx
        .description
        .poses
        .keys()
        .map(|name| quote(name))
        .collect();
    format!(
        "{{\"robot\":{},\"rate_hz\":{},\"connected\":{},\"poses\":[{}],\"joints\":[{}]}}",
        quote(&ctx.description.robot),
        ctx.description.rate_hz,
        connected,
        poses.join(","),
        joints.join(",")
    )
}

fn blocked_name(byte: u8) -> &'static str {
    match byte {
        1 => Blocked::NoTelemetry.as_str(),
        2 => Blocked::VoltageLow.as_str(),
        3 => Blocked::VoltageHigh.as_str(),
        4 => Blocked::ControllerFault.as_str(),
        5 => Blocked::NoPosition.as_str(),
        6 => Blocked::OutsideEnvelope.as_str(),
        7 => Blocked::Estop.as_str(),
        _ => "blocked",
    }
}

/// What teach mode has seen, next to what the description says today.
fn learn_json(ctx: &Panel) -> String {
    let learner = ctx.learner.lock().unwrap();
    let latest = ctx.client.latest().map(|(state, _)| state);
    let elapsed = learner
        .started
        .map(|at| at.elapsed().as_secs_f64())
        .unwrap_or(0.0);
    let remaining = if learner.running {
        (learner.seconds - elapsed).max(0.0)
    } else {
        0.0
    };
    let joints: Vec<String> = ctx
        .description
        .joints
        .iter()
        .map(|joint| {
            let index = joint.index;
            let current = latest
                .as_ref()
                .and_then(|s| s.state.get(index).map(|m| m.q as f64))
                .unwrap_or(f64::NAN);
            let span = learner.max[index] - learner.min[index];
            format!(
                "{{\"name\":{},\"min_rad\":{},\"max_rad\":{},\"current_rad\":{},\
                  \"span_rad\":{},\"samples\":{},\
                  \"configured_min_rad\":{},\"configured_max_rad\":{},\"direction\":{}}}",
                quote(&joint.name),
                number(learner.min[index]),
                number(learner.max[index]),
                number(current),
                number(span),
                learner.samples[index],
                joint.min_rad,
                joint.max_rad,
                // The runtime's description does not carry the sense; the
                // file does, and the file is what an edit goes back to.
                ctx.path
                    .as_ref()
                    .and_then(|path| Config::load(path).ok())
                    .and_then(|config| config.joint(&joint.name).map(|j| j.direction))
                    .unwrap_or(1.0),
            )
        })
        .collect();

    format!(
        "{{\"running\":{},\"elapsed_s\":{:.2},\"remaining_s\":{:.2},\"joints\":[{}]}}",
        learner.running,
        elapsed,
        remaining,
        joints.join(",")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The panel is served from the binary, so a build that forgot an asset is
    /// a build that serves a blank page.
    #[test]
    fn every_asset_is_compiled_in() {
        assert!(INDEX.contains("joint-template"), "the markup is missing");
        assert!(APP_JS.contains("/api/state"), "the script is missing");
        assert!(STYLE.contains("btn-estop"), "the styles are missing");
    }

    /// The controls that move the machine are served from the binary too, and a
    /// button whose markup, script or style went missing is a panel that looks
    /// finished and does nothing when pressed.
    #[test]
    fn the_move_controls_are_wired_end_to_end() {
        for id in ["go-zero", "move-velocity", "poses"] {
            assert!(INDEX.contains(id), "the markup has no {id}");
            assert!(APP_JS.contains(id), "the script never looks up {id}");
        }
        for class in ["joint-go", "joint-zero", "joint-goal"] {
            assert!(INDEX.contains(class), "the joint template has no {class}");
            assert!(APP_JS.contains(class), "the script never looks up {class}");
        }
        assert!(
            APP_JS.contains("/api/move"),
            "nothing in the panel plans a trajectory"
        );
        assert!(STYLE.contains(".joint-move"), "the move row is unstyled");
    }

    /// Teach mode has three moving parts — markup, script and the endpoints —
    /// and a button that reaches an endpoint nobody serves is a panel that
    /// releases the machine and then records nothing.
    #[test]
    fn teach_mode_is_wired_end_to_end() {
        for id in [
            "learn-seconds",
            "learn-start",
            "learn-stop",
            "learn-clear",
            "teach-results",
        ] {
            assert!(INDEX.contains(id), "the markup has no {id}");
            assert!(APP_JS.contains(id), "the script never looks up {id}");
        }
        for class in ["teach-min", "teach-max", "teach-direction"] {
            assert!(INDEX.contains(class), "the row template has no {class}");
            assert!(APP_JS.contains(class), "the script never reads {class}");
        }
        for route in [
            "/api/learn/start",
            "/api/learn/stop",
            "/api/learn/clear",
            "/api/learn",
            "/api/limits",
        ] {
            assert!(APP_JS.contains(route), "the script never calls {route}");
        }
        assert!(
            STYLE.contains(".teach-table"),
            "the results table is unstyled"
        );
    }

    /// Releasing a fifteen-degree-of-freedom machine drops it. The panel must
    /// say so before it happens, not after.
    #[test]
    fn releasing_the_machine_is_confirmed_first() {
        assert!(
            APP_JS.contains("releaseArmed"),
            "teach mode must ask twice before it releases every joint"
        );
        assert!(
            APP_JS.contains("goes limp"),
            "the confirmation has to say what actually happens"
        );
        assert!(
            !APP_JS.contains("confirm(") && !APP_JS.contains("alert("),
            "the panel must not use a blocking browser dialog"
        );
    }

    /// Every field the panel reads has to be one the panel sends.
    #[test]
    fn the_state_payload_carries_what_the_script_reads() {
        for field in [
            "moving_to_rad",
            "position_rad",
            "target_rad",
            "armed",
            "estop",
            "poses",
        ] {
            assert!(
                APP_JS.contains(field),
                "the script does not read {field}, so sending it is dead weight"
            );
        }
    }

    #[test]
    fn the_estop_is_not_one_button_among_others() {
        assert!(STYLE.contains("margin-left: auto"));
        assert!(INDEX.contains("btn btn-estop"));
    }

    #[test]
    fn restyling_an_element_keeps_the_class_it_was_found_by() {
        for marker in ["joint-armed", "joint-mode"] {
            assert!(
                APP_JS.contains(&format!(".{marker}")),
                "{marker} is never looked up"
            );
            assert!(
                APP_JS.contains(&format!("\"{marker} pill \"")),
                "the {marker} pill drops its own marker class when it is restyled"
            );
        }
    }

    /// Nothing safety-related may be drawn from what the operator pressed.
    #[test]
    fn the_script_renders_safety_state_only_from_the_robot() {
        let handler = APP_JS
            .split("getElementById(\"estop\")")
            .nth(1)
            .expect("an e-stop handler");
        let handler = &handler[..handler.find(';').unwrap_or(handler.len())];
        assert!(handler.contains("post("), "{handler}");
        assert!(
            !handler.contains("className") && !handler.contains("textContent"),
            "the e-stop handler must not paint the machine as stopped: {handler}"
        );
    }

    /// A move's progress and target are reported per joint while it runs,
    /// and the trapezoid is used in radians end to end.
    #[test]
    fn a_move_reports_where_each_joint_is_going() {
        let profile =
            Trapezoidal::new(Rev(0.0), Rev(1.0), RevPerSec(1.0), RevPerSec2(2.0)).unwrap();
        let current = Move {
            profiles: vec![(3, profile)],
            started: Instant::now(),
            total: profile.total_time(),
        };
        assert_eq!(current.target_of(3), Some(1.0));
        assert_eq!(current.target_of(0), None);
        assert!(current.progress() < 0.5);
    }
}
