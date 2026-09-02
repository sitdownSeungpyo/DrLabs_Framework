//! The bench panel: a browser, some sliders, and a live machine.
//!
//! The assets are compiled in rather than read from disk, so the panel travels
//! with the binary. That is the whole point of the layout: copy `drlabs` and a
//! JSON description onto a machine, run one command, and there is a robot under
//! control with a way to drive it.

mod http;

use std::net::TcpListener;
use std::sync::Arc;
use std::time::Duration;

use drlabs_core::units::{Rad, RadPerSec, RadPerSec2};
use drlabs_robot::{Config, RangeLearner, Robot};

use crate::flags::{Flags, Kind};
use http::{parse_learn, parse_limits, parse_move, parse_pose, quote, Request, Response};

const INDEX: &str = include_str!("static/index.html");
const APP_JS: &str = include_str!("static/app.js");
const STYLE: &str = include_str!("static/style.css");

const PANEL_FLAGS: &[(&str, Kind)] = &[("addr", Kind::Value), ("check", Kind::Switch)];

pub fn panel(args: &[String]) -> Result<(), String> {
    let flags = Flags::parse(args, PANEL_FLAGS).map_err(|error| error.to_string())?;
    let path = match flags.positional.first() {
        Some(path) => path.clone(),
        None => return Err("usage: drlabs panel [-addr host:port] <robot.json>".into()),
    };
    // Loopback by default. This server is a bench tool: it has no
    // authentication, and a machine that can be driven by anything that can
    // reach a port should not be reachable from a network by accident.
    let addr = flags.string("addr", "127.0.0.1:8080");

    let config = Config::load(std::path::Path::new(&path)).map_err(|error| error.to_string())?;
    if flags.switch("check") {
        println!("Description is valid. Nothing was opened.");
        return Ok(());
    }

    let mut robot = Robot::new(config).map_err(|error| error.to_string())?;
    println!("Bringing up {} joints…", robot.config.joints.len());
    robot.bring_up().map_err(|error| error.to_string())?;

    let listener = TcpListener::bind(&addr).map_err(|error| {
        // A port already in use is the commonest way this fails, and the
        // machine is live by now — say so rather than leaving a bare io error.
        format!("cannot listen on {addr}: {error}")
    })?;
    let bound = listener
        .local_addr()
        .map(|addr| addr.to_string())
        .unwrap_or(addr);

    println!("\npanel on http://{bound}");
    println!("  Ctrl+C to stop\n");

    let robot = Arc::new(robot);
    let learner = RangeLearner::new();

    // The deadman wants to hear from a supervisor. The panel polls, but a
    // browser that has been closed does not, and the machine should not be left
    // energised waiting for one — so the heartbeat comes from here, and the
    // panel's own liveness is what the operator watches.
    let beating = robot.clone();
    std::thread::spawn(move || loop {
        beating.heartbeat();
        std::thread::sleep(Duration::from_millis(200));
    });

    let ctx = Arc::new(Panel {
        robot: robot.clone(),
        learner,
        path: std::path::PathBuf::from(&path),
    });
    http::serve(listener, move |request| handle(&ctx, request));
    Ok(())
}

/// What a request handler needs: the machine, the recorder, and the file the
/// machine was described by.
struct Panel {
    robot: Arc<Robot>,
    learner: RangeLearner,
    /// The description this panel was started with. Edited limits go back to
    /// exactly this file and nowhere else.
    path: std::path::PathBuf,
}

fn handle(ctx: &Panel, request: &Request) -> Response {
    let robot: &Robot = &ctx.robot;
    match (request.method.as_str(), request.path.as_str()) {
        ("GET", "/") | ("GET", "/index.html") => Response::html(INDEX),
        ("GET", "/app.js") => Response::javascript(APP_JS),
        ("GET", "/style.css") => Response::css(STYLE),
        ("GET", "/api/state") => Response::json(state_json(robot)),

        ("POST", "/api/pose") => match parse_pose(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(targets) => {
                let pose: std::collections::BTreeMap<String, Rad> = targets
                    .into_iter()
                    .map(|(name, value)| (name, Rad(value)))
                    .collect();
                match robot.set_positions(&pose) {
                    Ok(()) => Response::json("{\"ok\":true}".into()),
                    Err(error) => Response::refused(&error.to_string()),
                }
            }
        },

        // A profiled move: the sliders set a target directly, this plans one.
        // Both the per-joint "go" and the whole-body buttons come through here,
        // because "every joint to zero" and "one joint to zero" differ only in
        // how many names are in the request.
        ("POST", "/api/move") => match parse_move(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(request) => {
                let result = match &request.pose {
                    Some(pose) => robot.move_to_pose(
                        pose,
                        RadPerSec(request.max_velocity),
                        RadPerSec2(request.accel),
                    ),
                    None => {
                        let targets = request
                            .targets
                            .iter()
                            .map(|(name, &value)| (name.clone(), Rad(value)))
                            .collect();
                        robot.move_many(
                            &targets,
                            RadPerSec(request.max_velocity),
                            RadPerSec2(request.accel),
                        )
                    }
                };
                match result {
                    Ok(()) => Response::json("{\"ok\":true}".into()),
                    Err(error) => Response::refused(&error.to_string()),
                }
            }
        },

        // Teach mode. Releasing the joints is the point rather than a side
        // effect: a joint under torque holds its target, so the only travel it
        // would report is the one the loop asked for.
        ("POST", "/api/learn/start") => match parse_learn(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(seconds) => match ctx.learner.start(ctx.robot.clone(), seconds) {
                Ok(()) => Response::json("{\"ok\":true}".into()),
                Err(reason) => Response::refused(&reason),
            },
        },
        ("POST", "/api/learn/stop") => {
            ctx.learner.stop();
            Response::json("{\"ok\":true}".into())
        }
        ("POST", "/api/learn/clear") => {
            ctx.learner.clear();
            Response::json("{\"ok\":true}".into())
        }
        ("GET", "/api/learn") => Response::json(learn_json(&ctx.learner, robot)),

        // Edited limits go back to the description. Nothing here changes the
        // running machine: the mapping and the envelope were built when it came
        // up, and rebuilding them underneath a live joint is not something to
        // do from a web request.
        ("POST", "/api/limits") => match parse_limits(&request.body) {
            Err(reason) => Response::refused(&reason),
            Ok(edits) => match Config::write_limits(&ctx.path, &edits) {
                Ok(_) => Response::json(format!(
                    "{{\"ok\":true,\"written\":{},\"restart_required\":true}}",
                    quote(&ctx.path.display().to_string())
                )),
                Err(error) => Response::refused(&error.0),
            },
        },

        ("POST", "/api/estop") => {
            robot.estop("the panel's E-STOP was pressed");
            Response::json("{\"ok\":true}".into())
        }
        ("POST", "/api/reset") => match robot.reset_estop() {
            Ok(()) => Response::json("{\"ok\":true}".into()),
            Err(error) => Response::refused(&error.to_string()),
        },
        ("POST", "/api/hold") => {
            let mut failures = Vec::new();
            for name in robot.joint_names() {
                if let Err(error) = robot.hold(name) {
                    failures.push(format!("{name}: {error}"));
                }
            }
            if failures.is_empty() {
                Response::json("{\"ok\":true}".into())
            } else {
                Response::refused(&failures.join(", "))
            }
        }
        ("POST", "/api/stop") => {
            for name in robot.joint_names() {
                let _ = robot.stop(name);
            }
            Response::json("{\"ok\":true}".into())
        }

        _ => Response::not_found(),
    }
}

/// The whole machine, as the panel wants it.
///
/// Hand-written rather than derived, because the panel's shape and the
/// framework's are not the same thing and pinning one to the other means every
/// internal rename is a UI change.
fn state_json(robot: &Robot) -> String {
    let states = robot.state();
    let mut joints = Vec::with_capacity(states.len());

    for state in &states {
        let config = robot
            .config
            .joint(&state.name)
            .expect("a state exists for every configured joint");

        let number = |value: Option<f64>| match value {
            // null, not zero. A driver that did not read a torque reports
            // nothing, and 0.000 Nm would be a different claim entirely.
            Some(value) if value.is_finite() => format!("{value}"),
            _ => "null".into(),
        };
        let blocked: Vec<String> = state.blocked.iter().map(|line| quote(line)).collect();

        joints.push(format!(
            "{{\"name\":{},\"driver\":{},\"bus\":{},\"id\":{},\
              \"min_rad\":{},\"max_rad\":{},\
              \"armed\":{},\"estop\":{},\"estop_reason\":{},\"mode\":{},\
              \"position_rad\":{},\"target_rad\":{},\"torque_nm\":{},\
              \"voltage_v\":{},\"rate_hz\":{},\"fault_name\":{},              \"moving_to_rad\":{},\"progress\":{},\"blocked\":[{}]}}",
            quote(&state.name),
            quote(&config.driver),
            quote(&config.bus),
            config.id,
            config.limits.min_rad,
            config.limits.max_rad,
            state.armed,
            state.estop,
            quote(&state.estop_reason),
            quote(state.mode.as_str()),
            number(state.position_rad),
            number(Some(state.target_rad)),
            number(state.torque_nm),
            number(state.voltage_v),
            number(Some(state.rate_hz)),
            quote(&state.fault_name),
            number(state.moving_to_rad),
            number(Some(state.progress)),
            blocked.join(",")
        ));
    }

    // The poses come from the description, so the panel grows a button when the
    // machine grows a pose and nothing has to be rebuilt.
    let poses: Vec<String> = robot.pose_names().iter().map(|name| quote(name)).collect();

    format!(
        "{{\"robot\":{},\"rate_hz\":{},\"connected\":{},\"poses\":[{}],\"joints\":[{}]}}",
        quote(&robot.config.robot),
        robot.config.rate_hz,
        !states.is_empty(),
        poses.join(","),
        joints.join(",")
    )
}

/// What teach mode has seen, next to what the description says today.
fn learn_json(learner: &RangeLearner, robot: &Robot) -> String {
    let status = learner.status(robot);
    let number = |value: Option<f64>| match value {
        Some(value) if value.is_finite() => format!("{value}"),
        _ => "null".into(),
    };

    let joints: Vec<String> = status
        .joints
        .iter()
        .map(|joint| {
            format!(
                "{{\"name\":{},\"min_rad\":{},\"max_rad\":{},\"current_rad\":{},                  \"span_rad\":{},\"samples\":{},                  \"configured_min_rad\":{},\"configured_max_rad\":{},\"direction\":{}}}",
                quote(&joint.name),
                number(joint.min_rad),
                number(joint.max_rad),
                number(joint.current_rad),
                number(joint.span_rad()),
                joint.samples,
                joint.configured_min_rad,
                joint.configured_max_rad,
                joint.direction,
            )
        })
        .collect();

    format!(
        "{{\"running\":{},\"elapsed_s\":{:.2},\"remaining_s\":{:.2},\"joints\":[{}]}}",
        status.running,
        status.elapsed_s,
        status.remaining_s,
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
        // A native dialog blocks the page, and a panel showing a live machine
        // must not stop rendering because one is open.
        assert!(
            !APP_JS.contains("confirm(") && !APP_JS.contains("alert("),
            "the panel must not use a blocking browser dialog"
        );
    }

    /// Every field the panel reads has to be one the panel sends. A rename on
    /// this side is a widget that silently renders nothing.
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

    /// The E-STOP is the one control that must never be hunted for, so it is
    /// pushed to its own end of the row and coloured.
    #[test]
    fn the_estop_is_not_one_button_among_others() {
        assert!(STYLE.contains("margin-left: auto"));
        assert!(INDEX.contains("btn btn-estop"));
    }

    /// A class assignment must not drop the class the element was found by.
    ///
    /// Found in a browser on this bench, and only there. `render` looked up
    /// `.joint-armed`, then assigned `className = "pill pill-ok"` — which threw
    /// the marker away. The first render worked; every one after it hit a null
    /// and threw halfway through, so the panel sat on stale numbers with the
    /// connection pill flickering to "disconnected" while the machine was
    /// perfectly alive.
    #[test]
    fn restyling_an_element_keeps_the_class_it_was_found_by() {
        for marker in ["joint-armed", "joint-mode"] {
            assert!(
                APP_JS.contains(&format!(".{marker}")),
                "{marker} is never looked up"
            );
            assert!(
                APP_JS.contains(&format!("\"{marker} pill \"")),
                "the {marker} pill drops its own marker class when it is restyled, so the second                  render cannot find it"
            );
        }
    }

    /// Nothing safety-related may be drawn from what the operator pressed.
    /// Armed, e-stopped and faulted come only from state the robot sent.
    #[test]
    fn the_script_renders_safety_state_only_from_the_robot() {
        // The E-STOP handler posts and does nothing else — no local repaint.
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
}
