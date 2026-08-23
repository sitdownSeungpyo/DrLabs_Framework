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

use drlabs_core::units::Rad;
use drlabs_robot::{Config, Robot};

use crate::flags::{Flags, Kind};
use http::{parse_pose, quote, Request, Response};

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

    // The deadman wants to hear from a supervisor. The panel polls, but a
    // browser that has been closed does not, and the machine should not be left
    // energised waiting for one — so the heartbeat comes from here, and the
    // panel's own liveness is what the operator watches.
    let beating = robot.clone();
    std::thread::spawn(move || loop {
        beating.heartbeat();
        std::thread::sleep(Duration::from_millis(200));
    });

    let serving = robot.clone();
    http::serve(listener, move |request| handle(&serving, request));
    Ok(())
}

fn handle(robot: &Robot, request: &Request) -> Response {
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
              \"voltage_v\":{},\"rate_hz\":{},\"fault_name\":{},\"blocked\":[{}]}}",
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
            blocked.join(",")
        ));
    }

    format!(
        "{{\"robot\":{},\"rate_hz\":{},\"connected\":{},\"joints\":[{}]}}",
        quote(&robot.config.robot),
        robot.config.rate_hz,
        !states.is_empty(),
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
