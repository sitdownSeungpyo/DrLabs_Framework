//! The framework's command line front end.
//!
//! ```text
//! drlabs ports                       list serial ports, CAN adapters first
//! drlabs drivers                     list the drivers this binary carries
//! drlabs probe   -driver … -id …     connect, identify, report, disconnect
//! drlabs run     -driver … -id …     run a control session on one axis
//! drlabs bringup [-check] robot.json bring a whole machine up
//! drlabs panel   robot.json          bring it up and drive it from a browser
//! drlabs serve   -driver … -id …     the servo role: energised, over a socket
//! ```
//!
//! `probe` goes through the same `Link` a driver test would; `run`, `bringup`
//! and `rt` start the runtime in this process and are clients of it, so
//! what the CLI can do is a lower bound on what the framework can do.

mod bringup;
mod flags;
mod machine;
mod panel;
mod probe;
mod rt;
mod serve;

use std::process::ExitCode;

use flags::{Flags, Kind};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(command) = args.first() else {
        usage();
        return ExitCode::from(2);
    };

    let rest = &args[1..];
    let result = match command.as_str() {
        "ports" => probe::ports(),
        "drivers" => probe::drivers(),
        "probe" => probe::probe(rest),
        "run" => probe::run(rest),
        "bringup" => bringup::bringup(rest),
        "panel" => panel::panel(rest),
        "rt" => rt::rt(rest),
        "serve" => serve::serve(rest),
        "-h" | "--help" | "help" => {
            usage();
            return ExitCode::SUCCESS;
        }
        other => {
            eprintln!("drlabs: unknown command {other:?}\n");
            usage();
            return ExitCode::from(2);
        }
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("drlabs: {error}");
            ExitCode::FAILURE
        }
    }
}

/// The flags every connecting subcommand shares.
pub const COMMON_FLAGS: &[(&str, Kind)] = &[
    ("driver", Kind::Value),
    ("port", Kind::Value),
    ("baud", Kind::Value),
    ("id", Kind::Value),
    ("opt", Kind::Repeated),
];

/// What one axis is being asked to connect to.
pub struct Target {
    pub driver: String,
    pub port: String,
    pub baud: u32,
    pub id: u8,
    pub options: std::collections::BTreeMap<String, String>,
}

impl Target {
    pub fn from(flags: &Flags) -> Result<Self, String> {
        Ok(Self {
            driver: flags.string("driver", "moteus"),
            port: flags.string("port", ""),
            baud: flags.number("baud", 1_000_000u32)?,
            id: flags.number("id", 1u8)?,
            options: flags.pairs("opt"),
        })
    }
}

fn usage() {
    eprint!(
        r#"drlabs — multi-vendor motor control framework

Usage:
  drlabs ports                       List serial ports, CAN adapters first
  drlabs drivers                     List the drivers this binary carries
  drlabs probe   [flags]             Connect, identify, print state, disconnect
  drlabs run     [flags]             Run a control session on one axis
  drlabs bringup [flags] <robot.json>
                                     Bring a whole machine up from a
                                     description and hold it there
  drlabs panel   [-addr host:port] [-domain N] [-prefix name] [robot.json]
                                     A browser panel on a running `drlabs rt`;
                                     robot.json only for writing edited limits
  drlabs serve   [flags]             The servo role: energise one axis and take
                                     newline-delimited JSON commands over TCP
  drlabs rt      [-check] [-seconds N] [-drive] <robot.json>
                                     The real-time runtime: one thread per bus,
                                     LowCmd/LowState records; -check validates
                                     the description and opens nothing

Common flags:
  -driver   moteus | dynamixel | robstride | odrive | sim   (default moteus)
  -port     serial port; empty auto-detects a CAN adapter
  -baud     serial baud rate, Dynamixel only (default 1000000)
  -id       bus address: CAN id or servo id (default 1)
  -opt      driver option, repeatable: -opt key=value

run flags (the axis is driven through the runtime, as a one-joint machine):
  -rate     bus rate in Hz (default 100)
  -seconds  how long to run; 0 runs until Ctrl+C (default 0)
  -position move to this target, in radians at the shaft; none holds
  -shuttle  A,B,dwell — move A↔B forever, dwell seconds at each end
  -min      travel envelope, lower bound in rad (default -π, half a turn)
  -max      travel envelope, upper bound (default π)
  -torque   torque ceiling in Nm (default 0.3)
  -velocity velocity ceiling and move speed in rad/s (default 6.0)
  -accel    acceleration ceiling and move ramp in rad/s² (default 10.0)

serve flags:
  -addr             address to listen on (default 127.0.0.1:7777)
  -rate             control rate in Hz (default 200)
  -state-rate       unsolicited state lines a second (default 50, 0 disables)
  -max-torque       torque ceiling in Nm (default 0.3)
  -velocity-limit   velocity ceiling in rad/s (default 6.0)
  -accel-limit      acceleration ceiling in rad/s² (default 30.0)
  -position-min     soft envelope, lower bound in rad (default -π)
  -position-max     soft envelope, upper bound (default π)
  -command-timeout  seconds of silence after which a velocity or torque
                    command reverts to holding position (default 1, 0 disables)
  -deadman          seconds without a client before stopping (0 disables)
  -no-auto-enable   connect but do not energise until told to

panel flags:
  -addr     address to listen on (default 127.0.0.1:8080)
  -check    validate the description and exit

bringup flags (the machine runs on the runtime; this process is its client):
  -check    validate the description and print the plan, opening nothing
  -seconds  how long to hold the machine up; 0 runs until Ctrl+C
  -sweep    drive every joint through this fraction of its travel, together
  -flood    command poses as fast as possible; the bus keeps its own rate
"#
    );
}
