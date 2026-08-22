# The Rust port

The framework is being moved to Rust on `feature/rust`. This file is the plan
and the running record of what is done, so the branch can be read without
guessing what is finished and what is scaffolding.

## Why

Not for speed — the Go version already holds 200 Hz on real hardware with
0.33 ms of jitter. Three other reasons:

**One binary.** The goal is that a robot is a JSON description and an
executable: copy two files onto a machine, run it, and the axis is under
control. No toolchain, no runtime, no package manager on the target. The
release profile is set up for that — fat LTO, one codegen unit, panics that
abort, symbols stripped — and the panel's assets are compiled in rather than
read from disk.

**Memory safety**, on code that drives physical hardware.

**Units that cannot be mixed up.** This is the one that came from a bug. The Go
version carried positions as bare `f64` and kept motor space and joint space
apart by naming and comments; a message built in the safety layer reported a
joint's position in revolutions to somebody who had written the limit in
radians, and at the precision printed the two numbers looked identical. Here
`Rev` and `Rad` are different types and the conversion has to be written down.

The same reasoning gets applied to the other place the Go version needed a
paragraph of documentation to stay safe: NaN. It meant both "not reported" and
"do not send" and, in one register, "hand control back to the controller".
Those are now `Option`, `Field::Unchanged` and `TorqueLimit::Controller` — three
shapes, so the wrong one cannot be written.

## What is deliberately *not* expected to improve

Windows serial timeouts and timer resolution are direct Win32 calls either way.
The risk in porting them is not that Rust is worse, it is that the behaviour
found the hard way on the bench — a 1 ms read timeout, clearing a latched bus
watchdog at open, capping the loop at the configured rate — gets quietly left
out. Each of those has a test in the Go tree, and each one is ported with its
test.

## Layout

```
Cargo.toml                     workspace
crates/
  drlabs-core/                 units, Command, Telemetry, Link trait, errors
  drlabs-timing/               scheduling accuracy (per platform)
  drlabs-transport/            serial (per platform), CAN abstraction, fdcanusb
  drlabs-safety/               envelope, monitor, trips
  drlabs-session/              one-axis control loop, trajectory, shuttle
  drlabs-robot/                description to machine, joint mapping
  drivers/
    drlabs-moteus/
    drlabs-dynamixel/
    drlabs-robstride/
    drlabs-odrive/
apps/
  drlabs-cli/                  probe / run / serve / bringup
  drlabs-panel/                web panel, assets embedded
configs/                       robot descriptions
tools/ros2_bridge/             ROS 2 package, stays Python
docs/
```

Three changes from the Go tree, all aimed at reading it six months from now:

* **Applications are not libraries.** `apps/` holds the things you run,
  `crates/` the things they are built from. The Go tree had two web
  applications and a Python client living under `test/`, which is neither where
  you look for them nor what they are.
* **Descriptions live in one place.** `configs/`, rather than split between
  `test/*.json` and `examples/robots/*.json`.
* **`legacy/dxl` is ported, not preserved as a second tree.** It exists in Go
  because callers had not moved over yet; the Rust tree has one Dynamixel
  driver.

## Progress

| Crate | State | Tests |
|---|---|---|
| `drlabs-core` | done | 19 passing |
| `drlabs-timing` | done | 7 passing |
| `drlabs-transport` | not started | |
| `drlabs-moteus` | protocol and frames done — **reference bytes match**; driver, console to come | 38 passing |
| `drlabs-dynamixel` | not started | |
| `drlabs-robstride` | not started | |
| `drlabs-odrive` | not started | |
| `drlabs-safety` | not started | |
| `drlabs-session` | not started | |
| `drlabs-robot` | not started | |
| `drlabs-cli` | not started | |
| `drlabs-panel` | not started | |

A crate joins the workspace `members` list when it builds and its tests pass,
so `cargo test` on this branch is always green.

### The reference bytes match

The eleven pinned moteus frames — stop, brake, query-only, a minimal position
cycle, a full parameter set, a deferred torque limit, velocity with its NaN
position, set-output and stay-within — encode byte for byte as the official
`moteus` Python package produces them. That is the evidence the port is
faithful at the wire level, and it was worth having: the first version of the
combiner wrote every subframe header before any value, producing a frame with
all the framing at the front. Nothing about it looked wrong, and the bytes
caught it in one run.

## How it gets verified

Two levels, and the first one carries most of the weight.

**Off the bench.** The protocol encoders have expectations pinned against
reference implementations — the moteus frames against output from the official
`moteus` Python package, byte for byte. Those port directly as Rust tests with
the same hex strings, which proves the port is faithful at the wire level
without any hardware attached. Everything derived from measurements taken on
the bench (a `%.3f` that hid a bound, a rate that ran away, a latched watchdog)
ports with its test as well.

**On the bench.** The same procedure the Go version was put through, on the
same hardware: a moteus r4.11 on an fdcanusb at COM5 and a Dynamixel
XL330-M288-T at COM6. Connect and identify, every control mode, position
clamping in both directions, e-stop and reset, a sustained loop at the
configured rate with no dropped replies, and two joints driven at once from the
panel.

Nothing is claimed as working on the strength of it having compiled.
