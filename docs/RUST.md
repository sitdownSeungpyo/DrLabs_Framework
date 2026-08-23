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
* **`legacy/dxl` is ported, not preserved as a second tree.** The Go tree
  carried three Dynamixel layers that had accumulated rather than been
  designed: `legacy/dxl`, `drivers/dynamixel/driver.go`, and the `Link` that
  replaced them. Every *capability* of the older two is in the Rust driver —
  register access, ping, reboot, factory reset, and the sync read and sync
  write that let a daisy chain move on one period instead of one per servo.
  What is gone is the goroutine-and-channel calling convention the oldest layer
  wrapped around them. That was a way of calling the driver, not something the
  driver could do, and a second copy of it would mean two places to fix the
  next time the wire turns out to work differently than anyone thought.

## Progress

| Crate | State | Tests |
|---|---|---|
| `drlabs-core` | done, plus the unit renderer | 23 passing |
| `drlabs-timing` | done | 7 passing |
| `drlabs-transport` | serial (3 platforms), CAN, fdcanusb | 27 passing |
| `drlabs-moteus` | complete: protocol, frames, driver, console | 66 passing |
| `drlabs-dynamixel` | complete: protocol, driver, sync read/write, link | 30 passing |
| `drlabs-robstride` | complete: protocol, parameters, controller, link | 46 passing |
| `drlabs-odrive` | **implemented, never bench tested** — see below | 23 passing |
| `drlabs-safety` | complete: clamps, trips, presets, preflight | 27 passing |
| `drlabs-session` | complete: loop, state machine, trajectory, shuttle | 40 passing |
| `drlabs-robot` | complete: description, joint mapping, atomic pose | 31 passing |
| `drlabs-cli` | ports, drivers, probe, run, bringup, panel | 17 passing |
| `drlabs-panel` | folded into the CLI, assets compiled in | — |

A crate joins the workspace `members` list when it builds and its tests pass,
so `cargo test` on this branch is always green.

`drlabs-sim` is one simulator, not one per driver. The Go tree had a `sim.go` in
the moteus package and another in the robstride one; the physics in both was the
same rotor, the same friction and the same inner loop, differing only in what
they called their modes. Two copies of an integrator is two places for it to
drift.

### The Go tree is gone from this branch

Every feature it had is here, and each was exercised on hardware or against the
simulator before the source was removed — including the two that were missing
when the port was first called complete: the simulator itself, and the servo
role (`drlabs serve`, newline-delimited JSON over TCP).

It is untouched on `main`, and in this repository's history. Keeping both on one
branch would mean two places to fix the next time a wire detail turns out to
work differently than anyone thought, which is the opposite of why this port
was done.

### The rate ceiling is checked by reproducing the runaway

The worst defect the Go version had on this bench was not a wrong number, it
was a loop that obeyed its callers instead of its configuration: a wake ran a
cycle immediately, so a slider drag drove a 100 Hz robot's CAN joint at 801 Hz
and its serial joint at 345 Hz, until the servo's own watchdog latched and
e-stopped it. A machine stopped by the rate of somebody's mouse.

`crates/drlabs-session/tests/loop_behaviour.rs` floods a session with commands
for six hundred milliseconds and asserts the loop stayed near its configured
rate. Removing the ceiling and re-running it gives **520,564 Hz against a
50 Hz configuration** — so the test is known to catch the thing it was written
for, rather than merely passing.

### Every platform's serial code is at least compiled

The Go tree had a Linux and a macOS serial implementation that nothing on this
bench had built since they were written. Here `cargo check --target
x86_64-unknown-linux-gnu` and `--target aarch64-apple-darwin` both pass, so the
termios path is known to compile even though it cannot be exercised here. That
is not the same as knowing it works, and it is not claimed to be.

### The reference bytes match

The eleven pinned moteus frames — stop, brake, query-only, a minimal position
cycle, a full parameter set, a deferred torque limit, velocity with its NaN
position, set-output and stay-within — encode byte for byte as the official
`moteus` Python package produces them. That is the evidence the port is
faithful at the wire level, and it was worth having: the first version of the
combiner wrote every subframe header before any value, producing a frame with
all the framing at the front. Nothing about it looked wrong, and the bytes
caught it in one run.

### ODrive is now code, and says what it is

In the Go tree `drivers/odrive` was eight lines of doc comment: the CAN
addressing scheme, the shape of an ASCII command, and no implementation. The
Rust crate implements what those eight lines described — CANSimple's
`(node_id << 5) | command_id` and its message layouts, and the line protocol —
with tests pinned against the published documentation.

That is a real standard, and not the same as the others. No ODrive has been on
this bench. So the driver reports its board status as unknown and puts "never
bench tested" in the board name, and it claims no envelope at all: an ODrive is
a controller rather than a motor, and its board ratings are not the axis's.
Everything else here has been watched moving; this has not, and the code says
so rather than leaving an operator to assume.

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
XL330-M288-T at COM6.

| | result |
|---|---|
| moteus identify — registers, and travel bounds through the console | r4.11, fw 0.0.5, −5.0 … 5.0 rev |
| Dynamixel identify — model number chooses the profile, undeclared | XL330-M288-T, 4.7 V, 3.7–6.0 V range |
| moteus loop | 100 Hz sustained, no dropped replies |
| Dynamixel loop | 100 Hz sustained |
| moteus motion — shuttle 0 ↔ 0.35 rev | arrives within 0.9 mrev, 0.7 rev/s cruise |
| Dynamixel motion — shuttle 0.65 ↔ 0.90 rev | arrives at both ends |
| both joints at once, per-bus rates | CAN 200 Hz **and** serial 100 Hz, together |
| flood: poses as fast as a thread can send them | both rates unchanged, nothing tripped |
| position clamping, both directions | ±5.0 rev commanded → ±0.199 rev against a ±0.2 ceiling |
| hard kill while armed, then reconnect | re-arms; the latched bus watchdog is cleared |
| the panel, in a browser | both joints driven from sliders, E-STOP and recovery |

The flood row is the one worth reading. In the Go version that exact scenario
drove the CAN joint to 801 Hz and the serial joint to 345 Hz until the servo's
own watchdog latched and e-stopped it.

### What the bench found that the tests had not

Two defects, both invisible off the hardware, both now pinned by a test that
fails without the fix.

**An unset register decoded as a very large number.** The verify query reads the
controller's live command registers; a velocity limit a moteus has never been
given comes back as the int32 NaN sentinel. Decoded as a plain integer and
scaled, that is −21474.836 rev/s — finite, plausible-looking, and wrong. The
session compared it against the 1.000 it had set, concluded the controller had
lost the parameter, and re-sent the whole set on every verify cycle. Nothing
looked broken: the axis held, the loop kept 100 Hz, and the log filled with a
mismatch that was not one. `nanify` existed; the reader simply never called it.

**A restyled element lost the class it was found by.** Only a browser has this
one. `render` looked up `.joint-armed`, then assigned `className = "pill
pill-ok"` — throwing the marker away. The first render worked; every one after
it hit a null and threw halfway through, so the panel sat on stale numbers with
its connection pill flickering to "disconnected" while the machine was
perfectly alive and the server was answering every request with a 200.

**Connecting never opened the link.** `Session::connect` described and
identified the link but never called `open`, which is where a driver takes
control of what the last session left — a moteus still driving, a Dynamixel
with a latched watchdog — and where it tunes the transport. The servo answered,
armed and held position, so nothing reported an error. It just ran at 44 Hz
against a 100 Hz configuration, because an unopened port keeps its default
ten-millisecond read timeout and the loop reads twice a cycle. Opening it took
the same loop to exactly 100.

### One binary, 0.7 MB

`cargo build --release` produces a single `drlabs.exe` of 0.7 MB: four drivers,
serial and CAN transports for three platforms, the safety layer, the control
loop, the robot layer, the command line and the web panel with its assets
compiled in. Nothing else has to be on the target — which was the point.

The release binary was run against the live pair, not just built.

Nothing here is claimed as working on the strength of it having compiled.
