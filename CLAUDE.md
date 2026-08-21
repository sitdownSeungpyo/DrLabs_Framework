# CLAUDE.md

Project instructions for DrLabs_Framework. The cross-cutting engineering rules
live in `miles-dev-skills/CLAUDE.md` — hardware is unforgiving, real-time paths
stay clean, debug layer by layer, design for disconnection, ship with a
rollback path, verify on target. This file is what those rules mean *here*, and
the handful of things about this codebase that are expensive to rediscover.

## What this is

A multi-vendor motor control SDK in Go. One `motor.Link` surface; drivers
translate it to each vendor's wire protocol; a runtime adds the control loop,
the safety envelope and whole-machine bring-up.

```
motor/       the vendor-neutral surface: Link, Command, Telemetry, units
drivers/     moteus, robstride, dynamixel, odrive
transport/   fdcanusb, serial, CAN abstraction, shared-bus locking
runtime/     session (loop, safety, trajectory) and robot (bring-up from JSON)
cmd/drlabs/  probe, run, serve, bringup
legacy/dxl/  the original single-package driver, kept working
```

## Two spaces, one conversion point

Everything below `runtime/robot` works in **motor space**: revolutions at the
shaft, torque at the rotor. Everything above it is authored in **joint space**:
radians at the output, torque at the mechanism. A gear ratio, a sign and a zero
offset separate them.

`runtime/robot/joint.go` (`Mapping`) is the only place that converts. Do not add
a second one. The same axis is legitimately `0.477 rev` and `3.000 rad` at the
same instant, so a number without its space is not a fact.

Anything addressed to a joint must be *said* in joint units. `motor.Units`
renders text at the point it is built, and `runtime/robot` supplies a joint
renderer; a nil renderer means motor space, which is what a single-axis session
speaks. `Position` and `Span` are separate because a place carries the offset
and a distance does not.

## The protocol tests are pinned, not descriptive

`TestFramesMatchReferenceEncoding` compares moteus frames byte for byte against
output from the official `moteus` Python package. If it fails, the encoder
changed — regenerate the expectations from the reference implementation, never
edit them to match new behaviour.

When touching anything under `runtime/` or `motor/`, prove the wire did not
move: dump the payloads before and after and diff them. It takes a minute and
it is the difference between a display change and a silent protocol change.

## Verify on the bench, not in your head

This framework's failures do not announce themselves. Every defect found so far
looked like working software: a servo that armed and then refused its first
command, a loop that ran at 67 Hz while reporting success, a config value read
back from the wrong key. Compiling proves nothing; `go test` proves the parts
that do not touch hardware.

The bench, when connected:

| | port | notes |
|---|---|---|
| mjbots qdd100 (moteus r4.11) | COM5, fdcanusb, CAN id 1 | 19:1 reduction, position reported at the output |
| ROBOTIS XL330-M288-T | COM6, serial, 1 Mbaud, id 1 | 5 V, 1 mA current unit |

`test/single_moteus.json`, `test/single_xl330.json` and `test/bench_pair.json`
bring these up. `bringup -check` validates a description without opening
anything.

Two habits worth keeping:

- **Release the hardware.** A killed process leaves a Dynamixel's bus watchdog
  latched and a moteus energised. `Open` now clears the watchdog, but a clean
  stop is still the difference between a bench you can pick up and one you have
  to debug.
- **Give the axis room to be seen.** An envelope of ±0.5 rad through a 19:1
  reduction is ±29° at the output, and a quarter of that travel is easy to
  watch happen and not notice. If somebody is meant to observe the motion, make
  it large enough to observe.

## Driver notes

Each driver has a `docs/` file recording the wire details that cost time:
[MOTEUS.md](docs/MOTEUS.md), [DYNAMIXEL.md](docs/DYNAMIXEL.md),
[ROBSTRIDE.md](docs/ROBSTRIDE.md), [SERVO.md](docs/SERVO.md),
[ROBOT.md](docs/ROBOT.md), [BRINGUP.md](docs/BRINGUP.md). When a hardware
session teaches something that is not in the code, it goes there.

The working tree is checked out with CRLF line endings. `gofmt -l` therefore
flags every file; that is the checkout, not the formatting. Judge formatting on
the files you changed.

## Commits

```
[태그] 한 줄 요약 — 세부 변경 사항을 쉼표로 나열
```

Tags: `[feat]` `[fix]` `[refactor]` `[docs]` `[test]` `[build]` `[chore]`.
Korean subject, imperative, ≤ 72 characters. Add a body only when the diff does
not carry the "why" — and for anything found on hardware, it usually does not:
record the measurement.

Never add Claude or AI attribution to a commit, PR body, tag or release note.
