# DrLabs Framework — a pure Go motor control framework

One vendor-neutral API, a real-time control loop, and a host-side safety layer,
for brushless controllers and smart servos alike.

**No Cgo. No DLLs. No vendor SDKs.** Every protocol — Dynamixel Protocol 2.0,
the mjlib multiplex register protocol, the RobStride CAN 2.0B protocol, CAN and
CAN-FD framing, the fdcanusb line protocol, and the platform serial ports
underneath them — is implemented in this repository in pure Go.

```
go build ./...
go run ./cmd/drlabs run -driver moteus-sim -mode position -position 0.25
```

That last command runs the complete stack — state machine, control loop, safety
monitor, driver, simulated physics — with nothing plugged in.

---

## What this is

**A low-level motor layer.** Something above it decides what the machine does;
its job is to be energised, waiting, and to execute what it is told — within an
envelope nothing above it can exceed. That role is `drlabs serve`, and
`docs/SERVO.md` describes it.

The operator-facing paths — `drlabs run`, the web UI — are the same runtime
with a human in the loop instead: they start disarmed and make someone read the
preflight checklist first.

The starting point was a Dynamixel SDK. A framework needs three things the SDK
did not have:

1. **A seam that is not vendor-shaped.** `motor.Link` is the only interface the
   runtime is written against. Applications say "go to 0.25 revolutions", not
   "write 1024 to control table address 116" or "set register 0x020 to 25000".
2. **A control loop that keeps its period.** Motor control is a real-time
   problem. A loop that silently runs at 64 Hz while claiming 100 Hz makes every
   watchdog timeout derived from the nominal rate wrong.
3. **A safety layer that protects the machine.** Controller firmware protects
   the *board*. The motor, the gearbox and whatever they are bolted to are not
   the controller manufacturer's products, so their envelope has to be declared
   by the operator and enforced on the host.

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│  Application            cmd/drlabs · examples/ · your code     │
├────────────────────────────────────────────────────────────────┤
│  runtime/session        state machine · fixed-rate loop        │
│                         telemetry history · event log          │
│  runtime/safety         command clamps · trip conditions       │
│  runtime/timing         1 ms timers · jitter measurement       │
├────────────────────────────────────────────────────────────────┤
│  motor                  Link · Command · Telemetry · Registry  │
│                         ── the only vendor-neutral seam ──     │
├────────────────────────────────────────────────────────────────┤
│  drivers/moteus         multiplex protocol · frames · console  │
│  drivers/dynamixel      Protocol 2.0 · control tables · units  │
│  drivers/robstride      CAN 2.0B · per-model scaling · params  │
│  drivers/odrive         (declared, not yet implemented)        │
├────────────────────────────────────────────────────────────────┤
│  transport/fdcanusb     CAN-FD over USB, ASCII line protocol   │
│  transport/can          CAN / CAN-FD frames and bus interface  │
│  transport/serial       Win32 · termios · IOSSIOSPEED          │
└────────────────────────────────────────────────────────────────┘
```

## Layout

| Path | What lives there |
|------|------------------|
| `motor/` | `Link`, `Command`, `Telemetry`, `Capabilities`, units, driver registry |
| `drivers/moteus/` | mjbots moteus: multiplex protocol, frame builders, diagnostic console, simulator |
| `drivers/dynamixel/` | ROBOTIS Dynamixel: Protocol 2.0 driver, control-table profiles, `Link` adapter |
| `drivers/robstride/` | RobStride RS00–RS06: CAN 2.0B protocol, per-model scaling, parameter table, simulator |
| `drivers/odrive/` | Declared interface, awaiting implementation |
| `transport/serial/` | Per-platform serial ports and port enumeration |
| `transport/can/` | CAN and CAN-FD frame types and the `Bus` interface |
| `transport/fdcanusb/` | mjbots fdcanusb adapter, including its optional line checksums |
| `runtime/session/` | Connection lifecycle, control loop, arming, emergency stop, A↔B shuttle |
| `runtime/safety/` | Operator envelope, command clamps, trip evaluation, sensitivity presets |
| `runtime/robot/` | A whole machine from one JSON description: joints, gearing, shared buses |
| `runtime/timing/` | High-resolution scheduling and loop statistics |
| `trajectory/` | Motor-agnostic trapezoidal motion profiles, driven by `session.StartTrajectory` |
| `cmd/drlabs/` | Command line front end, including the `serve` servo role |
| `test/webui/` | Browser control panel built on the runtime — see its own README |
| `examples/` | Runnable programs, `examples/framework` first; `examples/robots/` holds robot descriptions |
| `legacy/dxl/` | The original single-package Dynamixel SDK, kept working |
| `ros2/bridge/` | Planned gRPC bridge for ROS 2 |

## Units

Every shared interface speaks physical units. Drivers convert to and from their
own wire encodings internally, so application code never learns whether it is
talking to encoder counts or to 0.00001-revolution register steps.

| Quantity | Unit |
|----------|------|
| position | revolutions (rev) |
| velocity | revolutions per second (rev/s) |
| torque | newton-metres (Nm) |
| current | amperes (A) |
| voltage | volts (V) |
| temperature | degrees Celsius (°C) |
| time | seconds (s) |

A `float64` field carries `NaN` when the value is unknown or was not reported.
Use `motor.Valid` before doing arithmetic on one — an absent temperature sensor
must not read as a cold motor.

## Quick start

### Command line

```bash
drlabs drivers                     # what this binary can talk to
drlabs ports                       # serial ports, CAN adapters first

# The servo role: energise, then execute commands from an upper layer.
drlabs serve -driver moteus -port COM5 -id 1 -max-torque 0.2
#   {"cmd":"position","value":0.25}  ->  {"ok":true}

# A whole robot from one description, addressed by joint name.
drlabs bringup -check robot.json     # validate and print the plan
drlabs bringup robot.json
#   {"cmd":"pose","joints":{"r_sh_p":-0.3,"r_elb_p":-2.0}}

# Connect, identify, report, disconnect.  Commands nothing but a stop.
drlabs probe -driver moteus -port COM7 -id 1

# Drive an axis.
drlabs run -driver moteus -port COM7 -id 1 \
           -mode position -position 0.25 -max-torque 0.4

# Browser control panel (connection is made from the page).
go run ./test/webui -driver moteus -port COM5 -id 1

# Unattended A↔B endurance run.
drlabs run -driver moteus-sim -shuttle-a -0.25 -shuttle-b 0.25 \
           -shuttle-dwell 1.0 -shuttle-cycles 100
```

### Library

```go
s := session.New(session.Options{RateHz: 200})

err := s.Connect(ctx, motor.Config{
    Driver: "moteus", Port: "COM7", ID: 1,
}, false)

safety := s.Safety()
safety.MaxTorque = 0.5
safety.PositionMin, safety.PositionMax = -0.6, 0.6
s.SetSafety(safety)

if blocking, _ := s.Preflight(); len(blocking) > 0 {
    return fmt.Errorf("cannot arm: %s", blocking[0])
}
s.Arm(true)
s.SetMode(motor.CtrlPosition)
s.SetPosition(0.25)
```

See `examples/framework/main.go` for the complete version.

## Supported hardware

### mjbots moteus (`-driver moteus`)

Brushless controller on CAN-FD, reached through an fdcanusb adapter.

- Modes: stop, query, position, velocity, torque, hold (zero velocity), brake,
  stay-within, set-output
- Telemetry: mode, position, velocity, torque, q/d current, absolute position,
  power, bus voltage, board and motor temperature, fault, trajectory and homing
  state, plus the controller's live command registers
- The tunnelled diagnostic console, used to read `servopos.position_min/max`
- Fault and limiting codes decoded to a cause and a remedy

The frame encoder is pinned byte-for-byte against the official `moteus` Python
library in `drivers/moteus/frames_test.go`.

### ROBOTIS Dynamixel (`-driver dynamixel`)

Smart servos on Protocol 2.0 serial.

- Modes: stop, query, position (extended, multi-turn), velocity, torque, hold
- Telemetry: position, velocity, current, voltage, temperature, hardware error
- Control-table profiles for the X-series and XL330 families, with per-family
  current units
- The bus watchdog mapped to the framework's watchdog timeout

Torque is reported in newton-metres only when a motor-specific torque constant
is supplied (`-opt torque_constant=0.35`); otherwise it is reported as
unmeasured rather than fabricated.

### RobStride (`-driver robstride`)

Quasi-direct-drive actuators on classic CAN 2.0B at 1 Mbit/s, reached through
the same adapter as moteus.

All seven models are supported — RS00, RS01, RS02, RS03, RS04, RS05, RS06 — and
**the model is a required option**:

```bash
drlabs run -driver robstride -port COM7 -id 1 -opt model=RS03 \
           -mode position -position 0.25 -max-torque 8
```

That is not ceremony. The torque scale spans ±5.5 Nm on an RS05 to ±120 Nm on
an RS04, the gain ranges differ by a factor of ten between two halves of the
family, and **a RobStride cannot report its own model**. Configure the wrong one
and every torque command is scaled by that factor with nothing on the bus to
reveal it, so the driver refuses to guess and the session raises an advisory on
every arm.

- Modes: stop, query, position, velocity, torque, hold, set-output
- Position uses cyclic synchronous position mode by default, so the commanded
  velocity limit is honoured; `-opt position_mode=operation` selects the
  MIT-style mode for impedance control
- Brake and stay-within are reported as unsupported, because the controller has
  no equivalent
- Bus voltage and current live in the parameter table rather than the feedback
  frame, so they refresh on the wider query profiles

Written against the official manuals and tested against a simulator; see
`docs/ROBSTRIDE.md` for the protocol notes and the pre-power-up checklist.

### Simulator (`-driver moteus-sim`)

A trapezoidal trajectory generator feeding a torque-limited PD controller,
driving an inertia with viscous friction, with thermal rise and supply sag. It
exists so that every control path and every safety trip can be exercised — and
tested in CI — with no hardware attached.

## Safety

Two independent layers of protection, and this framework is the second one.

**Command clamps**, applied to every outgoing command before it reaches the bus:
position envelope, velocity ceiling, torque ceiling, gain scales, acceleration.

**Trips**, evaluated against telemetry every cycle, latching an emergency stop:

| Trip | Fires when |
|------|------------|
| `controller_fault` | The controller reports a latching fault |
| `over_voltage` / `under_voltage` | Bus voltage leaves the declared range |
| `over_current` | Current stays above the limit for longer than the trip time |
| `board_over_temperature` / `motor_over_temperature` | Either sensor exceeds its limit |
| `position_envelope` | The axis leaves the soft envelope plus its overshoot margin |
| `velocity_runaway` | Velocity exceeds the limit by the runaway factor, sustained |
| `stall` | Motion was commanded, current is high, and the axis is not moving |
| `comm_loss` | Too many consecutive cycles without a reply |
| `deadman` | The supervising client stopped sending heartbeats |
| `link_error` | The transport failed |

Three things about how these are designed are worth knowing:

- **Trips need duration, not a single sample.** A momentary current spike is
  what acceleration looks like. Every motion and electrical trip requires the
  condition to hold continuously.
- **The position envelope only trips while armed.** With the drive off, the
  operator's only way back inside is to move the axis by hand — latching on
  that would make the condition unrecoverable.
- **"Limiting" is not a fault.** Codes that say the output is being capped are
  surfaced as warnings. They are the single most useful thing to show when a
  motor accepts commands and does not move.

One-click sensitivity presets tighten or loosen all the motion trips together:
`conservative` for a first power-up with an unknown motor, `standard`,
`relaxed` for a machine that is known to overshoot.

## Real-time behaviour

Windows' default timer granularity is 15.6 ms. A naive 100 Hz loop overshoots
its period by more than the period itself, and runs at about 64 Hz. The runtime
raises the multimedia timer to 1 ms and spins the final approach to each
deadline.

Measured on Windows 11 with the simulator:

| Requested | Achieved | Jitter (σ) |
|-----------|----------|------------|
| 100 Hz | 100.0 Hz | 0.25 ms |
| 200 Hz | 200.0 Hz | 0.50 ms |

`Status().Stats` reports the rate actually achieved, not the one requested, and
the loop warns when it falls below 85% of target. A loop that cannot keep up is
the most common cause of surprising motor behaviour, and the only way to know is
to count cycles against the clock.

## Adding a driver

Implement `motor.Link` and register it:

```go
func init() {
    motor.Register("mymotor", "My controller over CAN", newLink)
}
```

The rest of the framework — the control loop, the safety monitor, the CLI, the
arming checklist — works without modification. Report honestly through
`Capabilities` and `Envelope`: a mode you claim and cannot deliver fails at
command time, and a rating you do not publish simply leaves the operator's
declared limits in force.

## Tests

```bash
go test ./...
```

Nothing in the suite needs hardware. The interesting parts:

- `drivers/moteus/frames_test.go` — every command frame compared byte-for-byte
  against output from the official `moteus` Python library. This is the only
  check that the encoder is compatible with real firmware rather than merely
  self-consistent, so regenerate it against the reference rather than adjusting
  it to match a change here.
- `drivers/moteus/multiplex_test.go` — varuint encoding, saturation, the NaN
  sentinel, subframe grouping, reply parsing including truncation and padding
- `runtime/safety/monitor_test.go` — every trip, including the cases that must
  *not* fire
- `drivers/robstride/protocol_test.go` — addressing against the manuals' own
  worked example, both byte orders, the per-model scaling table, fault bits
- `runtime/session/session_test.go` — the whole stack against the simulator:
  arming, emergency stop, fault handling, the shuttle, the deadman

Describing a machine is `docs/ROBOT.md`; bringing up a new axis is
`docs/BRINGUP.md`; the testing layers are in
`docs/TESTING.md`; the servo role is in `docs/SERVO.md`; the protocol notes are
in `docs/MOTEUS.md` and `docs/ROBSTRIDE.md`.

## Requirements

- Go 1.24+
- No external Go dependencies

For hardware: an fdcanusb (or compatible CAN-FD adapter) for moteus, a U2D2 or
OpenRB-150 for Dynamixel, and a supply matched to the motor.

## References

- [moteus reference](https://github.com/mjbots/moteus/blob/main/docs/reference.md) — register map, subframe encoding, value scaling
- [moteus Python library](https://pypi.org/project/moteus/) — the reference implementation these frames are pinned against
- [Dynamixel Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/) — packet structure, CRC, byte stuffing
- [Dynamixel control tables](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) — addresses and unit scales
- [RobStride Product_Information](https://github.com/RobStride/Product_Information) — the per-model user manuals these constants are quoted from
- [sirwart/robstride](https://github.com/sirwart/robstride) — an independent Python SDK, used to cross-check addressing and byte order

## License

MIT
