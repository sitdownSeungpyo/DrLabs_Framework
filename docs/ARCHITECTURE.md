# Architecture

Why the framework is shaped the way it is. The layout is in the README; this
file covers the decisions behind it.

## The one seam

`motor.Link` is the only interface the runtime is written against:

```go
type Link interface {
    Kind() string
    Describe() map[string]string
    Open(ctx context.Context) error
    Close() error
    Identify(ctx context.Context) (*Identity, error)
    Cycle(ctx context.Context, cmd Command, profile QueryProfile) (*Telemetry, error)
    Capabilities() Capabilities
}
```

Everything above it — the control loop, the safety monitor, the CLI, the arming
checklist — is vendor-neutral. Everything below it is one vendor's problem.

`Cycle` is a single method rather than a set of `SetPosition`/`ReadState` calls
because that is what the hardware does: a moteus command and the telemetry query
it piggybacks on travel in the same CAN frame. Splitting them into two calls
would either double the bus traffic or force the driver to invent a cache.

### Reporting what a driver cannot do

`Capabilities` and `Envelope` exist so that limitations are data rather than
runtime surprises. A Dynamixel has no stay-within mode, and a RobStride has
neither that nor a brake mode, so neither claims one and a UI can grey the
option out. Without a motor-specific torque constant a Dynamixel cannot report
newton-metres, so it reports torque as unmeasured rather than fabricating a
number a safety limit would then be set from.

`Envelope` is optional. A driver that publishes its hardware ratings lets the
safety layer refuse an operator setting looser than the board can survive; one
that does not simply leaves the declared limits in force.

### Identity that cannot be read

`BoardStatus` has four states, not three, because "the device answered with a
revision I do not recognise" and "the device has no identity register at all"
call for different responses.

A moteus reports a family and a revision, so an unrecognised one is worth
stopping for. A RobStride reports neither — its model comes from configuration
and nothing on the bus can confirm it. Demanding an override on every RobStride
connect would train operators to pass the override always, which would hollow
out the check for the devices that *do* report a revision. So `BoardDeclared`
connects without ceremony and surfaces a standing advisory instead: the
declaration is unverified, and it will stay unverified.

That matters more for RobStride than the wording suggests. The family's torque
scale spans a factor of twenty-two, so a mis-declared model mis-scales every
command and every reading with nothing to reveal it.

## Units and the meaning of NaN

Every shared interface is in physical units — revolutions, rev/s, Nm, A, V, °C,
seconds. Conversion lives in the driver, at the point where the wire encoding
is already being handled.

`NaN` means "unknown or not reported" in telemetry, and "do not send this
parameter" in commands. Both are load-bearing:

- A fast-profile cycle carries no motor temperature. Returning zero would let a
  thermal trip read an absent sensor as a cold motor. Returning NaN makes the
  monitor skip the check.
- Command registers on a moteus are sticky. Omitting one leaves the last value
  in force, which is what makes a 21-byte per-cycle frame possible.

The exception is documented at length in `docs/MOTEUS.md`: on the moteus wire,
NaN in a command register is itself a meaningful value. That is why
`motor.Command` has an explicit `DeferTorqueToController` flag instead of
overloading NaN, and why `motor.Clamp` returns NaN unchanged — `min`/`max`
against NaN silently produces the bound, which would turn "use the controller's
own torque limit" into "command zero torque".

## Who owns the bus

Exactly one goroutine: the session's control loop.

Callers never transact. They mutate request state under a mutex, and the loop
samples it at the top of each period. `Status()` returns a deep copy. Copying a
few hundred bytes at whatever rate a UI refreshes is not worth optimising away,
and a shared map read while the loop mutates it is a torn view at best.

This is what keeps a slow UI, a garbage collection pause or a blocking log
write out of motor control. The `signal()` channel lets an operator action wake
the loop immediately rather than waiting up to a full period, without letting
the caller do any I/O itself.

`ConnectLink` exists alongside `Connect` for the case of several controllers on
one shared bus: build the transport once, wrap it in one `Link` per controller,
give each `Link` its own `Session`.

## Keeping the period

Windows' default timer granularity is 15.6 ms. A 100 Hz loop that sleeps for
10 ms actually sleeps for about 15, runs at roughly 64 Hz, and makes every
watchdog timeout derived from the nominal rate wrong.

`runtime/timing` raises the multimedia timer to 1 ms (reference counted, so
nested sessions each get a matching release) and spins the final ~1.5 ms of each
wait rather than sleeping through it. The spin is bounded to a quarter of the
period, so it costs a fraction of a core, not a whole one.

The loop also resynchronises rather than catching up. If it falls more than a
period behind — a long pause, a laptop resuming from sleep — the next deadline
is set from now, not from the missed one. A burst of back-to-back cycles trying
to make up lost time is worse than a gap.

And it measures. `Status().Stats` reports the rate actually achieved, the jitter
as a standard deviation, and the transaction time as mean/p95/max. A p95 far
above the mean is the signature of an adapter that mostly keeps up but stalls —
a case a mean alone hides completely. The loop warns when it drops below 85% of
target, because a loop that cannot keep up is the most common cause of
surprising motor behaviour.

## Safety as a second layer

Controller firmware protects the board: over and under voltage, over
temperature, position bounds, command watchdog. Those are real and they stay.

`runtime/safety` protects the machine. The motor, the gearbox and whatever they
drive are not the controller manufacturer's products, so their envelope has to
be declared by the operator and enforced on the host, before anything reaches
the bus. Nothing in this layer is ever written to a controller's persistent
configuration.

Three design rules, each of which exists because the obvious alternative fails
in practice:

**Trips need duration.** A momentary current spike is what acceleration looks
like; a brief overshoot is what a fast approach looks like. Every motion and
electrical trip requires its condition to hold continuously for a configured
time. A single-sample trip produces a machine that stops constantly for no
reason, and operators who learn to disable it.

**Some conditions only trip while armed.** Being outside the position envelope
is a reason to refuse arming, but not a reason to latch while disarmed — with
the drive off, the operator's only way back inside is to move the axis by hand,
and latching on that would make the condition unrecoverable.

**Blockers and advisories are different.** Preflight returns two lists. A warm
board is an advisory; an invalid encoder is a blocker. A system that treats
every observation as a blocker teaches people to click through it.

The emergency stop is orthogonal to the state machine: it latches from any
state, forces a stop on every subsequent cycle, and must be cleared explicitly.
Clearing it leaves the axis disarmed, so a reset can never by itself put a
machine back in motion.

## The state machine

```
disconnected ──connect──▶ connected/disarmed ──arm──▶ connected/armed ──▶ mode
                              ▲                            │
                              └──────────disarm────────────┘
```

Two details worth calling out.

**A mode change is not a move.** Entering position mode adopts the current
measured position as the target. Keeping a stale setpoint would mean selecting a
mode flings the axis across its travel — the single most dangerous thing a UI
can do. `SetMode` also returns early when the mode is unchanged, so a repeated
call cannot drag a live target to wherever the axis happens to be.

**Connecting commands a stop.** A controller left driving by a previous session
keeps driving until told otherwise, and its command registers still hold that
session's limits. The connect sequence drains the transport and sends a short
burst of stops before anything else.

## Why the sequencers live in the loop

The A↔B shuttle and the host-generated trajectory — move, dwell, reverse,
count; or sample a profile — are both evaluated inside `buildCommandLocked`,
not by an application on top of the session.

They are mutually exclusive because both own the position setpoint, and each
one stops the other on start.

That is deliberate. Its timing does not depend on a UI staying responsive, and
it stops the instant the axis is disarmed, faulted or emergency-stopped, because
it is evaluated by the same code that decides whether to command anything at
all. A sequencer that lives outside the loop has to be told to stop, and every
path that fails to tell it is a machine that keeps moving after the operator
thought they had stopped it.

## Testing without hardware

The simulator is not a stub. It is a trapezoidal trajectory generator feeding a
torque-limited PD controller, driving an inertia with viscous friction, with
thermal rise and supply sag, integrated at 2 kHz independently of the host rate.

It exists so the interesting failures actually happen: overshoot, stall,
thermal runaway, escaping the position envelope. `InjectFault` raises controller
faults on demand. `runtime/session/session_test.go` drives the whole stack
through it, and the suite needs no hardware at all.

The frame encoder is checked separately and more strictly: every command frame
is compared byte-for-byte against output from the official `moteus` Python
library. That is the only check that the encoder is compatible with real
firmware rather than merely self-consistent.

## Known gaps

- `drivers/odrive` is declared but not implemented.
- Joints on a shared bus each run their own control loop rather than sharing
  one. Transactions are atomic, so this is correct, but joints on the same wire
  are only phase-aligned to within a control period. A machine needing tighter
  coordination wants one loop per bus cycling each joint in turn — see
  `docs/ROBOT.md` for what that would cost.
- `drivers/robstride` was written against documentation and a simulator; no
  hardware was available. Its protocol constants are covered by tests, but its
  timing, default gains and behaviour under load are unverified. See
  `docs/ROBSTRIDE.md`.
- SocketCAN is not implemented; CAN transport is fdcanusb only.
- The race detector needs cgo, so `go test -race` requires a C toolchain that
  a plain Go install does not provide.
- `ros2/drlabs_bridge` is implemented and host-tested, but its ROS 2 node has
  never been run: no machine with `rclpy` was available. The link underneath it
  is verified against a real bringup server. See `docs/ROS2.md`.
