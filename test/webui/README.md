# Web control panel

A browser front end for one motor, built on the framework's own runtime. It is
the same session, safety layer and control loop the CLI uses; this process only
translates HTTP into session calls and pushes snapshots back.

```bash
go run ./test/webui                                     # simulator
go run ./test/webui -driver moteus -port COM5 -id 1     # moteus over fdcanusb
go run ./test/webui -driver robstride -port COM5 -id 1 -model RS03
```

Then open <http://localhost:8080>. The flags only pre-fill the connect form —
the connection itself is made from the page, so it survives reconnects without
restarting the server.

## What it does

| Panel | Contents |
|---|---|
| Connection | driver, port, bus address, RobStride model, control rate, board override; identity and firmware once connected |
| Arming | the live preflight checklist — blockers in red, advisories in amber — plus arm, disarm, E-stop reset and set-position-zero |
| Control | mode buttons, position and velocity setpoints as slider plus number box, and the per-cycle parameters |
| Trajectory | host-generated trapezoidal profile: target, velocity limit, acceleration, with a progress bar and the live setpoint and feedforward |
| Shuttle | A ↔ B endurance run with dwell and a cycle count |
| Telemetry | eight readouts and three rolling charts (position, velocity, current), plus loop rate, jitter, bus timing and reply rate |
| Safety | the operator envelope, editable, with the three sensitivity presets |
| Event log | everything the session logged, with the detail line that says what to do about it |

**The space bar is the emergency stop**, from anywhere on the page except a
text field. It is the one control that should not require finding a button.

## How it is wired

Telemetry arrives over server-sent events rather than a websocket: it needs no
dependency and no handshake, and the traffic is one-way — the page reads state
and posts commands separately. Snapshots go out at 20 Hz regardless of the
control rate, because a browser has no use for 200 updates a second and the
loop should not pay to serialise them.

That stream doubles as the deadman's heartbeat. It is fed from the same place
that is reading, so **a page that stops reading stops the axis** — close the
tab, lose the network, or suspend the laptop, and the session latches a stop
within `DeadmanTimeout` (2 s by default). That is the guard working, not a bug;
it will show up as `estop: deadman` in the log.

JSON cannot represent NaN, and the framework uses NaN throughout to mean "not
measured". Optional numbers therefore cross as `null` and render as an em dash,
rather than being flattened to zero — an unread sensor must not look like a
sensor reading zero.

Nothing in the page decides anything about the machine. Every limit, refusal
and trip lives in `runtime/session` and `runtime/safety`; the browser posts
intents and renders what comes back.

## Verified

Against the simulator at 200 Hz, 0.5 ms jitter, 100% reply rate:

| Move | Velocity limit | Peak reached | Duration | Final error |
|---|---|---|---|---|
| −0.250 → +0.450 rev | 0.50 rev/s | 0.495 rev/s | 1.65 s | 0.0001 rev |
| +0.450 → −0.300 rev | 0.20 rev/s | 0.195 rev/s | 3.85 s | 0.0000 rev |

The deadman was confirmed by letting the stream drop: the session latched
`deadman` and disarmed within two seconds.

Not yet exercised against real hardware.

## Not here

**Multi-axis / daisy chain.** The driver layer can already share one bus
between several controllers — `moteus.NewLinkWithBus` and
`robstride.NewLinkWithBus` exist for exactly that — but there is no multi-axis
session, and `can.Bus` implementations are documented as not safe for
concurrent use, so two loops on one adapter would race. Driving several motors
needs a coordinator that owns the bus and cycles each controller in turn. Until
that exists, this panel controls exactly one axis.
