# Bringing up an axis

Bring-up is not a command. It is the procedure that turns an unknown motor into
a set of numbers you are willing to hand to `drlabs serve` — and the order
matters, because each stage is only safe given what the stage before it proved.

The output of bring-up is this table, filled in:

| Number | Where it comes from | Used as |
|---|---|---|
| serial port | stage 1 | `-port` |
| bus address | stage 2 | `-id` |
| direction of travel | stage 4 | how the upper layer signs its targets |
| torque that just moves the axis | stage 4 | the floor under `-max-torque` |
| torque ceiling | stage 5 | `-max-torque` |
| position envelope | stage 5 | `-position-min`, `-position-max` |
| zero reference | stage 5 | where the machine thinks it is |
| achievable rate | stage 6 | `-rate` |

Everything before stage 4 commands nothing but a stop, so it is safe to run
against a live machine.

---

## Stage 0 — before any software

Mechanical, and no tool here will do it for you:

- The axis is free to move through the range you intend to test, or is
  deliberately clamped and you know it.
- Nothing valuable is in the path.
- The supply can deliver the current the motor will draw at the torque limit
  you are about to set, and its voltage is inside the controller's rating.
- The CAN bus is terminated, if you are on CAN.

## Stage 1 — find the adapter

```
drlabs ports
```

CAN adapters are listed first where the platform reveals enough to tell. On
Windows it usually does not — the OS exposes only `COM5` — so if the list is
ambiguous, unplug the adapter, run it again, and take the port that
disappeared. macOS is the same: the list is `/dev/cu.usbmodem…` for a CDC
device such as the fdcanusb or OpenRB-150 and `/dev/cu.usbserial-…` for an
FTDI bridge such as the U2D2, with no product name. Use the `cu.` node, never
the `tty.` twin, which blocks until the adapter asserts carrier detect — which
a USB adapter never does.

## Stage 2 — identify, without commanding anything

```
drlabs probe -driver moteus -port COM5 -id 1
```

`probe` connects, reads identity and telemetry, and disconnects. The only thing
it puts on the bus is a stop, so it is safe against a machine that is already
running.

Read four things from its output:

```
Board             moteus r4.11          the framework recognised the hardware
  status          supported             …and has an envelope for it
  travel bounds   [-5.000, 5.000] rev   the controller's own limits
Voltage           24.00 V               the supply, as the controller sees it
Position          -0.0006 rev           where the axis thinks it is
Loop              99.6 Hz requested 100 replies 100%
```

**If the board is not recognised**, the framework says so and refuses to
continue without `-allow-unknown-board`. That is not a formality: an
unrecognised board has no verified electrical envelope, so the limits become
assumptions. Find out what it is before overriding.

**If `replies` is below 100%**, stop here. Wiring, termination and bit rate are
cheaper to fix now than to diagnose through a moving axis.

**If the travel bounds are narrow or absent**, note it. On a moteus, commanding
outside `servopos.position_min/max` makes the firmware clamp the target and
report "limiting: position bounds" — which from outside looks exactly like a
motor ignoring you.

## Stage 3 — decide the first envelope

Not measured yet; guessed deliberately low, so that stage 4 is survivable.

- **Torque:** start at a value you expect to be *too low to move the axis*. On a
  small motor, 0.05–0.1 Nm. You want the first surprise to be "it did not
  move", not "it moved further than I thought".
- **Position envelope:** the smallest range that proves anything. ±0.05 rev is
  18 degrees and plenty.

## Stage 4 — first motion, with a human watching

Use the web UI for this. It is the tool with a person in the loop, and it shows
position, velocity and current as curves while you work.

```
webui -driver moteus -port COM5 -id 1
```

Then, in the page: set the torque ceiling and position envelope from stage 3 in
the **Safety** panel, read the arming checklist, arm, and command a small
position step — 0.02 rev.

Three outcomes, all of them informative:

- **It moves the way you expected.** Note the direction. Positive revolutions
  going the wrong way is a fact to record now, not to discover in the upper
  layer later.
- **It does not move.** The torque ceiling is below what the axis needs. Raise
  it in steps and repeat. The value at which it *starts* to move is the most
  useful number in this whole procedure: it is the floor under every torque
  limit you will set afterwards, and it tells you how much of your ceiling is
  margin.
- **It trips.** Read the event log. The trip names the cause, and the detail
  line says what to do about it.

Then widen the range gradually and repeat, watching the current curve. Use the
**Trajectory** panel rather than bare position steps once you are past the
first few degrees: a profiled move bounds the approach speed, where a position
step does not.

## Stage 5 — settle the envelope

Now the numbers are measurements rather than guesses.

- **Torque ceiling:** enough to move the load with margin, not enough to break
  the mechanism. If the axis needed 0.15 Nm to move, 0.3 gives headroom without
  giving it authority to destroy anything.
- **Position envelope:** the travel you actually want, plus the overshoot you
  observed. The trip margin (`PositionTripMargin`, 0.05 rev by default) absorbs
  normal overshoot; the envelope should not have to.
- **Zero reference:** with the axis disarmed, put it where zero should be and
  press *Set position = 0*. On a moteus this is runtime only — it does not
  survive a power cycle unless written to the controller's configuration.
- **Current trip:** above the current you saw at working torque, below anything
  that would cook the motor.

The safety sensitivity presets are a starting point for the motion trips:
`conservative` while you are still learning the axis, `standard` afterwards.

## Stage 6 — check the rate you actually get

The web UI reports it, and so does `probe`:

```
Loop  199.8 Hz requested 200 · jitter 0.48 ms · replies 100%
```

Achieved, not requested. If it falls below 85% of target the loop says so.
A rate you cannot sustain makes every watchdog timeout derived from it wrong,
so lower the request until the two agree.

## Stage 7 — hand it to the servo

Bring-up is done when this line is written from measurements:

```
drlabs serve -driver moteus -port COM5 -id 1 \
  -max-torque 0.3 -position-min -0.2 -position-max 0.2 \
  -velocity-limit 1.0 -rate 200
```

The axis energises, holds where it is, and waits. Those flags are the envelope,
declared by whoever launches the process, and nothing the upper layer sends can
exceed them.

## Stage 8 — connect the upper layer

```
python test/client/drive.py state
python test/client/drive.py pos 0.05
```

`drive.py` is a working client in one file; the protocol is in
[SERVO.md](SERVO.md).

---

## When it goes wrong

| Symptom | Usual cause |
|---|---|
| No reply from the bus address | wrong id, wrong bit rate, termination, or the controller is unpowered |
| Board not recognised | genuinely unknown hardware, or a firmware this build cannot read the record from |
| Accepts commands, does not move | torque ceiling too low — the controller reports "limiting: max torque" |
| Moves, then trips immediately | position envelope narrower than the move, or a runaway/stall trip set too tight |
| Cannot re-arm after a trip | the axis coasted outside the envelope; move it back by hand while disarmed, or widen the envelope |
| Loop runs slower than requested | the transport cannot complete a transaction that fast; lower `-rate` |

## What is verified, and what is not

On real hardware — a moteus r4.11 over fdcanusb:

- Stages 1, 2, 6: adapter discovery, identification, telemetry, loop rate
  (200.2 Hz achieved against 200 requested, 100% replies)
- Stage 4: position steps and profiled trajectory moves through the web UI

Not yet exercised on hardware: velocity and torque modes, the A↔B shuttle, the
servo role end to end, and the whole RobStride driver. Those are covered against
the simulator only, and the simulator cannot tell you what your mechanism does.
