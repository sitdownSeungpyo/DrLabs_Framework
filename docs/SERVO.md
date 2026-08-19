# The servo role

This framework is the low level. Something above it decides what the machine
does; the framework's job is to be energised, to be waiting, and to execute
what it is told — promptly and within an envelope nobody above it can exceed.

```
drlabs serve -driver moteus -port COM5 -id 1 -allow-unknown-board \
             -max-torque 0.2 -position-min -0.2 -position-max 0.2
```

That starts a process which connects, energises the motor, holds it where it
already was, and listens on `127.0.0.1:7777` for commands.

```
drlabs servo on 127.0.0.1:7777
  driver moteus · id 1 · COM5 · 200 Hz
  energising and waiting for commands
OK    Armed automatically
      holding position at 0.0000 rev · torque limit 0.20 Nm
```

## What "running" means here

The operator-facing paths — `drlabs run`, the web UI — start disarmed and make
someone read a checklist and press Arm. That is right when a human is standing
at a bench. It is wrong for a subordinate layer: an axis that needs a person
before it will hold position cannot serve anything.

So `serve` sets `session.Options.AutoArm`, and the control loop energises as
soon as preflight is clear. Three details make that safe rather than merely
convenient:

- **Energising is not a move.** On arming, the setpoint adopts the position the
  axis is already at. Torque comes on and the shaft stays where it is.
- **It keeps trying.** The reasons an axis cannot arm — a supply still
  settling, a fault left over from power-up — are usually transient, so the
  loop retries every cycle and says why once a second. A servo that gave up on
  the first attempt would need a human to notice, which is the thing this role
  exists to avoid.
- **It will not drive through a latched emergency stop.** Clearing one stays an
  explicit act by whatever raised it. A stop that undid itself would be worse
  than no stop at all.

## The command interface

Newline-delimited JSON over TCP. One object per line in, one reply per line
back, plus an unsolicited state line at `-state-rate` Hz (50 by default) so the
caller never has to poll.

It is a deliberately plain protocol: callable from Python, ROS 2, C++ or a
shell, with no dependency on either side.

### Commands

| Line | Effect |
|---|---|
| `{"cmd":"position","value":0.25}` | position mode, target in revolutions |
| `{"cmd":"velocity","value":-1.0}` | velocity mode, rev/s |
| `{"cmd":"torque","value":0.3}` | torque mode, Nm feedforward |
| `{"cmd":"mode","mode":"hold"}` | select a mode without a setpoint |
| `{"cmd":"hold"}` | zero-velocity hold |
| `{"cmd":"stop"}` | release the bridge |
| `{"cmd":"trajectory","target":0.5,"max_velocity":1.0,"accel":2.0}` | host-profiled move |
| `{"cmd":"trajectory_stop"}` | abandon the profile |
| `{"cmd":"shuttle","a":-0.2,"b":0.2,"dwell":0.5,"cycles":10}` | A↔B run |
| `{"cmd":"shuttle_stop"}` | end it |
| `{"cmd":"params","max_torque":0.4,"kp_scale":0.8}` | per-cycle parameters; omitted fields keep their value |
| `{"cmd":"arm"}` / `{"cmd":"disarm"}` | energise / release |
| `{"cmd":"estop"}` / `{"cmd":"reset"}` | latch / clear the stop |
| `{"cmd":"zero"}` | re-reference the output to zero (disarmed only) |
| `{"cmd":"state"}` | one state line, on demand |
| `{"cmd":"ping"}` | liveness |

Every reply is `{"type":"reply","ok":true}` or
`{"type":"reply","ok":false,"error":"…"}`. A refusal is an answer, not a
failure: `arm the axis first` and `emergency stop is latched` arrive this way.

### State lines

```json
{"type":"state","connection":"connected","armed":true,"mode":"position",
 "estop":false,"position":0.2501,"velocity":0.0,"torque":0.02,
 "current":-1.6,"voltage":24.0,"temperature":30.5,"fault":0,
 "setpoint_position":0.25,"trajectory_running":false,"rate_hz":199.8}
```

Optional measurements are `null` when the driver did not read them, never `0`.
JSON cannot carry NaN, and a caller that reads a missing temperature as 0 °C
will draw the wrong conclusion from it.

While disarmed, a `blocked` array carries the preflight reasons — that is where
a client looks to find out why the axis is not live.

### A test client

`test/client/drive.py` is a working client and a bench tool in one file:

```
python test/client/drive.py state        one state line
python test/client/drive.py watch        stream it
python test/client/drive.py pos 0.05     move, wait for it to settle, report
python test/client/drive.py traj 0.1 --vel 0.2
python test/client/drive.py sweep 0.05 3
python test/client/drive.py stop | estop | reset
```

Every motion command waits for the axis to stop and prints the error against
what was asked, so a first bench test is one line and a readable answer. It
refuses to command motion into an axis that is not armed, and says why.

### A client, in full

```python
import json, socket

sock = socket.create_connection(('127.0.0.1', 7777))
stream = sock.makefile('rwb')

def send(**command):
    stream.write((json.dumps(command) + '\n').encode())
    stream.flush()

send(cmd='position', value=0.25)

for line in stream:
    message = json.loads(line)
    if message.get('type') == 'state' and message['position'] is not None:
        print('%.4f rev' % message['position'])
```

## What the framework still refuses to give up

Being subordinate does not make it credulous. Everything below applies whatever
the layer above sends:

- **The envelope clamps every command.** `-max-torque`, `-velocity-limit`,
  `-position-min/max` are declared on the command line by whoever launches the
  process, and no command can exceed them. They are the reason the flags exist
  rather than being settable over the socket.
- **The trips still stop the machine** — over-current, over-temperature,
  runaway, stall, position-envelope escape, communication loss.
- **The controller's own watchdog still runs.** The session sends it every
  cycle, so if this process dies the motor stops on its own.

### The command timeout

A position command is safe to leave standing: the axis arrives and holds. A
velocity or torque command is not — it runs until something countermands it, so
a caller that crashes mid-move would leave the machine turning.

`-command-timeout` (1.0 s by default) reverts open-ended motion to a position
hold after that much silence. It applies only to velocity and torque; a
position target is left alone.

### The deadman, and why it is off here

The operator-facing paths stop the axis when the supervising client goes quiet.
That is wrong in this role: waiting with no client attached is the normal state
of a servo, and a guard that stopped the axis for being idle would make it
useless. `-deadman` is therefore 0 by default. Set it to a number of seconds if
your upper layer is always connected and you want its disappearance to stop the
axis; the command timeout above covers the dangerous half of that case anyway.

## Verified

Against the simulator at 200 Hz:

| Behaviour | Result |
|---|---|
| Energise on start | armed automatically, holding 0.0000 rev |
| `position 0.3`, then `-0.2` | arrived 0.3000, then −0.2000 rev |
| `velocity 0.4` then silence | reverted to position hold 1.0 s later, at 0.4001 rev |
| `estop`, wait, `reset` | stayed disarmed under the latch; re-armed unaided after the reset |
| `trajectory 0.35 @ 0.4 rev/s` | arrived 0.3501 rev, peak 0.396 rev/s |
| velocity command run into the envelope | tripped `position_envelope` and stopped, as declared |

That last row is worth reading twice. A sustained velocity command will
eventually leave a bounded envelope, and the guard catches it — but the axis
then coasts to a stop *outside* the envelope, and preflight will refuse to
re-arm until it is back inside. On a bench somebody turns the shaft; an
unattended machine has nobody to do that. If you command open-ended velocity
inside a position envelope, plan for how the axis gets back.

Not yet exercised against real hardware.
