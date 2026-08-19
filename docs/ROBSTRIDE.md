# RobStride protocol notes

What this framework implements for the RobStride (灵足时代) actuator family, and
the details that are easy to get wrong.

Sources: the official user manuals for each model, `<model>User Manual260713.pdf`
in [RobStride/Product_Information](https://github.com/RobStride/Product_Information),
in both their English and Chinese editions. The addressing and byte order were
cross-checked against an independent implementation,
[sirwart/robstride](https://github.com/sirwart/robstride).

**No hardware was available while this driver was written.** The protocol
constants come from the manuals and are covered by tests; the timing, the
default gains and the behaviour under load have not been observed on a real
actuator. See [Before first power-up](#before-first-power-up).

## The wire

CAN 2.0B, 1 Mbit/s, 29-bit extended identifiers, 8-byte payloads. Not CAN-FD —
an adapter told to send FD frames will transmit something the motor cannot
receive, and the symptom is a motor that never answers.

### Arbitration identifier

```
bits 28..24   communication type
bits 23..8    data area 2
bits  7..0    destination address
```

Data area 2 usually carries the host's address in its low byte. Two commands
use it differently: the control command puts the whole 16-bit feedforward
torque there, and the set-address command puts the new address in its high byte.

Replies invert the addressing — the motor's address lands in bits 15..8 and the
host's in bits 7..0. Getting that backwards means every reply on a multi-motor
bus is attributed to the wrong actuator, which is why `Controller.send` filters
on both halves.

### Communication types

| Type | Meaning |
|------|---------|
| 0 | Get device id (64-bit MCU UID) |
| 1 | Operation control ("MIT") command |
| 2 | Motor feedback — the reply to almost everything |
| 3 | Enable |
| 4 | Stop, optionally clearing faults |
| 6 | Set mechanical zero |
| 7 | Set CAN address |
| 17 | Read one parameter |
| 18 | Write one parameter (volatile) |
| 21 | Fault feedback — the full fault and warning words |
| 22 | Save parameters to non-volatile storage |
| 23 | Set bit rate (takes effect on power-up) |
| 24 | Enable or disable unsolicited reporting |

### Byte order, which is not consistent

Control and feedback frames are **big-endian** ("the high byte is in front").
Parameter frames are **little-endian**. Mixing them up produces a driver that
talks confident nonsense, so the two encoders are kept in separate files and
both orders are pinned by tests.

### The 16-bit fields

Position, velocity, torque, Kp and Kd all travel as 16-bit codes over a linear
range. Position is ±4π radians on every model; the rest are per-model.

A symmetric range spread over 65536 codes has no code at its exact centre, so
zero is not representable — it encodes to the nearest code and decodes back half
a count away. On an RS01 that is 0.26 mNm. It matters only in that a test
comparing telemetry against an exact zero will never pass.

Out-of-range values are clamped, never wrapped. Wrapping would turn "more torque
than the codec can express" into full reverse.

## The model table, and why it is required

This is the single most important thing about driving a RobStride.

| Model | Torque | Velocity | Kp | Kd | Current | Supply (rated) |
|-------|--------|----------|-----|-----|---------|----------------|
| RS00 | ±14 Nm | ±33 rad/s | 0–500 | 0–5 | ±16 A | 24–60 V (48) |
| RS01 | ±17 Nm | ±44 rad/s | 0–500 | 0–5 | ±23 A | 24–50 V (36) |
| RS02 | ±17 Nm | ±44 rad/s | 0–500 | 0–5 | ±16 A | 24–60 V (48) |
| RS03 | ±60 Nm | ±20 rad/s | 0–5000 | 0–100 | ±43 A | 24–60 V (48) |
| RS04 | ±120 Nm | ±15 rad/s | 0–5000 | 0–100 | ±90 A | 24–60 V (48) |
| RS05 | ±5.5 Nm | ±50 rad/s | 0–500 | 0–5 | ±11 A | 15–60 V (48) |
| RS06 | ±36 Nm | ±50 rad/s | 0–5000 | 0–100 | ±57 A | 15–60 V (48) |

The torque column spans a factor of twenty-two. The same wire code means 17 Nm
on an RS01 and 120 Nm on an RS04. Configure the wrong model and every torque
command and every torque reading is scaled by that factor, in whichever
direction is worse — and nothing on the bus will say so, because **a RobStride
does not report its own model**.

That is why `model` is a required option with no default, and why `Identify`
reports `BoardDeclared` rather than claiming a verified identity. The session
raises it as an advisory on every arm: *check the label on the actuator.*

The gain ranges split the family in two. RS00, RS01, RS02 and RS05 use Kp 0–500
and Kd 0–5; RS03, RS04 and RS06 use 0–5000 and 0–100. A gain that is right for
one family is ten to twenty times wrong on the other.

### Two documented inconsistencies

**RS02.** Its control-frame table gives the codec a ±17 Nm and ±44 rad/s full
scale — stated identically in the English and Chinese manuals, and consistent
with its internal `0x2007 limit_torque` default of 17 — while its CAN parameter
table caps `limit_torque` at 14 Nm and `spd_ref` at 33 rad/s. The driver uses
the codec values for the wire format and the parameter-table values for
validating parameter writes, which is the reading that cannot mis-scale a
command in either direction.

**RS04.** Its control frame maps velocity over ±15 rad/s while `spd_ref` and
`limit_spd` allow 20. The codec range is what the frames use; a velocity beyond
it is clamped.

## Control modes

Each framework mode is mapped onto the controller loop that can actually
deliver its contract, rather than onto whichever one is most convenient.

| Framework mode | Run mode | Why |
|---|---|---|
| `CtrlPosition` | CSP (5) by default | Its profile generator honours the commanded velocity limit — which is what `CtrlPosition` promises. `position_mode=operation` switches to MIT for impedance control. |
| `CtrlVelocity` | Velocity (2) | Closes a PI loop and holds a setpoint under load, where MIT's damping term alone sags. |
| `CtrlTorque` | Operation (0) | Both gains at zero, so the feedforward torque is the whole command. |
| `CtrlHold` | Operation (0) | Zero stiffness, damping only — the manual's documented damping configuration. |
| `CtrlStop`, `CtrlQuery` | — | The stop frame. |
| `CtrlSetOutput` | — | Set mechanical zero. |
| `CtrlBrake`, `CtrlStayWithin` | — | **Unsupported.** No phase-short brake mode, no bounded-region mode. |

Steady-state cost is one transaction per cycle in every driving mode: a control
frame in operation mode, a `loc_ref` write in CSP, a `spd_ref` write in velocity
mode. Limits are written only when they change.

### Why a query sends a stop

A stop frame answers with a full feedback frame and leaves the output released.
That is exactly what a disarmed session wants — watch the axis without
commanding it — so `CtrlQuery` sends one rather than a zero-gain control frame
that would still be a command.

### Switching run modes stops the motor first

Handing the output to a different loop while it is driving gives control to a
controller whose state was never initialised for the current position.

## Telemetry

A feedback frame carries mode, six summary fault bits, angle, velocity, torque
and temperature. It does **not** carry bus voltage or current — those live in
the parameter table and cost a round trip each, so they are read on the wider
query profiles rather than every cycle.

The practical consequence: with a RobStride, `QCurrent` and `Voltage` refresh at
roughly a tenth of the control rate. The safety layer's over-current trip has a
0.4 s window by default, so that is still fast enough for it to fire — but it is
slower than on a controller that reports current in its feedback frame, and the
stall guard inherits the same latency.

### Position wraps at ±4π

The reported angle is periodic over the ±4π window. Without turn counting, a
multi-turn axis appears to jump 8π backwards every four revolutions, and a
position envelope checked against that trips on a perfectly healthy machine.
The driver counts turns on the host and reports a continuous position; commanded
targets are folded back into the window the protocol can express.

The count is lost across a reconnect, and the controller's own zero is only
persistent if committed with a save frame.

## Faults

A feedback frame's identifier carries six summary bits in 21..16:
undervoltage, three-phase overcurrent, overtemperature, magnetic encoder,
stall overload, uncalibrated. Any of them is treated as a hard fault.

A type-21 frame carries the full fault and warning words, which distinguish
which phase over-sampled and separate the over-temperature warning (≈93 °C) from
the fault (≈103 °C). `Controller.FaultReport` reads it; the control loop does
not, because the summary bits are enough to stop.

## Before first power-up

This driver has been written against documentation and tested against a
simulator. On a real actuator:

1. **Check the model** against the label before anything else. Everything else
   depends on it.
2. **Start with a low torque ceiling.** The framework's default is 1 Nm, which
   will move an unloaded RS05 and barely twitch an RS04. Raise it deliberately.
3. **The default gains are conservative starting points, not tuned values.**
   `DefaultKp`/`DefaultKd` were chosen as a fraction of each model's range with
   no load in mind. Set `kp` and `kd` explicitly once the axis is characterised.
4. **Verify the direction of travel** at low torque before trusting a position
   envelope.
5. **Expect the current and voltage readings to lag** the control rate, as above.

Things a real actuator may still disagree with the manuals about: whether the
firmware rounds or truncates when encoding the 16-bit fields (this driver
rounds; the difference is one count), the exact RS02 torque scale, and how
quickly a parameter write is reflected in a subsequent read.
