# Dynamixel notes

Working notes on the parts of the ROBOTIS Protocol 2.0 servo family this
framework implements, and on the details that cost time to get right. The
authority is the ROBOTIS e-Manual control tables; this file records what the
implementation depends on and why.

## A profile is a control table plus its unit scales

`Profile` in `profile.go` carries both halves of what a model needs: the
register addresses, and what the counts in those registers mean. The addresses
are largely shared across Protocol 2.0 — the scales are not.

| | XL330 | X-series (XM430, XL430, XH430, …) |
|---|---|---|
| current | 1 mA per count | 2.69 mA per count |
| supply | 3.7 – 6.0 V, rated 5 V | 10 – 14.8 V, rated 12 V |
| no-load speed | 6 rev/s | 4 rev/s |

Driving one through the other's profile is not a visible failure. The servo
still moves, and it holds position, and nothing in the logs looks wrong. What
changes is that every current reading is out by a factor of nearly three, which
makes a current trip meaningless, and the supply range belongs to a different
class of motor.

## The servo says which one it is, so the profile is never guessed

A Dynamixel answers every ping with its model number. `KnownModels` maps that
number to the profile that describes it, and `resolveProfile` settles the
question at open time:

- **Nothing declared, model recognised** — the model's profile is used. A
  config file that forgets `"profile"` is still driven correctly.
- **Declared, and the model says otherwise** — refused, naming both. One of the
  two is wrong, and connecting anyway means reporting numbers scaled for a
  motor that is not on the bus.
- **Model not in the table** — the configured profile stands, but `Identify`
  reports the board as unrecognised unless a profile was named. The choice is
  then acknowledged rather than assumed, which is the same bargain the moteus
  driver strikes with an unfamiliar board revision.

Before this, the model number was read and printed and then ignored: an XL330
fell back to the X-series table and the board still reported itself as
`supported`.

## The rated supply has to be published, not just known

The host safety layer opens with a bus voltage band of 10–42 V, which is a
sensible band for a 12 V or 24 V bus and an impossible one for a 5 V servo.
`safety.FitVoltageTo` exists to widen a band that excludes the connected
motor's rated supply — a band that excludes it catches no fault, it just trips
the instant the machine is powered.

That check keys on `Envelope.VoltageRated`, and it does nothing when the field
is zero. The Dynamixel profiles carried a min and a max but no rated value, so
the check never fired and an XL330 reporting a perfectly healthy 4.8 V could
not be armed at all:

```
Cannot arm:
  ✗ Bus voltage 4.8 V is below the configured minimum 11.0 V.
```

Every profile now declares `VoltageRated`, and a test fails if one does not.

## The serial read timeout was the loop rate

A serial port opens with a plain total read timeout — 10 ms — and a plain total
timeout means a read blocks until it has filled the *whole* buffer it asked for
or the timeout expires. `readPacketWithTimeout` asks for a buffer far larger
than any status packet, so it never fills, and every read cost the full 10 ms
however quickly the servo answered.

Measured on an XL330 at 1 Mbaud:

| | per cycle | rate |
|---|---|---|
| as the port opens | 15.01 ms | 66.6 Hz |
| with a 1 ms read timeout | 1.12 ms | 889.8 Hz |

`Link.Open` now asks the port for a 1 ms read timeout through the optional
`serial.ReadTimeouter` interface, which uses the Windows combination that
returns as soon as any byte arrives. A 100 Hz control loop went from a measured
70.0 Hz with 5.92 ms of jitter to 99.9 Hz with 0.44 ms.

This is the serial half of the same mistake the CAN side had; see the matching
section in [MOTEUS.md](MOTEUS.md).

## Telemetry is split across two query profiles

Position, velocity and current are one 10-byte read from 126. Voltage,
temperature and the hardware error register cost extra bytes and an extra
transaction, so they ride only on the wider profiles — a fast cycle leaves them
unreported rather than stale, and `Telemetry.Merge` keeps the last known value.

## Torque is not reported unless the constant is known

`Profile.TorqueConstant` is zero for the families here, because it varies per
model within a family. A zero means torque is reported as unmeasured rather
than as a number derived from a guess. Set `torque_constant` in the config to
get real torque readings.

## Verified

Against an XL330-M288-T (model 1200, firmware 53) on COM6 at 1 Mbaud:
open and identify, profile resolution in all three cases above, position
control, velocity control, the fast/full telemetry split, 885 cycles/s with no
failures, and `bringup` from a robot description with the host envelope
clamping a 99 rad request to the joint's 5.5 rad limit.
