# Describing a robot

One JSON file says what the machine is; one command brings it up.

```
drlabs bringup examples/robots/humanoid_arm.json
```

That opens each bus once, connects every joint on it, energises the machine,
and then takes joint-addressed commands:

```
{"cmd":"position","joint":"r_sh_p","rad":0.5}
{"cmd":"pose","joints":{"r_sh_p":-0.3,"r_elb_p":-2.0,"r_wri_p":0.8}}
```

Nothing above this layer knows that `r_sh_p` is a moteus at CAN id 1 on COM5,
geared six to one. That is the point of the file.

## The description

```json
{
  "robot": "right_arm",
  "rate_hz": 100,

  "buses": [
    { "name": "can0", "transport": "fdcanusb", "port": "COM5" },
    { "name": "dxl0", "transport": "serial", "port": "COM3", "baud": 1000000 }
  ],

  "joints": [
    {
      "name": "r_sh_p",
      "bus": "can0", "driver": "moteus", "id": 1,
      "gear_ratio": 6.0, "direction": 1, "offset_rad": 0.0,
      "limits": {
        "min_rad": -1.57, "max_rad": 1.57,
        "max_torque_nm": 6.0, "max_velocity_rad_s": 2.0, "max_accel_rad_s2": 8.0
      }
    },
    {
      "name": "r_elb_p",
      "bus": "can0", "driver": "robstride", "id": 3,
      "options": { "model": "RS02" },
      "gear_ratio": 1.0, "direction": 1, "offset_rad": 0.0,
      "limits": { "min_rad": -2.4, "max_rad": 0.0,
                  "max_torque_nm": 8.0, "max_velocity_rad_s": 3.0 }
    },
    {
      "name": "r_wri_p",
      "bus": "dxl0", "driver": "dynamixel", "id": 11,
      "gear_ratio": 1.0, "direction": 1, "offset_rad": 0.0,
      "limits": { "min_rad": -1.2, "max_rad": 1.2,
                  "max_torque_nm": 1.5, "max_velocity_rad_s": 4.0 }
    }
  ]
}
```

Buses and joints are separate because several joints share one bus. Naming the
bus once is what lets the runtime open the port a single time and hand the same
connection to every motor on it.

### The joint-to-motor mapping

Everything above this layer is in joint space — radians at the output, torque at
the mechanism. Everything below is in motor space — revolutions at the shaft,
torque at the rotor. Three numbers separate them, and each is a thing bring-up
measures:

| Field | Meaning |
|---|---|
| `gear_ratio` | motor turns per joint turn; `6.0` means the motor turns six times for one turn of the joint |
| `direction` | `1` or `-1`; recorded when somebody watches which way the axis actually goes |
| `offset_rad` | where the motor sits, in joint radians, when the joint reads zero |

Torque converts the other way from speed, because that is what a gearbox does:
a joint limited to 6 Nm behind a 6:1 reduction is 1 Nm at the rotor. Getting
that backwards gives a limit thirty-six times too generous, which is why it has
a test of its own.

Velocities are converted without the offset — an offset shifts a position, and
applying it to a rate would add a constant drift to every command.

### Limits are the envelope

`limits` is the joint's envelope, in joint units, and it becomes the host-side
safety configuration for that axis: the position envelope, the torque ceiling,
the velocity and acceleration ceilings. Nothing the upper layer sends can
exceed it, and the working parameters are set from it too — a joint that
declares six newton-metres is driven with six available, not with the
framework's cautious default.

These are the numbers [BRINGUP.md](BRINGUP.md) produces. The description is
where they end up.

## Validation happens before anything opens

```
drlabs bringup -check robot.json
```

prints the plan and touches no hardware:

```
JOINT    DRIVER     BUS   ID  GEAR  DIR  TRAVEL (rad)    TORQUE
r_sh_p   moteus     can0  1   6:1   +1   -1.570 … 1.570  6.00 Nm
r_elb_p  robstride  can0  3   1:1   +1   -2.400 … 0.000  8.00 Nm
r_wri_p  dynamixel  dxl0  11  1:1   +1   -1.200 … 1.200  1.50 Nm

BUS   TRANSPORT  PORT  LOAD
can0  fdcanusb   COM5  200 transactions/s
dxl0  serial     COM3  100 transactions/s
```

The loader is deliberately strict, because a description is written once and
then trusted by everything above it. It rejects a missing gear ratio, a
direction that is not ±1, an empty travel range, a joint with no torque
ceiling, two joints at one address, two joints with one name, an undefined bus —
and any key it does not recognise, so that `gear_ration` is an error rather
than a joint quietly geared 1:1.

**The bus load line is the one to read.** Every joint on a bus costs one
transaction per cycle, so eight joints at 200 Hz ask the adapter for 1600 round
trips a second. That number, not the joints individually, decides whether a
rate is achievable.

## The rate is a ceiling, and it belongs to the bus

`rate_hz` on the robot is the default control rate; a bus may override it:

```json
"buses": [
  { "name": "can0", "transport": "fdcanusb", "port": "COM5", "rate_hz": 200 },
  { "name": "dxl0", "transport": "serial", "port": "COM6", "baud": 1000000, "rate_hz": 100 }
]
```

The bus is where the constraint actually lives. A CAN-FD adapter carries
several hundred transactions a second without complaint and a Dynamixel chain
does not, so a machine with both is otherwise either throttled to what the
slowest bus can take or pushed past what it can. `bringup` prints the rate and
the load each bus is being asked for, which is the number that decides whether
the description is achievable at all.

It is a ceiling, not a suggestion. A command wakes the loop so that it is
noticed without polling, but it does not move the slot: the loop cycles at the
declared rate however fast commands arrive.

That is not decoration. Before it, a wake ran a cycle there and then, so a
client commanding faster than the rate simply got a faster loop — measured on
a two-actuator bench under a flood of poses from a slider drag:

| | configured | measured |
|---|---|---|
| moteus on CAN | 100 Hz | 801 Hz |
| XL330 on serial | 100 Hz | 345 Hz, then 13 Hz, then e-stopped |

The serial bus could not carry 345 Hz. It fell behind, the servo's own bus
watchdog latched, and that joint stopped — a machine halted by how fast
somebody moved a mouse. With the ceiling in place the same flood, 32,282 poses
a second for six seconds, leaves both joints at exactly their declared rates
and no fault.

## Bring-up is all or nothing

The first joint that cannot be brought up takes the whole robot down with it,
torque off, naming the joint and the reason.

A humanoid with one dead shoulder is more dangerous than one that never came
up: everything else will hold poses that assume the missing joint is holding
too. Bring-up also waits until every joint has actually energised before
reporting success — arming is not instant, and handing the upper layer a
machine that is only partly live is the same failure in slower motion.

## How several motors share one bus

A transaction is a send followed by a receive, and the receive takes whatever
frame arrives next. If two motors' transactions interleave, each reads the
other's reply, discards it as not its own, and both time out. Locking the
individual calls would not help — the *pair* has to be atomic.

So `can.Shared` and `serial.Shared` wrap a bus with a lock that drivers take
for a whole exchange, through a `Locker` interface that an unshared bus simply
does not implement. A single-motor setup pays nothing.

Each joint still runs its own control loop. The alternative — one loop per bus
cycling every joint in turn — would couple joints on the same wire more
tightly, and is the right shape if coordinated motion ever needs to be
simultaneous to better than a cycle. What it costs is the whole of
`runtime/session`: arming, the trips, the trajectory sequencer, the emergency
stop, all written per axis. Sharing the bus keeps all of that and gives up only
exact phase alignment, which at 100 Hz is a few milliseconds of skew between
joints.

If your machine needs better than that, this is the seam to change.

## Commands

| Line | Effect |
|---|---|
| `{"cmd":"position","joint":"r_sh_p","rad":0.5}` | one joint to an angle |
| `{"cmd":"pose","joints":{"r_sh_p":-0.3,"r_elb_p":-2.0}}` | several at once |
| `{"cmd":"velocity","joint":"r_sh_p","rad_s":0.4}` | one joint at a rate |
| `{"cmd":"move","joint":"r_sh_p","rad":1.0,"max_velocity":0.5,"accel":2.0}` | profiled move |
| `{"cmd":"hold","joint":"r_sh_p"}` / `{"cmd":"stop","joint":"r_sh_p"}` | hold / release one joint |
| `{"cmd":"estop"}` / `{"cmd":"reset"}` | stop or release the whole machine |
| `{"cmd":"joints"}` | the joint names |
| `{"cmd":"state"}` | every joint's state, on demand |

`pose` checks every name before commanding anything: a pose applied to half a
machine is not a pose. State arrives unsolicited at `-state-rate` Hz, in joint
units, with unread measurements as `null` rather than zero.

## Testing it

`test/client/robot.py` is a working client and a bench tool in one file, in
joint units:

```
python test/client/robot.py joints                  the joint names
python test/client/robot.py state                   every joint, once
python test/client/robot.py pos  r_sh_p 0.2         move one joint, radians
python test/client/robot.py move r_sh_p 0.3 --vel 0.5
python test/client/robot.py pose r_sh_p=0.1 r_elb_p=-0.4
python test/client/robot.py sweep r_sh_p 0.15 3
python test/client/robot.py estop | reset
```

Motion commands wait for the joints to settle and report the error against
what was asked. They refuse to command a joint that is not armed, and print the
preflight reasons instead.

Start on `examples/robots/sim_arm.json`, which needs no hardware at all, then
change `driver` and `port` to the real thing.

### A browser panel

`test/robotui` is the same thing with sliders, for driving a machine by hand
while watching what it does:

```
drlabs bringup test/shoulder_pair.json
go run ./test/robotui -robot test/shoulder_pair.json
```

Then open `http://localhost:8080`. It reads the description for the joint
names and travel limits, so the sliders are bounded — but only as a courtesy.
The panel is not privileged: it speaks the same JSON down the same socket as
any other client and is refused by the same envelope.

Two things it deliberately does not do. It never snaps a slider to the measured
position while somebody is holding it — the command and the measurement are
separate readouts, because a control that chases the measurement hides exactly
the disagreement worth seeing. And it never paints the machine as stopped
because the operator pressed stop; `armed`, `e-stopped` and the fault come only
from state the robot actually sent.

## Verified

Three simulated joints, 100 Hz, one with `direction: -1` and two geared 6:1:

| Behaviour | Result |
|---|---|
| Bring-up | all three connected and energised; torque ceilings 1.00, 1.00, 1.50 Nm from the 6 / 6 / 1.5 Nm joint declarations |
| `position r_sh_p 0.5` | arrived +0.5000 rad |
| `position r_elb_p -1.0` (direction −1) | arrived −1.0000 rad, reported in joint space |
| `pose` of three joints | all arrived, largest error 0.0005 rad |
| `move r_wri_p -0.9` profiled | arrived −0.9000 rad |
| `position r_wri_p 99` | clamped to +1.2002 rad, the declared travel |
| unknown joint name | refused, with the names it does have |

Not yet exercised on hardware. In particular: two motors sharing one real bus.
The locking is correct by construction and the mapping is covered by tests, but
neither has met a real adapter with two controllers on it.
