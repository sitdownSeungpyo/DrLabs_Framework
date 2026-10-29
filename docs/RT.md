# The real-time runtime (`drlabs rt`)

One thread per bus. Each tick the thread takes the latest `LowCmd` snapshot,
decides per axis whether it may be driven, converts joint space to motor
space, makes one call into the bus, converts back, evaluates the safety
guard, and publishes a snapshot. Nothing on that path takes a lock or
allocates. Crates: `drlabs-rt` (daemon), `drlabs-core::lowlevel` and
`drlabs-core::bus` (the records and the bus trait), `drlabs-safety::Guard`
(the trip logic without strings).

```
 client ──LowCmd──▶ [triple buffer] ──▶ BusWorker ──▶ Bus::cycle ──▶ wire
 client ◀─LowState─ [triple buffer] ◀── BusWorker ◀── replies    ◀── wire
```

What the runtime does *not* do: plan trajectories, move to poses, learn
ranges, run burn-in sequences. Those are clients.

## The two records

Joint space: radians at the output, newton-metres at the mechanism. `NaN`
means not reported (these records cross a process boundary where `Option`
has no shape). Little-endian, `repr(C)`; sizes are pinned by tests in
`drlabs-core/src/lowlevel.rs`.

`MotorCmd` (24 bytes): `mode u8, reserved[3], q f32, dq f32, kp f32, kd f32, tau f32`.
`mode` 0 = OFF (observe, torque off), 1 = DRIVE.

`MotorState` (28 bytes): `q, dq, tau, current, voltage, temp: f32; fault u16; status u8; blocked u8`.
`status` is an `AxisStatus` (0 ok, 1 silent, 2 alert, 3 refused, 4 off, 5 tripped);
`blocked` says why a DRIVE request is not honoured (`safety::Blocked`).

`LowCmd` (1560 bytes): `seq u32, flags u32, t_ns u64, n u16, reserved[6], cmd[64]`.
`flags`: bit 0 ESTOP, bit 1 RESET_ESTOP.

`LowState` (1832 bytes): `seq u32, cmd_seq u32, tick u64, t_ns u64, period_ns u32,
deadline_miss u32, estop_code u16, estop u8, reserved u8, n u16, reserved[2], state[64]`.

How a driver reads `{mode, q, dq, kp, kd, tau}`:

| driver | mode 0 | mode 1 |
|---|---|---|
| dynamixel | torque off | goal position = `q` through the mapping, clamped to the envelope; `dq`, `tau` not applied; `kp`/`kd` not applied in this version |
| sim | stop | position target with the simulator's own PD |
| moteus | (not on the bus trait yet) | |

The torque ceiling, velocity limit, acceleration limit and watchdog come from
the joint's declared limits every tick, whatever the client sends.

## The gate

The session's arm/disarm/e-stop state machine, driven by `mode`:

- `OFF` → the axis is read, not driven.
- `DRIVE` → honoured once the axis passes preflight (telemetry received,
  supply in band, no hard fault, position inside the envelope plus margin).
  Until then it stays off and `blocked` says why.
- A guard trip, `max_consecutive_timeouts` silent ticks on a driven axis,
  `deadline_miss_trip` consecutive late ticks, a transport failure, or the
  client's ESTOP flag latches a robot-wide stop: every axis gets Stop,
  `LowState.estop = 1`, `estop_code` is the `TripCode`.
- RESET_ESTOP clears the latch. Every axis is then OFF and has to be asked
  for again — clearing a stop drives nothing.
- A `LowCmd` older than `api.cmd_timeout_ms` is stale: `on_cmd_timeout:
  hold` keeps the last `q` with `dq`/`tau` zeroed, `release` turns every
  axis off. The servo's own bus watchdog is the second layer.

## Configuration

The robot description from `drlabs-robot` plus two sections, both optional:

```json
"rt":  { "cpus": [], "priority": 0, "deadline_miss_trip": 5 },
"api": { "udp": "127.0.0.1:7700", "state_div": 1, "cmd_timeout_ms": 100, "on_cmd_timeout": "hold" }
```

`buses[].transport` may be `sim`. `rate_hz` up to 1000. At most 64 joints.

## Measured

Windows 11, no real-time scheduling, release build, `configs/sim26.json`
(26 simulated joints on two buses, 500 Hz), `drlabs rt -drive -seconds 600`:

```
300002 ticks in 600 s = 500.0 Hz
59 deadline misses (a tick starting more than one period late): 0.02 %
late_max 3.73 ms; no e-stop
```

This is the floor the daemon itself sets. The same run on the robot PC
under Linux with `rt.priority` set is the number that matters, and it has
not been taken yet. No hardware bus has been run through this runtime yet
(COM7 was not connected on 2026-10-27).

macOS 26.5 on Apple Silicon, release build, the same `configs/sim26.json`,
`drlabs rt -drive -seconds 60` (2026-10-29):

```
before the time-constraint policy   29969 ticks   72 misses (0.24 %)   late_max 11.42 ms
rt.priority 0  (default policy)     30053 ticks    0 misses             late_max  0.14 ms
rt.priority 80 (policy at 2 ms)     30056 ticks    0 misses             late_max  1.44 ms
```

The first line is what a default macOS thread does: its sleeps overshoot by
about a quarter of their length, so a 1.5 ms spin lead cannot catch a 2 ms
sleep. The other two are the same binary with every bus thread under the mach
time-constraint policy, which `Resolution::raise` now applies on macOS
whether or not `rt.priority` is set. Single runs each; the difference between
the last two is within what one run shows.

## The wire

Frames over UDP; `crates/drlabs-api` is the Rust side, `tools/drlabs_client`
the Python reference. Both pin the same byte-exact test vectors.

```
0   "DLRT"          magic
4   u16 version     PROTOCOL_VERSION (1)
6   u16 kind        1 CMD, 2 STATE, 3 DESCRIBE, 4 DESCRIBE_REPLY
8   payload         CMD: LowCmd header (24 B) + n × MotorCmd (24 B)
                    STATE: LowState header (40 B) + n × MotorState (28 B)
                    DESCRIBE: empty
                    DESCRIBE_REPLY: UTF-8 JSON (joints in record order, limits, poses)
-4  u32 crc32       IEEE (zlib's), over everything before it
```

Only the `n` entries in use travel: 26 joints is a 660-byte command and a
776-byte state. The runtime sends a STATE to every address it has heard a
frame from in the last two seconds, every `api.state_div` ticks. A client
with nothing to command keeps the states coming by sending a DESCRIBE now
and then; `Client` in both languages does this.

`drlabs rt <robot.json>` serves `api.udp` (default `127.0.0.1:7700`).

## Clients that exist

- `tools/drlabs_client` — Python: `Client`, `move_to` (trapezoid, client side),
  `hold`, `release`, `estop`/`reset_estop`. `tests/live_sim.py` drives a
  simulated runtime end to end.
- `drlabs panel [-rt host:port] [robot.json]` — the browser panel, now a
  client of the runtime. Same HTTP API as before (`/api/state`, `/api/pose`,
  `/api/move`, `/api/hold`, `/api/stop`, `/api/estop`, `/api/reset`,
  `/api/learn/*`, `/api/limits`), so `tools/demo_*.py` and
  `tools/show_monitor.py` run unchanged against it. Moves are planned in the
  panel; `robot.json` is needed only to write edited limits back.
- `tools/ros2_bridge` — `link.py` speaks the frames through `drlabs_client`;
  the node's `port` parameter now defaults to 7700. `test/test_link.py`
  stands up a fake runtime on loopback.

## Real-time scheduling

`rt.priority` (Linux `SCHED_FIFO` 1..99) and `rt.cpus` (one core per bus,
cycled) are asked for when each bus thread starts, together with
`mlockall`. Each is granted or refused separately and the thread logs one
line saying which:

```
[dxl0] scheduling: priority 80: granted, cpu 2: pinned, memory locked, real-time
[dxl0] scheduling: priority 80: refused, cpu 2: pinned, memory not locked, best effort, not real-time
```

Unprivileged, `SCHED_FIFO` needs `CAP_SYS_NICE` (or a `rtprio` limit in
`/etc/security/limits.conf`) and `mlockall` a `memlock` limit. On Windows the
request becomes `THREAD_PRIORITY_TIME_CRITICAL` and an affinity mask, and is
reported as best effort whatever it returns. None of this has been run on a
Linux machine yet; the code type-checks for `x86_64-unknown-linux-gnu` and
`aarch64-unknown-linux-gnu`.

On macOS a non-zero `rt.priority` asks for the mach time-constraint policy,
declared for the bus's actual period (a quarter of it as computation, half as
the deadline); the number itself has no meaning there. It needs no privilege.
`rt.cpus` is ignored — Apple Silicon does not pin threads to cores, and
`drlabs rt` says so once at start-up — and there is no `mlockall`. The log
line is therefore always of this shape, and it is accurate:

```
[simA] scheduling: priority 80: granted (time-constraint), memory not locked, best effort, not real-time
```

"Best effort" because the policy is soft: a thread that overruns its declared
computation is scheduled as if it had not asked, for a while. In practice it
is what makes a loop on macOS keep time at all; see *Measured* above.
