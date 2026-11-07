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

| `mode` | name | what the axis does |
|---|---|---|
| 0 | OFF | observed, torque off |
| 1 | DRIVE | position `q`, with `dq` and `tau` as feed-forward |
| 2 | VELOCITY | runs at `dq`, no position target, `tau` feed-forward; the envelope edge ahead is sent as the stop position |
| 3 | TORQUE | `tau` only |
| 4 | HOLD | zero-velocity servo; a position servo holds where the hold began |
| 5 | STAY_WITHIN | torque only outside the joint's declared travel, `tau` inside it |
| 6 | ZERO | once, on the change into this mode: the current output position becomes `q`; the axis must be off and is stopped first |

`kp` and `kd` are carried but not applied in this version. A `ZERO` repeated
by a keep-alive is not a new request; it takes a change of mode away and
back. The torque ceiling, velocity limit, acceleration limit and watchdog
come from the joint's declared limits every tick, whatever the mode.

`MotorState` (28 bytes): `q, dq, tau, current, voltage, temp: f32; fault u16; status u8; blocked u8`.
`status` is an `AxisStatus` (0 ok, 1 silent, 2 alert, 3 refused, 4 off, 5 tripped,
6 held — powered by the stop policy, not limp);
`blocked` says why a DRIVE request is not honoured (`safety::Blocked`).

`LowCmd` (1560 bytes): `seq u32, flags u32, t_ns u64, n u16, reserved[6], cmd[64]`.
`flags`: bit 0 QUICK_STOP, bit 1 FAULT_RESET.

`LowState` (1832 bytes): `seq u32, cmd_seq u32, tick u64, t_ns u64, period_ns u32,
deadline_miss u32, estop_code u16, estop u8, reserved u8, n u16, reserved[2], state[64]`.

How a driver reads the modes:

| driver | OFF | DRIVE | VELOCITY, TORQUE, STAY_WITHIN | HOLD | ZERO |
|---|---|---|---|---|---|
| dynamixel | torque off | goal position = `q` through the mapping, clamped; `dq`, `tau` not applied | refused (`status` 3); the servo is a position servo | goal position = where the hold began | refused; use `offset_rad` in the description |
| sim | stop | position target with the simulator's own PD | native | native | native |
| moteus | stop | position with `dq`/`tau` feed-forward | native (moteus modes 10 and 13) | native (mode 12) | native (set output exact; runtime only) |

A refused mode leaves the axis torque-off and says so in `status`; it does
not trip the robot. Leaving any driving mode sends three stop frames before
the axis is merely observed, so a released motor is told rather than left
to its watchdog.

## The gate

The session's enable-operation / disable-operation / fault state machine (CiA 402 names, see [ARCHITECTURE.md](ARCHITECTURE.md)), driven by `mode`:

- `OFF` → the axis is read, not driven.
- `DRIVE`, `VELOCITY`, `TORQUE`, `HOLD`, `STAY_WITHIN` → honoured once the
  axis passes preflight (telemetry received, supply in band, no hard fault,
  position inside the envelope plus margin). Until then it stays off and
  `blocked` says why.
- `ZERO` → not a drive: the axis is stopped if it was driving, then
  re-referenced once.
- A guard trip, `max_consecutive_timeouts` silent ticks on a driven axis,
  `deadline_miss_trip` consecutive late ticks, a transport failure, or the
  client's QUICK_STOP flag latches a robot-wide fault: `LowState.estop = 1`,
  `estop_code` is the `TripCode`, and every axis does what its stop policy
  says (below).
- FAULT_RESET clears the latch. Every axis is then OFF and has to be asked
  for again — clearing a stop drives nothing — except a held one, which
  keeps its place until the client's next command (a later `seq` than the
  reset's): DRIVE carries on from the held place, anything else releases it.
  A reset is acted on once per `seq`; a client that restarts its counter
  must not reuse the number of its last reset.

### Stop policy (`rt.on_estop`, a joint's own `on_estop` overrides)

| policy | a driven axis on a stop |
|---|---|
| `release` (default) | torque off at once (category 0) — a gravity-loaded joint falls |
| `hold` | decelerates within its acceleration limit to the stopping point and holds it, powered (category 2); `status` 6 |
| `stop_then_release` | decelerates to rest (at most 2 s), then torque off (category 1) |

Only stops that holding makes safer hold: QUICK_STOP, deadline, velocity
runaway, position envelope, comm loss. A motor or supply fault (current,
temperature, voltage, controller fault, stall) and a transport failure
release. The axis whose own guard or silence raised the stop is never held,
whatever the code. A held axis that stops answering is released; after a
reset a held axis whose client has gone quiet follows `on_cmd_timeout`.
Measured 2026-11-07 on 7 XL430 (gravity-loaded arm, 400 Hz, `hold`): quick
stop mid-sine, decelerated in 0.25 s, sagged ≤ 0.007 rad over 3 s held,
resumed after reset with no jump, 0 deadline misses.
- A `LowCmd` older than `api.cmd_timeout_ms` is stale: `on_cmd_timeout:
  hold` keeps the last `q` with `dq`/`tau` zeroed (a stale `VELOCITY` or
  `TORQUE` becomes a `HOLD`), `release` turns every axis off. The servo's
  own bus watchdog is the second layer.

## Configuration

The robot description (`crates/drlabs-rt/src/config.rs`) plus two sections,
both optional:

```json
"rt":  { "cpus": [], "priority": 0, "deadline_miss_trip": 5 },
"api": { "dds": { "domain": 0, "prefix": "drlabs" }, "state_div": 1, "cmd_timeout_ms": 100, "on_cmd_timeout": "hold" }
```

`buses[].transport` is `sim`, `serial` (Dynamixel) or `fdcanusb` (moteus;
RobStride and ODrive have no bus driver yet). `rate_hz` up to 1000. At most
64 joints.

The moteus bus (`crates/drivers/drlabs-moteus/src/bus.rs`) sends every
axis's frame before waiting for any reply, then sorts the replies back by
the controller's source id, so the adapter's turnaround is paid once a tick.
Its frames are tested byte for byte against the one-axis link's, which are
pinned against the reference `moteus` package. It has not yet run on the
bench (2026-11-04): the cycle time of N frames through the fdcanusb's line
protocol is the measurement that decides how many moteus one adapter can
carry at 500 Hz.

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

CycloneDDS, under ROS 2 names (since 2026-11-03; before that, UDP frames of
this crate's own design). `crates/drlabs-api` is the Rust side,
`tools/drlabs_client` the Python reference, `tools/ros2/drlabs_msgs` the
ROS 2 interface package whose generated types match these byte for byte.

| ROS 2 topic | DDS type | QoS |
|---|---|---|
| `/<prefix>/lowcmd` | `drlabs_msgs::msg::dds_::LowCmd_` | best effort, volatile, keep last 4 |
| `/<prefix>/lowstate` | `drlabs_msgs::msg::dds_::LowState_` | best effort, volatile, keep last 1 |
| `/<prefix>/describe` | `drlabs_msgs::msg::dds_::Describe_` | reliable, transient local, keep last 1 |

`prefix` is `api.dds.prefix` (`drlabs` by default) and the runtime joins
`api.dds.domain` (0). On the DDS wire the topic names carry ROS 2's `rt/`
in front; a ROS 2 node never sees it. The sample body is XCDR1,
little-endian, with the 4-byte `00 01 00 00` encapsulation header, fields in
`.msg` order, each primitive aligned to its size from the start of the body
and fixed arrays laid out whole — so a command is always 1556 bytes and a
state 1848, whatever `n` says. `crates/drlabs-api/src/cdr.rs` writes that
by hand; `tools/drlabs_client/tests/test_cdr.py` pins its bytes against the
Eclipse `cyclonedds` Python package, the same way the moteus frames are
pinned against the official `moteus` package.

Late samples are dropped, not retried: a command older than the one after
it is worthless, which is what best-effort QoS says. The description is
written once and latched, so a client that arrives later still receives it
without asking. States go out every `api.state_div` ticks.

A ROS 2 node with `rmw_cyclonedds` on the same domain reads and writes these
directly — see [ROS2.md](ROS2.md). The transport is a client of the daemon's
handle, not part of it: the bus threads never see DDS.

## Clients that exist

- `drlabs run` and `drlabs bringup` — in-process clients: they build or
  load a description, start the daemon in the same process, and plan on top
  of it (`run` streams a trapezoid for `-position` and `-shuttle`;
  `bringup` energises every joint where it is and optionally sweeps). No
  DDS endpoint is opened; `drlabs rt` is the one that serves clients.

- `tools/drlabs_client` — Python on the Eclipse `cyclonedds` package:
  `Client(domain, prefix)`, `move_to` (trapezoid, client side), `hold`,
  `release`, `quick_stop`/`fault_reset`. `tests/live_sim.py` drives a
  simulated runtime end to end.
- `drlabs panel [-domain N] [-prefix name] [robot.json]` — the browser
  panel, a DDS client of the runtime. HTTP API: `/api/state`, `/api/pose`,
  `/api/move`, `/api/hold`, `/api/stop`, `/api/quick_stop`,
  `/api/fault_reset`, `/api/learn/*`, `/api/limits`; `tools/demo_*.py` and
  `tools/show_monitor.py` use it. Moves are planned in the panel;
  `robot.json` is needed only to write edited limits back.
- Any ROS 2 node with `rmw_cyclonedds` and `drlabs_msgs` — no bridge.

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
