# ROS 2

Since 2026-11-03 there is no bridge. `drlabs rt` speaks CycloneDDS under
ROS 2's own names, so a node built with `rmw_cyclonedds` on the runtime's
domain publishes and subscribes to the runtime's records directly:

```
ROS 2 node (rmw_cyclonedds)                 drlabs rt
  /drlabs/lowcmd    ── drlabs_msgs/LowCmd ──▶  bus threads
  /drlabs/lowstate  ◀─ drlabs_msgs/LowState ─  every api.state_div ticks
  /drlabs/describe  ◀─ drlabs_msgs/Describe ─  once, latched (JSON)
```

`drlabs` is `api.dds.prefix` in the robot description; the domain is
`api.dds.domain` (0). The runtime does not run ROS 2 and does not need it
installed: it hand-writes the same CDR the generated types produce
(`crates/drlabs-api/src/cdr.rs`, pinned byte for byte in tests).

## The interface package

`tools/ros2/drlabs_msgs` holds the five `.msg` files and nothing else:

| message | fields |
|---|---|
| `MotorCmd` | `uint8 mode` (0 OFF, 1 DRIVE), `float32 q, dq, kp, kd, tau` |
| `MotorState` | `float32 q, dq, tau, current, voltage, temp`, `uint16 fault`, `uint8 status, blocked` |
| `LowCmd` | `uint32 seq, flags`, `uint64 t_ns`, `uint16 n`, `MotorCmd[64] cmd` |
| `LowState` | `uint32 seq, cmd_seq`, `uint64 tick, t_ns`, `uint32 period_ns, deadline_miss`, `uint16 estop_code`, `uint8 estop`, `uint16 n`, `MotorState[64] state` |
| `Describe` | `string json` |

Units are joint space: radians at the output, rad/s, newton-metres at the
mechanism. `NaN` means not reported. `LowCmd.flags` bit 0 is `QUICK_STOP`
(latch the robot-wide fault), bit 1 `FAULT_RESET` (clear it; every axis is
then OFF). The meaning of every field is in [RT.md](RT.md).

```bash
cd <your_ws>/src && ln -s <this repo>/tools/ros2/drlabs_msgs .
cd <your_ws> && colcon build --packages-select drlabs_msgs
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Driving it from the command line

With `drlabs rt configs/sim26.json` running on the same machine:

```bash
ros2 topic echo /drlabs/describe --once          # the JSON description
ros2 topic hz   /drlabs/lowstate                 # the state cadence
ros2 topic pub -r 100 /drlabs/lowcmd drlabs_msgs/msg/LowCmd \
  "{seq: 1, n: 1, cmd: [{mode: 1, q: 0.3}]}"    # drive joint 0 to 0.3 rad
```

The runtime treats a command older than `api.cmd_timeout_ms` as stale (hold
or release, as configured), so a node has to keep publishing while it wants
a joint driven — `-r 100` above is not decoration.

## QoS

Match these or the reader will not see the writer:

| topic | reliability | durability | history |
|---|---|---|---|
| `lowcmd` | best effort | volatile | keep last |
| `lowstate` | best effort | volatile | keep last 1 |
| `describe` | reliable | transient local | keep last 1 |

`lowstate` is what `rclpy.qos.qos_profile_sensor_data` gives. For
`describe` use `QoSProfile(reliability=RELIABLE, durability=TRANSIENT_LOCAL,
depth=1)`.

## What has been verified

- A Rust client and the Python client, both on CycloneDDS, against a
  simulated runtime on this repository's test suite and `live_sim.py`.
- Not yet: a ROS 2 node. No machine with ROS 2 installed has been pointed at
  the runtime. The type names and the CDR are what `rmw_cyclonedds` expects,
  and the Python `cyclonedds` package agrees byte for byte, but until a
  `ros2 topic echo` has shown a state this section describes an intent.

## Why not a bridge any more

The bridge translated the runtime's UDP frames into `sensor_msgs/JointState`
and back. It had to be kept in step with two protocols and ran as a third
process between the controller and the robot. Speaking DDS from the runtime
removes the process and one of the protocols; the cost is CycloneDDS in the
binary (2.1 MB instead of 0.7) and CMake in the build. ROS 2's own naming
was chosen over a private one so that the interop needs no configuration
beyond the domain id.
