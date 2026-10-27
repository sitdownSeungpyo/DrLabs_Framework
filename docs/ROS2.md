# The ROS 2 bridge

`ros2/drlabs_bridge` is a ROS 2 package that fronts a running `drlabs bringup`
server:

```
ROS 2                                   drlabs bringup
joint_commands  ──┐                  ┌──  JSON lines
                  ├─> bridge_node ──>┤     over TCP
joint_states    <─┘                  └──  robot + safety envelope
```

```bash
drlabs bringup examples/robots/humanoid_arm.json &
ros2 launch drlabs_bridge bridge.launch.py host:=127.0.0.1 port:=7777
ros2 topic echo /joint_states
```

## Why it is not gRPC

The design note this replaced planned a gRPC service with a generated proto and
a Python node on the far side. That would have worked, and it would have cost
the property the framework is otherwise careful about: `go.mod` has no
dependencies, and `docs/SERVO.md` describes the command interface as
"deliberately plain… callable from Python, ROS 2, C++ or a shell, with no
dependency on either side."

gRPC would have added a protobuf toolchain and a runtime to both halves in
order to expose a command surface that already exists. The bridge speaks the
protocol that is already there, so the Go side needed no code at all and the
ROS 2 side needs nothing beyond `rclpy` and the standard message packages.

## What the node is, and is not

It is a **lifecycle node**, because it manages hardware. An axis that is live
should be live because somebody activated it, and deactivating releases it
rather than leaving it holding — `release_on_deactivate` is on by default for
that reason.

It owns no control logic and no limits. The envelope, the trips, the
trajectory generator and the controller's own watchdog all stay on the far side
of the socket. The node translates, and is careful about what it claims:
`armed` and `estop` are published only from state lines the robot actually
sent, never from what was commanded.

| | |
|---|---|
| `joint_states` | `sensor_msgs/JointState`, sensor QoS |
| `joint_commands` | `sensor_msgs/JointState`, reliable, depth 10 |
| `~/connection` | `std_msgs/String`, latched — `disconnected`/`connecting`/`ready`/`lost` |
| `~/estop`, `~/reset` | `std_srvs/Trigger` |

A command carrying `position` becomes one `pose` — the whole set of joints in a
single message, so they are commanded together rather than one per round trip.
A command carrying only `velocity` becomes one velocity command per joint.
`NaN` entries are skipped, which is how a command addresses a subset.

Position targets are absolute, so a resend is harmless. That is what makes
retrying safe on a link that may have dropped mid-write.

State lines are buffered and flushed by a timer at `publish_rate`, not
republished as they arrive. A 200 Hz robot does not need to wake every ROS
subscriber 200 times a second.

## Missing measurements stay missing

A driver that did not read a temperature reports `null`, and the bridge
publishes `NaN` — never zero. `JointState` has no way to say "this joint
reports no effort", and a consumer that reads a missing torque as zero torque
draws exactly the wrong conclusion. This is the same rule the JSON protocol
follows.

## The link is a state machine, because it will drop

`link.py` knows nothing about ROS. It is a supervised connection: connecting →
ready → lost → connecting, with exponential backoff and jitter, and it
publishes every transition to `~/connection`. Commands issued while the link is
down raise rather than being silently dropped.

Two things that only showed up on the bench:

**Closing a socket's file object from another thread deadlocks.**
`socket.makefile()` returns a buffered reader that holds its own lock while it
waits for bytes. Closing *it* from the thread that wants to shut the link down
blocks behind the reader thread sitting inside it — and the reader is waiting
for data that is never coming, so it blocks forever. `_close_socket` shuts down
the socket instead, which is what makes the parked read return. Measured
before: a `stop()` that never returned. After: 0.71 s.

**Asking for the joint roster from the supervisor deadlocks too.** The first
version sent `{"cmd":"joints"}` right after connecting, from the same thread
that was about to start reading replies — so nothing was reading, and it timed
out on every connection. The roster is in every state line anyway, so the
request is gone and the names come from the stream.

## Verified

`ros2/drlabs_bridge/test/test_link.py` runs on a host with no ROS and no
hardware: nine tests against a fake bringup server covering the wire format,
refusals, the disconnect path and reconnection.

```bash
cd ros2/drlabs_bridge && pytest test
```

The link was also driven against a real `drlabs bringup` — connect, roster from
the stream, `pose` moving two joints, envelope clamping a 99 rad request,
e-stop and reset, reconnection after the server was killed and restarted, and a
clean `stop()` with the server gone.

**The node itself is unverified.** No machine with `rclpy` was available, so
`bridge_node.py` and `bridge.launch.py` have not been run: they are written to
the ROS 2 conventions but nothing has confirmed the lifecycle transitions, the
QoS settings or the launch file on a real distro. That is the first thing to do
on a machine with ROS 2 installed:

```bash
colcon build --symlink-install --packages-select drlabs_bridge
source install/setup.bash
ros2 launch drlabs_bridge bridge.launch.py
ros2 topic hz /joint_states        # should match publish_rate
ros2 topic info -v /joint_states   # QoS endpoints agree
```

## 2026-10-27: the bridge now fronts `drlabs rt`

The JSON-lines server this document describes was the Go `bringup`; the
Rust tree never implemented it. The bridge's `link.py` now speaks the
runtime's LowCmd/LowState frames through `tools/drlabs_client` (see
`docs/RT.md`), and the node's `port` parameter defaults to 7700. The ROS
surface — `joint_states`, `joint_commands`, `~/estop`, `~/reset`,
`~/connection` — is unchanged. Velocity commands are refused: the runtime
drives position servos in this version. `test/test_link.py` stands up a fake
runtime on loopback and needs no ROS.
