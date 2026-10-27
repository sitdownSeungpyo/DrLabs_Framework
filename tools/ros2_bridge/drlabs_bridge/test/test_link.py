"""Host tests for the link.

These need no ROS installation and no hardware: they stand up a fake runtime
on a loopback UDP socket and drive the real link against it.  That covers the
half of the bridge most likely to be wrong — the wire format, the command
mapping, and what happens when the runtime goes quiet — while the node itself
needs a machine with rclpy.

    python tools/ros2_bridge/drlabs_bridge/test/test_link.py
    pytest tools/ros2_bridge/drlabs_bridge/test
"""

import json
import os
import socket
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "..", "drlabs_client"))

from drlabs_bridge.link import LOST, READY, JointState, RobotLink  # noqa: E402
from drlabs_client.codec import (  # noqa: E402
    CMD, DESCRIBE, DRIVE, ESTOP, OFF, LowState, MotorState, decode,
    encode_describe_reply, encode_state,
)


class FakeRuntime:
    """A runtime that describes two joints, streams states to whoever has
    spoken, and records every command."""

    def __init__(self, joints=("a", "b"), state_hz=100.0):
        self.joints = list(joints)
        self.period = 1.0 / state_hz
        self.commands = []
        self.positions = [0.1, -0.2]
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.05)
        self.port = self._sock.getsockname()[1]
        self._subscribers = {}
        self._running = True
        self._seq = 0
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def describe_json(self):
        return json.dumps({
            "protocol": 1, "robot": "fake", "rate_hz": 500, "period_ns": 2_000_000,
            "cmd_timeout_ms": 100, "on_cmd_timeout": "hold",
            "joints": [{"index": i, "name": n, "bus": "sim", "driver": "sim", "id": i + 1,
                        "min_rad": -1.0, "max_rad": 1.0, "max_torque_nm": 1.0,
                        "max_velocity_rad_s": 2.0, "max_accel_rad_s2": 4.0}
                       for i, n in enumerate(self.joints)],
            "poses": {"zero": {n: 0.0 for n in self.joints}},
        })

    def _serve(self):
        next_state = time.monotonic()
        while self._running:
            try:
                data, addr = self._sock.recvfrom(65536)
                kind, payload = decode(data)
                self._subscribers[addr] = time.monotonic()
                if kind == DESCRIBE:
                    self._sock.sendto(encode_describe_reply(self.describe_json()), addr)
                elif kind == CMD:
                    self.commands.append(payload)
            except (socket.timeout, ValueError):
                pass
            if time.monotonic() >= next_state:
                next_state += self.period
                self._seq += 1
                driving = self.commands[-1].cmd if self.commands else []
                state = LowState(seq=self._seq, tick=self._seq, period_ns=2_000_000, state=[
                    MotorState(q=self.positions[i], dq=0.0, voltage=12.0, temp=30.0,
                               status=0 if i < len(driving) and driving[i].mode == DRIVE else 4)
                    for i in range(len(self.joints))])
                frame = encode_state(state)
                now = time.monotonic()
                for addr, seen in list(self._subscribers.items()):
                    if now - seen < 2.0:
                        self._sock.sendto(frame, addr)

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._sock.close()


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_link_connects_and_reports_joints():
    runtime = FakeRuntime()
    states, transitions = [], []
    link = RobotLink("127.0.0.1", runtime.port,
                     on_state=states.append, on_connection=transitions.append)
    try:
        link.start()
        assert wait_for(lambda: link.state == READY), transitions
        assert link.joint_names == ["a", "b"]
        assert wait_for(lambda: len(states) > 3)
        joints = states[-1]
        assert [j.name for j in joints] == ["a", "b"]
        assert isinstance(joints[0], JointState)
        assert abs(joints[0].position_rad - 0.1) < 1e-6
        assert joints[0].voltage_v == 12.0
        assert joints[0].torque_nm is None, "an unreported torque is None, not zero"
        assert not joints[0].armed
    finally:
        link.stop()
        runtime.stop()


def test_commands_become_frames():
    runtime = FakeRuntime()
    link = RobotLink("127.0.0.1", runtime.port)
    try:
        link.start()
        assert wait_for(lambda: link.state == READY)
        assert link.set_positions({"b": 0.5}) == {"ok": True}
        assert wait_for(lambda: runtime.commands and runtime.commands[-1].cmd[1].mode == DRIVE)
        cmd = runtime.commands[-1]
        assert abs(cmd.cmd[1].q - 0.5) < 1e-6
        assert cmd.cmd[0].mode == OFF, "a joint not named stays as it was"

        assert link.stop_joint("b") == {"ok": True}
        assert wait_for(lambda: runtime.commands[-1].cmd[1].mode == OFF)

        assert link.estop("test") == {"ok": True}
        assert wait_for(lambda: runtime.commands[-1].flags & ESTOP)

        assert link.set_velocity("a", 1.0)["ok"] is False
    finally:
        link.stop()
        runtime.stop()


def test_a_quiet_runtime_is_reported_lost():
    runtime = FakeRuntime()
    transitions = []
    link = RobotLink("127.0.0.1", runtime.port, on_connection=transitions.append)
    try:
        link.start()
        assert wait_for(lambda: link.state == READY)
        runtime.stop()
        assert wait_for(lambda: link.state == LOST, timeout=5.0), transitions
    finally:
        link.stop(release=False)


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
