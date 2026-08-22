"""Host tests for the link.

These need no ROS installation and no hardware: they stand up a fake bringup
server on a loopback socket and drive the real client against it.  That covers
the half of the bridge most likely to be wrong — the wire format, and what
happens when the link drops — while the node itself needs a machine with rclpy.

    pytest ros2/drlabs_bridge/test
"""

import json
import socket
import threading
import time

import pytest

from drlabs_bridge.link import (CONNECTING, DISCONNECTED, LOST, READY, Backoff,
                                JointState, RobotLink)


class FakeServer:
    """A bringup server that answers commands and streams state."""

    def __init__(self, joints=("a", "b"), state_hz=50.0, port=0):
        self.joints = list(joints)
        self.state_period = 1.0 / state_hz
        self.received = []
        self.refuse = False
        self.port = port
        self._listener = None
        self._running = False
        self._thread = None
        self.open()

    def open(self):
        """Start listening, on the same port as last time when there was one."""
        listener = socket.socket()
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", self.port))
        listener.listen(1)
        self.port = listener.getsockname()[1]
        self._listener = listener
        self._running = True
        self._sessions = []
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        while self._running:
            try:
                self._listener.settimeout(0.2)
                conn, _ = self._listener.accept()
            except (socket.timeout, OSError):
                continue
            self._sessions.append(conn)
            threading.Thread(target=self._session, args=(conn,), daemon=True).start()

    def _session(self, conn):
        stream = conn.makefile("rwb")
        stop = threading.Event()

        def push_state():
            while self._running and not stop.is_set():
                payload = {"type": "state", "joints": [
                    {"name": name, "armed": True, "mode": "position",
                     "estop": False, "position_rad": 0.25, "velocity_rad_s": 0.0,
                     "torque_nm": None, "fault": 0, "fault_name": "No fault"}
                    for name in self.joints]}
                try:
                    stream.write((json.dumps(payload) + "\n").encode())
                    stream.flush()
                except OSError:
                    return
                time.sleep(self.state_period)

        threading.Thread(target=push_state, daemon=True).start()
        try:
            for raw in stream:
                command = json.loads(raw)
                self.received.append(command)
                reply = {"type": "reply", "ok": not self.refuse}
                if self.refuse:
                    reply["error"] = "arm the axis first"
                if command.get("cmd") == "joints":
                    reply["value"] = list(self.joints)
                stream.write((json.dumps(reply) + "\n").encode())
                stream.flush()
        except (OSError, ValueError):
            pass
        finally:
            stop.set()
            conn.close()

    def close(self):
        """Stop listening *and* drop every accepted session.

        Closing only the listener leaves existing clients connected, which is
        not what a bringup process being killed looks like — and a test that
        models it that way never exercises the reconnect path at all.
        """
        self._running = False
        if self._listener is not None:
            self._listener.close()
            self._listener = None
        for conn in getattr(self, "_sessions", []):
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass
        self._sessions = []
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)


@pytest.fixture
def server():
    fake = FakeServer()
    yield fake
    fake.close()


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_connects_and_streams_state(server):
    seen = []
    link = RobotLink("127.0.0.1", server.port, on_state=seen.append)
    link.start()
    try:
        assert wait_until(lambda: link.state == READY)
        assert wait_until(lambda: len(seen) >= 3)
        assert [j.name for j in seen[-1]] == server.joints
    finally:
        link.stop()


def test_joint_names_come_from_the_state_stream(server):
    """The roster arrives with the state, so it costs no round trip.

    It also must not be requested by the thread that reads the answer, which is
    what an earlier version did — and it timed out every time.
    """
    link = RobotLink("127.0.0.1", server.port)
    link.start()
    try:
        assert wait_until(lambda: link.joint_names == server.joints)
        assert not any(c.get("cmd") == "joints" for c in server.received)
    finally:
        link.stop()


def test_pose_command_is_one_message(server):
    link = RobotLink("127.0.0.1", server.port)
    link.start()
    try:
        assert wait_until(lambda: link.state == READY)
        reply = link.set_positions({"a": 0.1, "b": -0.2})
        assert reply["ok"] is True
        pose = [c for c in server.received if c.get("cmd") == "pose"]
        assert pose and pose[-1]["joints"] == {"a": 0.1, "b": -0.2}
    finally:
        link.stop()


def test_a_refusal_is_an_answer_not_an_exception(server):
    """The robot saying no is information, and the caller has to see it."""
    server.refuse = True
    link = RobotLink("127.0.0.1", server.port)
    link.start()
    try:
        assert wait_until(lambda: link.state == READY)
        reply = link.set_position("a", 0.1)
        assert reply["ok"] is False
        assert "arm the axis first" in reply["error"]
    finally:
        link.stop()


def test_commands_fail_loudly_while_disconnected():
    link = RobotLink("127.0.0.1", 1)  # nothing listens on port 1
    link.start()
    try:
        assert wait_until(lambda: link.state in (CONNECTING, LOST))
        with pytest.raises(OSError):
            link.set_position("a", 0.1)
    finally:
        link.stop(release=False)


def test_reconnects_after_the_server_returns():
    """Every link drops; what matters is that it comes back by itself."""
    server = FakeServer()
    seen = []
    link = RobotLink("127.0.0.1", server.port, on_state=seen.append)
    link.start()
    try:
        assert wait_until(lambda: link.state == READY)

        server.close()
        assert wait_until(lambda: link.state in (LOST, CONNECTING), timeout=5.0)

        server.open()  # same port, as a restarted bringup would be
        try:
            before = len(seen)
            assert wait_until(lambda: link.state == READY, timeout=10.0)
            assert wait_until(lambda: len(seen) > before, timeout=5.0)
            assert link.set_positions({"a": 0.0})["ok"] is True
        finally:
            server.close()
    finally:
        link.stop(release=False)


def test_stop_reports_disconnected(server):
    states = []
    link = RobotLink("127.0.0.1", server.port, on_connection=states.append)
    link.start()
    assert wait_until(lambda: link.state == READY)
    link.stop()
    assert link.state == DISCONNECTED
    assert states[-1] == DISCONNECTED


def test_missing_measurements_stay_missing():
    """None is not zero, and the difference is the whole point of reporting it."""
    state = JointState.from_json({"name": "a", "position_rad": 0.5, "torque_nm": None})
    assert state.position_rad == 0.5
    assert state.torque_nm is None
    assert state.temperature_c is None


def test_backoff_grows_and_stays_bounded():
    backoff = Backoff(initial=0.1, maximum=1.0, factor=2.0)
    delays = [backoff.next_delay() for _ in range(10)]
    assert all(0 < d <= 1.0 for d in delays)
    assert delays[-1] > delays[0]
    backoff.reset()
    assert backoff.next_delay() <= 0.1
