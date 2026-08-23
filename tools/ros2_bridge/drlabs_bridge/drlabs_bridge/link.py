"""The link to a running ``drlabs bringup`` server.

This module deliberately knows nothing about ROS.  It is the half of the bridge
that can be tested on a host with a real robot server and no ROS installation,
which is most of the behaviour worth testing: the wire format, the command
mapping, and what happens when the link drops.

The protocol is newline-delimited JSON over TCP — one object per line out, one
reply per line back, plus unsolicited state lines.  See ``docs/SERVO.md``.

Every link drops eventually, so the drop is modelled rather than treated as an
error: the connection is a state machine, reconnection backs off with jitter,
and nothing above this module is told the axis is live unless a state line said
so.
"""

from __future__ import annotations

import json
import random
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

#: Connection states, in the order they are normally visited.
DISCONNECTED = "disconnected"
CONNECTING = "connecting"
READY = "ready"
LOST = "lost"


@dataclass
class JointState:
    """One joint as the server reports it.

    A measurement the driver did not read is ``None``, never zero.  A caller
    that reads a missing temperature as 0 °C draws the wrong conclusion from
    it, and a missing torque is not zero torque.
    """

    name: str
    armed: bool = False
    estop: bool = False
    mode: str = ""
    position_rad: Optional[float] = None
    velocity_rad_s: Optional[float] = None
    torque_nm: Optional[float] = None
    current_a: Optional[float] = None
    voltage_v: Optional[float] = None
    temperature_c: Optional[float] = None
    target_rad: Optional[float] = None
    fault: int = 0
    fault_name: str = ""
    rate_hz: Optional[float] = None

    @classmethod
    def from_json(cls, payload: dict) -> "JointState":
        return cls(
            name=payload.get("name", ""),
            armed=bool(payload.get("armed", False)),
            estop=bool(payload.get("estop", False)),
            mode=payload.get("mode", ""),
            position_rad=payload.get("position_rad"),
            velocity_rad_s=payload.get("velocity_rad_s"),
            torque_nm=payload.get("torque_nm"),
            current_a=payload.get("current_a"),
            voltage_v=payload.get("voltage_v"),
            temperature_c=payload.get("temperature_c"),
            target_rad=payload.get("target_rad"),
            fault=int(payload.get("fault", 0) or 0),
            fault_name=payload.get("fault_name", ""),
            rate_hz=payload.get("rate_hz"),
        )


@dataclass
class Backoff:
    """Exponential backoff with jitter.

    The jitter matters when a bringup process restarts underneath a fleet of
    clients: without it they all retry on the same schedule and arrive
    together, which is the one moment the server can least afford it.
    """

    initial: float = 0.25
    maximum: float = 5.0
    factor: float = 2.0
    _current: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self._current = 0.0

    def next_delay(self) -> float:
        self._current = self.initial if self._current == 0.0 else min(
            self._current * self.factor, self.maximum)
        return self._current * (0.5 + random.random() * 0.5)


class RobotLink:
    """A supervised connection to one ``drlabs bringup`` server.

    ``on_state`` is called with a list of :class:`JointState` for every state
    line, and ``on_connection`` with one of the state constants whenever the
    link changes.  Both run on the reader thread, so they must not block.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7777,
        on_state: Optional[Callable[[list], None]] = None,
        on_connection: Optional[Callable[[str], None]] = None,
        connect_timeout: float = 2.0,
        reply_timeout: float = 2.0,
    ) -> None:
        self.host = host
        self.port = port
        self._on_state = on_state
        self._on_connection = on_connection
        self._connect_timeout = connect_timeout
        self._reply_timeout = reply_timeout

        self._sock: Optional[socket.socket] = None
        self._stream = None
        # _lock guards the socket itself; _send_lock serialises whole
        # command/reply exchanges so a reply cannot be matched to the wrong
        # command.  They are separate so that closing the socket never has to
        # wait out a reply that is not coming.
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._backoff = Backoff()

        self._state = DISCONNECTED
        self._joint_names: list = []
        self._replies: "list[dict]" = []
        self._reply_ready = threading.Condition()

    # --- lifecycle --------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def joint_names(self) -> list:
        """The joints the server reported at connect, in its own order."""
        return list(self._joint_names)

    def start(self) -> None:
        """Begin connecting, and keep the connection up until :meth:`stop`."""
        if self._running.is_set():
            return
        self._running.set()
        self._reader = threading.Thread(
            target=self._supervise, name="drlabs-link", daemon=True)
        self._reader.start()

    def stop(self, release: bool = True) -> None:
        """Close the link.

        ``release`` sends a stop to every joint first.  It is best effort: a
        link that has already dropped cannot be told anything, and the
        controller's own watchdog is what covers that case.
        """
        self._running.clear()
        if release:
            try:
                self.stop_all()
            except OSError:
                pass
        self._close_socket()
        reader, self._reader = self._reader, None
        if reader and reader is not threading.current_thread():
            reader.join(timeout=2.0)
        self._set_state(DISCONNECTED)

    # --- commands ---------------------------------------------------------
    #
    # Every position command carries an absolute target, so a resend is
    # harmless: the axis is asked to be somewhere, not to move by something.
    # That is what makes retrying safe on a link that may have dropped
    # mid-write.

    def set_positions(self, targets: dict) -> dict:
        """Command a whole pose at once: ``{joint: radians}``."""
        return self.request({"cmd": "pose", "joints": targets})

    def set_position(self, joint: str, radians: float) -> dict:
        return self.request({"cmd": "position", "joint": joint, "rad": radians})

    def set_velocity(self, joint: str, rad_per_s: float) -> dict:
        return self.request({"cmd": "velocity", "joint": joint, "rad_s": rad_per_s})

    def move(self, joint: str, radians: float,
             max_velocity: Optional[float] = None,
             accel: Optional[float] = None) -> dict:
        command = {"cmd": "move", "joint": joint, "rad": radians}
        if max_velocity is not None:
            command["max_velocity"] = max_velocity
        if accel is not None:
            command["accel"] = accel
        return self.request(command)

    def hold(self, joint: str) -> dict:
        return self.request({"cmd": "hold", "joint": joint})

    def stop_joint(self, joint: str) -> dict:
        return self.request({"cmd": "stop", "joint": joint})

    def stop_all(self) -> None:
        for name in self._joint_names:
            try:
                self.stop_joint(name)
            except (OSError, TimeoutError):
                return

    def estop(self, reason: str = "") -> dict:
        return self.request({"cmd": "estop", "reason": reason})

    def reset_estop(self) -> dict:
        return self.request({"cmd": "reset"})

    def ping(self) -> dict:
        return self.request({"cmd": "ping"})

    def read_joint_names(self) -> list:
        reply = self.request({"cmd": "joints"})
        if reply.get("ok") and isinstance(reply.get("value"), list):
            self._joint_names = list(reply["value"])
        return self.joint_names

    def request(self, command: dict) -> dict:
        """Send one command and wait for its reply.

        Replies are matched by order, which the protocol guarantees: one reply
        per command line, in the order they were sent.  Sending is serialised
        so that ordering cannot be broken by two callers at once.
        """
        line = (json.dumps(command) + "\n").encode()
        with self._send_lock:
            with self._lock:
                stream = self._stream
                if stream is None:
                    raise OSError("not connected")
            with self._reply_ready:
                self._replies.clear()
            stream.write(line)
            stream.flush()

            deadline = time.monotonic() + self._reply_timeout
            with self._reply_ready:
                while not self._replies:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"no reply to {command.get('cmd')!r}")
                    self._reply_ready.wait(remaining)
                return self._replies.pop(0)

    # --- the supervised connection ---------------------------------------

    def _supervise(self) -> None:
        while self._running.is_set():
            self._set_state(CONNECTING)
            try:
                self._connect()
            except OSError:
                self._set_state(LOST)
                self._sleep(self._backoff.next_delay())
                continue

            self._backoff.reset()
            self._set_state(READY)

            try:
                self._read_lines()
            except OSError:
                pass
            finally:
                self._close_socket()

            if self._running.is_set():
                self._set_state(LOST)
                self._sleep(self._backoff.next_delay())

    def _connect(self) -> None:
        sock = socket.create_connection(
            (self.host, self.port), timeout=self._connect_timeout)
        sock.settimeout(None)
        # Control commands are small and latency matters more than packing.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._lock:
            self._sock = sock
            self._stream = sock.makefile("rwb")

    def _read_lines(self) -> None:
        stream = self._stream
        if stream is None:
            return
        for raw in stream:
            if not self._running.is_set():
                return
            line = raw.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except ValueError:
                continue
            self._dispatch(message)

    def _dispatch(self, message: dict) -> None:
        kind = message.get("type")
        if kind == "state":
            joints = [JointState.from_json(j) for j in message.get("joints", [])]
            if not joints:
                return
            # The state line carries the roster, so the names need no round
            # trip — and asking for them from the supervisor would be asking
            # the thread that reads the answer to wait for it.
            names = [j.name for j in joints]
            if names != self._joint_names:
                self._joint_names = names
            if self._on_state:
                self._on_state(joints)
            return
        if kind == "reply":
            with self._reply_ready:
                self._replies.append(message)
                self._reply_ready.notify_all()

    def _close_socket(self) -> None:
        """Drop the connection from whichever thread noticed it has to go.

        Only the socket is closed here, deliberately.  ``makefile`` returns a
        buffered reader that holds its own lock for as long as it is waiting
        for bytes, so closing *it* from another thread blocks behind the reader
        thread sitting inside it — forever, since the reader is waiting for
        data that is not coming.  Shutting the socket down is what makes that
        read return; the file object is then just garbage, and the reader
        thread lets go of it on its way out.
        """
        with self._lock:
            self._stream = None
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        # Nobody is going to answer an outstanding request now.
        with self._reply_ready:
            self._reply_ready.notify_all()

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        if self._on_connection:
            self._on_connection(state)

    def _sleep(self, seconds: float) -> None:
        """Sleep, but wake immediately if the link is being shut down."""
        deadline = time.monotonic() + seconds
        while self._running.is_set() and time.monotonic() < deadline:
            time.sleep(min(0.05, deadline - time.monotonic()))
