"""The link to a running ``drlabs rt`` runtime.

This module deliberately knows nothing about ROS.  It is the half of the bridge
that can be tested on a host with a real runtime and no ROS installation.

The wire is the runtime's own: ``LowCmd`` in, ``LowState`` out, as UDP frames
(``tools/drlabs_client``).  The runtime publishes a state to whoever has
spoken recently, so "connected" here means "a state arrived lately"; the link
keeps speaking so they keep coming, and reports the loss when they stop.

What the old JSON-lines server did for a client — profiled moves, holds —
the runtime does not.  Those are done here, in the client, which is where the
runtime's design puts them.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

# ``tools/drlabs_client`` is the reference codec; the bridge rides on it
# rather than carrying a second copy of the frame layout.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (
    os.path.join(_HERE, "..", "..", "..", "drlabs_client"),
    os.path.join(_HERE, "..", "..", "drlabs_client"),
):
    if os.path.isdir(os.path.join(_candidate, "drlabs_client")):
        sys.path.insert(0, os.path.abspath(_candidate))
        break

from drlabs_client import Client, DRIVE, OFF, Trapezoidal  # noqa: E402
from drlabs_client.codec import STATUS_NAMES, TRIP_NAMES  # noqa: E402

#: Connection states, in the order they are normally visited.
DISCONNECTED = "disconnected"
CONNECTING = "connecting"
READY = "ready"
LOST = "lost"

#: A state older than this means the runtime has gone quiet.
STALE_AFTER = 0.5


@dataclass
class JointState:
    """One joint as the runtime reports it.

    A measurement the driver did not read is ``None``, never zero.
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


def _opt(value: float) -> Optional[float]:
    return None if math.isnan(value) else float(value)


class RobotLink:
    """A supervised connection to one ``drlabs rt`` runtime.

    ``on_state`` is called with a list of :class:`JointState` whenever a fresh
    state is seen (at most ``poll_hz`` times a second), and ``on_connection``
    with one of the state constants whenever the link changes.  Both run on
    the link's thread, so they must not block.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 7700,
        on_state: Optional[Callable[[list], None]] = None,
        on_connection: Optional[Callable[[str], None]] = None,
        connect_timeout: float = 2.0,
        reply_timeout: float = 2.0,
        poll_hz: float = 100.0,
    ) -> None:
        self.host = host
        self.port = port
        self._on_state = on_state
        self._on_connection = on_connection
        self._connect_timeout = connect_timeout
        self._reply_timeout = reply_timeout
        self._poll = 1.0 / max(poll_hz, 1.0)

        self._client: Optional[Client] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._state = DISCONNECTED
        self._joint_names: List[str] = []
        self._targets: Dict[str, float] = {}
        self._moves: Dict[str, threading.Thread] = {}
        self._lock = threading.Lock()

    # --- lifecycle --------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def joint_names(self) -> List[str]:
        return list(self._joint_names)

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._supervise, name="drlabs-link", daemon=True)
        self._thread.start()

    def stop(self, release: bool = True) -> None:
        """Close the link.  ``release`` turns every joint off first, best effort."""
        self._running.clear()
        if release:
            try:
                self.stop_all()
            except (OSError, TimeoutError, RuntimeError):
                pass
        thread, self._thread = self._thread, None
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        client, self._client = self._client, None
        if client is not None:
            client.close()
        self._set_state(DISCONNECTED)

    # --- commands ---------------------------------------------------------
    #
    # Every position command carries an absolute target, so a resend is
    # harmless: the axis is asked to be somewhere, not to move by something.

    def _ready(self) -> Client:
        client = self._client
        if client is None or self._state != READY:
            raise OSError("not connected")
        return client

    def set_positions(self, targets: dict) -> dict:
        """Command a whole pose at once: ``{joint: radians}``."""
        client = self._ready()
        for name in targets:
            self._cancel_move(name)
        client.command({name: float(rad) for name, rad in targets.items()})
        with self._lock:
            self._targets.update({name: float(rad) for name, rad in targets.items()})
        return {"ok": True}

    def set_position(self, joint: str, radians: float) -> dict:
        return self.set_positions({joint: radians})

    def set_velocity(self, joint: str, rad_per_s: float) -> dict:
        """Not available: the runtime drives position servos in this version."""
        return {"ok": False, "error": "velocity control is not available on the runtime"}

    def move(self, joint: str, radians: float,
             max_velocity: Optional[float] = None,
             accel: Optional[float] = None) -> dict:
        """Starts a profiled move and returns at once, as the old server did."""
        client = self._ready()
        description = client.describe()
        info = description.joints[description.index(joint)]
        start = client.positions().get(joint)
        if start is None or math.isnan(start):
            return {"ok": False, "error": f"{joint!r} reports no position"}
        target = min(max(float(radians), info.min_rad), info.max_rad)
        velocity = min(max_velocity or 0.35, info.max_velocity_rad_s)
        acceleration = accel or 1.0
        if info.max_accel_rad_s2 > 0:
            acceleration = min(acceleration, info.max_accel_rad_s2)
        profile = Trapezoidal(start, target, velocity, acceleration)
        self._cancel_move(joint)

        def run() -> None:
            started = time.monotonic()
            while self._running.is_set():
                t = time.monotonic() - started
                q, _ = profile.sample(t)
                try:
                    client.command({joint: q})
                except OSError:
                    return
                with self._lock:
                    self._targets[joint] = q
                    if self._moves.get(joint) is not threading.current_thread():
                        return
                if t >= profile.total_time:
                    return
                time.sleep(0.01)

        thread = threading.Thread(target=run, name=f"drlabs-move-{joint}", daemon=True)
        with self._lock:
            self._moves[joint] = thread
        thread.start()
        return {"ok": True}

    def _cancel_move(self, joint: str) -> None:
        with self._lock:
            self._moves.pop(joint, None)

    def hold(self, joint: str) -> dict:
        client = self._ready()
        self._cancel_move(joint)
        client.hold([joint])
        return {"ok": True}

    def stop_joint(self, joint: str) -> dict:
        client = self._ready()
        self._cancel_move(joint)
        client.release([joint])
        with self._lock:
            self._targets.pop(joint, None)
        return {"ok": True}

    def stop_all(self) -> None:
        client = self._client
        if client is None:
            return
        with self._lock:
            self._moves.clear()
            self._targets.clear()
        client.release()

    def estop(self, reason: str = "") -> dict:
        client = self._ready()
        with self._lock:
            self._moves.clear()
        client.estop()
        return {"ok": True}

    def reset_estop(self) -> dict:
        client = self._ready()
        client.reset_estop()
        with self._lock:
            self._targets.clear()
        return {"ok": True}

    def ping(self) -> dict:
        client = self._ready()
        state = client.state()
        return {"ok": True, "value": state.tick}

    def read_joint_names(self) -> list:
        client = self._ready()
        self._joint_names = client.describe().names
        return self.joint_names

    # --- the supervised connection ---------------------------------------

    def _supervise(self) -> None:
        while self._running.is_set():
            self._set_state(CONNECTING)
            try:
                client = Client(self.host, self.port, timeout=self._connect_timeout)
                description = client.describe(self._connect_timeout)
                client.wait_state(self._connect_timeout)
            except (OSError, TimeoutError):
                self._set_state(LOST)
                self._sleep(1.0)
                continue

            self._client = client
            self._joint_names = description.names
            self._set_state(READY)
            last_seq = -1
            while self._running.is_set():
                time.sleep(self._poll)
                try:
                    state = client.state(max_age=STALE_AFTER)
                except TimeoutError:
                    break
                if state.seq == last_seq:
                    continue
                last_seq = state.seq
                if self._on_state:
                    self._on_state(self._joints(description, state))

            client.close()
            if self._client is client:
                self._client = None
            if self._running.is_set():
                self._set_state(LOST)
                self._sleep(0.5)

    def _joints(self, description, state) -> List[JointState]:
        with self._lock:
            targets = dict(self._targets)
        out = []
        estop = state.stopped
        reason = TRIP_NAMES.get(state.estop_code, "")
        for joint in description.joints:
            if joint.index >= state.n:
                break
            m = state.state[joint.index]
            out.append(JointState(
                name=joint.name,
                armed=m.driving,
                estop=estop,
                mode=STATUS_NAMES.get(m.status, str(m.status)),
                position_rad=_opt(m.q),
                velocity_rad_s=_opt(m.dq),
                torque_nm=_opt(m.tau),
                current_a=_opt(m.current),
                voltage_v=_opt(m.voltage),
                temperature_c=_opt(m.temp),
                target_rad=targets.get(joint.name, _opt(m.q)),
                fault=int(m.fault),
                fault_name=(reason if estop else (f"fault 0x{m.fault:x}" if m.fault else "")),
                rate_hz=float(description.rate_hz),
            ))
        return out

    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        if self._on_connection:
            self._on_connection(state)

    def _sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while self._running.is_set() and time.monotonic() < deadline:
            time.sleep(min(0.05, deadline - time.monotonic()))
