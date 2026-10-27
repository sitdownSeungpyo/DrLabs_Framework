"""The socket, and the things every client needs above it.

The runtime sends a state to whoever has spoken in the last two seconds, so a
client that only watches still has to say something now and then; ``Client``
does that with a DESCRIBE when it has nothing else to send.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from . import motion
from .codec import (
    DESCRIBE_REPLY,
    DRIVE,
    ESTOP,
    OFF,
    RESET_ESTOP,
    STATE,
    LowCmd,
    LowState,
    MotorCmd,
    MotorState,
    decode,
    encode_cmd,
    encode_describe,
)


@dataclass
class Joint:
    index: int
    name: str
    bus: str
    driver: str
    id: int
    min_rad: float
    max_rad: float
    max_torque_nm: float
    max_velocity_rad_s: float
    max_accel_rad_s2: float


@dataclass
class Description:
    protocol: int
    robot: str
    rate_hz: int
    period_ns: int
    cmd_timeout_ms: int
    on_cmd_timeout: str
    joints: List[Joint]
    poses: Dict[str, Dict[str, float]]

    @classmethod
    def from_json(cls, text: str) -> "Description":
        raw = json.loads(text)
        joints = [Joint(**j) for j in raw["joints"]]
        return cls(
            protocol=int(raw["protocol"]),
            robot=raw.get("robot", ""),
            rate_hz=int(raw["rate_hz"]),
            period_ns=int(raw["period_ns"]),
            cmd_timeout_ms=int(raw.get("cmd_timeout_ms", 100)),
            on_cmd_timeout=raw.get("on_cmd_timeout", "hold"),
            joints=joints,
            poses=raw.get("poses", {}),
        )

    def index(self, name: str) -> int:
        for joint in self.joints:
            if joint.name == name:
                return joint.index
        raise KeyError(f"no joint named {name!r}")

    @property
    def names(self) -> List[str]:
        return [joint.name for joint in self.joints]


class Client:
    """A UDP client of ``drlabs rt``.

    States arrive on a reader thread and the latest is kept; commands go out
    when a method is called.  The command record is sticky: ``set`` changes
    some joints and ``send`` ships the whole record, so a joint not mentioned
    keeps what it was last given.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7700,
                 timeout: float = 2.0) -> None:
        self.address = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.2)
        self._timeout = timeout
        self._lock = threading.Lock()
        self._cmd = LowCmd()
        self._seq = 0
        self._latest: Optional[LowState] = None
        self._latest_at = 0.0
        self._description: Optional[Description] = None
        self._described = threading.Event()
        self._running = threading.Event()
        self._running.set()
        self._reader = threading.Thread(target=self._read, name="drlabs-client", daemon=True)
        self._reader.start()

    # --- lifecycle --------------------------------------------------------

    def close(self) -> None:
        self._running.clear()
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self) -> "Client":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # --- description ------------------------------------------------------

    def describe(self, timeout: Optional[float] = None) -> Description:
        """Asks the runtime what it is driving.  Cached after the first answer."""
        if self._description is not None:
            return self._description
        deadline = time.monotonic() + (timeout or self._timeout)
        while time.monotonic() < deadline:
            self._sock.sendto(encode_describe(), self.address)
            if self._described.wait(0.25) and self._description is not None:
                return self._description
        raise TimeoutError(f"no description from {self.address[0]}:{self.address[1]}")

    @property
    def description(self) -> Description:
        return self.describe()

    # --- state ------------------------------------------------------------

    def state(self, max_age: Optional[float] = None) -> LowState:
        """The latest state.  With ``max_age``, waits for one at most that old."""
        if max_age is None:
            if self._latest is None:
                return self.wait_state()
            return self._latest
        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            if self._latest is not None and time.monotonic() - self._latest_at <= max_age:
                return self._latest
            self._keepalive()
            time.sleep(0.002)
        raise TimeoutError("no fresh state from the runtime")

    def wait_state(self, timeout: Optional[float] = None) -> LowState:
        deadline = time.monotonic() + (timeout or self._timeout)
        while time.monotonic() < deadline:
            if self._latest is not None:
                return self._latest
            self._keepalive()
            time.sleep(0.02)
        raise TimeoutError("no state from the runtime")

    def joint_state(self, name: str) -> MotorState:
        state = self.state()
        index = self.describe().index(name)
        if index >= state.n:
            raise KeyError(f"the runtime reports {state.n} joints, {name!r} is index {index}")
        return state.state[index]

    def positions(self) -> Dict[str, float]:
        """Every joint's position by name, ``NaN`` where not reported."""
        state = self.state()
        return {joint.name: state.state[joint.index].q
                for joint in self.describe().joints if joint.index < state.n}

    # --- commands ---------------------------------------------------------

    def set(self, targets: Dict[str, float], mode: int = DRIVE,
            dq: float = 0.0, tau: float = 0.0) -> None:
        """Changes some joints' entries in the command record without sending it."""
        description = self.describe()
        with self._lock:
            self._ensure_entries(len(description.joints))
            for name, q in targets.items():
                entry = self._cmd.cmd[description.index(name)]
                entry.mode = mode
                entry.q = float(q)
                entry.dq = float(dq)
                entry.tau = float(tau)

    def send(self, flags: int = 0) -> None:
        """Ships the command record as it stands."""
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFFFFFF or 1
            self._cmd.seq = self._seq
            self._cmd.flags = flags
            self._cmd.t_ns = time.monotonic_ns()
            data = encode_cmd(self._cmd)
        self._sock.sendto(data, self.address)

    def command(self, targets: Dict[str, float], **kw) -> None:
        """``set`` then ``send``."""
        self.set(targets, **kw)
        self.send()

    def release(self, names: Optional[Iterable[str]] = None) -> None:
        """Turns joints off (torque off).  All of them by default."""
        description = self.describe()
        names = list(names) if names is not None else description.names
        with self._lock:
            self._ensure_entries(len(description.joints))
            for name in names:
                self._cmd.cmd[description.index(name)].mode = OFF
        self.send()

    def hold(self, names: Optional[Iterable[str]] = None) -> None:
        """Drives joints to where they are now."""
        current = self.positions()
        names = list(names) if names is not None else list(current)
        self.command({name: current[name] for name in names
                      if current[name] == current[name]})  # not NaN

    def estop(self) -> None:
        self.send(flags=ESTOP)

    def reset_estop(self) -> None:
        """Clears a latched stop.  Every joint is off afterwards."""
        with self._lock:
            for entry in self._cmd.cmd:
                entry.mode = OFF
        self.send(flags=RESET_ESTOP)

    # --- motion -----------------------------------------------------------

    def move_to(self, targets: Dict[str, float], max_velocity: float = 0.35,
                accel: float = 1.0, rate_hz: float = 200.0,
                settle_tolerance: float = 0.03, settle_timeout: float = 5.0) -> None:
        """A profiled move: streams trapezoidal setpoints until every joint
        arrives.  Raises on an e-stop."""
        motion.move_to(self, targets, max_velocity, accel, rate_hz,
                       settle_tolerance, settle_timeout)

    def move_to_pose(self, name: str, **kw) -> None:
        self.move_to(self.describe().poses[name], **kw)

    # --- internals --------------------------------------------------------

    def _ensure_entries(self, n: int) -> None:
        while len(self._cmd.cmd) < n:
            self._cmd.cmd.append(MotorCmd())

    def _keepalive(self) -> None:
        """Say something so the runtime keeps sending states."""
        self._sock.sendto(encode_describe(), self.address)

    def _read(self) -> None:
        # A description for sixty-four joints is well over 4 kB; a datagram
        # larger than the buffer is an error on Windows, not a truncation.
        buf = bytearray(65536)
        while self._running.is_set():
            try:
                n, _ = self._sock.recvfrom_into(buf)
            except socket.timeout:
                continue
            except OSError:
                # Windows reports an ICMP "port unreachable" from an earlier
                # send as an error on the next receive. The socket is fine.
                if not self._running.is_set():
                    return
                time.sleep(0.01)
                continue
            try:
                kind, payload = decode(bytes(buf[:n]))
            except ValueError:
                continue
            if kind == STATE:
                self._latest = payload
                self._latest_at = time.monotonic()
            elif kind == DESCRIBE_REPLY:
                try:
                    self._description = Description.from_json(payload)
                    self._described.set()
                except (ValueError, KeyError, TypeError):
                    continue
