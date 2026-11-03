"""The DDS participant, and the things every client needs above it.

The runtime publishes its description once, latched, so a client gets it
without asking; states arrive on Cyclone's own thread and the latest is
kept.  The command record is sticky: ``set`` changes some joints and
``send`` ships the whole record, so a joint not mentioned keeps what it was
last given.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from cyclonedds.core import Listener, Policy, Qos
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from . import motion
from .types import (
    DRIVE,
    FAULT_RESET,
    OFF,
    QUICK_STOP,
    Describe,
    LowCmd,
    LowState,
    MotorCmd,
    MotorState,
    topic_name,
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


#: Latest wins, as the runtime's own QoS says.
BEST_EFFORT = Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile,
                  Policy.History.KeepLast(1))
#: The description: written once, kept for whoever turns up later.
LATCHED = Qos(Policy.Reliability.Reliable(duration(seconds=10)),
              Policy.Durability.TransientLocal, Policy.History.KeepLast(1))


class Client:
    """A CycloneDDS client of ``drlabs rt``.

    ``domain`` and ``prefix`` are the runtime's ``api.dds`` settings (0 and
    ``drlabs`` by default): the topics are ``rt/<prefix>/lowcmd``,
    ``…/lowstate`` and ``…/describe``.
    """

    def __init__(self, domain: int = 0, prefix: str = "drlabs",
                 timeout: float = 2.0) -> None:
        self.domain = domain
        self.prefix = prefix
        self._timeout = timeout
        self._lock = threading.Lock()
        self._cmd = LowCmd()
        self._seq = 0
        self._latest: Optional[LowState] = None
        self._latest_at = 0.0
        self._description: Optional[Description] = None
        self._described = threading.Event()

        self._participant = DomainParticipant(domain)
        self._cmd_topic = Topic(self._participant, topic_name(prefix, "lowcmd"), LowCmd)
        self._state_topic = Topic(self._participant, topic_name(prefix, "lowstate"), LowState)
        self._describe_topic = Topic(self._participant, topic_name(prefix, "describe"), Describe)
        self._writer = DataWriter(self._participant, self._cmd_topic, qos=BEST_EFFORT)
        self._state_reader = DataReader(
            self._participant, self._state_topic, qos=BEST_EFFORT,
            listener=Listener(on_data_available=self._on_state))
        self._describe_reader = DataReader(
            self._participant, self._describe_topic, qos=LATCHED,
            listener=Listener(on_data_available=self._on_describe))

    # --- lifecycle --------------------------------------------------------

    def close(self) -> None:
        for entity in (self._state_reader, self._describe_reader, self._writer):
            try:
                del entity
            except Exception:  # noqa: BLE001 — closing is best effort
                pass
        self._participant = None  # type: ignore[assignment]

    def __enter__(self) -> "Client":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # --- description ------------------------------------------------------

    def describe(self, timeout: Optional[float] = None) -> Description:
        """The machine's description.  Latched by the runtime, so it arrives
        on its own; this only waits for it."""
        if self._description is not None:
            return self._description
        if self._described.wait(timeout or self._timeout) and self._description is not None:
            return self._description
        raise TimeoutError(f"no description on domain {self.domain}, rt/{self.prefix}: "
                           "is `drlabs rt` running?")

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
            time.sleep(0.002)
        raise TimeoutError("no fresh state from the runtime")

    def wait_state(self, timeout: Optional[float] = None) -> LowState:
        deadline = time.monotonic() + (timeout or self._timeout)
        while time.monotonic() < deadline:
            if self._latest is not None:
                return self._latest
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
            self._cmd.n = max(self._cmd.n, len(description.joints))
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
            sample = LowCmd(seq=self._cmd.seq, flags=flags, t_ns=self._cmd.t_ns,
                            n=self._cmd.n, cmd=[MotorCmd(**vars(e)) for e in self._cmd.cmd])
        self._writer.write(sample)

    def command(self, targets: Dict[str, float], **kw) -> None:
        """``set`` then ``send``."""
        self.set(targets, **kw)
        self.send()

    def release(self, names: Optional[Iterable[str]] = None) -> None:
        """Turns joints off (torque off).  All of them by default."""
        description = self.describe()
        names = list(names) if names is not None else description.names
        with self._lock:
            self._cmd.n = max(self._cmd.n, len(description.joints))
            for name in names:
                self._cmd.cmd[description.index(name)].mode = OFF
        self.send()

    def hold(self, names: Optional[Iterable[str]] = None) -> None:
        """Drives joints to where they are now."""
        current = self.positions()
        names = list(names) if names is not None else list(current)
        self.command({name: current[name] for name in names
                      if current[name] == current[name]})  # not NaN

    def quick_stop(self) -> None:
        """Latches the robot-wide fault (CiA 402 quick stop)."""
        self.send(flags=QUICK_STOP)

    def fault_reset(self) -> None:
        """Clears a latched fault.  Every joint is off afterwards."""
        with self._lock:
            for entry in self._cmd.cmd:
                entry.mode = OFF
        self.send(flags=FAULT_RESET)

    # --- motion -----------------------------------------------------------

    def move_to(self, targets: Dict[str, float], max_velocity: float = 0.35,
                accel: float = 1.0, rate_hz: float = 200.0,
                settle_tolerance: float = 0.03, settle_timeout: float = 5.0) -> None:
        """A profiled move: streams trapezoidal setpoints until every joint
        arrives.  Raises on a latched fault."""
        motion.move_to(self, targets, max_velocity, accel, rate_hz,
                       settle_tolerance, settle_timeout)

    def move_to_pose(self, name: str, **kw) -> None:
        self.move_to(self.describe().poses[name], **kw)

    # --- internals (Cyclone's thread) -------------------------------------

    def _on_state(self, reader: DataReader) -> None:
        for sample in reader.take(1):
            if isinstance(sample, LowState):
                self._latest = sample
                self._latest_at = time.monotonic()

    def _on_describe(self, reader: DataReader) -> None:
        for sample in reader.take(1):
            if isinstance(sample, Describe):
                try:
                    self._description = Description.from_json(sample.json)
                    self._described.set()
                except (ValueError, KeyError, TypeError):
                    continue
