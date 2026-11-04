"""The records, as CycloneDDS types under their ROS 2 names.

Mirrors ``crates/drlabs-core/src/lowlevel.rs`` and the ``.msg`` files in
``tools/ros2/drlabs_msgs``.  The ``typename`` on each struct is the name
``rmw_cyclonedds`` gives the generated ROS 2 type, so a sample written here
is indistinguishable on the wire from one a ROS 2 node publishes.

Angles are radians at the joint output, rates rad/s, torque Nm at the
mechanism.  ``NaN`` in a state means "not reported".
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float32, uint8, uint16, uint32, uint64

MAX_AXES = 64

#: MotorCmd.mode — see docs/RT.md for what each does on each driver.
OFF = 0
DRIVE = 1          # position q, dq and tau as feed-forward
VELOCITY = 2       # run at dq, no position target
TORQUE = 3         # tau only
HOLD = 4           # zero-velocity servo; a position servo holds where it is
STAY_WITHIN = 5    # torque only outside the joint's declared travel
ZERO = 6           # once, on the change into this mode: output position becomes q
MODE_NAMES = {0: "off", 1: "drive", 2: "velocity", 3: "torque", 4: "hold",
              5: "stay_within", 6: "zero"}

#: LowCmd.flags — the CiA 402 verbs: a quick stop latches the robot-wide
#: fault, a fault reset clears it and leaves every axis OFF.
QUICK_STOP = 1 << 0
FAULT_RESET = 1 << 1

#: MotorState.status
STATUS_NAMES = {0: "ok", 1: "silent", 2: "alert", 3: "refused", 4: "off", 5: "tripped"}
#: MotorState.blocked
BLOCKED_NAMES = {
    0: "",
    1: "no telemetry received from the controller",
    2: "bus voltage below the configured minimum",
    3: "bus voltage above the configured maximum",
    4: "controller reports a hard fault",
    5: "output position is invalid",
    6: "outside the safety envelope; move the axis back by hand",
    7: "a fault is latched; reset it first",
}
#: LowState.estop_code
TRIP_NAMES = {
    0: "",
    1: "controller_fault",
    2: "over_voltage",
    3: "under_voltage",
    4: "over_current",
    5: "board_over_temperature",
    6: "motor_over_temperature",
    7: "position_envelope",
    8: "velocity_runaway",
    9: "stall",
    10: "comm_loss",
    11: "deadline",
    12: "quick_stop_requested",
    13: "transport",
}

#: What the runtime and a ROS 2 node both call the topics (without the
#: ``rt/`` DDS prefix a ROS 2 node never sees).
TYPE_LOWCMD = "drlabs_msgs::msg::dds_::LowCmd_"
TYPE_LOWSTATE = "drlabs_msgs::msg::dds_::LowState_"
TYPE_DESCRIBE = "drlabs_msgs::msg::dds_::Describe_"


def topic_name(prefix: str, leaf: str) -> str:
    """The DDS topic name: ROS 2 puts ``rt/`` in front of every topic."""
    return f"rt/{prefix}/{leaf}"


@dataclass
class MotorCmd(IdlStruct, typename="drlabs_msgs::msg::dds_::MotorCmd_"):
    mode: uint8 = OFF
    q: float32 = 0.0
    dq: float32 = 0.0
    kp: float32 = 0.0
    kd: float32 = 0.0
    tau: float32 = 0.0


def _off_entries() -> List[MotorCmd]:
    return [MotorCmd() for _ in range(MAX_AXES)]


@dataclass
class LowCmd(IdlStruct, typename=TYPE_LOWCMD):
    seq: uint32 = 0
    flags: uint32 = 0
    t_ns: uint64 = 0
    n: uint16 = 0
    cmd: array[MotorCmd, 64] = field(default_factory=_off_entries)


@dataclass
class MotorState(IdlStruct, typename="drlabs_msgs::msg::dds_::MotorState_"):
    q: float32 = math.nan
    dq: float32 = math.nan
    tau: float32 = math.nan
    current: float32 = math.nan
    voltage: float32 = math.nan
    temp: float32 = math.nan
    fault: uint16 = 0
    status: uint8 = 1
    blocked: uint8 = 0

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, f"status {self.status}")

    @property
    def blocked_name(self) -> str:
        return BLOCKED_NAMES.get(self.blocked, f"blocked {self.blocked}")

    @property
    def driving(self) -> bool:
        return self.status in (0, 2, 3)

    @staticmethod
    def _opt(value: float) -> Optional[float]:
        return None if math.isnan(value) else value

    @property
    def position(self) -> Optional[float]:
        """``q``, or ``None`` when not reported."""
        return self._opt(self.q)


def _silent_entries() -> List[MotorState]:
    return [MotorState() for _ in range(MAX_AXES)]


@dataclass
class LowState(IdlStruct, typename=TYPE_LOWSTATE):
    seq: uint32 = 0
    cmd_seq: uint32 = 0
    tick: uint64 = 0
    t_ns: uint64 = 0
    period_ns: uint32 = 0
    deadline_miss: uint32 = 0
    estop_code: uint16 = 0
    estop: uint8 = 0
    n: uint16 = 0
    state: array[MotorState, 64] = field(default_factory=_silent_entries)

    @property
    def stopped(self) -> bool:
        return self.estop != 0

    @property
    def estop_name(self) -> str:
        return TRIP_NAMES.get(self.estop_code, f"code {self.estop_code}")


@dataclass
class Describe(IdlStruct, typename=TYPE_DESCRIBE):
    json: str = ""
