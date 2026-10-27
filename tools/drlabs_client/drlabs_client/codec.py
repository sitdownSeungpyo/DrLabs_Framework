"""The frame layout, byte for byte.

Mirrors ``crates/drlabs-api/src/frame.rs``; the test vectors in
``tests/test_codec.py`` are the same bytes the Rust tests pin.

    0   "DLRT"          magic
    4   u16 version
    6   u16 kind        1 CMD, 2 STATE, 3 DESCRIBE, 4 DESCRIBE_REPLY
    8   payload
    -4  u32 crc32       IEEE (zlib.crc32), over everything before it

Little-endian.  Only the ``n`` entries in use travel.  ``NaN`` in a state
means "not reported".
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, field
from typing import List, Optional

MAGIC = b"DLRT"
PROTOCOL_VERSION = 1
MAX_AXES = 64

CMD = 1
STATE = 2
DESCRIBE = 3
DESCRIBE_REPLY = 4

#: MotorCmd.mode
OFF = 0
DRIVE = 1

#: LowCmd.flags
ESTOP = 1 << 0
RESET_ESTOP = 1 << 1

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
    7: "emergency stop is latched",
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
    12: "estop_requested",
    13: "transport",
}

_HEAD = struct.Struct("<4sHH")
_CMD_HEADER = struct.Struct("<IIQH6x")
_MOTOR_CMD = struct.Struct("<B3xfffff")
_STATE_HEADER = struct.Struct("<IIQQIIHBBH2x")
_MOTOR_STATE = struct.Struct("<ffffffHBB")

assert _CMD_HEADER.size == 24 and _MOTOR_CMD.size == 24
assert _STATE_HEADER.size == 40 and _MOTOR_STATE.size == 28


class FrameError(ValueError):
    """A frame that could not be read, and why."""


@dataclass
class MotorCmd:
    mode: int = OFF
    q: float = 0.0
    dq: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0


@dataclass
class LowCmd:
    seq: int = 0
    flags: int = 0
    t_ns: int = 0
    cmd: List[MotorCmd] = field(default_factory=list)

    @property
    def n(self) -> int:
        return len(self.cmd)


@dataclass
class MotorState:
    q: float = math.nan
    dq: float = math.nan
    tau: float = math.nan
    current: float = math.nan
    voltage: float = math.nan
    temp: float = math.nan
    fault: int = 0
    status: int = 1
    blocked: int = 0

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


@dataclass
class LowState:
    seq: int = 0
    cmd_seq: int = 0
    tick: int = 0
    t_ns: int = 0
    period_ns: int = 0
    deadline_miss: int = 0
    estop_code: int = 0
    estop: int = 0
    state: List[MotorState] = field(default_factory=list)

    @property
    def n(self) -> int:
        return len(self.state)

    @property
    def stopped(self) -> bool:
        return self.estop != 0

    @property
    def estop_name(self) -> str:
        return TRIP_NAMES.get(self.estop_code, f"code {self.estop_code}")


def _close(body: bytearray) -> bytes:
    body += struct.pack("<I", zlib.crc32(bytes(body)) & 0xFFFFFFFF)
    return bytes(body)


def encode_cmd(cmd: LowCmd) -> bytes:
    if cmd.n > MAX_AXES:
        raise FrameError(f"{cmd.n} entries, at most {MAX_AXES}")
    body = bytearray(_HEAD.pack(MAGIC, PROTOCOL_VERSION, CMD))
    body += _CMD_HEADER.pack(cmd.seq & 0xFFFFFFFF, cmd.flags, cmd.t_ns, cmd.n)
    for motor in cmd.cmd:
        body += _MOTOR_CMD.pack(motor.mode, motor.q, motor.dq, motor.kp, motor.kd, motor.tau)
    return _close(body)


def encode_state(state: LowState) -> bytes:
    if state.n > MAX_AXES:
        raise FrameError(f"{state.n} entries, at most {MAX_AXES}")
    body = bytearray(_HEAD.pack(MAGIC, PROTOCOL_VERSION, STATE))
    body += _STATE_HEADER.pack(
        state.seq, state.cmd_seq, state.tick, state.t_ns, state.period_ns,
        state.deadline_miss, state.estop_code, state.estop, 0, state.n)
    for motor in state.state:
        body += _MOTOR_STATE.pack(
            motor.q, motor.dq, motor.tau, motor.current, motor.voltage, motor.temp,
            motor.fault, motor.status, motor.blocked)
    return _close(body)


def encode_describe() -> bytes:
    return _close(bytearray(_HEAD.pack(MAGIC, PROTOCOL_VERSION, DESCRIBE)))


def encode_describe_reply(json_text: str) -> bytes:
    body = bytearray(_HEAD.pack(MAGIC, PROTOCOL_VERSION, DESCRIBE_REPLY))
    body += json_text.encode("utf-8")
    return _close(body)


def decode(data: bytes):
    """Returns ``(kind, payload)``: ``(CMD, LowCmd)``, ``(STATE, LowState)``,
    ``(DESCRIBE, None)`` or ``(DESCRIBE_REPLY, str)``."""
    if len(data) < _HEAD.size + 4:
        raise FrameError("frame too short")
    magic, version, kind = _HEAD.unpack_from(data, 0)
    if magic != MAGIC:
        raise FrameError("not a DrLabs frame")
    body, crc = data[:-4], struct.unpack("<I", data[-4:])[0]
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise FrameError("CRC mismatch")
    if version != PROTOCOL_VERSION:
        raise FrameError(f"protocol version {version}, this client speaks {PROTOCOL_VERSION}")
    at = _HEAD.size
    try:
        if kind == CMD:
            seq, flags, t_ns, n = _CMD_HEADER.unpack_from(body, at)
            at += _CMD_HEADER.size
            if n > MAX_AXES:
                raise FrameError(f"{n} entries, at most {MAX_AXES}")
            cmd = LowCmd(seq=seq, flags=flags, t_ns=t_ns)
            for _ in range(n):
                cmd.cmd.append(MotorCmd(*_MOTOR_CMD.unpack_from(body, at)))
                at += _MOTOR_CMD.size
            return CMD, cmd
        if kind == STATE:
            (seq, cmd_seq, tick, t_ns, period_ns, deadline_miss, estop_code, estop,
             _reserved, n) = _STATE_HEADER.unpack_from(body, at)
            at += _STATE_HEADER.size
            if n > MAX_AXES:
                raise FrameError(f"{n} entries, at most {MAX_AXES}")
            state = LowState(seq=seq, cmd_seq=cmd_seq, tick=tick, t_ns=t_ns,
                             period_ns=period_ns, deadline_miss=deadline_miss,
                             estop_code=estop_code, estop=estop)
            for _ in range(n):
                state.state.append(MotorState(*_MOTOR_STATE.unpack_from(body, at)))
                at += _MOTOR_STATE.size
            return STATE, state
    except struct.error as error:
        raise FrameError("payload shorter than its count says") from error
    if kind == DESCRIBE:
        return DESCRIBE, None
    if kind == DESCRIBE_REPLY:
        try:
            return DESCRIBE_REPLY, body[at:].decode("utf-8")
        except UnicodeDecodeError as error:
            raise FrameError("description is not UTF-8") from error
    raise FrameError(f"unknown frame kind {kind}")
