"""The Python client for the DrLabs real-time runtime (``drlabs rt``).

Two records cross the wire: a ``LowCmd`` in, a ``LowState`` out, as UDP
frames.  This package is the reference encoding — ``codec`` is the byte
layout, ``client`` the socket, ``motion`` the things a client does above the
runtime (trapezoidal moves, poses, holds) because the runtime itself does
not.  Standard library only.

    from drlabs_client import Client, MotorCmd, DRIVE
    with Client() as robot:
        robot.describe()                      # joint names, limits, poses
        robot.move_to({"l_sh_p": 0.3}, max_velocity=0.5, accel=1.0)
        state = robot.state()                 # the latest LowState
        print(state.joint("l_sh_p").q)
"""

from .codec import (  # noqa: F401
    DESCRIBE,
    DESCRIBE_REPLY,
    DRIVE,
    ESTOP,
    MAX_AXES,
    OFF,
    PROTOCOL_VERSION,
    RESET_ESTOP,
    STATUS_NAMES,
    BLOCKED_NAMES,
    TRIP_NAMES,
    FrameError,
    LowCmd,
    LowState,
    MotorCmd,
    MotorState,
    decode,
    encode_cmd,
    encode_describe,
    encode_state,
)
from .client import Client, Description, Joint  # noqa: F401
from .motion import Trapezoidal  # noqa: F401

__all__ = [
    "Client",
    "Description",
    "Joint",
    "Trapezoidal",
    "LowCmd",
    "LowState",
    "MotorCmd",
    "MotorState",
    "OFF",
    "DRIVE",
    "ESTOP",
    "RESET_ESTOP",
    "FrameError",
    "PROTOCOL_VERSION",
]
