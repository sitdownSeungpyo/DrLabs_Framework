"""The Python client for the DrLabs real-time runtime (``drlabs rt``).

Two records cross the wire, a ``LowCmd`` in and a ``LowState`` out, as
CycloneDDS samples under their ROS 2 names — ``types`` is the layout,
``client`` the participant, ``motion`` the things a client does above the
runtime (trapezoidal moves, poses, holds) because the runtime itself does
not.  Needs the Eclipse ``cyclonedds`` package (``pip install cyclonedds``).

    from drlabs_client import Client
    with Client() as robot:                   # domain 0, rt/drlabs/*
        robot.describe()                      # joint names, limits, poses
        robot.move_to({"l_sh_p": 0.3}, max_velocity=0.5, accel=1.0)
        state = robot.state()                 # the latest LowState
        print(robot.joint_state("l_sh_p").q)
"""

from .client import Client, Description, Joint  # noqa: F401
from .motion import Trapezoidal  # noqa: F401
from .types import (  # noqa: F401
    BLOCKED_NAMES,
    DRIVE,
    FAULT_RESET,
    MAX_AXES,
    OFF,
    QUICK_STOP,
    STATUS_NAMES,
    TRIP_NAMES,
    TYPE_DESCRIBE,
    TYPE_LOWCMD,
    TYPE_LOWSTATE,
    Describe,
    LowCmd,
    LowState,
    MotorCmd,
    MotorState,
    topic_name,
)

__all__ = [
    "Client",
    "Description",
    "Joint",
    "Trapezoidal",
    "Describe",
    "LowCmd",
    "LowState",
    "MotorCmd",
    "MotorState",
    "OFF",
    "DRIVE",
    "QUICK_STOP",
    "FAULT_RESET",
    "MAX_AXES",
    "topic_name",
]
