"""Time-synchronized moves: the Python side of ``synchronized`` in
crates/drlabs-core/src/trajectory.rs. The same cases are asserted on both sides.

Loads ``motion.py`` by path so it needs neither ``cyclonedds`` nor a runtime.

    python -m pytest tools/drlabs_client/tests/test_motion.py
"""

import importlib.util
import os
import sys

import pytest

_PATH = os.path.join(os.path.dirname(__file__), "..", "drlabs_client", "motion.py")
_SPEC = importlib.util.spec_from_file_location("drlabs_motion", _PATH)
motion = importlib.util.module_from_spec(_SPEC)
# A dataclass looks its own module up in sys.modules, so register it first.
sys.modules["drlabs_motion"] = motion
_SPEC.loader.exec_module(motion)

LEGS = [
    (0.0, 3.0, 1.0, 2.0),    # long: trapezoid, the slowest
    (0.0, 0.5, 1.0, 2.0),    # shorter: stretched
    (1.0, 0.98, 4.0, 1.0),   # tiny and backwards: a triangle
]


def test_synchronized_axes_arrive_together():
    profiles = motion.synchronized(LEGS)
    slowest = profiles[0].total_time
    for profile, leg in zip(profiles, LEGS):
        assert profile.total_time == pytest.approx(slowest, abs=1e-9)
        assert profile.sample(profile.total_time)[0] == leg[1]


def test_synchronizing_never_exceeds_a_limit():
    for profile, (_, _, max_velocity, accel) in zip(motion.synchronized(LEGS), LEGS):
        steps = 2000
        for i in range(steps + 1):
            _, velocity = profile.sample(profile.total_time * i / steps)
            assert abs(velocity) <= max_velocity + 1e-9
        # The acceleration is the slope of the velocity: check it numerically.
        dt = profile.total_time / steps
        for i in range(steps):
            _, v0 = profile.sample(i * dt)
            _, v1 = profile.sample((i + 1) * dt)
            assert abs(v1 - v0) / dt <= accel + 1e-6


def test_the_slowest_axis_keeps_its_own_profile():
    profiles = motion.synchronized([LEGS[0], LEGS[1]])
    alone = motion.Trapezoidal(*LEGS[0])
    assert profiles[0] == alone


def test_an_axis_with_nowhere_to_go_stays_put():
    profiles = motion.synchronized([(0.0, 2.0, 1.0, 2.0), (0.7, 0.7, 1.0, 2.0)])
    assert profiles[1].total_time == 0.0
    assert profiles[1].sample(0.5)[0] == 0.7
    assert profiles[0].total_time > 0.0


def test_no_axes_is_an_empty_plan():
    assert motion.synchronized([]) == []


def test_one_unusable_limit_refuses_the_whole_move():
    with pytest.raises(ValueError):
        motion.synchronized([(0.0, 1.0, 1.0, 2.0), (0.0, 1.0, 0.0, 2.0)])
