"""What a client does above the runtime: profiled moves.

A port of ``crates/drlabs-session/src/trajectory.rs``.  The runtime takes
setpoints; a move is the client sending the right setpoint at the right
time, and this is where "right" is worked out.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class Trapezoidal:
    """Constant acceleration, constant velocity, constant deceleration.

    A move too short to reach the cruise speed is a triangle, handled by
    computing the cruise speed the distance allows rather than special-casing.
    """

    start: float
    target: float
    max_velocity: float
    accel: float

    def __post_init__(self) -> None:
        if self.max_velocity <= 0:
            raise ValueError("the velocity limit must be positive")
        if self.accel <= 0:
            raise ValueError("the acceleration limit must be positive")
        distance = abs(self.target - self.start)
        if distance == 0:
            cruise, accel_time, cruise_time = 0.0, 0.0, 0.0
        else:
            time_to_max = self.max_velocity / self.accel
            ramp = self.max_velocity * time_to_max
            if ramp > distance:
                peak = math.sqrt(self.accel * distance)
                cruise, accel_time, cruise_time = peak, peak / self.accel, 0.0
            else:
                cruise = self.max_velocity
                accel_time = time_to_max
                cruise_time = (distance - ramp) / self.max_velocity
        object.__setattr__(self, "_distance", distance)
        object.__setattr__(self, "_cruise", cruise)
        object.__setattr__(self, "_accel_time", accel_time)
        object.__setattr__(self, "_cruise_time", cruise_time)
        object.__setattr__(self, "total_time", 2 * accel_time + cruise_time)

    def sample(self, t: float) -> Tuple[float, float]:
        """``(position, velocity)`` at ``t`` seconds from the start."""
        if t <= 0:
            return self.start, 0.0
        if t >= self.total_time:
            return self.target, 0.0
        direction = -1.0 if self.target < self.start else 1.0
        a = self.accel
        ta, tc, vc = self._accel_time, self._cruise_time, self._cruise  # type: ignore[attr-defined]
        if t <= ta:
            d, v = 0.5 * a * t * t, a * t
        elif t <= ta + tc:
            d, v = 0.5 * a * ta * ta + vc * (t - ta), vc
        else:
            s = t - ta - tc
            d = 0.5 * a * ta * ta + vc * tc + vc * s - 0.5 * a * s * s
            v = vc - a * s
        return self.start + direction * d, direction * v


def move_to(client, targets: Dict[str, float], max_velocity: float, accel: float,
            rate_hz: float, settle_tolerance: float, settle_timeout: float) -> None:
    """Streams one trapezoid per joint, all starting now, then waits until
    every joint is within tolerance of its target."""
    description = client.describe()
    current = client.positions()
    profiles: Dict[str, Trapezoidal] = {}
    for name, target in targets.items():
        joint = description.joints[description.index(name)]
        start = current.get(name)
        if start is None or start != start:
            raise RuntimeError(f"{name!r} reports no position; cannot plan a move from it")
        target = min(max(float(target), joint.min_rad), joint.max_rad)
        velocity = min(max_velocity, joint.max_velocity_rad_s)
        acceleration = min(accel, joint.max_accel_rad_s2) if joint.max_accel_rad_s2 > 0 else accel
        profiles[name] = Trapezoidal(start, target, velocity, acceleration)

    period = 1.0 / rate_hz
    started = time.monotonic()
    total = max((p.total_time for p in profiles.values()), default=0.0)
    while True:
        t = time.monotonic() - started
        setpoints = {}
        for name, profile in profiles.items():
            q, dq = profile.sample(t)
            setpoints[name] = q
        client.set(setpoints)
        client.send()
        state = client.state()
        if state.stopped:
            raise RuntimeError(f"e-stop: {state.estop_name}")
        if t >= total:
            break
        time.sleep(period)

    deadline = time.monotonic() + settle_timeout
    while time.monotonic() < deadline:
        client.send()  # keep the command fresh while settling
        state = client.state()
        if state.stopped:
            raise RuntimeError(f"e-stop: {state.estop_name}")
        positions = client.positions()
        errors = {name: abs(positions[name] - p.target) for name, p in profiles.items()}
        if all(error < settle_tolerance for error in errors.values()):
            return
        time.sleep(0.05)
    worst = max(errors.items(), key=lambda kv: kv[1])
    raise RuntimeError(f"joints did not settle in {settle_timeout:.0f} s "
                       f"(worst {worst[0]} off by {worst[1]:.3f} rad)")
