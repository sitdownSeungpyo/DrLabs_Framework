"""Drives a running ``drlabs rt`` through the client, end to end.

    drlabs rt configs/sim26.json          # in one terminal
    python tools/drlabs_client/tests/live_sim.py [domain] [prefix]

Not a unit test: it needs the runtime.  It exercises describe, a profiled
move, a hold, a release, an e-stop and its reset, and prints what it saw.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drlabs_client import Client  # noqa: E402


def main() -> int:
    domain = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    prefix = sys.argv[2] if len(sys.argv) > 2 else "drlabs"
    with Client(domain, prefix) as robot:
        d = robot.describe()
        print(f"{d.robot}: {len(d.joints)} joints at {d.rate_hz} Hz, "
              f"cmd timeout {d.cmd_timeout_ms} ms ({d.on_cmd_timeout})")
        state = robot.state()
        print(f"tick {state.tick}, estop {state.stopped}, "
              f"first joint {state.state[0].status_name}")

        names = d.names[:4]
        targets = {name: 0.4 for name in names}
        t0 = time.monotonic()
        robot.move_to(targets, max_velocity=1.0, accel=4.0)
        took = time.monotonic() - t0
        got = {name: round(robot.joint_state(name).q, 3) for name in names}
        print(f"move_to {targets} arrived in {took:.2f} s: {got}")
        assert all(abs(q - 0.4) < 0.03 for q in got.values()), got

        robot.hold(names)
        time.sleep(0.2)
        assert robot.joint_state(names[0]).driving

        robot.release(names)
        time.sleep(0.2)
        s = robot.joint_state(names[0])
        print(f"released: {names[0]} is {s.status_name}")
        assert s.status_name == "off", s

        robot.command({names[0]: 0.0})
        time.sleep(0.5)
        robot.quick_stop()
        time.sleep(0.2)
        state = robot.state()
        print(f"e-stop: stopped={state.stopped} ({state.estop_name})")
        assert state.stopped and state.estop_name == "quick_stop_requested"

        robot.fault_reset()
        time.sleep(0.2)
        state = robot.state()
        print(f"reset: stopped={state.stopped}, {names[0]} is "
              f"{state.state[0].status_name}")
        assert not state.stopped and state.state[0].status_name == "off"

        print(f"deadline misses so far: {state.deadline_miss}")
        print("ok")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # noqa: BLE001
        print(f"FAILED: {error!r}")
        sys.exit(1)
