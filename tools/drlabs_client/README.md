# drlabs_client

The Python client for the DrLabs real-time runtime (`drlabs rt`). Needs the
Eclipse `cyclonedds` package (`pip install cyclonedds`; it builds against a
CycloneDDS install named by `CYCLONEDDS_HOME`). Copy the `drlabs_client/`
directory next to your code or put `tools/drlabs_client` on `PYTHONPATH`.

```python
from drlabs_client import Client

with Client(domain=0, prefix="drlabs") as robot:
    d = robot.describe()                         # joints in record order, limits, poses
    robot.move_to({"l_sh_p": 0.3}, max_velocity=0.5, accel=1.0)
    print(robot.joint_state("l_sh_p").q)          # rad; NaN = not reported
    robot.hold()                                 # drive every joint to where it is
    robot.release()                              # torque off, every joint
    robot.quick_stop(); robot.fault_reset()      # the robot-wide fault and its reset
```

The runtime takes setpoints and nothing else: a profiled move is this client
sending the right setpoint at the right time (`motion.Trapezoidal`). A
command older than the runtime's `cmd_timeout_ms` is stale; `Client` keeps
the last command fresh for as long as it is open.

Wire format: `docs/RT.md`. The types in `types.py` carry the ROS 2 type
names, so a sample from here and one from a `rmw_cyclonedds` node are the
same bytes; `tests/test_cdr.py` pins those bytes against the hand-written
encoder in `crates/drlabs-api`, so the two cannot drift apart quietly.
`tests/live_sim.py` drives a running `drlabs rt configs/sim26.json` end to
end.
