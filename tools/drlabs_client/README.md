# drlabs_client

The Python client for the DrLabs real-time runtime (`drlabs rt`). Standard
library only; copy the `drlabs_client/` directory next to your code or put
`tools/drlabs_client` on `PYTHONPATH`.

```python
from drlabs_client import Client

with Client("127.0.0.1", 7700) as robot:
    d = robot.describe()                         # joints in record order, limits, poses
    robot.move_to({"l_sh_p": 0.3}, max_velocity=0.5, accel=1.0)
    print(robot.joint_state("l_sh_p").q)          # rad; NaN = not reported
    robot.hold()                                 # drive every joint to where it is
    robot.release()                              # torque off, every joint
    robot.estop(); robot.reset_estop()           # the robot-wide stop and its reset
```

The runtime takes setpoints and nothing else: a profiled move is this client
sending the right setpoint at the right time (`motion.Trapezoidal`). A
command older than the runtime's `cmd_timeout_ms` is stale; `Client` keeps
the last command fresh for as long as it is open.

Wire format: `docs/RT.md`. The byte-exact test vectors in `tests/test_codec.py`
are the same ones `crates/drlabs-api` pins, so the two cannot drift apart
quietly. `tests/live_sim.py` drives a running `drlabs rt configs/sim26.json`
end to end.
