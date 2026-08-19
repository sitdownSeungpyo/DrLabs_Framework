# Testing

Three layers, in increasing order of what they can hurt.

## 1. The test suite — no hardware

```bash
go test ./...
go test ./... -cover
```

Everything here runs against simulators and in-memory transports. Nothing
touches a bus, so it is safe to run anywhere and fast enough to run on every
change.

The parts worth knowing about:

| Package | What it pins |
|---|---|
| `drivers/moteus` | every command frame compared byte-for-byte against the official `moteus` Python library; the 64-byte frame budget across every refresh and query combination |
| `drivers/robstride` | addressing against the manuals' own worked example, both byte orders, the per-model scaling table, fault bits |
| `drivers/dynamixel` | control-table unit conversions, per-family current units, watchdog rounding |
| `transport/fdcanusb` | line framing, DLC padding, the optional CRC-8 checksum mode |
| `runtime/safety` | every trip, including the cases that must *not* fire |
| `runtime/session` | the whole stack against the simulator: arming, emergency stop, fault handling, trajectory, shuttle, deadman, auto-arm |

Useful subsets:

```bash
go test ./drivers/moteus/ -run Frames -v     # the reference-encoding goldens
go test ./runtime/safety/ -v                 # the trip matrix
go test ./runtime/session/ -run Trajectory   # profiled moves end to end
```

The race detector needs cgo, so `go test -race` requires a C toolchain that a
plain Go install does not provide.

## 2. The simulator — no hardware, but it moves

The simulators are not stubs. A trapezoidal trajectory generator feeds a
torque-limited PD controller driving an inertia with viscous friction, with
thermal rise and supply sag, integrated at 2 kHz independently of the host
rate. Overshoot, stall, thermal rise and envelope escape all actually happen.

```bash
# The whole stack, including the safety layer, with nothing plugged in.
drlabs run -driver moteus-sim -mode position -position 0.25 -max-torque 0.5

# The servo role, then drive it as an upper layer would.
drlabs serve -driver moteus-sim -id 1 -max-torque 0.8 -position-min -0.6 -position-max 0.6
python test/client/drive.py pos 0.1
python test/client/drive.py traj -0.2 --vel 0.3
python test/client/drive.py sweep 0.15 2

# The browser panel.
webui                       # then http://localhost:8080
```

`robstride-sim` works the same way and requires the same `-opt model=RS0x` the
real driver does, so the per-model scaling is exercised too.

What the simulator cannot tell you: anything about your mechanism. It will
happily reach a target that a real axis cannot, at a torque a real motor would
refuse.

## 3. Hardware

Start at [BRINGUP.md](BRINGUP.md). It is ordered so that each stage only
commands what the previous stage proved safe, and everything before first
motion puts nothing on the bus but a stop.

```bash
drlabs ports                                   # find the adapter
drlabs probe -driver moteus -port COM5 -id 1   # identify; commands only a stop
```

`probe` is the one to reach for whenever something is behaving oddly: it
reports the board, the firmware, the controller's own travel bounds, the live
telemetry and the achieved loop rate, and then disconnects.

### Cross-checking against the reference

For moteus, the official Python library is on the same bus and can read the
same registers, which makes it the arbiter when a value looks wrong:

```python
import asyncio, moteus
from moteus.fdcanusb_device import FdcanusbDevice

async def main():
    device = FdcanusbDevice("COM5")
    transport = moteus.Transport([device])
    controller = moteus.Controller(id=1, transport=transport)
    result = (await transport.cycle([controller.make_stop(query=True)]))[0]
    print(result.values)
    device.close()

asyncio.run(main())
```

This is how the decoder was validated: position, voltage, current and
temperature read within sampling noise of what this framework reported.

### One port, one process

The servo, the web UI and any Python client all want the same serial port. Run
one at a time; the second will fail to open it.
