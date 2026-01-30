# Pure Go Dynamixel Control

A high-performance, native Go implementation of **Dynamixel Protocol 2.0** for Windows/Linux.

**No Cgo, No DLLs, Just Pure Go.**

## Features

- **Pure Go** - No C dependencies, native serial port implementation
- **Protocol 2.0** - Packet construction, parsing, byte stuffing, CRC16
- **Sync Read/Write** - Efficient multi-motor control (3-5x faster)
- **Trajectory Generation** - Trapezoidal velocity profile for smooth motion
- **Thread-safe** - Concurrent controller with context-based cancellation
- **Multi-platform** - Windows and Linux support

## Project Structure

```
dxl/
  controller.go      # Multi-motor control loop
  driver.go          # High-level API (Ping, Read, Write, SyncRead/Write)
  protocol.go        # Protocol 2.0 (CRC, Packet, Stuffing)
  trajectory.go      # Trapezoidal velocity profile generation
  serial_windows.go  # Windows serial port
  serial_linux.go    # Linux serial port
  *_test.go          # Unit tests

test/
  position_run.go    # Position control example
  velocity_run.go    # Velocity control example
  torque_run.go      # PWM/Torque control example
  multi_motor_run.go # Multi-motor sync control
  trajectory_run.go  # Trajectory execution example
  smoke_hw_run.go    # Hardware smoke test
  sync_benchmark.go  # Performance benchmark

docs/
  TESTING.md         # Test execution guide
```

## Quick Start

### Installation

```bash
git clone https://github.com/sitdownSeungpyo/dxl_go.git
cd dxl_go/src
go build ./...
```

### Basic Usage

```go
ctrl := dxl.NewController("COM3", 1000000, dxl.ModelXSeries)
ctrl.SetMotorIDs([]uint8{1})
ctrl.Start()
defer ctrl.Stop()

// Set mode
ctrl.SetOperatingMode(1, dxl.OpModePosition)

// Send command
ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: 2048}}

// Receive feedback
fb := <-ctrl.FeedbackChan
fmt.Printf("Position: %d\n", fb[0].Value)
```

### Multi-Motor Control

```go
ctrl.SetMotorIDs([]uint8{1, 2, 3}) // Auto-enables sync mode

ctrl.CommandChan <- []dxl.Command{
    {ID: 1, Value: 2048},
    {ID: 2, Value: 3072},
    {ID: 3, Value: 1024},
}
```

### Trajectory Execution

```go
profile, _ := dxl.NewTrapezoidalProfile(0, 4096, 1000, 5000)
executor := dxl.NewTrajectoryExecutor(ctrl, 1)
executor.Execute(profile, 100) // 100Hz update rate
```

## Running Tests

```bash
# Unit tests
go test ./dxl/... -v

# Unit tests with coverage
go test ./dxl/... -cover

# Hardware tests
go run test/position_run.go -port COM4 -id 1
go run test/trajectory_run.go -port COM4 -id 1
go run test/smoke_hw_run.go -port COM4 -id 1
```

See [docs/TESTING.md](docs/TESTING.md) for detailed test commands.

## API Reference

| Function | Description |
|----------|-------------|
| `NewController(port, baud, model)` | Create controller |
| `SetMotorIDs(ids)` | Configure motor IDs (0-252) |
| `SetOperatingMode(id, mode)` | Set control mode |
| `Start()` / `Stop()` | Start/stop control loop |
| `NewTrapezoidalProfile(start, end, vel, accel)` | Create trajectory |
| `TrajectoryExecutor.Execute(profile, rate)` | Run trajectory |

## Control Modes

| Mode | Constant | Description |
|------|----------|-------------|
| Position | `OpModePosition` | Position control |
| Velocity | `OpModeVelocity` | Velocity control |
| PWM | `OpModePWM` | Torque/PWM control |
| Extended Position | `OpModeExtendedPosition` | Multi-turn position |
| Current-based Position | `OpModeCurrentBasedPos` | Position with current limit |

## Supported Motors

- X-Series: XM430, XC430, XL430, etc.
- MX-Series: MX-106, MX-64, MX-28 (Protocol 2.0 firmware)
- Pro-Series: H54, H42

## Requirements

- Go 1.18+
- Windows or Linux
- Dynamixel Protocol 2.0 compatible motor

## License

MIT
