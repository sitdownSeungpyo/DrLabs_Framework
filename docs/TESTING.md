# Testing Guide

## Unit Tests

### Run All Tests
```bash
go test ./dxl/... -v
```

### Run Tests with Coverage
```bash
go test ./dxl/... -cover
```

### Run Specific Test Categories
```bash
# Protocol tests
go test ./dxl/... -v -run "Protocol|CRC|Stuff|Packet"

# Driver tests
go test ./dxl/... -v -run "Driver|Ping|Read|Write|Sync"

# Trajectory tests
go test ./dxl/... -v -run "Trapezoidal|Trajectory"
```

### Run Benchmarks
```bash
# All benchmarks
go test ./dxl/... -bench=. -benchmem

# Trajectory benchmarks only
go test ./dxl/... -bench=Trapezoidal -benchmem
```

---

## Hardware Tests

All hardware tests are located in the `test/` directory.

### Common Flags
| Flag | Default | Description |
|------|---------|-------------|
| `-port` | COM4 | Serial port |
| `-baud` | 1000000 | Baudrate |
| `-id` | 1 | Motor ID |

---

### Position Control Test
Basic position control with target positions.

```bash
go run test/position_run.go
go run test/position_run.go -port COM4 -id 1
```

---

### Velocity Control Test
Velocity mode control test.

```bash
go run test/velocity_run.go
go run test/velocity_run.go -port COM4 -id 1
```

---

### Torque Control Test
Current-based torque control test.

```bash
go run test/torque_run.go
go run test/torque_run.go -port COM4 -id 1
```

---

### Multi-Motor Test
Simultaneous control of multiple motors.

```bash
go run test/multi_motor_run.go
go run test/multi_motor_run.go -port COM4 -ids 1,2,3
```

---

### Sync Read/Write Benchmark
Performance benchmark for sync operations.

```bash
go run test/sync_benchmark.go
go run test/sync_benchmark.go -port COM4 -iterations 1000
```

---

### Trajectory Test
Trapezoidal velocity profile trajectory execution.

```bash
# Basic execution (0 -> 2048)
go run test/trajectory_run.go

# Custom parameters
go run test/trajectory_run.go -port COM4 -id 1 -start 0 -target 4095 -vel 1000 -accel 3000

# Continuous loop mode
go run test/trajectory_run.go -port COM4 -loop
```

**Trajectory-specific flags:**
| Flag | Default | Description |
|------|---------|-------------|
| `-start` | 0 | Start position (0-4095) |
| `-target` | 2048 | Target position (0-4095) |
| `-vel` | 500 | Max velocity (units/sec) |
| `-accel` | 2000 | Acceleration (units/sec^2) |
| `-rate` | 100 | Update rate (Hz) |
| `-loop` | false | Loop back and forth |

---

## Main Application

```bash
# Build
go build -o main.exe main.go

# Run
go run main.go
```
