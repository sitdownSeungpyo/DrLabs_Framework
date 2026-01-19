# 🚀 Pure Go Dynamixel Control

> **"No Cgo, No DLLs, Just Pure Go."**

A high-performance, native Go implementation of the **Dynamixel Protocol 2.0** for Windows.
Designed for developers who want the concurrency and simplicity of Go without the headache of C/C++ dependencies.

## ✨ Key Features

- **Pure Go Implementation**:
  - Zero dependencies on `dxl_x64_c.dll` or `gcc`.
  - Native Windows & Linux serial port support.
- **Protocol 2.0 Full Support**:
  - Complete implementation of Packet Construction, Parsing, Byte Stuffing, and CRC16 validation.
  - **Sync Read/Write**: Efficient multi-motor control in a single packet (up to 3-5x faster).
  - **Bulk Read/Write**: Future support for different addresses per motor.
- **Robust Control Architecture**:
  - **Verified Startup**: Checks Ping and Torque Enable before motion.
  - **Closed Loop Control**: Feedback-based motion (Move -> Verify Arrival -> Move).
  - **Concurrency**: Goroutine-based non-blocking controller loop.
  - **Multi-Motor Support**: Control multiple motors simultaneously with automatic sync optimization.
- **Configurable Motor Models**:
  - Supports X-Series (`XM430`, `XC430`) and Pro-Series out of the box.
- **Multiple Control Modes**:
  - Position Control, Velocity Control, PWM (Torque) Control

## 📁 Project Structure

```bash
dxl_go/
├── dxl/
│   ├── driver.go         # 🎮 High-Level API (Ping, Read, Write, Sync Read/Write)
│   ├── protocol.go       # 🧠 Protocol 2.0 Logic (CRC, Packet)
│   ├── controller.go     # ⚡ Concurrent Multi-Motor Control Loop
│   ├── serial_windows.go # 🔌 Native Windows Serial Port
│   └── serial_linux.go   # 🔌 Native Linux Serial Port
├── test/
│   ├── position_run.go   # Position control example
│   ├── velocity_run.go   # Velocity control example
│   ├── torque_run.go     # PWM/Torque control example
│   ├── multi_motor_run.go# Multi-motor sync control example
│   └── sync_benchmark.go # Performance comparison
└── main.go               # 🚀 Main entry point
```

## 🚀 Getting Started

### Prerequisites
- Go 1.18+
- Windows OS (for `syscall` support)
- Dynamixel X-Series or compatible motor (Protocol 2.0)

### Installation

```bash
git clone https://github.com/sitdownSeungpyo/dxl_go.git
cd dxl_go/src
go build
```

### Usage Examples

**Position Control (Single Motor):**
```bash
cd test
go run position_run.go
```

**Velocity Control:**
```bash
cd test
go run velocity_run.go
```

**Multi-Motor Control (Sync Read/Write):**
```bash
cd test
go run multi_motor_run.go
```

**Performance Benchmark:**
Compare individual writes vs sync write performance.
```bash
cd test
go run sync_benchmark.go
# Expected: Sync Write is 3-5x faster for 3+ motors
```

### API Quick Reference

**Single Motor Control:**
```go
// Individual read/write
driver.Write4Byte(id, addr, value)
driver.Read4Byte(id, addr)
```

**Multi-Motor Control (Recommended):**
```go
// Sync Write - Send to multiple motors in one packet
values := map[uint8]uint32{
    1: 2048,  // Motor 1 -> position 2048
    2: 3072,  // Motor 2 -> position 3072
    3: 1024,  // Motor 3 -> position 1024
}
driver.SyncWrite4Byte(goalPositionAddr, values)

// Sync Read - Read from multiple motors efficiently
ids := []uint8{1, 2, 3}
positions, _ := driver.SyncRead4Byte(presentPositionAddr, ids)
// Returns: map[uint8]uint32{1: 2048, 2: 3072, 3: 1024}
```

**Controller with Auto-Optimization:**
```go
ctrl := dxl.NewController("COM3", 57600, dxl.ModelXSeries)
ctrl.SetMotorIDs([]uint8{1, 2, 3}) // Automatically enables sync read/write
ctrl.Start()

// Send commands - automatically uses sync write for efficiency
ctrl.CommandChan <- []dxl.Command{
    {ID: 1, Value: 2048},
    {ID: 2, Value: 3072},
    {ID: 3, Value: 1024},
}

// Receive feedback - automatically uses sync read
feedbacks := <-ctrl.FeedbackChan // Returns all motor positions
```

## 🗺️ Roadmap & TBD

Recent updates:
- [x] **Sync Read/Write**: Implemented for efficient multi-motor control (3-5x faster)
- [x] **Cross-Platform Support**: Linux support added
- [x] **Multiple Control Modes**: Position, Velocity, and PWM control

Future enhancements:
- [ ] **Bulk Read/Write**: Per-motor custom address/length support
- [ ] **Real-time 1kHz Cycle**: Optimize for <1ms loop times with OS timer tuning
- [ ] **Trajectory Generation**: Trapezoidal velocity profile generation in Go
- [ ] **macOS Support**: Add `serial_darwin.go`

## 🛠️ Tech Stack

- **Language**: Go (Golang)
- **OS**: Windows (Native API)
- **Hardware**: Robotis Dynamixel Series

---
