# 🚀 Pure Go Dynamixel Control

> **"No Cgo, No DLLs, Just Pure Go."**

A high-performance, native Go implementation of the **Dynamixel Protocol 2.0** for Windows.
Designed for developers who want the concurrency and simplicity of Go without the headache of C/C++ dependencies.

## ✨ Key Features

- **Pure Go Implementation**: 
  - Zero dependencies on `dxl_x64_c.dll` or `gcc`.
  - Uses native Windows `syscall` for direct serial port access.
- **Protocol 2.0 Full Support**: 
  - Complete implementation of Packet Construction, Parsing, Byte Stuffing, and CRC16 validation.
- **Robust Control Architecture**: 
  - **Verified Startup**: Checks Ping and Torque Enable before motion.
  - **Closed Loop Control**: Feedback-based motion (Move -> Verify Arrival -> Move).
  - **Concurrency**: Goroutine-based non-blocking controller loop.
- **Configurable Motor Models**: 
  - Supports X-Series (`XM430`, `XC430`) and Pro-Series out of the box.

## 📁 Project Structure

```bash
dxl_go/
├── dxl/
│   ├── driver.go         # 🎮 High-Level API (Ping, Read, Write)
│   ├── protocol.go       # 🧠 Protocol 2.0 Logic (CRC, Packet)
│   ├── controller.go     # ⚡ Concurrent Control Loop
│   └── serial_windows.go # 🔌 Native Serial Port (Syscalls)
└── main.go               # 🚀 Example: Robust Rotation Test
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

### Usage

**Run Rotation Test:**
Rotates Motor ID 1 from 0 to 4096 (closed-loop).

```bash
# Default: COM3, 1Mbps
.\go_dxl.exe

# Custom Port/Baud
.\go_dxl.exe -port COM4 -baud 57600
```

## 🗺️ Roadmap & TBD

The following features were part of the initial high-performance requirements and are planned for future updates:

- [ ] **Real-time 1kHz Cycle**: Ensure <1ms loop times with precise OS timer tuning.
- [ ] **Sync Write / Bulk Read**: Optimize bandwidth for multi-motor robots.
- [ ] **Cross-Platform Support**: Add `serial_linux.go` and `serial_darwin.go`.
- [ ] **Trajectory Generation**: Trapezoidal velocity profile generation in Go.

## 🛠️ Tech Stack

- **Language**: Go (Golang)
- **OS**: Windows (Native API)
- **Hardware**: Robotis Dynamixel Series

---
