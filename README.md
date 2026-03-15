# dxl_go — Pure Go Dynamixel SDK

A high-performance, zero-dependency Go implementation of **Dynamixel Protocol 2.0**.

**No Cgo, No DLLs, Just Pure Go.**

> Inspired by the official [ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK). This project reimplements the core protocol and driver layers in pure Go for cross-platform compatibility and easy deployment.

## Features

- **Pure Go** — Native serial port implementation per platform (no C dependencies)
- **Protocol 2.0** — Full packet construction, parsing, CRC-16 validation, byte stuffing
- **Sync Read/Write** — Efficient multi-motor communication (3-5x faster than individual commands)
- **Trajectory Generation** — Trapezoidal velocity profile with smooth acceleration/deceleration
- **Concurrent Controller** — Channel-based command/feedback loop with context cancellation
- **Cross-Platform** — Windows, Linux, macOS (including Apple Silicon)

## Architecture

```
┌──────────────────────────────────────────────────┐
│  Application (main.go, test/*.go)                │
│  - Position/Velocity/Torque control              │
│  - Trajectory execution                          │
├──────────────────────────────────────────────────┤
│  Controller (controller.go)                      │
│  - Goroutine-based control loop                  │
│  - Channel I/O (CommandChan / FeedbackChan)      │
│  - Auto sync mode for multi-motor                │
│  - Operating mode management                     │
├──────────────────────────────────────────────────┤
│  Driver (driver.go)                              │
│  - Ping / Read / Write / SyncRead / SyncWrite    │
│  - Packet timeout & CRC validation               │
│  - Input buffer flush before each Transfer       │
├──────────────────────────────────────────────────┤
│  Protocol (protocol.go)                          │
│  - BuildPacket / ParsePacket                     │
│  - CRC-16 (XMODEM variant)                      │
│  - Byte stuffing / destuffing                    │
├──────────────────────────────────────────────────┤
│  Serial Port (per-platform)                      │
│  - serial_windows.go  (Win32 API)                │
│  - serial_linux.go    (termios + ioctl)          │
│  - serial_darwin.go   (termios + IOSSIOSPEED)    │
└──────────────────────────────────────────────────┘
```

## Requirements

### Software
- Go 1.18+

### Hardware
- **Motor**: Dynamixel Protocol 2.0 compatible (see [Supported Motors](#supported-motors))
- **Interface**: USB-Serial adapter that supports your motor's baud rate
  - [U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/) (ROBOTIS 공식)
  - [OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/) (Arduino 호환 보드, USB CDC ACM)
- **Power**: 모터에 맞는 전원 공급 (예: XL330은 5V, XM430은 12V)
- **Cable**: JST 3-pin 또는 4-pin 커넥터 케이블

### Verified Setup
| 항목 | 사양 |
|------|------|
| Motor | XL330-M288-T (Model 1200) |
| Interface | OpenRB-150 (CDC ACM) |
| Baudrate | 1,000,000 bps |
| OS | Windows 10/11, macOS (ARM64) |

## Quick Start

### Installation

```bash
git clone https://github.com/sitdownSeungpyo/dxl_go.git
cd dxl_go/dxl_go
go build ./...
```

### Serial Port 확인

```bash
# macOS — cu.* 포트 사용 권장 (tty.* 대신)
ls /dev/cu.usb*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*

# Windows
# 장치 관리자에서 COM 포트 번호 확인
```

### Connection Test

```bash
# macOS
go run main.go -port /dev/cu.usbmodem14301 -baud 1000000

# Linux
go run main.go -port /dev/ttyACM0 -baud 1000000

# Windows
go run main.go -port COM3 -baud 1000000
```

## Usage

### Basic: Single Motor Position Control

```go
package main

import (
    "fmt"
    "go_dxl/dxl"
)

func main() {
    ctrl := dxl.NewController("/dev/cu.usbmodem14301", 1000000, dxl.ModelXSeries)
    ctrl.SetMotorIDs([]uint8{1})

    if err := ctrl.Start(); err != nil {
        panic(err)
    }
    defer ctrl.Stop()

    // Set position mode
    ctrl.SetOperatingMode(1, dxl.OpModePosition)

    // Move to position 2048 (center)
    ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: 2048}}

    // Read feedback
    fb := <-ctrl.FeedbackChan
    fmt.Printf("Position: %d\n", fb[0].Value)
}
```

### Multi-Motor Sync Control

```go
// 2개 이상이면 자동으로 Sync Read/Write 모드 활성화
ctrl.SetMotorIDs([]uint8{1, 2, 3})

ctrl.CommandChan <- []dxl.Command{
    {ID: 1, Value: 2048},
    {ID: 2, Value: 3072},
    {ID: 3, Value: 1024},
}
```

### Trajectory Execution

```go
// Trapezoidal velocity profile: start=0, target=4096, maxVel=1000, accel=5000
profile, _ := dxl.NewTrapezoidalProfile(0, 4096, 1000, 5000)
executor := dxl.NewTrajectoryExecutor(ctrl, 1)

// Execute at 100Hz update rate
executor.Execute(profile, 100)
```

### Low-Level Driver Access

```go
sp, _ := dxl.OpenSerial("/dev/cu.usbmodem14301", 1000000)
defer sp.Close()

driver := dxl.NewDriver(sp)

// Ping
model, _ := driver.Ping(1)
fmt.Printf("Model: %d\n", model)

// Read 4 bytes from address 132 (Present Position)
pos, _ := driver.Read4Byte(1, 132)
fmt.Printf("Position: %d\n", pos)

// Write 4 bytes to address 116 (Goal Position)
driver.Write4Byte(1, 116, 2048)
```

## Control Modes

| Mode | Constant | Value | Description |
|------|----------|-------|-------------|
| Current | `OpModeCurrent` | 0 | 전류(토크) 제어 |
| Velocity | `OpModeVelocity` | 1 | 속도 제어 |
| Position | `OpModePosition` | 3 | 위치 제어 (0~4095) |
| Extended Position | `OpModeExtendedPosition` | 4 | 다중 회전 위치 제어 |
| Current-based Position | `OpModeCurrentBasedPos` | 5 | 전류 제한 위치 제어 |
| PWM | `OpModePWM` | 16 | PWM 직접 제어 |

## Supported Motors

Protocol 2.0을 지원하는 모든 Dynamixel 모터에서 동작합니다:

| Series | Models | Control Table |
|--------|--------|---------------|
| **X-Series** | XL330, XL430, XC330, XC430, XM430, XM540, XH430, XH540, XW430, XW540 | `ModelXSeries` |
| **MX-Series** | MX-28, MX-64, MX-106 (Protocol 2.0 펌웨어 필요) | `ModelXSeries` |
| **Pro-Series** | H54, H42 | `ModelProSeries` |

> Control Table 주소가 다른 모터는 `MotorModel` 구조체를 커스텀하여 사용할 수 있습니다.

## Project Structure

```
dxl_go/
├── main.go                     # CLI entry point
├── dxl/
│   ├── protocol.go             # Protocol 2.0 (CRC, Packet, Stuffing)
│   ├── driver.go               # Driver (Ping, Read, Write, SyncRead/Write)
│   ├── controller.go           # Concurrent control loop
│   ├── trajectory.go           # Trapezoidal motion profile
│   ├── serial_windows.go       # Windows serial (Win32 API)
│   ├── serial_linux.go         # Linux serial (termios)
│   ├── serial_darwin.go        # macOS serial (termios + IOSSIOSPEED)
│   ├── protocol_test.go        # Protocol unit tests
│   ├── driver_test.go          # Driver unit tests
│   ├── trajectory_test.go      # Trajectory unit tests
│   └── smoke_test.go           # Integration tests
├── test/
│   ├── position_run.go         # Position control test
│   ├── velocity_run.go         # Velocity control test
│   ├── torque_run.go           # Torque/PWM control test
│   ├── trajectory_run.go       # Single motor trajectory
│   ├── multi_motor_run.go      # Multi-motor sync control
│   ├── multi_trajectory_run.go # Multi-motor trajectory
│   ├── smoke_hw_run.go         # Hardware smoke test
│   └── sync_benchmark.go       # Sync vs individual benchmark
└── docs/
    └── TESTING.md              # Test execution guide
```

## Running Tests

### Unit Tests (하드웨어 불필요)

```bash
go test ./dxl/ -v
go test ./dxl/ -cover
```

### Hardware Tests

```bash
# 포트/ID를 환경에 맞게 변경하세요
PORT=/dev/cu.usbmodem14301
BAUD=1000000
ID=1

# Ping & 연결 테스트
go run main.go -port $PORT -baud $BAUD

# Position 제어
go run test/position_run.go -port $PORT -baud $BAUD -id $ID

# Velocity 제어
go run test/velocity_run.go -port $PORT -baud $BAUD -id $ID

# Trajectory 실행
go run test/trajectory_run.go -port $PORT -baud $BAUD -id $ID

# 전체 스모크 테스트
go run test/smoke_hw_run.go -port $PORT -baud $BAUD -id $ID

# Sync Read/Write 벤치마크
go run test/sync_benchmark.go
```

## macOS 참고사항

- **`/dev/cu.*` 포트 사용** — `/dev/tty.*` 대신 `/dev/cu.*`를 사용하세요. CDC ACM 디바이스에서 더 안정적입니다.
- **OpenRB-150**은 `/dev/cu.usbmodemXXXX` 형태로 인식됩니다.
- **U2D2**는 `/dev/cu.usbserial-XXXX` 형태로 인식됩니다.

## References

- [ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) — 공식 C/C++/Python SDK (이 프로젝트의 프로토콜 구현 참고)
- [Dynamixel Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/) — 패킷 구조, CRC, Byte Stuffing 명세
- [XL330-M288-T e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/) — Control Table, 전기적 사양
- [OpenRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/) — USB-Serial 인터페이스 보드

## License

MIT
