package motor

import (
	"errors"
	"fmt"
)

// ErrUnsupportedMode is returned by Link.Cycle when the driver cannot express
// the requested ControlMode.  Check Capabilities first to avoid it.
var ErrUnsupportedMode = errors.New("control mode not supported by this driver")

// ErrClosed is returned when a Link is used before Open or after Close.
var ErrClosed = errors.New("link is closed")

// LinkError is a transport-level failure: the adapter is gone, wedged, or
// answering nonsense.  It is distinct from a timeout — a timeout is a single
// missed reply, a LinkError means the connection should be torn down.
type LinkError struct {
	Op  string
	Err error
}

func (e *LinkError) Error() string {
	if e.Err == nil {
		return "link error: " + e.Op
	}
	return fmt.Sprintf("link error: %s: %v", e.Op, e.Err)
}

func (e *LinkError) Unwrap() error { return e.Err }

// TimeoutError indicates a communication timeout.
type TimeoutError struct {
	MotorID uint8
	Op      string // Operation that timed out (e.g., "ping", "read", "write")
}

func (e *TimeoutError) Error() string {
	return fmt.Sprintf("timeout: motor %d %s", e.MotorID, e.Op)
}

// CommError indicates a communication failure (CRC, framing, etc.).
type CommError struct {
	MotorID uint8
	Code    uint8
	Message string
}

func (e *CommError) Error() string {
	return fmt.Sprintf("comm error: motor %d code 0x%02X: %s", e.MotorID, e.Code, e.Message)
}

// HardwareError indicates a motor hardware fault.
type HardwareError struct {
	MotorID uint8
	Status  uint8 // Bitmask of hardware error flags
}

func (e *HardwareError) Error() string {
	return fmt.Sprintf("hardware error: motor %d status 0x%02X", e.MotorID, e.Status)
}
