package motor

import "fmt"

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
