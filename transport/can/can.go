// Package can provides CAN bus abstraction for motor communication.
// Used by Robstride (CAN 2.0B extended) and ODrive (standard CAN).
//
// Linux: SocketCAN (AF_CAN) via pure Go syscalls
// macOS/Windows: USB-CAN adapters via SLCAN protocol over serial
package can

// Frame represents a CAN bus frame.
type Frame struct {
	ID       uint32 // CAN identifier (11-bit standard or 29-bit extended)
	Data     [8]byte
	DLC      uint8 // Data Length Code (0-8)
	Extended bool  // True for 29-bit extended ID
}

// Bus is the interface for CAN bus communication.
type Bus interface {
	// Open initializes the CAN bus interface.
	Open() error

	// Close releases the CAN bus interface.
	Close() error

	// Send transmits a CAN frame.
	Send(frame Frame) error

	// Recv receives a CAN frame (blocking with timeout).
	Recv() (Frame, error)
}
