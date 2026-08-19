// Package serial provides the serial port abstraction and platform-specific implementations.
package serial

import "time"

// Port defines the contract for serial port operations.
// Implementations handle platform-specific serial I/O (macOS/Linux/Windows).
// This interface enables dependency injection and mocking for unit tests.
type Port interface {
	// Read reads up to len(b) bytes from the serial port.
	// Returns (0, nil) when no data is available (non-blocking).
	Read(b []byte) (int, error)

	// Write writes len(b) bytes to the serial port.
	Write(b []byte) (int, error)

	// Close closes the serial port and releases resources.
	Close() error
}

// Flusher is an optional interface for serial ports that support buffer flushing.
type Flusher interface {
	Flush() error
}

// ReadTimeouter is an optional interface for ports whose blocking read timeout
// can be tuned.
//
// The right value depends entirely on what is on the other end.  A Dynamixel
// packet is worth waiting ten milliseconds for; a CAN adapter answering a
// 200 Hz control loop has five milliseconds for the whole round trip, and a
// read that blocks for ten is the single largest cost in the cycle.  A port
// that does not implement this keeps whatever default it was opened with.
type ReadTimeouter interface {
	SetReadTimeout(timeout time.Duration) error
}
