// Package serial provides the serial port abstraction and platform-specific implementations.
package serial

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
