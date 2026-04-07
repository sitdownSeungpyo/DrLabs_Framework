package motor

// Bus represents a communication channel to motors (serial, CAN, USB, etc.).
type Bus interface {
	// Open initializes the communication channel.
	Open() error

	// Close releases the communication channel.
	Close() error

	// Transfer sends a request and returns the response.
	Transfer(request []byte) ([]byte, error)
}

// BulkBus extends Bus with support for multi-device bulk operations.
type BulkBus interface {
	Bus

	// BulkTransfer sends a request expecting n responses.
	BulkTransfer(request []byte, n int) ([][]byte, error)
}
