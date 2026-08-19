// Package can provides the CAN / CAN-FD abstraction used by the CAN based
// motor drivers.
//
// Two frame widths matter here.  Classic CAN 2.0 carries at most 8 data bytes
// and is what Robstride and ODrive use.  CAN-FD carries up to 64 and is what
// moteus needs: a moteus command and the telemetry query it piggybacks on
// share a single frame, and the useful query sets do not fit in 8 bytes.
//
// Implementations:
//
//	transport/fdcanusb   mjbots fdcanusb, ASCII over USB CDC — all platforms
//	transport/socketcan  Linux SocketCAN (AF_CAN), classic and FD
package can

import (
	"errors"
	"fmt"
	"time"
)

// MaxFDPayload is the largest CAN-FD data field.
const MaxFDPayload = 64

// MaxClassicPayload is the largest classic CAN 2.0 data field.
const MaxClassicPayload = 8

// ErrTimeout is returned by Recv when no frame arrived in time.  It is an
// ordinary condition on a polled bus, not a transport failure.
var ErrTimeout = errors.New("can: receive timeout")

// Frame is a single CAN or CAN-FD frame.
type Frame struct {
	// ID is the arbitration identifier: 11 bits when Extended is false,
	// 29 bits when it is true.
	ID uint32

	// Data is the payload, at most MaxFDPayload bytes.
	Data []byte

	// Extended selects the 29-bit identifier format.
	Extended bool

	// FD marks a CAN-FD frame, which permits payloads above 8 bytes.
	FD bool

	// BRS requests the CAN-FD bit rate switch, so the data phase runs at the
	// higher data bitrate.  Without it a 64-byte frame occupies the bus long
	// enough to matter at kHz control rates.
	BRS bool
}

func (f Frame) String() string {
	return fmt.Sprintf("Frame{id=%03x len=%d fd=%v brs=%v}", f.ID, len(f.Data), f.FD, f.BRS)
}

// ValidDLCs are the payload sizes a CAN-FD frame can actually carry.  Anything
// between them is padded up by the sender.
var ValidDLCs = [...]int{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64}

// RoundUpDLC returns the on-wire payload size for a frame of size bytes.
// CAN-FD has no byte-granular length above 8, so a 9-byte payload is
// transmitted as 12 with the remainder padded.
func RoundUpDLC(size int) int {
	for _, dlc := range ValidDLCs {
		if size <= dlc {
			return dlc
		}
	}
	return MaxFDPayload
}

// Bus is a CAN interface that can send and receive frames.
//
// Recv is expected to block for at most timeout and then return ErrTimeout.
// Implementations must be safe for one sender and one receiver used from the
// same goroutine; they are not required to be safe for concurrent use.
type Bus interface {
	// Open acquires the interface.
	Open() error

	// Close releases the interface.  It is safe to call more than once.
	Close() error

	// Send transmits one frame.
	Send(frame Frame) error

	// Recv waits up to timeout for the next frame.
	Recv(timeout time.Duration) (Frame, error)

	// Drain discards frames already queued, so a fresh transaction is not
	// answered by a stale reply.  It returns when the bus has been quiet for
	// the given interval.
	Drain(quiet time.Duration) error
}

// Describer is implemented by buses that can report what they are attached to.
type Describer interface {
	Describe() map[string]string
}
