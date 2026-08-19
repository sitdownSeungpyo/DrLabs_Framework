package serial

import (
	"sync"
	"time"
)

// Shared makes one serial port usable by several devices — a Dynamixel daisy
// chain, where a dozen servos answer on the same two wires.
//
// The unit that has to be atomic is the transaction, not the call.  A
// Protocol 2.0 exchange is flush, write, then read until a packet arrives; if
// two servos' exchanges interleave, each read consumes the other's reply and
// both time out.  Locking Read and Write individually would not help.
//
// Callers take the lock for the whole exchange, through the Locker interface.
// A port with a single device on it does not implement Locker and pays nothing.
type Shared struct {
	port Port
	mu   sync.Mutex
}

// NewShared wraps a port for use by several devices.
func NewShared(port Port) *Shared { return &Shared{port: port} }

// Locker is implemented by ports that need a transaction held together.
type Locker interface {
	Lock()
	Unlock()
}

// Lock claims the port for one transaction.
func (s *Shared) Lock() { s.mu.Lock() }

// Unlock releases it.
func (s *Shared) Unlock() { s.mu.Unlock() }

// Read passes through.  The caller is expected to hold the lock.
func (s *Shared) Read(b []byte) (int, error) { return s.port.Read(b) }

// Write passes through.  The caller is expected to hold the lock.
func (s *Shared) Write(b []byte) (int, error) { return s.port.Write(b) }

// Flush clears the buffers if the underlying port can.
func (s *Shared) Flush() error {
	if flusher, ok := s.port.(Flusher); ok {
		return flusher.Flush()
	}
	return nil
}

// SetReadTimeout passes through if the underlying port can be tuned.
func (s *Shared) SetReadTimeout(timeout time.Duration) error {
	if tuner, ok := s.port.(ReadTimeouter); ok {
		return tuner.SetReadTimeout(timeout)
	}
	return nil
}

// Close releases the underlying port.  Whoever owns the machine owns the
// shutdown order; this does not count users.
func (s *Shared) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.port.Close()
}
