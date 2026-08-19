package can

import (
	"sync"
	"time"
)

// Shared makes one bus usable by several controllers at once.
//
// The problem it solves is not merely that two goroutines might write at the
// same time.  A transaction here is a send followed by a receive, and the
// receive takes whatever frame arrives next — so if two controllers interleave,
// one of them reads the other's reply, discards it as not its own, and both
// time out.  Locking the individual calls would not help; the *pair* has to be
// atomic.
//
// So a caller that shares a bus takes the lock for the whole exchange:
//
//	bus.Lock()
//	defer bus.Unlock()
//	bus.Send(request)
//	reply, err := bus.Recv(timeout)
//
// Drivers do this through the Locker interface, which a bus that is not shared
// simply does not implement — a single controller pays nothing.
//
// What this does not do is make the bus faster.  Serialised transactions mean
// the achievable rate per controller is the bus rate divided by the number of
// controllers on it, and that arithmetic is the real constraint on a
// multi-joint machine.
type Shared struct {
	bus Bus
	mu  sync.Mutex
}

// NewShared wraps a bus so that several controllers can use it.
func NewShared(bus Bus) *Shared { return &Shared{bus: bus} }

// Locker is implemented by buses that need a transaction held together.
//
// A driver performing a send-then-receive exchange should take this lock for
// the duration if the bus offers it.
type Locker interface {
	Lock()
	Unlock()
}

// Lock claims the bus for one transaction.
func (s *Shared) Lock() { s.mu.Lock() }

// Unlock releases it.
func (s *Shared) Unlock() { s.mu.Unlock() }

// Open acquires the underlying bus.
func (s *Shared) Open() error { return s.bus.Open() }

// Close releases it.  The wrapper does not track how many controllers are
// still using it: whoever owns the robot owns the shutdown order.
func (s *Shared) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.bus.Close()
}

// Send transmits one frame.  The caller is expected to hold the lock.
func (s *Shared) Send(frame Frame) error { return s.bus.Send(frame) }

// Recv waits for the next frame.  The caller is expected to hold the lock.
func (s *Shared) Recv(timeout time.Duration) (Frame, error) { return s.bus.Recv(timeout) }

// Drain discards queued frames.
func (s *Shared) Drain(quiet time.Duration) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.bus.Drain(quiet)
}

// Describe passes through to the wrapped bus, so status output still names the
// real transport rather than the wrapper.
func (s *Shared) Describe() map[string]string {
	if describer, ok := s.bus.(Describer); ok {
		return describer.Describe()
	}
	return map[string]string{}
}

// Unwrap returns the bus underneath.
func (s *Shared) Unwrap() Bus { return s.bus }
