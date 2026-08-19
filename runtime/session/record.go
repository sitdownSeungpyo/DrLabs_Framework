package session

import (
	"sync"
	"time"

	"drlabs/motor"
)

// Level classifies a log event.
type Level string

const (
	LevelInfo  Level = "info"
	LevelOK    Level = "ok"
	LevelWarn  Level = "warn"
	LevelError Level = "error"
)

// Event is one entry in the session log.
//
// Message says what happened; Detail says why and what to do about it.  The
// split exists so a UI can show a readable line and keep the explanation for
// whoever needs it — most operator confusion with motor controllers comes from
// a true but unactionable message.
type Event struct {
	Seq     uint64
	At      time.Time
	Level   Level
	Message string
	Detail  string
}

// Sample is one cycle of recorded telemetry, kept for plotting.
//
// It is a flat struct rather than a full Telemetry because these accumulate at
// the control rate: at 500 Hz a two-minute history is 60,000 of them, and the
// difference between eight floats and forty is the difference between a few
// megabytes and a few tens.
type Sample struct {
	Seq         uint64
	Elapsed     time.Duration
	Position    float64
	Velocity    float64
	Current     float64
	Torque      float64
	Voltage     float64
	Temperature float64
}

// eventRing is a bounded log.
type eventRing struct {
	mu     sync.Mutex
	items  []Event
	limit  int
	nextID uint64
}

func newEventRing(limit int) *eventRing {
	return &eventRing{limit: limit, items: make([]Event, 0, limit)}
}

func (r *eventRing) append(level Level, message, detail string) Event {
	r.mu.Lock()
	defer r.mu.Unlock()

	r.nextID++
	event := Event{
		Seq:     r.nextID,
		At:      time.Now(),
		Level:   level,
		Message: message,
		Detail:  detail,
	}
	if len(r.items) >= r.limit {
		copy(r.items, r.items[1:])
		r.items = r.items[:r.limit-1]
	}
	r.items = append(r.items, event)
	return event
}

// since returns the events newer than seq, oldest first.
func (r *eventRing) since(seq uint64) []Event {
	r.mu.Lock()
	defer r.mu.Unlock()

	var out []Event
	for _, event := range r.items {
		if event.Seq > seq {
			out = append(out, event)
		}
	}
	return out
}

// sampleRing is a bounded telemetry history.
type sampleRing struct {
	mu     sync.Mutex
	items  []Sample
	limit  int
	nextID uint64
}

func newSampleRing(limit int) *sampleRing {
	return &sampleRing{limit: limit, items: make([]Sample, 0, min(limit, 4096))}
}

func (r *sampleRing) append(elapsed time.Duration, telemetry motor.Telemetry) {
	r.mu.Lock()
	defer r.mu.Unlock()

	r.nextID++
	sample := Sample{
		Seq:         r.nextID,
		Elapsed:     elapsed,
		Position:    telemetry.Position,
		Velocity:    telemetry.Velocity,
		Current:     telemetry.QCurrent,
		Torque:      telemetry.Torque,
		Voltage:     telemetry.Voltage,
		Temperature: telemetry.Temperature,
	}
	if len(r.items) >= r.limit {
		copy(r.items, r.items[1:])
		r.items = r.items[:r.limit-1]
	}
	r.items = append(r.items, sample)
}

// since returns up to limit samples newer than seq, oldest first.
//
// It walks backwards from the newest so that a client which has fallen far
// behind gets the most recent window rather than an ancient one — a plot that
// is minutes stale is worse than a plot with a gap.
func (r *sampleRing) since(seq uint64, limit int) []Sample {
	r.mu.Lock()
	defer r.mu.Unlock()

	if limit <= 0 {
		limit = 600
	}
	var out []Sample
	for i := len(r.items) - 1; i >= 0; i-- {
		if r.items[i].Seq <= seq {
			break
		}
		out = append(out, r.items[i])
		if len(out) >= limit {
			break
		}
	}
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// log records an event.  It must not be called while the session mutex is
// held; use logLocked for that.
func (s *Session) log(level Level, message, detail string) {
	s.events.append(level, message, detail)
}

// logLocked is log for callers that already hold the session mutex.  The
// event ring has its own lock, so this is safe — the distinction exists only
// to document which paths are already inside the critical section.
func (s *Session) logLocked(level Level, message, detail string) {
	s.events.append(level, message, detail)
}

// Events returns the log entries newer than seq.
func (s *Session) Events(seq uint64) []Event { return s.events.since(seq) }

// Samples returns up to limit telemetry samples newer than seq.
func (s *Session) Samples(seq uint64, limit int) []Sample { return s.samples.since(seq, limit) }
