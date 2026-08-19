package moteus

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	"drlabs/transport/can"
)

// noise is one line of console output the host never asked for.
const noise = "motor.cogging_dq_comp.1 0\n"

// consoleBus is a moteus that carries a text console.
//
// It answers a poll subframe with the next slice of whatever it has to say and
// records what was written into it.
//
// noisyPolls is the point of the fake: it is how many more polls the
// controller answers with output nobody asked for — the tail of a previous
// command, or telemetry.  Counting polls rather than bytes is what makes the
// tests deterministic, because the condition being modelled is a controller
// that is *still emitting*, not one with a particular amount left over.  Each
// noisy poll costs a little wall clock, so a settle budget expires against it
// the way it would against real hardware.
type consoleBus struct {
	noisyPolls int
	pollCost   time.Duration

	out     []byte
	written []byte
	replies map[string]string
	pending bool
}

func (b *consoleBus) Open() error               { return nil }
func (b *consoleBus) Close() error              { return nil }
func (b *consoleBus) Drain(time.Duration) error { return nil }

func (b *consoleBus) Send(frame can.Frame) error {
	r := &reader{data: frame.Data}
	command, err := r.readU8()
	if err != nil {
		return nil
	}
	switch command {
	case streamClientData:
		_, _ = r.readVaruint() // channel
		length, _ := r.readVaruint()
		b.written = append(b.written, r.data[r.offset:r.offset+int(length)]...)
		// Answer once a whole command line has arrived.
		if line, done := strings.CutSuffix(string(b.written), "\n"); done {
			b.written = nil
			if reply, ok := b.replies[strings.TrimSpace(line)]; ok {
				b.out = append(b.out, reply...)
			}
		}
	case streamClientPoll:
		b.pending = true
	}
	return nil
}

func (b *consoleBus) Recv(time.Duration) (can.Frame, error) {
	if !b.pending {
		return can.Frame{}, can.ErrTimeout
	}
	b.pending = false

	if b.noisyPolls > 0 {
		b.noisyPolls--
		b.out = append(b.out, noise...)
		if b.pollCost > 0 {
			time.Sleep(b.pollCost)
		}
	}

	size := maxPollChunk
	if size > len(b.out) {
		size = len(b.out)
	}
	chunk := b.out[:size]
	b.out = b.out[size:]

	w := &writer{}
	w.writeU8(streamServerData)
	w.writeVaruint(diagnosticChannel)
	w.writeVaruint(uint32(len(chunk)))
	w.buf = append(w.buf, chunk...)

	// The reply carries the controller's id in the source field, which is what
	// exchange demultiplexes on.
	return can.Frame{ID: uint32(1) << 8, Data: w.bytes()}, nil
}

func newConsoleStream(bus *consoleBus) *Stream {
	s := NewStream(&Controller{ID: 1, Bus: bus, Timeout: time.Second})
	s.PollInterval = time.Millisecond
	s.SettleQuiet = 5 * time.Millisecond
	s.SettleBudget = 60 * time.Millisecond
	s.DrainQuiet = 5 * time.Millisecond
	s.DrainBudget = 60 * time.Millisecond
	return s
}

// busyBus keeps talking for longer than any budget in these tests, and starts
// with a line already queued.
//
// The queued line is what makes the regression bite: it is served before any
// reply to the command about to be sent, so an unguarded read returns it and
// calls it the answer.
func busyBus(replies map[string]string) *consoleBus {
	return &consoleBus{
		noisyPolls: 1 << 30,
		pollCost:   time.Millisecond,
		out:        []byte(noise),
		replies:    replies,
	}
}

// A console with nothing left over answers the question it was asked.
func TestConfGetReadsTheValueOnAQuietConsole(t *testing.T) {
	bus := &consoleBus{replies: map[string]string{
		"conf get servopos.position_max": "5\n",
	}}

	value, err := newConsoleStream(bus).ConfGetFloat(context.Background(), "servopos.position_max")
	if err != nil {
		t.Fatalf("ConfGetFloat: %v", err)
	}
	if value != 5 {
		t.Fatalf("servopos.position_max = %v, want 5", value)
	}
}

// The regression this file exists for.
//
// A command that produced more output than anyone read leaves the controller
// emitting.  Because `conf get` echoes no key, the next line read is
// indistinguishable from a correct answer — so the only defence is refusing to
// read while the console is still talking.  Before the fix this returned a
// line belonging to a different key, with a nil error, and Identify built a
// travel envelope out of it.
func TestConfGetRefusesToReadALineItCannotAttribute(t *testing.T) {
	bus := busyBus(map[string]string{"conf get servopos.position_min": "-5\n"})

	value, err := newConsoleStream(bus).ConfGet(context.Background(), "servopos.position_min")
	if !errors.Is(err, ErrConsoleBusy) {
		t.Fatalf("ConfGet on a busy console returned (%q, %v); want ErrConsoleBusy", value, err)
	}
}

// Command is written into the same console and needs the same protection.
func TestCommandRefusesABusyConsole(t *testing.T) {
	_, err := newConsoleStream(busyBus(nil)).Command(context.Background(), "tel stop")
	if !errors.Is(err, ErrConsoleBusy) {
		t.Fatalf("Command on a busy console returned (%v); want ErrConsoleBusy", err)
	}
}

// Drain has to say whether it succeeded, because a caller that cannot tell is
// a caller that carries on and trusts the next line.
func TestDrainReportsWhetherTheConsoleWentQuiet(t *testing.T) {
	if !newConsoleStream(&consoleBus{}).Drain(context.Background()) {
		t.Error("Drain reported busy on a console with nothing to say")
	}
	if newConsoleStream(busyBus(nil)).Drain(context.Background()) {
		t.Error("Drain reported quiet while the console was still emitting")
	}
}

// Once the backlog ends the console is usable again: the failure is temporary,
// not a wedged session.
func TestConsoleRecoversOnceTheBacklogIsGone(t *testing.T) {
	bus := &consoleBus{
		noisyPolls: 300,
		pollCost:   time.Millisecond,
		replies:    map[string]string{"conf get servopos.position_min": "-5\n"},
	}
	stream := newConsoleStream(bus)

	if _, err := stream.ConfGet(context.Background(), "servopos.position_min"); !errors.Is(err, ErrConsoleBusy) {
		t.Fatalf("first ConfGet returned %v; want ErrConsoleBusy", err)
	}

	// Draining repeatedly is the documented recovery: one call is bounded by
	// DrainBudget and a real backlog outlasts it.
	rounds := 0
	for ; rounds < 200 && !stream.Drain(context.Background()); rounds++ {
	}
	if rounds == 0 {
		t.Fatal("the console went quiet without draining; the fixture is not exercising the path")
	}

	value, err := stream.ConfGetFloat(context.Background(), "servopos.position_min")
	if err != nil {
		t.Fatalf("ConfGetFloat after draining: %v", err)
	}
	if value != -5 {
		t.Fatalf("servopos.position_min = %v, want -5", value)
	}
}
