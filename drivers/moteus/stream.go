package moteus

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"time"
)

// The tunnelled diagnostic stream.
//
// Alongside the register interface, moteus carries a text console over the
// same CAN frames.  It is what tview and moteus_tool talk to, and it is the
// only way to read the controller's stored configuration — most importantly
// servopos.position_min/max, the travel envelope the firmware enforces.
//
// That envelope matters more than its obscurity suggests.  Starting position
// control while the axis sits outside it raises fault 39 immediately, and
// commanding past it makes the firmware clamp the target and report fault 103.
// Both look from the outside exactly like "the motor ignores me".  Reading the
// bounds at connect time turns that into a sentence the operator can act on.
//
// Subframes (see multiplex.go for the framing):
//
//	0x40  client -> server data:  channel, length, bytes
//	0x41  server -> client data:  channel, length, bytes
//	0x42  client polls server:    channel, maximum length
const (
	streamClientData = 0x40
	streamServerData = 0x41
	streamClientPoll = 0x42

	// diagnosticChannel is the console channel; channel 0 is the firmware's
	// own boot log.
	diagnosticChannel = 1

	// maxStreamChunk keeps a write subframe inside one CAN-FD frame after the
	// command byte, channel and length varuints are accounted for.
	maxStreamChunk = 48

	// maxPollChunk is the largest reply that still fits: 64 bytes less the
	// command byte and the channel and length varuints.
	maxPollChunk = 61
)

// Stream is a text console session with one controller.
//
// It is not safe for concurrent use, and it should not be interleaved with
// control cycles: the console is slow enough that a `conf get` inside a
// running loop will blow the period budget.  Use it at connect time, or while
// the axis is stopped.
type Stream struct {
	controller *Controller

	// PollInterval is the pause between polls when the controller has nothing
	// to say yet.
	PollInterval time.Duration

	// Timeout bounds one complete command/response exchange.
	Timeout time.Duration

	// SettleQuiet is how long the console must say nothing before a command
	// is written into it, and SettleBudget bounds the wait.  See settle.
	SettleQuiet  time.Duration
	SettleBudget time.Duration

	// DrainQuiet is how long Drain needs the console to say nothing before it
	// calls it quiet.  It is longer than SettleQuiet because Drain is used
	// where the console may be mid-record rather than merely between lines.
	DrainQuiet time.Duration

	// DrainBudget bounds Drain.  A controller that was asked for more output
	// than anyone read — `conf enumerate` is thousands of lines — keeps
	// emitting for far longer than a connect should wait, so Drain gives up
	// and says so rather than blocking.
	DrainBudget time.Duration

	buf []byte
}

// ErrConsoleBusy means the console was still emitting when a command was about
// to be written, so nothing read afterwards could be attributed to it.
//
// This is deliberately an error rather than a best guess.  `conf get` answers
// with a bare value and echoes no key, so a reply cannot be matched to its
// request: the only thing separating an answer from the tail of somebody
// else's output is that the console was quiet beforehand.  Without that, a
// stale line is indistinguishable from a correct one — and a wrong travel
// bound or torque limit reported confidently is worse than no answer.
var ErrConsoleBusy = errors.New("moteus: the console is still emitting; its next line cannot be trusted")

// NewStream opens a console session on the diagnostic channel.
func NewStream(c *Controller) *Stream {
	return &Stream{
		controller:   c,
		PollInterval: 2 * time.Millisecond,
		Timeout:      3 * time.Second,
		SettleQuiet:  30 * time.Millisecond,
		SettleBudget: time.Second,
		DrainQuiet:   150 * time.Millisecond,
		DrainBudget:  5 * time.Second,
	}
}

// Command sends one console command and returns the lines it produced, not
// counting the trailing "OK".
//
// An "ERR" response is returned as an error carrying the controller's own
// message, which is almost always more specific than anything this layer
// could infer.
func (s *Stream) Command(ctx context.Context, command string) ([]string, error) {
	if !s.settle(ctx) {
		return nil, ErrConsoleBusy
	}
	if err := s.write(ctx, []byte(command+"\n")); err != nil {
		return nil, err
	}

	deadline := time.Now().Add(s.Timeout)
	var lines []string

	for {
		line, err := s.readLine(ctx, deadline)
		if err != nil {
			return lines, err
		}
		// The terminator is matched as a prefix, not as a whole line.  A board
		// that was streaming telemetry when we connected delivers binary bytes
		// on the same line, and an equality test would never see the "OK" that
		// follows them.
		switch {
		case strings.HasPrefix(line, "OK"):
			return lines, nil
		case strings.HasPrefix(line, "ERR"):
			return lines, fmt.Errorf("moteus: %s", line)
		default:
			lines = append(lines, line)
		}
	}
}

// Query sends a command and returns the single line it answers with.
//
// Some console commands — `conf get` among them — reply with the value and no
// "OK" terminator, so waiting for one just burns the timeout and then reports
// a failure for a query that actually succeeded.  The reference tooling reads
// these with its "allow any response" mode; this is the same thing.
func (s *Stream) Query(ctx context.Context, command string) (string, error) {
	if !s.settle(ctx) {
		return "", ErrConsoleBusy
	}
	if err := s.write(ctx, []byte(command+"\n")); err != nil {
		return "", err
	}
	line, err := s.readLine(ctx, time.Now().Add(s.Timeout))
	if err != nil {
		return "", err
	}
	if strings.HasPrefix(line, "ERR") {
		return "", fmt.Errorf("moteus: %s", line)
	}
	return line, nil
}

// readLine returns the next non-empty console line.
//
// Lines are terminated by either carriage return or newline: the console uses
// both, and splitting on only one of them merges responses into a single line
// that then matches nothing.
func (s *Stream) readLine(ctx context.Context, deadline time.Time) (string, error) {
	for {
		if idx := indexEOL(s.buf); idx >= 0 {
			raw := s.buf[:idx]
			s.buf = s.buf[idx+1:]
			if line := strings.TrimRight(string(raw), "\r\n \t"); line != "" {
				return line, nil
			}
			continue
		}

		if time.Now().After(deadline) {
			return "", fmt.Errorf("moteus: console read timed out")
		}
		if err := ctx.Err(); err != nil {
			return "", err
		}

		chunk, err := s.poll(ctx)
		if err != nil {
			return "", err
		}
		if len(chunk) == 0 {
			time.Sleep(s.PollInterval)
			continue
		}
		s.buf = append(s.buf, chunk...)
	}
}

// indexEOL finds the first carriage return or newline, or -1.
func indexEOL(data []byte) int {
	cr := bytes.IndexByte(data, '\r')
	lf := bytes.IndexByte(data, '\n')
	switch {
	case cr < 0:
		return lf
	case lf < 0:
		return cr
	case cr < lf:
		return cr
	default:
		return lf
	}
}

// ConfGet reads one configuration value.
//
// The controller answers `conf get servopos.position_min` with the bare value,
// so the response is parsed as a float.  A configured "nan" means the bound is
// not set, which is reported as (NaN, nil) rather than an error — an unbounded
// axis is a perfectly ordinary configuration.
func (s *Stream) ConfGet(ctx context.Context, key string) (string, error) {
	line, err := s.Query(ctx, "conf get "+key)
	if err != nil {
		return "", err
	}
	return strings.TrimSpace(line), nil
}

// ConfGetFloat is ConfGet parsed as a number.  A value of "nan" comes back as
// NaN with no error.
func (s *Stream) ConfGetFloat(ctx context.Context, key string) (float64, error) {
	text, err := s.ConfGet(ctx, key)
	if err != nil {
		return 0, err
	}
	if strings.EqualFold(text, "nan") || text == "" {
		return nan(), nil
	}
	value, err := strconv.ParseFloat(text, 64)
	if err != nil {
		return 0, fmt.Errorf("moteus: %q returned %q, which is not a number", key, text)
	}
	return value, nil
}

// write pushes bytes into the controller's console, in frame-sized pieces.
func (s *Stream) write(ctx context.Context, data []byte) error {
	for len(data) > 0 {
		chunk := data
		if len(chunk) > maxStreamChunk {
			chunk = chunk[:maxStreamChunk]
		}
		data = data[len(chunk):]

		w := &writer{}
		w.writeU8(streamClientData)
		w.writeVaruint(diagnosticChannel)
		w.writeVaruint(uint32(len(chunk)))
		w.buf = append(w.buf, chunk...)

		if _, err := s.controller.exchange(ctx, w.bytes(), false, nil); err != nil {
			return err
		}
	}
	return nil
}

// poll asks the controller for whatever console output it has buffered.
func (s *Stream) poll(ctx context.Context) ([]byte, error) {
	w := &writer{}
	w.writeU8(streamClientPoll)
	w.writeVaruint(diagnosticChannel)
	w.writeVaruint(maxPollChunk)

	reply, err := s.controller.exchange(ctx, w.bytes(), true, isStreamReply)
	if err != nil || reply == nil {
		return nil, err
	}
	return parseStreamData(reply, diagnosticChannel)
}

// isStreamReply accepts server-to-client console frames.
func isStreamReply(data []byte) bool { return data[0] == streamServerData }

// parseStreamData extracts the console bytes from a 0x41 subframe addressed to
// the given channel.
func parseStreamData(data []byte, channel uint32) ([]byte, error) {
	r := &reader{data: data}

	command, err := r.readU8()
	if err != nil {
		return nil, err
	}
	if command != streamServerData {
		return nil, nil
	}

	frameChannel, err := r.readVaruint()
	if err != nil {
		return nil, err
	}
	if frameChannel != channel {
		return nil, nil
	}

	length, err := r.readVaruint()
	if err != nil {
		return nil, err
	}
	if int(length) > r.remaining() {
		return nil, errShort
	}
	return append([]byte(nil), r.data[r.offset:r.offset+int(length)]...), nil
}

// settle waits for the console to go quiet, so that the next line read really
// is the answer to the command about to be written.
//
// Every console read is a read of "whatever the controller says next".  If
// anything is still queued — the tail of a previous command, telemetry, the
// remains of a read that timed out half way — that is what comes back, and it
// is attributed to the wrong question.  A quiet console beforehand is the only
// thing that makes the attribution sound, because `conf get` echoes no key to
// check the reply against.
//
// It reports false when the console would not go quiet inside SettleBudget.
func (s *Stream) settle(ctx context.Context) bool {
	return s.quieten(ctx, s.settleQuiet(), s.settleBudget())
}

// Drain discards whatever the console has buffered and waits for it to stop
// producing.
//
// This is not optional housekeeping.  A board that was emitting telemetry when
// the host connected keeps emitting into the same channel that carries command
// responses, so the first command's reply arrives interleaved with binary
// records.  Draining until quiet is what separates the two, and it is why
// Identify drains, stops telemetry, and drains again.
//
// It reports whether the console actually went quiet.  A false means the
// controller is still emitting more than DrainBudget's worth — a `conf
// enumerate` backlog runs to tens of seconds — and the caller should either
// call again, raise DrainBudget, or treat the console as unavailable rather
// than believe whatever it says next.
func (s *Stream) Drain(ctx context.Context) bool {
	return s.quieten(ctx, s.drainQuiet(), s.drainBudget())
}

// quieten polls until nothing has arrived for quiet, or budget runs out.
func (s *Stream) quieten(ctx context.Context, quiet, budget time.Duration) bool {
	s.buf = s.buf[:0]

	deadline := time.Now().Add(budget)
	lastData := time.Now()

	for time.Now().Before(deadline) {
		if ctx.Err() != nil {
			return false
		}
		chunk, err := s.poll(ctx)
		if err != nil {
			return false
		}
		if len(chunk) > 0 {
			lastData = time.Now()
			continue
		}
		if time.Since(lastData) > quiet {
			return true
		}
		time.Sleep(s.PollInterval)
	}
	return false
}

func (s *Stream) settleQuiet() time.Duration {
	if s.SettleQuiet <= 0 {
		return 30 * time.Millisecond
	}
	return s.SettleQuiet
}

func (s *Stream) settleBudget() time.Duration {
	if s.SettleBudget <= 0 {
		return time.Second
	}
	return s.SettleBudget
}

func (s *Stream) drainQuiet() time.Duration {
	if s.DrainQuiet <= 0 {
		return 150 * time.Millisecond
	}
	return s.DrainQuiet
}

func (s *Stream) drainBudget() time.Duration {
	if s.DrainBudget <= 0 {
		return 5 * time.Second
	}
	return s.DrainBudget
}

// FirmwareRecord is the controller's own description of itself.
//
// It is the only place the board family and hardware revision are available:
// there is no register for them.  The record is normally emitted as a binary
// telemetry frame, which would need the schema parser this driver deliberately
// does without — but the console can be asked to emit it as text instead, and
// then it is six lines of key and value.
type FirmwareRecord struct {
	Version int
	Model   int
	Family  int
	HWRev   int
}

// ReadFirmware asks the controller to describe itself.
//
// Switching the channel to text format is a runtime setting on an emission
// that is already stopped, so it disturbs nothing; what it buys is the
// difference between a board this framework can check against a known envelope
// and one it can only be told about.
func (s *Stream) ReadFirmware(ctx context.Context) (*FirmwareRecord, error) {
	if _, err := s.Command(ctx, "tel fmt firmware 1"); err != nil {
		return nil, err
	}

	// The record may or may not be terminated by an OK depending on firmware,
	// so whatever arrived is parsed either way — a partial answer that carries
	// the family is worth more than an error.
	lines, err := s.Command(ctx, "tel get firmware")
	record := parseFirmwareRecord(lines)
	if record == nil {
		if err != nil {
			return nil, err
		}
		return nil, fmt.Errorf("moteus: the firmware record carried no family or revision")
	}
	return record, nil
}

// parseFirmwareRecord reads the "firmware.<field> <value>" lines.
func parseFirmwareRecord(lines []string) *FirmwareRecord {
	record := &FirmwareRecord{Version: -1, Model: -1, Family: -1, HWRev: -1}
	seen := false

	for _, line := range lines {
		key, value, found := strings.Cut(strings.TrimSpace(line), " ")
		if !found {
			continue
		}
		number, err := strconv.Atoi(strings.TrimSpace(value))
		if err != nil {
			continue
		}
		switch key {
		case "firmware.version":
			record.Version, seen = number, true
		case "firmware.model":
			record.Model, seen = number, true
		case "firmware.family":
			record.Family, seen = number, true
		case "firmware.hwrev":
			record.HWRev, seen = number, true
		}
	}

	if !seen || (record.Family < 0 && record.HWRev < 0) {
		return nil
	}
	return record
}
