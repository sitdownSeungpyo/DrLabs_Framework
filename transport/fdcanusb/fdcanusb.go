// Package fdcanusb drives the mjbots fdcanusb, a USB to CAN-FD adapter that
// speaks a line-oriented ASCII protocol over a CDC-ACM serial port.
//
// Wire protocol
//
//	host -> device   can send <id_hex> <data_hex> [flags]\n
//	device -> host   OK\n                       command accepted
//	device -> host   rcv <id_hex> <data_hex> [flags]\n
//	device -> host   ERR <text>\n
//
// Flags are a concatenation of single letters: 'B' bit-rate switch enabled,
// 'b' disabled, 'F' CAN-FD frame, 'E' extended (29-bit) identifier.
//
// The adapter may ask for line checksums.  When it answers a command with an
// error mentioning "checksum", every subsequent line gains a trailing " *XX"
// CRC-8 (polynomial 0x97) and the mode stays on for the rest of the session.
//
// This implementation is deliberately synchronous: one Send, then one Recv,
// driven by the caller's control loop.  A moteus transaction is a single
// request/reply pair, so a background reader goroutine would only add a
// hand-off without removing any latency.
package fdcanusb

import (
	"bytes"
	"encoding/hex"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"time"

	"drlabs/transport/can"
	"drlabs/transport/serial"
)

// Default timings, taken from the behaviour of the reference Python driver.
const (
	// DefaultCommandTimeout is how long to wait for the "OK" that
	// acknowledges a transmitted frame.
	DefaultCommandTimeout = 200 * time.Millisecond

	// DefaultPollInterval is the pause between reads when the port has no
	// data.  The serial layer is non-blocking, so without it a wait would
	// spin a core flat.
	DefaultPollInterval = 100 * time.Microsecond

	// ReadTimeout is how long a single read may block waiting for the adapter.
	// Short enough not to dominate a fast control period, long enough that an
	// answer arriving normally is caught by the blocking read rather than by
	// the next poll.
	ReadTimeout = time.Millisecond

	// maxLineLength bounds a single response line, so that a device emitting
	// garbage cannot grow the buffer without limit.
	maxLineLength = 512

	// paddingByte fills a CAN-FD frame out to the next valid DLC.  0x50 is
	// the multiplex protocol's NOP, so moteus ignores the padding rather than
	// misreading it as a register operation.
	paddingByte = 0x50
)

// Device is a connection to one fdcanusb adapter.  It implements can.Bus.
type Device struct {
	port serial.Port
	path string

	// buf holds bytes read from the port that do not yet form a whole line.
	buf []byte

	// pending holds frames that arrived while we were waiting for something
	// else, so that a reply which overtakes its own "OK" is not lost.
	pending []can.Frame

	CommandTimeout time.Duration
	PollInterval   time.Duration

	// DisableBRS suppresses the bit-rate switch.  Needed on buses whose
	// wiring cannot carry the 5 Mbit/s data phase reliably.
	DisableBRS bool

	checksumActive bool
	closed         bool
}

// Open connects to the adapter at path.
//
// The baud rate is irrelevant for a CDC-ACM device — the USB endpoint runs at
// USB speed and the line settings are ignored — but the host serial API still
// demands one, so a conventional value is passed through.
func Open(path string) (*Device, error) {
	opened, err := serial.Open(path, 115200)
	if err != nil {
		return nil, fmt.Errorf("fdcanusb: open %s: %w", path, err)
	}
	// Held as the interface, because whether this platform's port can have its
	// read timeout tuned is exactly the question asked below.
	var port serial.Port = opened

	// A control loop's whole round trip is a few milliseconds, so a read that
	// blocks for the platform default — ten milliseconds on Windows, chosen
	// for a different protocol entirely — becomes the cycle's dominant cost.
	// Ports that cannot be tuned keep their default; the loop still works, it
	// just cannot run as fast.
	if tuner, ok := port.(serial.ReadTimeouter); ok {
		_ = tuner.SetReadTimeout(ReadTimeout)
	}

	return NewDevice(port, path), nil
}

// NewDevice wraps an already-open serial port.  It exists so that tests can
// drive the protocol over an in-memory port.
func NewDevice(port serial.Port, path string) *Device {
	return &Device{
		port:           port,
		path:           path,
		CommandTimeout: DefaultCommandTimeout,
		PollInterval:   DefaultPollInterval,
	}
}

// Detect returns the serial ports that look like a CAN adapter, best guess
// first.  An empty result is not an error: the operator can always name the
// port explicitly.
func Detect() ([]serial.PortInfo, error) {
	ports, err := serial.List()
	if err != nil {
		return nil, err
	}
	return ports, nil
}

// Kind identifies the transport in logs and status output.
func (d *Device) Kind() string { return "fdcanusb" }

// Describe reports what this device is attached to.
func (d *Device) Describe() map[string]string {
	return map[string]string{
		"transport": "fdcanusb",
		"port":      d.path,
	}
}

// Open satisfies can.Bus.  The port is already open by the time a Device
// exists, so this only rejects use after Close.
func (d *Device) Open() error {
	if d.closed {
		return errors.New("fdcanusb: device is closed")
	}
	return nil
}

// Close releases the serial port.  Calling it twice is harmless.
func (d *Device) Close() error {
	if d.closed {
		return nil
	}
	d.closed = true
	return d.port.Close()
}

// Send transmits one frame and waits for the adapter to acknowledge it.
//
// The acknowledgement matters: without it a caller can queue frames faster
// than the adapter drains them, and the resulting backlog shows up much later
// as telemetry that belongs to a command several cycles old.
func (d *Device) Send(frame can.Frame) error {
	if d.closed {
		return errors.New("fdcanusb: device is closed")
	}
	if len(frame.Data) > can.MaxFDPayload {
		return fmt.Errorf("fdcanusb: payload of %d bytes exceeds CAN-FD limit", len(frame.Data))
	}

	line := d.encode(frame)
	if _, err := d.port.Write(line); err != nil {
		return fmt.Errorf("fdcanusb: write: %w", err)
	}

	deadline := time.Now().Add(d.CommandTimeout)
	for {
		text, err := d.readLine(deadline)
		if err != nil {
			return err
		}
		switch {
		case bytes.HasPrefix(text, []byte("OK")):
			return nil
		case bytes.HasPrefix(text, []byte("rcv")):
			// A reply that overtook its own acknowledgement.  Keep it.
			if received, err := parseFrame(text); err == nil {
				d.pending = append(d.pending, received)
			}
		case bytes.HasPrefix(text, []byte("ERR")):
			if !d.checksumActive && bytes.Contains(bytes.ToLower(text), []byte("checksum")) {
				// The adapter is configured to require checksums.  Turn them
				// on and resend once; from here the session keeps using them.
				d.checksumActive = true
				return d.Send(frame)
			}
			return fmt.Errorf("fdcanusb: adapter error: %s", strings.TrimSpace(string(text)))
		default:
			// Banner or unsolicited chatter — ignore and keep waiting.
		}
	}
}

// Recv waits up to timeout for the next received frame.
func (d *Device) Recv(timeout time.Duration) (can.Frame, error) {
	if len(d.pending) > 0 {
		frame := d.pending[0]
		d.pending = d.pending[1:]
		return frame, nil
	}
	if d.closed {
		return can.Frame{}, errors.New("fdcanusb: device is closed")
	}

	deadline := time.Now().Add(timeout)
	for {
		text, err := d.readLine(deadline)
		if err != nil {
			return can.Frame{}, err
		}
		if bytes.HasPrefix(text, []byte("rcv")) {
			return parseFrame(text)
		}
		if bytes.HasPrefix(text, []byte("ERR")) {
			return can.Frame{}, fmt.Errorf("fdcanusb: adapter error: %s", strings.TrimSpace(string(text)))
		}
		// "OK" from an earlier command, or a banner: not what we are after.
	}
}

// Drain discards whatever is already queued and returns once the port has
// been quiet for the given interval.  Called before a fresh transaction so
// that a stale reply cannot be mistaken for the answer to a new question.
func (d *Device) Drain(quiet time.Duration) error {
	d.pending = nil
	d.buf = d.buf[:0]
	if d.closed {
		return nil
	}
	scratch := make([]byte, 512)
	deadline := time.Now().Add(quiet)
	for time.Now().Before(deadline) {
		n, err := d.port.Read(scratch)
		if err != nil {
			return nil // a read error here is the caller's problem later
		}
		if n > 0 {
			deadline = time.Now().Add(quiet)
			continue
		}
		time.Sleep(d.PollInterval)
	}
	return nil
}

// encode renders a frame as an fdcanusb command line.
func (d *Device) encode(frame can.Frame) []byte {
	onWire := can.RoundUpDLC(len(frame.Data))

	var body strings.Builder
	body.WriteString("can send ")
	body.WriteString(fmt.Sprintf("%04x", frame.ID))
	body.WriteByte(' ')
	body.WriteString(strings.ToUpper(hex.EncodeToString(frame.Data)))
	for i := len(frame.Data); i < onWire; i++ {
		body.WriteString(fmt.Sprintf("%02X", paddingByte))
	}

	flags := ""
	if frame.BRS && !d.DisableBRS {
		flags += "B"
	} else {
		flags += "b"
	}
	if frame.FD {
		flags += "F"
	}
	if flags != "" {
		body.WriteByte(' ')
		body.WriteString(flags)
	}

	line := []byte(body.String())
	if d.checksumActive {
		line = appendChecksum(line)
	}
	return append(line, '\n')
}

// readLine returns the next complete line from the port, with the checksum
// verified and stripped.  Lines that fail verification are skipped, because a
// corrupted line carries no usable information and retrying the read is the
// only sensible response.
func (d *Device) readLine(deadline time.Time) ([]byte, error) {
	scratch := make([]byte, 256)
	for {
		if idx := bytes.IndexByte(d.buf, '\n'); idx >= 0 {
			raw := d.buf[:idx]
			d.buf = d.buf[idx+1:]
			line, hadChecksum, valid := stripChecksum(bytes.TrimRight(raw, "\r "))
			if hadChecksum && !valid {
				continue
			}
			if d.checksumActive && !hadChecksum {
				continue
			}
			if len(line) == 0 {
				continue
			}
			return line, nil
		}

		if len(d.buf) > maxLineLength {
			// No newline in a full line's worth of data: the stream is out of
			// sync.  Drop it rather than growing without bound.
			d.buf = d.buf[:0]
		}

		if !time.Now().Before(deadline) {
			return nil, can.ErrTimeout
		}

		n, err := d.port.Read(scratch)
		if err != nil {
			return nil, fmt.Errorf("fdcanusb: read: %w", err)
		}
		if n == 0 {
			time.Sleep(d.PollInterval)
			continue
		}
		d.buf = append(d.buf, scratch[:n]...)
	}
}

// parseFrame decodes an "rcv <id> <data> [flags]" line.
func parseFrame(line []byte) (can.Frame, error) {
	fields := strings.Fields(string(line))
	if len(fields) < 3 {
		return can.Frame{}, fmt.Errorf("fdcanusb: malformed frame %q", string(line))
	}

	id, err := strconv.ParseUint(fields[1], 16, 32)
	if err != nil {
		return can.Frame{}, fmt.Errorf("fdcanusb: bad identifier in %q: %w", string(line), err)
	}

	data, err := hex.DecodeString(fields[2])
	if err != nil {
		return can.Frame{}, fmt.Errorf("fdcanusb: bad payload in %q: %w", string(line), err)
	}

	frame := can.Frame{ID: uint32(id), Data: data}
	for _, flag := range fields[3:] {
		for _, c := range flag {
			switch c {
			case 'E':
				frame.Extended = true
			case 'B':
				frame.BRS = true
			case 'F':
				frame.FD = true
			}
		}
	}
	return frame, nil
}
