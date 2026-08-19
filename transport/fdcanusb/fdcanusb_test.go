package fdcanusb

import (
	"strings"
	"testing"
	"time"

	"drlabs/transport/can"
)

// fakePort is an in-memory serial port: it records what the driver writes and
// replays a scripted set of response lines.  The real port is non-blocking and
// returns (0, nil) when idle, so this one does too — that behaviour is what
// the line reader's polling loop is written against.
type fakePort struct {
	written strings.Builder
	replies []string
	closed  bool
}

func (p *fakePort) Read(b []byte) (int, error) {
	if len(p.replies) == 0 {
		return 0, nil
	}
	line := p.replies[0]
	p.replies = p.replies[1:]
	n := copy(b, line)
	return n, nil
}

func (p *fakePort) Write(b []byte) (int, error) {
	p.written.Write(b)
	return len(b), nil
}

func (p *fakePort) Close() error {
	p.closed = true
	return nil
}

func TestSendEncodesFrame(t *testing.T) {
	port := &fakePort{replies: []string{"OK\n"}}
	device := NewDevice(port, "fake")

	err := device.Send(can.Frame{
		ID:   0x8001,
		Data: []byte{0x01, 0x00, 0x0a},
		FD:   true,
		BRS:  true,
	})
	if err != nil {
		t.Fatalf("Send: %v", err)
	}

	const want = "can send 8001 01000A BF\n"
	if got := port.written.String(); got != want {
		t.Errorf("got %q, want %q", got, want)
	}
}

func TestSendPadsToAValidDLC(t *testing.T) {
	// CAN-FD has no byte-granular length above 8, so a 9-byte payload goes on
	// the wire as 12.  The padding is the multiplex NOP (0x50), so the
	// receiver skips it instead of reading it as register operations.
	port := &fakePort{replies: []string{"OK\n"}}
	device := NewDevice(port, "fake")

	if err := device.Send(can.Frame{
		ID:   0x8001,
		Data: []byte{1, 2, 3, 4, 5, 6, 7, 8, 9},
		FD:   true,
		BRS:  true,
	}); err != nil {
		t.Fatalf("Send: %v", err)
	}

	const want = "can send 8001 010203040506070809505050 BF\n"
	if got := port.written.String(); got != want {
		t.Errorf("got %q, want %q", got, want)
	}
}

func TestSendWithoutBitRateSwitch(t *testing.T) {
	port := &fakePort{replies: []string{"OK\n"}}
	device := NewDevice(port, "fake")
	device.DisableBRS = true

	if err := device.Send(can.Frame{ID: 1, Data: []byte{0x50}, FD: true, BRS: true}); err != nil {
		t.Fatalf("Send: %v", err)
	}
	if got := port.written.String(); !strings.HasSuffix(got, " bF\n") {
		t.Errorf("bit rate switch should be disabled, got %q", got)
	}
}

func TestRecvParsesFrame(t *testing.T) {
	port := &fakePort{replies: []string{"rcv 0100 2400010A BF\n"}}
	device := NewDevice(port, "fake")

	frame, err := device.Recv(time.Second)
	if err != nil {
		t.Fatalf("Recv: %v", err)
	}
	if frame.ID != 0x0100 {
		t.Errorf("id: got %#x, want 0x100", frame.ID)
	}
	if len(frame.Data) != 4 || frame.Data[0] != 0x24 {
		t.Errorf("data: got %#v", frame.Data)
	}
	if !frame.BRS || !frame.FD {
		t.Errorf("flags: got brs=%v fd=%v, want both true", frame.BRS, frame.FD)
	}
}

func TestSendKeepsAReplyThatOvertakesItsAcknowledgement(t *testing.T) {
	// The adapter can deliver a received frame before the "OK" for the frame
	// that provoked it.  Dropping it would lose one cycle of telemetry for no
	// reason, so it is queued and handed to the next Recv.
	port := &fakePort{replies: []string{"rcv 0100 2400 BF\n", "OK\n"}}
	device := NewDevice(port, "fake")

	if err := device.Send(can.Frame{ID: 0x8001, Data: []byte{0x11}, FD: true}); err != nil {
		t.Fatalf("Send: %v", err)
	}

	frame, err := device.Recv(time.Second)
	if err != nil {
		t.Fatalf("Recv: %v", err)
	}
	if frame.ID != 0x0100 {
		t.Errorf("the early reply was lost: got id %#x", frame.ID)
	}
}

func TestSendReportsAdapterErrors(t *testing.T) {
	port := &fakePort{replies: []string{"ERR bus off\n"}}
	device := NewDevice(port, "fake")

	err := device.Send(can.Frame{ID: 1, Data: []byte{0}, FD: true})
	if err == nil {
		t.Fatal("an ERR response should produce an error")
	}
	if !strings.Contains(err.Error(), "bus off") {
		t.Errorf("the adapter's own message should be preserved, got %v", err)
	}
}

func TestSendEnablesChecksumModeOnDemand(t *testing.T) {
	// The adapter can be configured to require line checksums.  It says so by
	// rejecting the first command, after which every line carries one — in
	// both directions, so the acknowledgement to the retry is checksummed too.
	port := &fakePort{replies: []string{
		"ERR checksum required\n",
		string(appendChecksum([]byte("OK"))) + "\n",
	}}
	device := NewDevice(port, "fake")

	if err := device.Send(can.Frame{ID: 0x8001, Data: []byte{0x11}, FD: true, BRS: true}); err != nil {
		t.Fatalf("Send should have retried with a checksum: %v", err)
	}

	lines := strings.Split(strings.TrimSuffix(port.written.String(), "\n"), "\n")
	if len(lines) != 2 {
		t.Fatalf("expected an original and a retry, got %d lines: %q", len(lines), lines)
	}
	if strings.Contains(lines[0], "*") {
		t.Error("the first attempt should not carry a checksum")
	}
	if !strings.Contains(lines[1], " *") {
		t.Errorf("the retry should carry a checksum, got %q", lines[1])
	}
}

func TestChecksumModeRejectsUncheckedLines(t *testing.T) {
	// Once the adapter has demanded checksums, a line arriving without one is
	// not trustworthy and must not be acted on.  Accepting it would defeat
	// the point of turning the mode on.
	port := &fakePort{replies: []string{"ERR checksum required\n", "OK\n"}}
	device := NewDevice(port, "fake")
	device.CommandTimeout = 50 * time.Millisecond

	if err := device.Send(can.Frame{ID: 1, Data: []byte{0x11}, FD: true}); err == nil {
		t.Error("an unchecksummed acknowledgement should not be accepted in checksum mode")
	}
}

func TestRecvTimesOut(t *testing.T) {
	port := &fakePort{}
	device := NewDevice(port, "fake")

	start := time.Now()
	_, err := device.Recv(50 * time.Millisecond)
	if err != can.ErrTimeout {
		t.Errorf("got %v, want ErrTimeout", err)
	}
	if elapsed := time.Since(start); elapsed < 40*time.Millisecond {
		t.Errorf("returned after %v, which is before the timeout", elapsed)
	}
}

func TestChecksumRoundTrip(t *testing.T) {
	line := []byte("can send 8001 01000A BF")
	withChecksum := appendChecksum(line)

	stripped, present, valid := stripChecksum(withChecksum)
	if !present {
		t.Fatal("the checksum should be detected")
	}
	if !valid {
		t.Error("a checksum we just computed should verify")
	}
	if string(stripped) != string(line) {
		t.Errorf("stripped to %q, want %q", stripped, line)
	}
}

func TestCorruptedChecksumIsRejected(t *testing.T) {
	line := appendChecksum([]byte("rcv 0100 2400"))
	line[len(line)-1] ^= 0x01 // flip a hex digit

	if _, present, valid := stripChecksum(line); !present || valid {
		t.Errorf("a corrupted checksum should be detected as invalid (present=%v valid=%v)", present, valid)
	}
}

func TestLineWithoutChecksumIsReportedAsAbsent(t *testing.T) {
	stripped, present, _ := stripChecksum([]byte("OK"))
	if present {
		t.Error("a line with no checksum should report absent")
	}
	if string(stripped) != "OK" {
		t.Errorf("the line should be returned unchanged, got %q", stripped)
	}
}

func TestRoundUpDLC(t *testing.T) {
	cases := map[int]int{0: 0, 1: 1, 8: 8, 9: 12, 12: 12, 13: 16, 21: 24, 33: 48, 64: 64}
	for size, want := range cases {
		if got := can.RoundUpDLC(size); got != want {
			t.Errorf("RoundUpDLC(%d): got %d, want %d", size, got, want)
		}
	}
}
