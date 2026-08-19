package robstride

import (
	"context"
	"encoding/binary"
	"math"
	"testing"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
)

// fakeBus is an in-memory CAN bus that records what was sent and replies with
// a scripted queue.  It is enough to check addressing, reply matching and the
// order of transactions, none of which a codec test can reach.
type fakeBus struct {
	sent    []can.Frame
	replies []can.Frame
	closed  bool
}

func (b *fakeBus) Open() error  { return nil }
func (b *fakeBus) Close() error { b.closed = true; return nil }

func (b *fakeBus) Send(frame can.Frame) error {
	copied := frame
	copied.Data = append([]byte(nil), frame.Data...)
	b.sent = append(b.sent, copied)
	return nil
}

func (b *fakeBus) Recv(time.Duration) (can.Frame, error) {
	if len(b.replies) == 0 {
		return can.Frame{}, can.ErrTimeout
	}
	frame := b.replies[0]
	b.replies = b.replies[1:]
	return frame, nil
}

func (b *fakeBus) Drain(time.Duration) error {
	b.replies = nil
	return nil
}

// feedbackFrame builds the reply a motor sends to almost every command.
func feedbackFrame(motorID, hostID uint8, mode Mode, flags uint16, angle, velocity, torque float64, temp float64, model Model) can.Frame {
	id := uint32(TypeFeedback)<<24 |
		uint32(mode)<<22 |
		uint32(flags)<<16 |
		uint32(motorID)<<8 |
		uint32(hostID)

	data := make([]byte, 8)
	binary.BigEndian.PutUint16(data[0:2], encodeUint16(angle, PositionRange))
	binary.BigEndian.PutUint16(data[2:4], encodeUint16(velocity, model.VelocityRange))
	binary.BigEndian.PutUint16(data[4:6], encodeUint16(torque, model.TorqueRange))
	binary.BigEndian.PutUint16(data[6:8], uint16(int16(math.Round(temp*10))))

	return can.Frame{ID: id, Data: data, Extended: true}
}

func newTestController(bus *fakeBus, model Model) *Controller {
	c := NewController(bus, 0x7F, model)
	c.HostID = 0xFD
	c.Timeout = 50 * time.Millisecond
	return c
}

func TestControllerSendsClassicCANFrames(t *testing.T) {
	// RobStride is CAN 2.0B, not CAN-FD.  An adapter told to send FD would
	// transmit a frame the motor cannot receive, and the symptom would be a
	// motor that never answers.
	bus := &fakeBus{}
	c := newTestController(bus, RS01)
	bus.replies = append(bus.replies, feedbackFrame(0x7F, 0xFD, ModeRun, 0, 0, 0, 0, 25, RS01))

	if _, err := c.Enable(context.Background()); err != nil {
		t.Fatalf("Enable: %v", err)
	}
	if len(bus.sent) != 1 {
		t.Fatalf("sent %d frames, want 1", len(bus.sent))
	}
	frame := bus.sent[0]
	if frame.FD || frame.BRS {
		t.Errorf("frame was marked CAN-FD (fd=%v brs=%v)", frame.FD, frame.BRS)
	}
	if !frame.Extended {
		t.Error("frame was not marked as an extended identifier")
	}
	if len(frame.Data) != 8 {
		t.Errorf("payload is %d bytes, want 8", len(frame.Data))
	}
}

func TestControllerIgnoresOtherMotorsTraffic(t *testing.T) {
	// On a shared bus the reply queue contains everyone's frames.  Accepting
	// the wrong one would attribute another actuator's position to this one.
	bus := &fakeBus{}
	c := newTestController(bus, RS01)

	bus.replies = append(bus.replies,
		feedbackFrame(0x02, 0xFD, ModeRun, 0, 1.0, 0, 0, 90, RS01), // another motor
		feedbackFrame(0x7F, 0xAA, ModeRun, 0, 2.0, 0, 0, 90, RS01), // another host
		feedbackFrame(0x7F, 0xFD, ModeRun, 0, 0.5, 0, 0, 25, RS01), // ours
	)

	feedback, err := c.Enable(context.Background())
	if err != nil {
		t.Fatalf("Enable: %v", err)
	}
	if feedback == nil {
		t.Fatal("no feedback returned")
	}
	if math.Abs(feedback.Angle-0.5) > 0.01 {
		t.Errorf("angle: got %v, want 0.5 — a frame from another device was accepted", feedback.Angle)
	}
	if math.Abs(feedback.Temperature-25) > 0.05 {
		t.Errorf("temperature: got %v, want 25", feedback.Temperature)
	}
}

func TestControllerReportsATimeoutAsADroppedCycle(t *testing.T) {
	// One missed reply is normal on a noisy bus.  It has to come back as "no
	// data" rather than as an error, or a control loop would tear the link
	// down over a single dropped frame.
	bus := &fakeBus{}
	c := newTestController(bus, RS01)

	feedback, err := c.Enable(context.Background())
	if err != nil {
		t.Fatalf("a timeout should not be an error, got %v", err)
	}
	if feedback != nil {
		t.Error("a timeout should return no feedback")
	}
}

func TestStopCanClearFaults(t *testing.T) {
	bus := &fakeBus{}
	c := newTestController(bus, RS01)
	bus.replies = append(bus.replies,
		feedbackFrame(0x7F, 0xFD, ModeReset, 0, 0, 0, 0, 25, RS01),
		feedbackFrame(0x7F, 0xFD, ModeReset, 0, 0, 0, 0, 25, RS01))

	if _, err := c.Stop(context.Background(), false); err != nil {
		t.Fatal(err)
	}
	if got := bus.sent[0].Data[0]; got != 0 {
		t.Errorf("a plain stop set the fault-clear byte to %d", got)
	}

	if _, err := c.Stop(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	if got := bus.sent[1].Data[0]; got != 1 {
		t.Errorf("a fault-clearing stop left the byte at %d, want 1", got)
	}
}

func TestReadParameterRejectsAMismatchedReply(t *testing.T) {
	// If the motor answers with a different index, the value belongs to
	// another parameter.  Using it would put, say, a bus voltage into a
	// torque limit.
	bus := &fakeBus{}
	c := newTestController(bus, RS01)

	data := make([]byte, 8)
	binary.LittleEndian.PutUint16(data[0:2], uint16(ParamVBus))
	binary.LittleEndian.PutUint32(data[4:8], math.Float32bits(48.0))
	bus.replies = append(bus.replies, can.Frame{
		ID:   uint32(TypeReadParam)<<24 | 0x7F<<8 | 0xFD,
		Data: data,
	})

	if _, _, err := c.ReadParameter(context.Background(), ParamLimitTorque); err == nil {
		t.Error("a reply for the wrong parameter should be rejected")
	}
}

func TestReadParameterDecodesAFloat(t *testing.T) {
	bus := &fakeBus{}
	c := newTestController(bus, RS01)

	data := make([]byte, 8)
	binary.LittleEndian.PutUint16(data[0:2], uint16(ParamVBus))
	binary.LittleEndian.PutUint32(data[4:8], math.Float32bits(47.5))
	bus.replies = append(bus.replies, can.Frame{
		ID:   uint32(TypeReadParam)<<24 | 0x7F<<8 | 0xFD,
		Data: data,
	})

	value, ok, err := c.ReadParameter(context.Background(), ParamVBus)
	if err != nil || !ok {
		t.Fatalf("ReadParameter: value=%v ok=%v err=%v", value, ok, err)
	}
	if math.Abs(value-47.5) > 1e-5 {
		t.Errorf("got %v, want 47.5", value)
	}
}

func TestSetCANIDFollowsTheMotorToItsNewAddress(t *testing.T) {
	// The change takes effect immediately, so a Controller that kept talking
	// to the old address would be talking to nothing.
	bus := &fakeBus{}
	c := newTestController(bus, RS01)
	bus.replies = append(bus.replies, can.Frame{
		ID:   uint32(TypeGetDeviceID)<<24 | 0x05<<8 | uint32(broadcastReplyAddress),
		Data: make([]byte, 8),
	})

	if err := c.SetCANID(context.Background(), 0x05); err != nil {
		t.Fatalf("SetCANID: %v", err)
	}
	if c.ID != 0x05 {
		t.Errorf("controller still addresses %#02x, want 0x05", c.ID)
	}

	// The new address travels in the high byte of data area 2.
	_, dataArea2, target := parseArbitrationID(bus.sent[0].ID)
	if target != 0x7F {
		t.Errorf("the request went to %#02x, want the old address 0x7F", target)
	}
	if uint8(dataArea2>>8) != 0x05 {
		t.Errorf("new id field: got %#02x, want 0x05", uint8(dataArea2>>8))
	}
	if uint8(dataArea2&0xff) != 0xFD {
		t.Errorf("host id field: got %#02x, want 0xFD", uint8(dataArea2&0xff))
	}
}

func TestSetCANIDRejectsAnUnusableAddress(t *testing.T) {
	c := newTestController(&fakeBus{}, RS01)
	if err := c.SetCANID(context.Background(), 0); err == nil {
		t.Error("address 0 should be rejected")
	}
	if err := c.SetCANID(context.Background(), 200); err == nil {
		t.Error("an address above 127 should be rejected")
	}
}

func TestControllerRefusesToUseAClosedBus(t *testing.T) {
	c := NewController(nil, 1, RS01)
	if _, err := c.Enable(context.Background()); err != motor.ErrClosed {
		t.Errorf("got %v, want ErrClosed", err)
	}
}
