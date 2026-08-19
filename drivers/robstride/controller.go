package robstride

import (
	"context"
	"encoding/hex"
	"fmt"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
)

// Controller is one RobStride actuator on a CAN bus.
//
// It owns framing and addressing, not timing: a caller drives it from its own
// control loop, and every method performs one bus transaction and returns.
type Controller struct {
	// ID is the motor's CAN address, 1..127.
	ID uint8

	// HostID is the address this host answers to.  Replies are addressed back
	// to it, so two programs sharing a bus need different values.
	HostID uint8

	// Model supplies the per-variant scaling.  It is not optional: the wrong
	// model mis-scales every torque command and every torque reading.
	Model Model

	// Bus is the transport.  It is not owned; Close does not close it.
	Bus can.Bus

	// Timeout bounds a single transaction.  RobStride replies to a command
	// within a millisecond or so on a healthy 1 Mbit/s bus; this is generous
	// enough to survive an adapter hiccup and short enough that a wedged bus
	// is noticed within one control period at modest rates.
	Timeout time.Duration
}

// DefaultTimeout is the transaction timeout a Controller starts with.
const DefaultTimeout = 100 * time.Millisecond

// NewController returns a Controller for the given address on bus.
func NewController(bus can.Bus, id uint8, model Model) *Controller {
	return &Controller{
		ID:      id,
		HostID:  DefaultHostID,
		Model:   model,
		Bus:     bus,
		Timeout: DefaultTimeout,
	}
}

func (c *Controller) timeout() time.Duration {
	if c.Timeout <= 0 {
		return DefaultTimeout
	}
	return c.Timeout
}

// send transmits one frame and waits for a reply that accept recognises.
//
// A nil reply with a nil error means nothing answered in time — a dropped
// cycle rather than a transport failure, which is what a noisy bus produces
// occasionally and what a control loop should ride out.
func (c *Controller) send(ctx context.Context, id uint32, data []byte, accept func(uint32, []byte) bool) (uint32, []byte, error) {
	if c.Bus == nil {
		return 0, nil, motor.ErrClosed
	}

	// A send and its reply are one transaction.  On a bus shared with other
	// controllers, letting them interleave means each reads the other's
	// answer, discards it as not its own, and both time out — so the pair is
	// held together.  A bus with a single controller offers no lock and pays
	// nothing for this.
	if locker, shared := c.Bus.(can.Locker); shared {
		locker.Lock()
		defer locker.Unlock()
	}

	payload := make([]byte, 8)
	copy(payload, data)

	frame := can.Frame{
		ID:       id,
		Data:     payload,
		Extended: true,
		// RobStride speaks classic CAN 2.0B, not CAN-FD.  Setting the FD flag
		// would make an adapter transmit a frame the motor cannot receive.
		FD:  false,
		BRS: false,
	}

	if err := c.Bus.Send(frame); err != nil {
		return 0, nil, &motor.LinkError{Op: "send", Err: err}
	}
	if accept == nil {
		return 0, nil, nil
	}

	deadline := time.Now().Add(c.timeout())
	if ctxDeadline, ok := ctx.Deadline(); ok && ctxDeadline.Before(deadline) {
		deadline = ctxDeadline
	}

	for {
		remaining := time.Until(deadline)
		if remaining <= 0 {
			return 0, nil, nil
		}
		if err := ctx.Err(); err != nil {
			return 0, nil, err
		}

		reply, err := c.Bus.Recv(remaining)
		if err == can.ErrTimeout {
			return 0, nil, nil
		}
		if err != nil {
			return 0, nil, &motor.LinkError{Op: "receive", Err: err}
		}
		if accept(reply.ID, reply.Data) {
			return reply.ID, reply.Data, nil
		}
		// Somebody else's traffic on a shared bus.  Discarding it is the
		// point: decoding it would attribute another motor's state to this one.
	}
}

// expectFeedback accepts the type-2 frame this motor sends in reply to almost
// every command.
func (c *Controller) expectFeedback(id uint32, data []byte) bool {
	commType, dataArea2, address := parseArbitrationID(id)
	return commType == TypeFeedback &&
		uint8(dataArea2&0xff) == c.ID &&
		address == c.HostID &&
		len(data) >= 8
}

// expectType accepts a reply of a particular type from this motor.
func (c *Controller) expectType(want CommunicationType) func(uint32, []byte) bool {
	return func(id uint32, data []byte) bool {
		commType, dataArea2, address := parseArbitrationID(id)
		return commType == want &&
			uint8(dataArea2&0xff) == c.ID &&
			address == c.HostID &&
			len(data) >= 8
	}
}

// feedbackFrom decodes a reply that has already been accepted.
func (c *Controller) feedbackFrom(id uint32, data []byte) (*Feedback, error) {
	if data == nil {
		return nil, nil
	}
	feedback, err := decodeFeedback(id, data, c.Model)
	if err != nil {
		return nil, err
	}
	return &feedback, nil
}

// Enable puts the motor into run mode, after which it acts on commands.
func (c *Controller) Enable(ctx context.Context) (*Feedback, error) {
	id, data, err := c.send(ctx,
		arbitrationID(TypeEnable, uint16(c.HostID), c.ID),
		nil, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(id, data)
}

// Stop takes the motor out of run mode, releasing the output.
//
// With clearFaults set it also clears latched faults, which is the documented
// way to recover from one.  Stopping is the safest state the motor has, and it
// still answers with a full feedback frame — which is why a disarmed session
// can poll with it rather than having to command something.
func (c *Controller) Stop(ctx context.Context, clearFaults bool) (*Feedback, error) {
	data := make([]byte, 8)
	if clearFaults {
		data[0] = 1
	}
	id, reply, err := c.send(ctx,
		arbitrationID(TypeStop, uint16(c.HostID), c.ID),
		data, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(id, reply)
}

// Control issues one operation-mode command.
func (c *Controller) Control(ctx context.Context, cmd ControlCommand) (*Feedback, error) {
	id, data := encodeControl(cmd, c.Model, c.ID)
	replyID, reply, err := c.send(ctx, id, data, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(replyID, reply)
}

// SetMechanicalZero declares the current shaft position to be zero.
//
// Runtime only unless committed with SaveParameters.  The motor must be
// stopped: re-defining where the axis thinks it is while it is being driven
// would move the machine by the size of the correction.
func (c *Controller) SetMechanicalZero(ctx context.Context) (*Feedback, error) {
	data := make([]byte, 8)
	data[0] = 1
	id, reply, err := c.send(ctx,
		arbitrationID(TypeSetZero, uint16(c.HostID), c.ID),
		data, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(id, reply)
}

// WriteParameter sets one parameter.  The change is volatile until
// SaveParameters commits it.
func (c *Controller) WriteParameter(ctx context.Context, index Parameter, value float64) (*Feedback, error) {
	data, err := encodeParamWrite(index, value)
	if err != nil {
		return nil, err
	}
	id, reply, err := c.send(ctx,
		arbitrationID(TypeWriteParam, uint16(c.HostID), c.ID),
		data, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(id, reply)
}

// ReadParameter reads one parameter.
//
// A nil-safe zero with ok=false means the motor did not answer this period.
func (c *Controller) ReadParameter(ctx context.Context, index Parameter) (value float64, ok bool, err error) {
	_, reply, err := c.send(ctx,
		arbitrationID(TypeReadParam, uint16(c.HostID), c.ID),
		encodeParamRead(index), c.expectType(TypeReadParam))
	if err != nil || reply == nil {
		return 0, false, err
	}

	got, value, err := decodeParamReply(reply)
	if err != nil {
		return 0, false, err
	}
	if got != index {
		return 0, false, fmt.Errorf(
			"robstride: asked for parameter %s, motor answered with %s", index, got)
	}
	return value, true, nil
}

// SetRunMode selects which control loop is in charge.
func (c *Controller) SetRunMode(ctx context.Context, mode RunMode) (*Feedback, error) {
	return c.WriteParameter(ctx, ParamRunMode, float64(mode))
}

// SaveParameters commits the volatile parameter set to non-volatile storage.
//
// It is deliberately not called by the control loop.  Writing flash on a motor
// that is being driven is a good way to produce an axis whose stored
// configuration reflects a transient, and the wear budget is finite.
func (c *Controller) SaveParameters(ctx context.Context) (*Feedback, error) {
	data := []byte{1, 2, 3, 4, 5, 6, 7, 8}
	id, reply, err := c.send(ctx,
		arbitrationID(TypeSaveParams, uint16(c.HostID), c.ID),
		data, c.expectFeedback)
	if err != nil {
		return nil, err
	}
	return c.feedbackFrom(id, reply)
}

// DeviceID reads the motor's 64-bit MCU unique identifier.
//
// The reply is addressed to the fixed broadcast address rather than to this
// host, so it needs its own acceptance rule.
func (c *Controller) DeviceID(ctx context.Context) (string, error) {
	accept := func(id uint32, data []byte) bool {
		commType, dataArea2, address := parseArbitrationID(id)
		return commType == TypeGetDeviceID &&
			uint8(dataArea2&0xff) == c.ID &&
			address == broadcastReplyAddress &&
			len(data) >= 8
	}

	_, reply, err := c.send(ctx,
		arbitrationID(TypeGetDeviceID, uint16(c.HostID), c.ID),
		nil, accept)
	if err != nil || reply == nil {
		return "", err
	}
	return hex.EncodeToString(reply[:8]), nil
}

// SetCANID changes the motor's own address, effective immediately.
//
// The motor answers at its *new* address from that moment, so the Controller's
// ID is updated on success — otherwise every subsequent call would be talking
// to an address that no longer exists.
func (c *Controller) SetCANID(ctx context.Context, newID uint8) error {
	if newID < 1 || newID > 127 {
		return fmt.Errorf("robstride: CAN id %d is out of range (1..127)", newID)
	}

	// Data area 2 carries the host id in its low byte and the new address in
	// its high byte.
	dataArea2 := uint16(c.HostID) | uint16(newID)<<8

	accept := func(id uint32, data []byte) bool {
		commType, _, address := parseArbitrationID(id)
		return commType == TypeGetDeviceID && address == broadcastReplyAddress
	}
	if _, _, err := c.send(ctx,
		arbitrationID(TypeSetCANID, dataArea2, c.ID),
		nil, accept); err != nil {
		return err
	}

	c.ID = newID
	return nil
}

// FaultReport reads the detailed fault and warning words.
//
// A feedback frame carries only six summary bits; this asks for the full
// picture, which is what distinguishes "over current" from which phase.
func (c *Controller) FaultReport(ctx context.Context) (*FaultReport, error) {
	_, reply, err := c.send(ctx,
		arbitrationID(TypeFaultFeedback, uint16(c.HostID), c.ID),
		nil, c.expectType(TypeFaultFeedback))
	if err != nil || reply == nil {
		return nil, err
	}
	report, err := decodeFaultReport(reply)
	if err != nil {
		return nil, err
	}
	return &report, nil
}

// SetActiveReporting turns the motor's unsolicited 10 ms feedback stream on or
// off.
//
// This driver polls instead, because a shared bus with several motors all
// reporting on their own schedule is bandwidth spent whether or not anyone is
// listening.  The control is here for callers who want it.
func (c *Controller) SetActiveReporting(ctx context.Context, enabled bool) error {
	data := []byte{1, 2, 3, 4, 5, 6, 0, 0}
	if enabled {
		data[6] = 1
	}
	_, _, err := c.send(ctx,
		arbitrationID(TypeSetReporting, uint16(c.HostID), c.ID),
		data, nil)
	return err
}

// String identifies the controller in logs.
func (c *Controller) String() string {
	return fmt.Sprintf("robstride[%s id=%d]", c.Model.Name, c.ID)
}
