package moteus

import (
	"context"
	"fmt"
	"math"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
)

// Controller is one moteus on a CAN bus.
//
// It owns the framing, not the timing: a caller drives it from its own control
// loop.  Every method performs one bus transaction and returns.
type Controller struct {
	// ID is the controller's CAN identifier, 1..126.
	ID uint8

	// SourceID is the identifier this host answers to.  Replies are addressed
	// back to it, so leaving it at 0 is right unless several hosts share a bus.
	SourceID uint8

	// CANPrefix is an optional 13-bit prefix that segments one physical bus
	// into several logical ones.
	CANPrefix uint16

	// Bus is the transport.  It is not owned: Close does not close it.
	Bus can.Bus

	// PositionResolution controls how wide each command field is on the wire.
	PositionResolution PositionResolution

	// Timeout bounds a single transaction.
	//
	// It is deliberately generous.  Cutting a transaction short because the
	// control period elapsed leaves the adapter's pending-request state
	// inconsistent, after which every following transaction returns empty —
	// a 9 ms cap on a 100 Hz loop turns a working link into a total loss.
	// The transport has its own retries; this only catches a wedged adapter,
	// and when it fires the link should be torn down rather than reused.
	Timeout time.Duration
}

// DefaultTimeout is the transaction timeout a Controller starts with.
const DefaultTimeout = 2 * time.Second

// NewController returns a Controller for the given CAN id on bus.
func NewController(bus can.Bus, id uint8) *Controller {
	return &Controller{
		ID:                 id,
		Bus:                bus,
		PositionResolution: DefaultPositionResolution(),
		Timeout:            DefaultTimeout,
	}
}

// arbitrationID builds the CAN identifier for a frame to this controller.
//
//	bits 0-7    destination id
//	bit  15     set when a reply is required
//	bits 8-14   source id
//	bits 16-28  optional bus prefix
func (c *Controller) arbitrationID(replyRequired bool) uint32 {
	id := uint32(c.ID) | uint32(c.SourceID)<<8 | uint32(c.CANPrefix)<<16
	if replyRequired {
		id |= 0x8000
	}
	return id
}

// transact sends one payload and decodes the register values that come back.
func (c *Controller) transact(ctx context.Context, payload []byte, expectReply bool) (map[Register]regValue, error) {
	reply, err := c.exchange(ctx, payload, expectReply, isRegisterReply)
	if err != nil || reply == nil {
		return nil, err
	}
	return parseReply(reply)
}

// isRegisterReply accepts the frames that answer a register request: a reply
// subframe, or the read-error subframe the controller sends when it refuses.
func isRegisterReply(data []byte) bool {
	return data[0]&0xf0 == replyBase || data[0] == readError
}

// exchange sends one payload and, when a reply is expected, waits for a frame
// this controller sent that accept recognises.
//
// Frames from other controllers on a shared bus are discarded rather than
// misread: the reply's source field has to match the controller we addressed.
// A nil result with a nil error means nothing answered in time, which is a
// missed cycle rather than a transport failure — one dropped reply on a noisy
// bus is normal and the control loop should keep running.
func (c *Controller) exchange(ctx context.Context, payload []byte, expectReply bool, accept func([]byte) bool) ([]byte, error) {
	if c.Bus == nil {
		return nil, motor.ErrClosed
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

	frame := can.Frame{
		ID:       c.arbitrationID(expectReply),
		Data:     payload,
		Extended: c.arbitrationID(expectReply) > 0x7ff,
		FD:       true,
		BRS:      true,
	}

	if err := c.Bus.Send(frame); err != nil {
		return nil, &motor.LinkError{Op: "send", Err: err}
	}
	if !expectReply {
		return nil, nil
	}

	deadline := time.Now().Add(c.timeout())
	if ctxDeadline, ok := ctx.Deadline(); ok && ctxDeadline.Before(deadline) {
		deadline = ctxDeadline
	}

	for {
		remaining := time.Until(deadline)
		if remaining <= 0 {
			return nil, nil
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}

		reply, err := c.Bus.Recv(remaining)
		if err == can.ErrTimeout {
			return nil, nil
		}
		if err != nil {
			return nil, &motor.LinkError{Op: "receive", Err: err}
		}

		// The controller answers with its own id in the source field.
		if uint8(reply.ID>>8)&0x7f != c.ID {
			continue
		}
		if len(reply.Data) == 0 || !accept(reply.Data) {
			continue
		}
		return reply.Data, nil
	}
}

func (c *Controller) timeout() time.Duration {
	if c.Timeout <= 0 {
		return DefaultTimeout
	}
	return c.Timeout
}

// Query reads telemetry without changing the controller's mode.
func (c *Controller) Query(ctx context.Context, q QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, queryFrame(&q), true)
}

// Stop releases the bridge.  It is the safest state the controller has and is
// what every error path falls back to.
func (c *Controller) Stop(ctx context.Context, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, stopFrame(q), q != nil)
}

// Brake shorts the phases for a passive stop.
func (c *Controller) Brake(ctx context.Context, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, brakeFrame(q), q != nil)
}

// ZeroVelocity holds zero velocity without holding a position target.
func (c *Controller) ZeroVelocity(ctx context.Context, kdScale float64, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, zeroVelocityFrame(kdScale, c.PositionResolution, q), q != nil)
}

// Position issues a position-mode command.
func (c *Controller) Position(ctx context.Context, cmd positionCommand, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, positionFrame(cmd, c.PositionResolution, q), q != nil)
}

// StayWithin issues a stay-within command.
func (c *Controller) StayWithin(ctx context.Context, cmd stayWithinCommand, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, stayWithinFrame(cmd, c.PositionResolution, q), q != nil)
}

// SetOutputExact re-references the output position to the given value.
func (c *Controller) SetOutputExact(ctx context.Context, position float64, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, setOutputFrame(RegSetOutputExact, position, q), q != nil)
}

// SetOutputNearest re-references the output to the nearest equivalent of the
// given position, which preserves the shaft's sub-revolution phase.
func (c *Controller) SetOutputNearest(ctx context.Context, position float64, q *QueryResolution) (map[Register]regValue, error) {
	return c.transact(ctx, setOutputFrame(RegSetOutputNearest, position, q), q != nil)
}

// ReadRegisters reads an arbitrary set of registers, scaled into physical
// units.  It is the escape hatch for registers the typed API does not cover.
func (c *Controller) ReadRegisters(ctx context.Context, want map[Register]Resolution) (map[Register]float64, error) {
	if len(want) == 0 {
		return map[Register]float64{}, nil
	}
	values, err := c.transact(ctx, queryFrame(&QueryResolution{
		Mode: Ignore, Position: Ignore, Velocity: Ignore, Torque: Ignore,
		QCurrent: Ignore, DCurrent: Ignore, AbsPosition: Ignore, Power: Ignore,
		MotorTemperature: Ignore, TrajectoryComplete: Ignore, HomeState: Ignore,
		Voltage: Ignore, Temperature: Ignore, Fault: Ignore,
		Extra: want,
	}), true)
	if err != nil {
		return nil, err
	}
	out := make(map[Register]float64, len(values))
	for register, value := range values {
		out[register] = scaleRegister(register, value)
	}
	return out, nil
}

// decode turns a register map into framework-unit telemetry.
func decode(values map[Register]regValue) *motor.Telemetry {
	if values == nil {
		return nil
	}
	telemetry := motor.NewTelemetry()

	if value, ok := values[RegMode]; ok && !math.IsNaN(value.raw) {
		telemetry.Mode = int(value.raw)
		telemetry.ModeName = ModeName(telemetry.Mode)
	}
	if value, ok := values[RegFault]; ok && !math.IsNaN(value.raw) {
		telemetry.Fault = int(value.raw)
		fault := FaultText(telemetry.Fault)
		telemetry.FaultName = fault.Name
		telemetry.FaultDetail = fault.Detail
		telemetry.HardFault = IsHardFault(telemetry.Fault)
	}
	if value, ok := values[RegTrajectoryComplete]; ok && !math.IsNaN(value.raw) {
		telemetry.TrajectoryComplete = int(value.raw)
	}
	if value, ok := values[RegHomeState]; ok && !math.IsNaN(value.raw) {
		telemetry.HomeState = int(value.raw)
	}

	for _, field := range []struct {
		register Register
		dst      *float64
	}{
		{RegPosition, &telemetry.Position},
		{RegVelocity, &telemetry.Velocity},
		{RegTorque, &telemetry.Torque},
		{RegQCurrent, &telemetry.QCurrent},
		{RegDCurrent, &telemetry.DCurrent},
		{RegAbsPosition, &telemetry.AbsPosition},
		{RegPower, &telemetry.Power},
		{RegVoltage, &telemetry.Voltage},
		{RegTemperature, &telemetry.Temperature},
		{RegMotorTemperature, &telemetry.MotorTemperature},
		{RegCommandPositionMaxTorque, &telemetry.ActiveMaxTorque},
		{RegCommandTimeout, &telemetry.ActiveTimeout},
		{RegCommandVelocityLimit, &telemetry.ActiveVelocityLimit},
		{RegCommandAccelLimit, &telemetry.ActiveAccelLimit},
	} {
		if value, ok := values[field.register]; ok {
			*field.dst = scaleRegister(field.register, value)
		}
	}

	return &telemetry
}

// String identifies the controller in logs.
func (c *Controller) String() string {
	return fmt.Sprintf("moteus[id=%d]", c.ID)
}
