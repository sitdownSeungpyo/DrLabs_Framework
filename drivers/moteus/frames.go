package moteus

import "math"

// Frame builders.  Each returns the multiplex payload for one command,
// optionally with a telemetry query appended so that the reply carries state
// without costing a second bus transaction.

// opt is a command field that may or may not be transmitted.
//
// Presence has to be tracked separately from the value, because NaN is
// *meaningful* on this protocol rather than being a stand-in for "absent".
// Sending NaN for the position register means "no position target" — which is
// exactly how velocity mode is expressed — and sending it for the maximum
// torque register means "use the controller's own configured limit".  Letting
// NaN double as "omit" would silently turn a velocity command into a stale
// position hold, because a command register that is not written keeps whatever
// was last put in it.
type opt struct {
	value float64
	send  bool
}

// val marks a field for transmission with the given value, NaN included.
func val(v float64) opt { return opt{value: v, send: true} }

// present marks a field for transmission only if the value is a real number.
// It is for the parameters where NaN genuinely does mean "leave it alone".
func present(v float64) opt { return opt{value: v, send: !math.IsNaN(v)} }

// unset omits the field entirely.
func unset() opt { return opt{value: math.NaN()} }

// resolution returns the wire width for this field, or Ignore when it is not
// being transmitted.
func (o opt) resolution(res Resolution) Resolution {
	if !o.send {
		return Ignore
	}
	return res
}

// setMode emits the two-byte "write mode register" prologue every command
// begins with.
func setMode(w *writer, mode Mode) {
	w.writeU8(writeCommands[Int8] | writeBase | 0x01)
	w.writeU8(byte(RegMode))
	w.writeInt8(int8(mode))
}

// stopFrame commands mode 0: the bridge is released and the shaft turns free.
func stopFrame(q *QueryResolution) []byte {
	w := &writer{}
	setMode(w, ModeStopped)
	appendQuery(w, q)
	return w.bytes()
}

// brakeFrame commands mode 15: the phases are shorted, which brakes the axis
// without needing the host to keep commanding it.
func brakeFrame(q *QueryResolution) []byte {
	w := &writer{}
	setMode(w, ModeBrake)
	appendQuery(w, q)
	return w.bytes()
}

// queryFrame reads telemetry without changing the controller's mode, which is
// what a disarmed session sends: it wants to see the axis, not drive it.
func queryFrame(q *QueryResolution) []byte {
	w := &writer{}
	appendQuery(w, q)
	return w.bytes()
}

// zeroVelocityFrame commands mode 12: hold zero velocity without holding a
// position, so the axis resists motion but does not fight back to a target.
func zeroVelocityFrame(kdScale float64, pr PositionResolution, q *QueryResolution) []byte {
	w := &writer{}
	setMode(w, ModeZeroVelocity)

	kd := present(kdScale)
	c := newCombiner(w, writeBase, uint32(RegCommandKdScale),
		[]Resolution{kd.resolution(pr.KdScale)})
	if c.maybeWrite() {
		w.writeMapped(kd.value, pwmScales[0], pwmScales[1], pwmScales[2], pr.KdScale)
	}

	appendQuery(w, q)
	return w.bytes()
}

// positionCommand is the full set of position-mode parameters.  A field left
// unset is omitted from the frame, which leaves the controller's existing
// value in force — that is what makes it affordable to send only the handful
// of registers that change every period.
type positionCommand struct {
	Position             opt
	Velocity             opt
	FeedforwardTorque    opt
	KpScale              opt
	KdScale              opt
	MaximumTorque        opt
	StopPosition         opt
	WatchdogTimeout      opt
	VelocityLimit        opt
	AccelLimit           opt
	ILimitScale          opt
	IgnorePositionBounds opt
}

// positionFrame builds a mode 10 command.
func positionFrame(cmd positionCommand, pr PositionResolution, q *QueryResolution) []byte {
	w := &writer{}
	setMode(w, ModePosition)

	// Registers 0x020..0x02d, in order.
	c := newCombiner(w, writeBase, uint32(RegCommandPosition), []Resolution{
		cmd.Position.resolution(pr.Position),
		cmd.Velocity.resolution(pr.Velocity),
		cmd.FeedforwardTorque.resolution(pr.FeedforwardTorque),
		cmd.KpScale.resolution(pr.KpScale),
		cmd.KdScale.resolution(pr.KdScale),
		cmd.MaximumTorque.resolution(pr.MaximumTorque),
		cmd.StopPosition.resolution(pr.StopPosition),
		cmd.WatchdogTimeout.resolution(pr.WatchdogTimeout),
		cmd.VelocityLimit.resolution(pr.VelocityLimit),
		cmd.AccelLimit.resolution(pr.AccelLimit),
		Ignore, // fixed voltage override
		cmd.ILimitScale.resolution(pr.ILimitScale),
		Ignore, // fixed current override
		cmd.IgnorePositionBounds.resolution(pr.IgnorePositionBounds),
	})

	writeIf(c, w, cmd.Position, positionScales, pr.Position)
	writeIf(c, w, cmd.Velocity, velocityScales, pr.Velocity)
	writeIf(c, w, cmd.FeedforwardTorque, torqueScales, pr.FeedforwardTorque)
	writeIf(c, w, cmd.KpScale, pwmScales, pr.KpScale)
	writeIf(c, w, cmd.KdScale, pwmScales, pr.KdScale)
	writeIf(c, w, cmd.MaximumTorque, torqueScales, pr.MaximumTorque)
	writeIf(c, w, cmd.StopPosition, positionScales, pr.StopPosition)
	writeIf(c, w, cmd.WatchdogTimeout, timeScales, pr.WatchdogTimeout)
	writeIf(c, w, cmd.VelocityLimit, velocityScales, pr.VelocityLimit)
	writeIf(c, w, cmd.AccelLimit, accelScales, pr.AccelLimit)
	writeIf(c, w, unset(), voltageScales, Ignore)
	writeIf(c, w, cmd.ILimitScale, pwmScales, pr.ILimitScale)
	writeIf(c, w, unset(), currentScales, Ignore)
	writeIntIf(c, w, cmd.IgnorePositionBounds, pr.IgnorePositionBounds)

	appendQuery(w, q)
	return w.bytes()
}

// stayWithinCommand is the parameter set for mode 13.
type stayWithinCommand struct {
	LowerBound           opt
	UpperBound           opt
	FeedforwardTorque    opt
	KpScale              opt
	KdScale              opt
	MaximumTorque        opt
	WatchdogTimeout      opt
	ILimitScale          opt
	IgnorePositionBounds opt
}

// stayWithinFrame builds a mode 13 command: the controller applies no torque
// while the axis is inside the bounds and pushes it back when it leaves.  It
// is the right shape for a soft end-stop, where a position hold would fight
// the operator throughout the whole range.
func stayWithinFrame(cmd stayWithinCommand, pr PositionResolution, q *QueryResolution) []byte {
	w := &writer{}
	setMode(w, ModeStayWithin)

	// Registers 0x040..0x048, in order.
	c := newCombiner(w, writeBase, uint32(RegWithinLowerBound), []Resolution{
		cmd.LowerBound.resolution(pr.Position),
		cmd.UpperBound.resolution(pr.Position),
		cmd.FeedforwardTorque.resolution(pr.FeedforwardTorque),
		cmd.KpScale.resolution(pr.KpScale),
		cmd.KdScale.resolution(pr.KdScale),
		cmd.MaximumTorque.resolution(pr.MaximumTorque),
		cmd.WatchdogTimeout.resolution(pr.WatchdogTimeout),
		cmd.ILimitScale.resolution(pr.ILimitScale),
		cmd.IgnorePositionBounds.resolution(pr.IgnorePositionBounds),
	})

	writeIf(c, w, cmd.LowerBound, positionScales, pr.Position)
	writeIf(c, w, cmd.UpperBound, positionScales, pr.Position)
	writeIf(c, w, cmd.FeedforwardTorque, torqueScales, pr.FeedforwardTorque)
	writeIf(c, w, cmd.KpScale, pwmScales, pr.KpScale)
	writeIf(c, w, cmd.KdScale, pwmScales, pr.KdScale)
	writeIf(c, w, cmd.MaximumTorque, torqueScales, pr.MaximumTorque)
	writeIf(c, w, cmd.WatchdogTimeout, timeScales, pr.WatchdogTimeout)
	writeIf(c, w, cmd.ILimitScale, pwmScales, pr.ILimitScale)
	writeIntIf(c, w, cmd.IgnorePositionBounds, pr.IgnorePositionBounds)

	appendQuery(w, q)
	return w.bytes()
}

// setOutputFrame re-references the output position.
//
// It writes register 0x130 (nearest) or 0x131 (exact) at runtime; neither
// touches the controller's stored configuration, so the change is gone after a
// power cycle.  That is the intended behaviour — re-zeroing an axis should not
// silently rewrite what the machine believes about itself across reboots.
func setOutputFrame(register Register, position float64, q *QueryResolution) []byte {
	w := &writer{}
	w.writeU8(writeCommands[F32] | writeBase | 0x01)
	w.writeVaruint(uint32(register))
	w.writeF32(float32(position))
	appendQuery(w, q)
	return w.bytes()
}

// appendQuery adds the read subframes for q, if any.
func appendQuery(w *writer, q *QueryResolution) {
	if q == nil {
		return
	}
	data, _ := queryData(*q)
	w.buf = append(w.buf, data...)
}

// writeIf emits one value if the combiner says this register is being sent.
func writeIf(c *combiner, w *writer, field opt, scales [3]float64, res Resolution) {
	if c.maybeWrite() {
		w.writeMapped(field.value, scales[0], scales[1], scales[2], res)
	}
}

// writeIntIf emits a register whose value is a plain integer with no scaling,
// such as the ignore-position-bounds flag.
func writeIntIf(c *combiner, w *writer, field opt, res Resolution) {
	if c.maybeWrite() {
		w.writeMapped(math.Trunc(field.value), 1, 1, 1, res)
	}
}
