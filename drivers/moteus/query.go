package moteus

import "sort"

// QueryResolution selects which telemetry registers a cycle asks for, and how
// precisely.
//
// This is a bandwidth budget, not a preference.  A moteus command and the
// query it piggybacks on travel in one CAN-FD frame of at most 64 bytes, so
// asking for everything at full precision every period is what stops a loop
// from reaching its target rate.  The three profiles below rotate: the fast
// set every cycle, the wider ones every tenth and every ninety-seventh.
type QueryResolution struct {
	Mode               Resolution
	Position           Resolution
	Velocity           Resolution
	Torque             Resolution
	QCurrent           Resolution
	DCurrent           Resolution
	AbsPosition        Resolution
	Power              Resolution
	MotorTemperature   Resolution
	TrajectoryComplete Resolution
	HomeState          Resolution
	Voltage            Resolution
	Temperature        Resolution
	Fault              Resolution

	// Extra requests registers that have no field of their own, which is how
	// the verify profile reads back the controller's active command limits.
	Extra map[Register]Resolution
}

// FastQuery is the minimum a control loop needs: enough to drive the axis and
// enough for every safety trip to fire.  About 28 bytes of reply.
func FastQuery() QueryResolution {
	return QueryResolution{
		Mode:               Int8,
		Position:           Int32, // 0.00001 rev
		Velocity:           Int32, // 0.00001 rev/s
		Torque:             Int16, // 0.01 Nm
		QCurrent:           Int16, // 0.1 A
		DCurrent:           Ignore,
		AbsPosition:        Ignore,
		Power:              Ignore,
		MotorTemperature:   Ignore,
		TrajectoryComplete: Ignore,
		HomeState:          Ignore,
		Voltage:            Int16,
		Temperature:        Int16,
		Fault:              Int8,
	}
}

// FullQuery adds the diagnostic registers.  Roughly 40 bytes of reply, so it
// is sent at a fraction of the control rate.
func FullQuery() QueryResolution {
	q := FastQuery()
	q.DCurrent = Int16
	q.AbsPosition = Int32
	q.Power = Int16
	q.MotorTemperature = Int16
	q.TrajectoryComplete = Int8
	q.HomeState = Int8
	return q
}

// VerifyQuery adds the controller's live command registers.
//
// This closes a real gap: the command registers are sticky, so a limit the
// host believes it set at connect time can be silently different — after a
// controller reset, after a dropped frame, or after another program on the
// same bus wrote them.  Reading them back turns "the machine behaved
// unexpectedly" into "the controller's torque limit is not what we set".
func VerifyQuery() QueryResolution {
	q := FastQuery()
	q.Extra = map[Register]Resolution{
		RegCommandPositionMaxTorque: Int16,
		RegCommandTimeout:           Int16,
		RegCommandVelocityLimit:     Int32,
		RegCommandAccelLimit:        Int32,
	}
	return q
}

// PositionResolution selects the width of each field in a position-mode
// command.  Narrower fields leave more of the frame for the query.
type PositionResolution struct {
	Position             Resolution
	Velocity             Resolution
	FeedforwardTorque    Resolution
	KpScale              Resolution
	KdScale              Resolution
	MaximumTorque        Resolution
	StopPosition         Resolution
	WatchdogTimeout      Resolution
	VelocityLimit        Resolution
	AccelLimit           Resolution
	FixedVoltageOverride Resolution
	ILimitScale          Resolution
	FixedCurrentOverride Resolution
	IgnorePositionBounds Resolution
}

// DefaultPositionResolution is tuned for a small motor on a fast loop.
//
// MaximumTorque gets int32 rather than the int16 the reference default uses:
// at 0.01 Nm per count a small motor's entire useful torque range is a handful
// of counts, and rounding a 0.05 Nm limit to 0.05 or 0.04 changes whether the
// axis moves at all.
func DefaultPositionResolution() PositionResolution {
	return PositionResolution{
		Position:             Int32, // 0.00001 rev
		Velocity:             Int32, // 0.00001 rev/s
		FeedforwardTorque:    Int16, // 0.01 Nm
		KpScale:              Int16,
		KdScale:              Int16,
		MaximumTorque:        Int32, // 0.001 Nm
		StopPosition:         Int32,
		WatchdogTimeout:      Int16, // 0.001 s
		VelocityLimit:        Int32,
		AccelLimit:           Int32,
		FixedVoltageOverride: Ignore,
		ILimitScale:          Int16,
		FixedCurrentOverride: Ignore,
		IgnorePositionBounds: Int8,
	}
}

// queryData renders a QueryResolution into read subframes, and reports how
// many bytes the reply will occupy.
//
// The registers are requested in four runs that mirror how they are laid out
// in the register map, so that consecutive registers of the same resolution
// collapse into one subframe instead of one framing byte each.
func queryData(q QueryResolution) (data []byte, replySize int) {
	w := &writer{}

	first := newCombiner(w, readBase, uint32(RegMode), []Resolution{
		q.Mode,
		q.Position,
		q.Velocity,
		q.Torque,
		q.QCurrent,
		q.DCurrent,
		q.AbsPosition,
		q.Power,
	})
	first.writeAll()
	replySize += first.replySize

	second := newCombiner(w, readBase, uint32(RegMotorTemperature), []Resolution{
		q.MotorTemperature,
		q.TrajectoryComplete,
		q.HomeState,
		q.Voltage,
		q.Temperature,
		q.Fault,
	})
	second.writeAll()
	replySize += second.replySize

	if len(q.Extra) > 0 {
		registers := make([]int, 0, len(q.Extra))
		for register := range q.Extra {
			registers = append(registers, int(register))
		}
		sort.Ints(registers)

		low, high := registers[0], registers[len(registers)-1]
		resolutions := make([]Resolution, 0, high-low+1)
		for register := low; register <= high; register++ {
			res, ok := q.Extra[Register(register)]
			if !ok {
				res = Ignore
			}
			resolutions = append(resolutions, res)
		}

		extra := newCombiner(w, readBase, uint32(low), resolutions)
		extra.writeAll()
		replySize += extra.replySize
	}

	return w.bytes(), replySize
}
