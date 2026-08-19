package motor

// ControlMode is the vendor-neutral control intent for one cycle.
//
// Not every driver supports every mode; Link.Cycle returns ErrUnsupportedMode
// for the ones it cannot express.  The set is deliberately modelled on the
// moteus command surface because it is the richest of the supported families —
// simpler controllers implement a subset.
type ControlMode uint8

const (
	// CtrlStop releases the bridge: no torque, no holding.
	CtrlStop ControlMode = iota
	// CtrlQuery reads telemetry without changing the controller's mode.
	CtrlQuery
	// CtrlPosition drives to Command.Position with the configured limits.
	CtrlPosition
	// CtrlVelocity spins at Command.Velocity (Position is left NaN).
	CtrlVelocity
	// CtrlTorque commands Command.FeedforwardTorque directly.
	CtrlTorque
	// CtrlHold servos velocity to zero without holding a position target.
	CtrlHold
	// CtrlBrake shorts the phases for a passive, regenerative stop.
	CtrlBrake
	// CtrlStayWithin applies torque only outside [LowerBound, UpperBound].
	CtrlStayWithin
	// CtrlSetOutput re-references the output position to Command.Position.
	// Runtime only — it must not touch persistent controller configuration.
	CtrlSetOutput
)

var controlModeNames = map[ControlMode]string{
	CtrlStop:       "stop",
	CtrlQuery:      "query",
	CtrlPosition:   "position",
	CtrlVelocity:   "velocity",
	CtrlTorque:     "torque",
	CtrlHold:       "hold",
	CtrlBrake:      "brake",
	CtrlStayWithin: "within",
	CtrlSetOutput:  "set_output",
}

func (m ControlMode) String() string {
	if name, ok := controlModeNames[m]; ok {
		return name
	}
	return "unknown"
}

// ParseControlMode maps the textual name used by the CLI and config files.
func ParseControlMode(name string) (ControlMode, bool) {
	for mode, text := range controlModeNames {
		if text == name {
			return mode, true
		}
	}
	return CtrlStop, false
}

// Command is one control cycle's worth of intent.
//
// Every float field uses NaN to mean "do not send this parameter", which for
// register-based controllers leaves whatever value was last written in place.
// NewCommand returns a Command with all of them already set to NaN.
type Command struct {
	Mode ControlMode

	Position float64 // rev
	Velocity float64 // rev/s

	FeedforwardTorque float64 // Nm
	MaxTorque         float64 // Nm
	StopPosition      float64 // rev
	WatchdogTimeout   float64 // s; controller stops if no command arrives in time

	// DeferTorqueToController asks the controller to apply its own configured
	// torque limit instead of the host's, which register-based drivers send
	// as the protocol's NaN sentinel.  It is a separate flag rather than a
	// NaN in MaxTorque because NaN already means "omit this field", and the
	// two are not the same thing: omitting the field leaves whatever limit
	// was last written in force, while deferring actively hands control back.
	// While it is set, the host torque ceiling does not apply — the current,
	// stall and runaway guards are what remain.
	DeferTorqueToController bool

	VelocityLimit float64 // rev/s, trajectory limit
	AccelLimit    float64 // rev/s^2, trajectory limit

	KpScale     float64 // 0..1 scale on the position gain
	KdScale     float64 // 0..1 scale on the damping gain
	ILimitScale float64 // 0..1 scale on the integrator limit

	LowerBound float64 // rev, CtrlStayWithin
	UpperBound float64 // rev, CtrlStayWithin

	// IgnorePositionBounds bypasses the controller's own stored travel limits
	// for this command only (moteus `d pos … b1`).
	IgnorePositionBounds bool

	// Full requests that the complete parameter set be transmitted this cycle
	// rather than only the fields that change every period.  Drivers that keep
	// command registers sticky use it to re-assert limits periodically.
	Full bool

	// VelocityTarget is the motion the host expects to see as a result of this
	// command.  It is not transmitted; the safety monitor uses it to tell a
	// stall apart from a deliberate hold.
	VelocityTarget float64
}

// NewCommand returns a Command in the given mode with every optional
// parameter unset (NaN).
func NewCommand(mode ControlMode) Command {
	nan := NaN()
	return Command{
		Mode:              mode,
		Position:          nan,
		Velocity:          nan,
		FeedforwardTorque: nan,
		MaxTorque:         nan,
		StopPosition:      nan,
		WatchdogTimeout:   nan,
		VelocityLimit:     nan,
		AccelLimit:        nan,
		KpScale:           nan,
		KdScale:           nan,
		ILimitScale:       nan,
		LowerBound:        nan,
		UpperBound:        nan,
		VelocityTarget:    0,
	}
}
