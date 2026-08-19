package robstride

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
	"drlabs/transport/fdcanusb"
)

func init() {
	motor.Register("robstride", "RobStride quasi-direct-drive actuator over CAN 2.0B", newLink)
	motor.Register("robstride-sim", "simulated RobStride, for exercising control and safety paths", newSimLink)
}

// Link adapts a RobStride actuator to the framework's motor.Link.
//
// Two translations happen here.
//
// The first is units: RobStride speaks radians and rad/s, the framework speaks
// revolutions and rev/s.
//
// The second is control modes, and each framework mode is mapped onto the
// controller loop that can actually deliver its contract:
//
//	CtrlPosition  cyclic synchronous position by default, because its profile
//	              generator honours the commanded velocity limit; the
//	              operation ("MIT") mode is available for impedance control
//	CtrlVelocity  the dedicated velocity run mode, which closes a PI loop and
//	              holds a setpoint under load where MIT's damping term sags
//	CtrlTorque    operation mode with both gains at zero, so the feedforward
//	              torque is the whole command
//	CtrlHold      operation mode with stiffness at zero and damping only —
//	              the controller's documented damping configuration
//	CtrlStop      the stop frame, which also answers with full telemetry and
//	              is therefore what CtrlQuery sends too
type Link struct {
	config motor.Config
	model  Model

	controller *Controller
	bus        can.Bus
	ownsBus    bool

	// kp and kd are what a gain scale of 1.0 maps to.
	kp float64
	kd float64

	// runMode is the mode believed to be selected in the motor.  Switching it
	// costs a parameter write, so it is tracked rather than re-sent.
	runMode    RunMode
	runModeSet bool

	enabled bool

	// positionMode selects how CtrlPosition is expressed: cyclic synchronous
	// position, or the operation ("MIT") mode.
	positionMode RunMode

	// lastTorqueLimit and lastVelocityLimit are the values last written, so
	// the writes happen on change rather than every cycle.
	lastTorqueLimit   float64
	lastVelocityLimit float64

	// turns tracks full revolutions beyond the controller's ±4π window, so a
	// multi-turn axis reports a continuous position instead of wrapping.
	turns        float64
	lastRawAngle float64
	haveRawAngle bool
}

// newLink builds a RobStride Link from a Config.
//
// Recognised options:
//
//	model         RS00 | RS01 | RS02 | RS03 | RS04 | RS05 | RS06  (required)
//	host_id       CAN address this host answers to (default 253)
//	position_mode csp | operation  (default csp)
//	kp            Nm/rad that a gain scale of 1.0 means (default per model)
//	kd            Nm/(rad/s) that a gain scale of 1.0 means (default per model)
//	timeout_ms    transaction timeout (default 100)
//
// position_mode picks how a position command is expressed.  "csp" uses the
// controller's cyclic synchronous position mode, whose own profile generator
// respects the commanded velocity limit — the behaviour the framework's
// CtrlPosition promises.  "operation" uses the MIT-style mode instead, where
// the gain scales and the feedforward torque take effect and the axis behaves
// as a spring-damper; that is what impedance control needs, and it ignores the
// velocity limit because the mode has no trajectory generator.
func newLink(cfg motor.Config) (motor.Link, error) {
	if cfg.ID < 1 || cfg.ID > 127 {
		return nil, fmt.Errorf("robstride: CAN id %d is out of range (1..127)", cfg.ID)
	}
	if err := checkOptions(cfg, "model", "host_id", "position_mode", "kp", "kd", "timeout_ms"); err != nil {
		return nil, err
	}

	model, err := requireModel(cfg)
	if err != nil {
		return nil, err
	}

	hostID, err := intOption(cfg, "host_id", int(DefaultHostID), 0, 255)
	if err != nil {
		return nil, err
	}
	timeoutMS, err := intOption(cfg, "timeout_ms", int(DefaultTimeout/time.Millisecond), 1, 60000)
	if err != nil {
		return nil, err
	}
	kp, err := floatOption(cfg, "kp", model.DefaultKp, 0, model.KpMax)
	if err != nil {
		return nil, err
	}
	kd, err := floatOption(cfg, "kd", model.DefaultKd, 0, model.KdMax)
	if err != nil {
		return nil, err
	}
	positionMode, err := positionModeOption(cfg)
	if err != nil {
		return nil, err
	}

	return &Link{
		config:            cfg,
		model:             model,
		kp:                kp,
		kd:                kd,
		positionMode:      positionMode,
		lastTorqueLimit:   math.NaN(),
		lastVelocityLimit: math.NaN(),
		controller: &Controller{
			ID:      uint8(cfg.ID),
			HostID:  uint8(hostID),
			Model:   model,
			Timeout: time.Duration(timeoutMS) * time.Millisecond,
		},
	}, nil
}

// requireModel resolves the model option.  There is no default on purpose: the
// scaling differs by up to a factor of twenty across the family, so a guess
// would mean commanding an unknown multiple of the intended torque.
func requireModel(cfg motor.Config) (Model, error) {
	name := cfg.Option("model", "")
	if name == "" {
		return Model{}, fmt.Errorf(
			"robstride: the model option is required (one of %v) — "+
				"the torque and velocity scaling differ between models, so it cannot be guessed",
			ModelNames())
	}
	return LookupModel(name)
}

// NewLinkWithBus builds a Link on a bus the caller already owns.  Close will
// not close that bus, which is what makes it possible to drive several
// actuators over one adapter.
func NewLinkWithBus(bus can.Bus, id uint8, model Model) *Link {
	return &Link{
		config:            motor.Config{Driver: "robstride", ID: int(id)},
		model:             model,
		bus:               bus,
		kp:                model.DefaultKp,
		kd:                model.DefaultKd,
		positionMode:      RunPositionCSP,
		lastTorqueLimit:   math.NaN(),
		lastVelocityLimit: math.NaN(),
		controller:        NewController(bus, id, model),
	}
}

// Kind identifies the driver.
func (l *Link) Kind() string { return "robstride" }

// Controller exposes the low-level interface, for callers that need the
// parameter table or the run modes this adapter does not use.
func (l *Link) Controller() *Controller { return l.controller }

// Describe reports the connection.
func (l *Link) Describe() map[string]string {
	out := map[string]string{
		"driver": "robstride",
		"model":  l.model.Name,
		"can_id": strconv.Itoa(int(l.controller.ID)),
	}
	if describer, ok := l.bus.(can.Describer); ok {
		for key, value := range describer.Describe() {
			out[key] = value
		}
	} else if l.config.Port != "" {
		out["port"] = l.config.Port
	}
	return out
}

// Capabilities report what a RobStride can be asked to do.
//
// Brake and stay-within are absent because the controller has no equivalent:
// there is no phase-short brake mode, and no bounded-region mode.  Claiming
// them and approximating would be worse than saying so — a caller that asks
// for a brake and silently gets a damped hold has a machine that coasts
// further than it was told to.
func (l *Link) Capabilities() motor.Capabilities {
	return motor.Capabilities{
		Modes: []motor.ControlMode{
			motor.CtrlStop, motor.CtrlQuery, motor.CtrlPosition,
			motor.CtrlVelocity, motor.CtrlTorque, motor.CtrlHold,
			motor.CtrlSetOutput,
		},
		// Classic CAN at 1 Mbit/s completes a command and its reply in well
		// under a millisecond, but the USB adapter round trip is what actually
		// bounds the loop.
		MaxRateHz: 500,
		// canTimeout (0x7028) is a command watchdog, but it is off by default
		// and this driver does not turn it on: enabling it changes stored
		// behaviour the operator did not ask for.
		HasWatchdog:        false,
		ReportsTemperature: true,
		ReportsCurrent:     true,
		ReportsVoltage:     true,
	}
}

// Envelope reports what the model's documentation rates it for.
func (l *Link) Envelope() motor.Envelope {
	return motor.Envelope{
		VoltageMin:   l.model.VoltageMin,
		VoltageMax:   l.model.VoltageMax,
		VoltageRated: l.model.VoltageRated,
		PeakCurrent:  l.model.CurrentRefMax,
		VelocityMax:  motor.RadPerSecToRevPerSec(l.model.VelocityRange),
	}
}

// Open acquires the transport and leaves the motor stopped.
func (l *Link) Open(ctx context.Context) error {
	if l.bus == nil {
		port := l.config.Port
		if port == "" {
			detected, err := autoDetectPort()
			if err != nil {
				return err
			}
			port = detected
		}
		device, err := fdcanusb.Open(port)
		if err != nil {
			return err
		}
		l.bus = device
		l.ownsBus = true
	}
	l.controller.Bus = l.bus

	if err := l.bus.Drain(20 * time.Millisecond); err != nil {
		return err
	}

	// A motor left enabled by a previous session keeps acting on its last
	// command until told otherwise, so the connect sequence stops it first.
	feedback, err := l.controller.Stop(ctx, false)
	if err != nil {
		l.closeBus()
		return err
	}
	if feedback == nil {
		l.closeBus()
		return &motor.LinkError{
			Op: "open",
			Err: fmt.Errorf("no reply from CAN id %d — check the id, the bus wiring and "+
				"termination, the 1 Mbit/s bit rate, and that the motor is powered",
				l.controller.ID),
		}
	}
	l.enabled = false
	l.runModeSet = false
	return nil
}

// Close stops the motor and, if this Link opened the bus, releases it.
func (l *Link) Close() error {
	if l.bus == nil {
		return nil
	}
	stopCtx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	_, _ = l.controller.Stop(stopCtx, false)
	cancel()
	l.enabled = false

	return l.closeBus()
}

func (l *Link) closeBus() error {
	bus := l.bus
	l.bus = nil
	l.controller.Bus = nil
	if bus == nil || !l.ownsBus {
		return nil
	}
	return bus.Close()
}

// Identify reports what can be learned about the motor.
//
// A RobStride does not report its own model over the bus, so the board name
// here is the *configured* model, and the status is reported as unverified.
// That distinction matters: if the configuration says RS01 and an RS04 is
// attached, every torque command is scaled by a factor of seven and nothing on
// the wire would reveal it — so the operator is told the identification is an
// assumption rather than a reading.
func (l *Link) Identify(ctx context.Context) (*motor.Identity, error) {
	identity := motor.NewIdentity("robstride")
	identity.Board = l.model.Name
	identity.Status = motor.BoardDeclared

	if uid, err := l.controller.DeviceID(ctx); err == nil && uid != "" {
		identity.SerialNumber = uid
		identity.Known = true
	}
	return identity, nil
}

// Cycle transmits one command and returns the telemetry that came back.
func (l *Link) Cycle(ctx context.Context, cmd motor.Command, profile motor.QueryProfile) (*motor.Telemetry, error) {
	feedback, err := l.apply(ctx, cmd)
	if err != nil {
		return nil, err
	}
	if feedback == nil {
		return nil, nil
	}

	telemetry := l.telemetry(*feedback)

	// The feedback frame carries no bus voltage and no current; both live in
	// the parameter table.  They are read on the wider profiles so that the
	// fast path stays a single transaction, which is what lets the loop keep
	// its period.
	if profile != motor.ProfileFast {
		if value, ok, err := l.controller.ReadParameter(ctx, ParamVBus); err == nil && ok {
			telemetry.Voltage = value
		}
		if value, ok, err := l.controller.ReadParameter(ctx, ParamIqFilt); err == nil && ok {
			telemetry.QCurrent = value
		}
	}
	if profile == motor.ProfileVerify {
		// limit_torque governs in every run mode, so it is always worth
		// verifying against what the host believes it set.
		if value, ok, err := l.controller.ReadParameter(ctx, ParamLimitTorque); err == nil && ok {
			telemetry.ActiveMaxTorque = value
		}
		// limit_spd only governs in CSP position mode.  Reporting it while the
		// operation mode is in charge would have the host compare its own
		// velocity limit against a parameter that is not doing anything, and
		// warn about a mismatch on every verify cycle for ever.
		if l.runModeSet && l.runMode == RunPositionCSP {
			if value, ok, err := l.controller.ReadParameter(ctx, ParamLimitSpd); err == nil && ok {
				telemetry.ActiveVelocityLimit = motor.RadPerSecToRevPerSec(value)
			}
		}
	}

	return &telemetry, nil
}

// apply performs the one transaction that this command amounts to.
func (l *Link) apply(ctx context.Context, cmd motor.Command) (*Feedback, error) {
	switch cmd.Mode {
	case motor.CtrlStop:
		l.enabled = false
		return l.controller.Stop(ctx, false)

	case motor.CtrlQuery:
		// A stop frame answers with a full feedback frame and leaves the motor
		// released, which is exactly what a disarmed session wants: watch the
		// axis without commanding it.
		return l.controller.Stop(ctx, false)

	case motor.CtrlSetOutput:
		// The controller can only declare the *current* position to be zero;
		// it cannot be told "you are at 0.3 rev".  Anything else is refused
		// rather than approximated.
		if motor.Valid(cmd.Position) && math.Abs(cmd.Position) > 1e-9 {
			return nil, fmt.Errorf(
				"robstride: the output can only be re-referenced to zero, not to %.4f rev", cmd.Position)
		}
		if l.enabled {
			return nil, fmt.Errorf("robstride: stop the motor before re-referencing the output")
		}
		l.turns, l.haveRawAngle = 0, false
		return l.controller.SetMechanicalZero(ctx)

	case motor.CtrlBrake, motor.CtrlStayWithin:
		return nil, motor.ErrUnsupportedMode
	}

	if err := l.applyTorqueLimit(ctx, cmd); err != nil {
		return nil, err
	}

	switch cmd.Mode {
	case motor.CtrlVelocity:
		if err := l.ensureRunMode(ctx, RunVelocity); err != nil {
			return nil, err
		}
		if err := l.ensureEnabled(ctx); err != nil {
			return nil, err
		}
		velocity := 0.0
		if motor.Valid(cmd.Velocity) {
			velocity = motor.RevPerSecToRadPerSec(cmd.Velocity)
		}
		velocity = motor.Clamp(velocity, -l.model.SpeedRefMax, l.model.SpeedRefMax)
		return l.controller.WriteParameter(ctx, ParamSpdRef, velocity)

	case motor.CtrlPosition:
		if l.positionMode == RunPositionCSP {
			return l.applyCSP(ctx, cmd)
		}
		fallthrough

	case motor.CtrlTorque, motor.CtrlHold:
		if err := l.ensureRunMode(ctx, RunOperation); err != nil {
			return nil, err
		}
		if err := l.ensureEnabled(ctx); err != nil {
			return nil, err
		}
		return l.controller.Control(ctx, l.controlCommand(cmd))
	}

	return nil, motor.ErrUnsupportedMode
}

// applyCSP drives a position target through cyclic synchronous position mode.
//
// This is the mapping that honours the framework's contract for CtrlPosition:
// "drive to this position, respecting the configured velocity limit".  The
// controller's own profile generator does the limiting, so a step change in
// the setpoint produces a bounded move rather than a full-torque lunge.
//
// The operation mode cannot do that — it has no trajectory generator, so a
// step in the target is a step in the error and the axis accelerates until the
// torque limit stops it.  That is the right behaviour for impedance control
// and the wrong one for "go here at no more than this speed".
func (l *Link) applyCSP(ctx context.Context, cmd motor.Command) (*Feedback, error) {
	if err := l.ensureRunMode(ctx, RunPositionCSP); err != nil {
		return nil, err
	}
	if err := l.applyVelocityLimit(ctx, cmd); err != nil {
		return nil, err
	}
	if err := l.ensureEnabled(ctx); err != nil {
		return nil, err
	}

	target := motor.RevToRad(valueOr(cmd.Position, 0))
	return l.controller.WriteParameter(ctx, ParamLocRef, target)
}

// applyVelocityLimit writes limit_spd when it changes.
func (l *Link) applyVelocityLimit(ctx context.Context, cmd motor.Command) error {
	if !motor.Valid(cmd.VelocityLimit) {
		return nil
	}
	limit := motor.Clamp(motor.RevPerSecToRadPerSec(cmd.VelocityLimit), 0, l.model.SpeedLimitMax)
	if !cmd.Full && limit == l.lastVelocityLimit {
		return nil
	}
	if _, err := l.controller.WriteParameter(ctx, ParamLimitSpd, limit); err != nil {
		return err
	}
	l.lastVelocityLimit = limit
	return nil
}

// controlCommand builds the operation-mode frame for a command.
func (l *Link) controlCommand(cmd motor.Command) ControlCommand {
	out := ControlCommand{}

	// Gain scales are 0..1 in the framework; here they select a fraction of
	// the configured stiffness rather than of the codec's full scale, which
	// would be far too stiff to use as a default.
	kpScale, kdScale := 1.0, 1.0
	if motor.Valid(cmd.KpScale) {
		kpScale = cmd.KpScale
	}
	if motor.Valid(cmd.KdScale) {
		kdScale = cmd.KdScale
	}

	switch cmd.Mode {
	case motor.CtrlPosition:
		// The controller's position field wraps at ±4π, so a target beyond
		// that has to be folded back into the window it can express.  The turn
		// counter on the feedback side undoes the fold when reporting.
		out.Angle = wrapAngle(motor.RevToRad(valueOr(cmd.Position, 0)))
		out.Velocity = 0
		out.Kp = l.kp * kpScale
		out.Kd = l.kd * kdScale

	case motor.CtrlHold:
		// Damping without a position target: zero stiffness, zero velocity
		// target, damping only.  This is the controller's documented damping
		// configuration.
		out.Angle = 0
		out.Velocity = 0
		out.Kp = 0
		out.Kd = l.kd * kdScale

	case motor.CtrlTorque:
		// Pure feedforward: no stiffness, no damping.
		out.Kp = 0
		out.Kd = 0
	}

	if motor.Valid(cmd.FeedforwardTorque) {
		out.Torque = cmd.FeedforwardTorque
	}
	return out
}

// applyTorqueLimit writes limit_torque when it changes.
//
// This is the parameter that actually bounds what the machine can do, whatever
// the gains are, so it is worth a transaction whenever the operator moves it —
// and worth not spending one when they have not.
func (l *Link) applyTorqueLimit(ctx context.Context, cmd motor.Command) error {
	if !motor.Valid(cmd.MaxTorque) {
		return nil
	}
	limit := motor.Clamp(cmd.MaxTorque, 0, l.model.TorqueLimitMax)
	if !cmd.Full && limit == l.lastTorqueLimit {
		return nil
	}
	if _, err := l.controller.WriteParameter(ctx, ParamLimitTorque, limit); err != nil {
		return err
	}
	l.lastTorqueLimit = limit
	return nil
}

// ensureRunMode switches the controller's active loop if needed.
//
// The motor must be stopped for the switch: changing which loop is in charge
// while it is driving hands the output to a controller whose state has not
// been initialised for the current position.
func (l *Link) ensureRunMode(ctx context.Context, mode RunMode) error {
	if l.runModeSet && l.runMode == mode {
		return nil
	}
	if l.enabled {
		if _, err := l.controller.Stop(ctx, false); err != nil {
			return err
		}
		l.enabled = false
	}
	if _, err := l.controller.SetRunMode(ctx, mode); err != nil {
		return err
	}
	l.runMode = mode
	l.runModeSet = true
	return nil
}

// positionModeOption resolves the position_mode setting.
func positionModeOption(cfg motor.Config) (RunMode, error) {
	switch cfg.Option("position_mode", "csp") {
	case "csp":
		return RunPositionCSP, nil
	case "operation", "mit":
		return RunOperation, nil
	default:
		return 0, fmt.Errorf(
			"robstride: option position_mode=%q must be \"csp\" or \"operation\"",
			cfg.Option("position_mode", ""))
	}
}

func (l *Link) ensureEnabled(ctx context.Context) error {
	if l.enabled {
		return nil
	}
	if _, err := l.controller.Enable(ctx); err != nil {
		return err
	}
	l.enabled = true
	return nil
}

// telemetry converts a feedback frame into framework units.
func (l *Link) telemetry(feedback Feedback) motor.Telemetry {
	out := motor.NewTelemetry()

	out.Mode = int(feedback.Mode)
	out.ModeName = ModeName(out.Mode)

	name, detail := feedback.FaultText()
	out.Fault = int(feedback.Flags)
	out.FaultName = name
	out.FaultDetail = detail
	out.HardFault = feedback.Faulted()

	out.Position = motor.RadToRev(l.unwrap(feedback.Angle))
	out.Velocity = motor.RadPerSecToRevPerSec(feedback.Velocity)
	out.Torque = feedback.Torque
	out.Temperature = feedback.Temperature
	// A RobStride is an integrated actuator: the temperature it reports is the
	// one inside the case, and it stands for both board and motor.
	out.MotorTemperature = feedback.Temperature

	return out
}

// unwrap turns the controller's ±4π position into a continuous one.
//
// The reported angle wraps at the ends of its range, so a multi-turn axis
// would otherwise appear to jump eight π backwards every four revolutions —
// and a position envelope checked against that would trip on a perfectly
// healthy machine.  A jump larger than half the window is read as a wrap.
func (l *Link) unwrap(raw float64) float64 {
	if !l.haveRawAngle {
		l.lastRawAngle = raw
		l.haveRawAngle = true
		return raw
	}

	delta := raw - l.lastRawAngle
	window := 2 * PositionRange
	switch {
	case delta > PositionRange:
		l.turns -= window
	case delta < -PositionRange:
		l.turns += window
	}
	l.lastRawAngle = raw
	return raw + l.turns
}

// wrapAngle folds a commanded angle into the ±4π window the protocol can
// express.
func wrapAngle(angle float64) float64 {
	window := 2 * PositionRange
	wrapped := math.Mod(angle+PositionRange, window)
	if wrapped < 0 {
		wrapped += window
	}
	return wrapped - PositionRange
}

func valueOr(value, fallback float64) float64 {
	if motor.Valid(value) {
		return value
	}
	return fallback
}

// autoDetectPort picks the most likely CAN adapter when none was configured.
func autoDetectPort() (string, error) {
	ports, err := fdcanusb.Detect()
	if err != nil {
		return "", fmt.Errorf("robstride: could not enumerate serial ports: %w", err)
	}
	if len(ports) == 0 {
		return "", fmt.Errorf("robstride: no serial ports found; plug in a CAN adapter or name the port explicitly")
	}
	if ports[0].IsCANAdapter {
		return ports[0].Path, nil
	}
	names := make([]string, 0, len(ports))
	for _, port := range ports {
		names = append(names, port.Path)
	}
	return "", fmt.Errorf("robstride: no CAN adapter recognised among %v; name the port explicitly", names)
}

// checkOptions rejects unrecognised option keys, so that a typo in a config
// file is reported rather than silently ignored.
func checkOptions(cfg motor.Config, allowed ...string) error {
	for key := range cfg.Options {
		known := false
		for _, name := range allowed {
			if key == name {
				known = true
				break
			}
		}
		if !known {
			return fmt.Errorf("robstride: unknown option %q (accepted: %v)", key, allowed)
		}
	}
	return nil
}

func intOption(cfg motor.Config, key string, def, low, high int) (int, error) {
	text := cfg.Option(key, "")
	if text == "" {
		return def, nil
	}
	value, err := strconv.Atoi(text)
	if err != nil {
		return 0, fmt.Errorf("robstride: option %s=%q is not a number", key, text)
	}
	if value < low || value > high {
		return 0, fmt.Errorf("robstride: option %s=%d is out of range (%d..%d)", key, value, low, high)
	}
	return value, nil
}

func floatOption(cfg motor.Config, key string, def, low, high float64) (float64, error) {
	text := cfg.Option(key, "")
	if text == "" {
		return def, nil
	}
	value, err := strconv.ParseFloat(text, 64)
	if err != nil {
		return 0, fmt.Errorf("robstride: option %s=%q is not a number", key, text)
	}
	if value < low || value > high {
		return 0, fmt.Errorf("robstride: option %s=%g is out of range (%g..%g)", key, value, low, high)
	}
	return value, nil
}
