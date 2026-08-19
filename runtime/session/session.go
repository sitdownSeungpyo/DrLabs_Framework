// Package session owns the connection lifecycle, the real-time control loop
// and the state machine that sits between an application and one controller.
//
// Everything that touches the bus happens inside the loop goroutine, on a
// fixed period.  Callers never transact: they push requests into fields the
// loop samples, and read snapshots it publishes.  That is what keeps a slow
// UI, a garbage collection pause or a blocking log write from injecting
// jitter into motor control.
//
// State machine:
//
//	disconnected ──connect──▶ connected/disarmed ──arm──▶ connected/armed ──▶ mode
//	                              ▲                            │
//	                              └──────────disarm────────────┘
//
// The emergency stop is orthogonal.  It latches from any state, forces a stop
// command on every subsequent cycle, and must be cleared explicitly — a stop
// that clears itself is not a stop.
package session

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"drlabs/motor"
	"drlabs/runtime/safety"
	"drlabs/trajectory"
)

// Connection is the transport-level state.
type Connection string

const (
	Disconnected Connection = "disconnected"
	Connecting   Connection = "connecting"
	Connected    Connection = "connected"
)

// Rate limits.  The ceiling is what a USB-CAN adapter can sustain in practice;
// the floor keeps the watchdog arithmetic meaningful.
const (
	MinRateHz     = 10
	MaxRateHz     = 500
	DefaultRateHz = 100
)

// Query profile rotation.  The verify divisor is prime so that verify and full
// cycles do not repeatedly coincide — two wide queries landing on the same
// period is exactly the frame that overruns.
const (
	fullQueryDivisor   = 10
	verifyQueryDivisor = 97

	// paramRefresh is how often the complete parameter set is re-sent even
	// when nothing changed.  Command registers are sticky, so a controller
	// that reset underneath us would otherwise keep stale limits until the
	// operator happened to touch something.
	paramRefresh = 2 * time.Second

	// boundsInset keeps commands this far inside the controller's own limits.
	// Commanding exactly the bound leaves no room for overshoot, and the
	// firmware faults the moment position control starts at or beyond it.
	boundsInset = 0.05
)

// Options configure a Session.
type Options struct {
	// Safety is the starting envelope.  Zero value means DefaultConfig.
	Safety *safety.Config

	// Units renders the positions, speeds and torques this session puts in
	// its log.  Nil means motor space — revolutions at the shaft, torque at
	// the rotor — which is what somebody driving one controller directly is
	// thinking in.  A robot supplies a joint renderer instead, so that a
	// limit written in radians is not reported back in revolutions.
	Units motor.Units

	// RateHz is the initial control rate.
	RateHz int

	// SampleBuffer is how many telemetry samples to retain (about two minutes
	// at 100 Hz by default).
	SampleBuffer int

	// EventBuffer is how many log events to retain.
	EventBuffer int

	// RequireAcknowledgement makes Arm refuse until the caller confirms it has
	// shown the operator the preflight advisories.  Left on by default: an
	// arm that cannot be refused is not a checklist.
	//
	// AutoArm turns it off implicitly — there is nobody to acknowledge.
	RequireAcknowledgement *bool

	// AutoArm makes the session energise the motor as soon as it can and hold
	// it there, waiting for commands.
	//
	// This is the servo role rather than the operator role: the framework is
	// the low level, something above it decides what the machine does, and an
	// axis that needs a human to press Arm first cannot serve that.  With it
	// set, the loop arms the moment preflight is clear and re-arms if it later
	// becomes clear — a supply that was still settling at start-up should not
	// leave the axis dead until someone notices.
	//
	// It never re-arms through a latched emergency stop.  A stop that undid
	// itself would be worse than no stop at all, so clearing one stays an
	// explicit act.
	AutoArm bool

	// InitialMode is what an auto-armed session commands once armed.  The zero
	// value is CtrlPosition, which holds the position the axis was already at:
	// torque on, standing still, waiting.
	InitialMode motor.ControlMode
}

// Params are the control parameters the loop sends every cycle.
type Params struct {
	MaxTorque     float64
	VelocityLimit float64
	AccelLimit    float64
	KpScale       float64
	KdScale       float64
	ILimitScale   float64
	Feedforward   float64

	// DeferTorqueToController hands the torque limit back to the firmware.
	// While it is set the host torque ceiling does not apply; the current,
	// stall and runaway guards still do.
	DeferTorqueToController bool

	// IgnorePositionBounds bypasses the controller's stored travel envelope
	// for every command, leaving only the host envelope.
	IgnorePositionBounds bool
}

// DefaultParams are deliberately gentle: a third of a newton-metre and one
// revolution per second will move a small motor and will not break a machine.
func DefaultParams() Params {
	return Params{
		MaxTorque:     0.30,
		VelocityLimit: 1.0,
		AccelLimit:    5.0,
		KpScale:       1.0,
		KdScale:       1.0,
		ILimitScale:   1.0,
		Feedforward:   0.0,
	}
}

// Setpoint is what the operator is asking the axis to do.
type Setpoint struct {
	Position    float64 // rev
	Velocity    float64 // rev/s
	WithinLower float64 // rev
	WithinUpper float64 // rev
}

// Session drives exactly one controller.
type Session struct {
	mu sync.RWMutex

	link     motor.Link
	linkDesc map[string]string
	identity *motor.Identity

	safety  safety.Config
	units   motor.Units
	monitor *safety.Monitor
	preset  string

	connection    Connection
	connectError  string
	armed         bool
	armBlockers   []string
	armAdvisories []string

	estop       bool
	estopCode   string
	estopReason string
	estopDetail string

	mode     motor.ControlMode
	rateHz   int
	setpoint Setpoint
	params   Params
	shuttle  Shuttle

	// A host-profiled move.  It and the shuttle both own the position
	// setpoint, so starting either one stops the other.
	trajectory          TrajectoryStatus
	profile             trajectory.Profile
	trajectoryStartedAt time.Time

	telemetry    motor.Telemetry
	latchedFault int
	lastReplyAt  time.Time

	requireAck  bool
	autoArm     bool
	initialMode motor.ControlMode
	// autoArmReportedAt throttles the "cannot arm" log, so a blocked axis
	// explains itself once rather than at the control rate.
	autoArmReportedAt time.Time

	// Loop-owned bookkeeping.  Written only by the loop goroutine, read under
	// the mutex by Status.
	stats   loopStats
	samples *sampleRing
	events  *eventRing

	paramsDirty    bool
	paramsSentAt   time.Time
	stopBurst      int
	cycleCount     uint64
	pendingSpecial *motor.Command

	heartbeatAt time.Time

	cancel context.CancelFunc
	done   chan struct{}
	wake   chan struct{}
	t0     time.Time
}

// New creates a disconnected Session.
func New(opts Options) *Session {
	config := safety.DefaultConfig()
	if opts.Safety != nil {
		config = *opts.Safety
	}
	config.Normalise(nil)

	rate := opts.RateHz
	if rate == 0 {
		rate = DefaultRateHz
	}
	sampleBuffer := opts.SampleBuffer
	if sampleBuffer == 0 {
		sampleBuffer = 12000
	}
	eventBuffer := opts.EventBuffer
	if eventBuffer == 0 {
		eventBuffer = 400
	}
	requireAck := true
	if opts.RequireAcknowledgement != nil {
		requireAck = *opts.RequireAcknowledgement
	}
	if opts.AutoArm {
		// There is no operator in this role, so there is nobody to
		// acknowledge a checklist.  The blockers still apply.
		requireAck = false
	}
	initialMode := opts.InitialMode
	if initialMode == motor.CtrlStop {
		initialMode = motor.CtrlPosition
	}

	s := &Session{
		safety:      config,
		units:       opts.Units,
		preset:      "standard",
		connection:  Disconnected,
		mode:        motor.CtrlStop,
		rateHz:      clampRate(rate),
		params:      DefaultParams(),
		setpoint:    Setpoint{WithinLower: -1.0, WithinUpper: 1.0},
		shuttle:     defaultShuttle(),
		telemetry:   motor.NewTelemetry(),
		requireAck:  requireAck,
		autoArm:     opts.AutoArm,
		initialMode: initialMode,
		samples:     newSampleRing(sampleBuffer),
		events:      newEventRing(eventBuffer),
		wake:        make(chan struct{}, 1),
		t0:          time.Now(),
	}
	s.monitor = safety.NewMonitor(&s.safety)
	s.monitor.Units = s.units
	return s
}

// Connect opens a Link built from cfg and starts the control loop.
//
// allowUnknownBoard lets the operator proceed past a controller this build
// cannot positively identify.  It is a separate argument rather than a config
// field because it is a decision, not a setting: someone has to look at the
// hardware and say yes.
func (s *Session) Connect(ctx context.Context, cfg motor.Config, allowUnknownBoard bool) error {
	s.mu.Lock()
	if s.connection != Disconnected {
		s.mu.Unlock()
		return errors.New("session: already connected")
	}
	s.mu.Unlock()

	s.log(LevelInfo, "Connecting…", fmt.Sprintf("driver=%s port=%s id=%d",
		cfg.Driver, orAuto(cfg.Port), cfg.ID))

	link, err := motor.New(cfg)
	if err != nil {
		s.failConnect(err)
		return err
	}
	return s.ConnectLink(ctx, link, allowUnknownBoard)
}

// ConnectLink is Connect for a Link the caller built.
//
// It is what makes several controllers on one shared bus possible: build the
// transport once, wrap it in one Link per controller, and give each Link its
// own Session.  Tests use it to drive a simulator they hold a reference to.
//
// The Session takes ownership from here: Disconnect closes the Link.
func (s *Session) ConnectLink(ctx context.Context, link motor.Link, allowUnknownBoard bool) error {
	s.mu.Lock()
	if s.connection == Connected {
		s.mu.Unlock()
		return errors.New("session: already connected")
	}
	s.connection = Connecting
	s.connectError = ""
	s.identity = nil
	s.armBlockers = nil
	s.armAdvisories = nil
	s.mu.Unlock()

	if err := link.Open(ctx); err != nil {
		s.failConnect(err)
		return err
	}

	identity, err := link.Identify(ctx)
	if err != nil {
		s.log(LevelWarn, "Board identification failed", err.Error())
	}

	if identity != nil && identity.Status == motor.BoardRejected {
		_ = link.Close()
		message := fmt.Sprintf("Unsupported board: %s.", identity.Board)
		s.failConnect(errors.New(message))
		s.log(LevelError, "Refused to control an unsupported board", message)
		return errors.New(message)
	}
	// A device that answered with a revision this build does not recognise is
	// worth stopping for — the table may be stale, but so may the assumption.
	// A device that cannot report its identity at all is not: see
	// motor.BoardDeclared.
	if identity != nil && identity.Status == motor.BoardUnknown && !allowUnknownBoard {
		_ = link.Close()
		// Name the control the operator can actually see.  Telling someone to
		// pass "allowUnknownBoard" when the screen in front of them says
		// "Allow unidentified board" is a dead end dressed as an instruction.
		message := fmt.Sprintf(
			"Could not confirm what is attached: %s%s. "+
				"Tick \"Allow unidentified board\" (or pass -allow-unknown-board) "+
				"if you know what the hardware is.",
			identity.Board, revisionSuffix(identity))
		s.failConnect(errors.New(message))
		s.log(LevelWarn, "Board not recognised", message)
		return errors.New(message)
	}
	if identity != nil {
		identity.Accepted = true
		identity.Overridden = identity.Status == motor.BoardUnknown
	}

	// Fit the voltage trip band to the hardware before the loop starts, so a
	// band inherited from a different class of controller cannot stop the axis
	// on the first cycle.  Everything else in the envelope stays as declared.
	var voltageNotes []string
	if envelope := envelopeFor(link); envelope != nil {
		s.mu.Lock()
		voltageNotes = s.safety.FitVoltageTo(*envelope)
		s.mu.Unlock()
	}

	loopCtx, cancel := context.WithCancel(context.Background())

	s.mu.Lock()
	s.link = link
	s.linkDesc = link.Describe()
	s.identity = identity
	s.connection = Connected
	s.armed = false
	s.mode = motor.CtrlStop
	s.estop = false
	s.estopCode, s.estopReason, s.estopDetail = "", "", ""
	s.latchedFault = 0
	s.telemetry = motor.NewTelemetry()
	s.paramsDirty = true
	// A short burst of stops at connect time, because a controller left
	// driving by a previous session keeps driving until it is told otherwise.
	s.stopBurst = 5
	s.cycleCount = 0
	s.stats = loopStats{}
	s.heartbeatAt = time.Now()
	s.monitor.Reset()
	s.cancel = cancel
	s.done = make(chan struct{})
	rate := s.rateHz
	s.mu.Unlock()

	go s.run(loopCtx)

	board := "controller"
	firmware := "?"
	if identity != nil {
		board, firmware = identity.Board, identity.Firmware
	}
	s.log(LevelOK, "Connected to "+board,
		fmt.Sprintf("firmware %s · %d Hz control period", firmware, rate))
	for _, note := range voltageNotes {
		s.log(LevelWarn, "Safety limits adjusted for this hardware", note)
	}
	return nil
}

func (s *Session) failConnect(err error) {
	s.mu.Lock()
	s.connection = Disconnected
	s.connectError = err.Error()
	s.mu.Unlock()
	s.log(LevelError, "Connection failed", err.Error())
}

// Disconnect stops the loop and releases the transport.
func (s *Session) Disconnect() error {
	s.mu.Lock()
	cancel, done := s.cancel, s.done
	link := s.link
	s.cancel, s.done, s.link = nil, nil, nil
	s.mu.Unlock()

	if cancel != nil {
		cancel()
		s.signal()
		select {
		case <-done:
		case <-time.After(2 * time.Second):
			// The loop is wedged in a transaction.  Closing the link below
			// unblocks it; we do not wait longer than this for the courtesy.
		}
	}

	var err error
	if link != nil {
		err = link.Close()
	}

	s.mu.Lock()
	s.connection = Disconnected
	s.armed = false
	s.mode = motor.CtrlStop
	s.telemetry = motor.NewTelemetry()
	s.identity = nil
	s.linkDesc = nil
	s.armBlockers, s.armAdvisories = nil, nil
	s.mu.Unlock()

	s.log(LevelInfo, "Disconnected", "")
	return err
}

// Preflight re-runs the arming checklist and returns what stands in the way.
func (s *Session) Preflight() (blocking, advisory []string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.preflightLocked()
}

func (s *Session) preflightLocked() (blocking, advisory []string) {
	blocking, advisory = s.monitor.Preflight(s.identity, s.telemetry, s.connection == Connected)

	lower, upper, hasBounds := s.identity.Bounds()
	if hasBounds && !s.params.IgnorePositionBounds {
		if motor.Valid(s.telemetry.Position) &&
			(s.telemetry.Position < lower || s.telemetry.Position > upper) {
			units := motor.Render(s.units)
			blocking = append(blocking, fmt.Sprintf(
				"Axis is at %s, outside the controller's own range [%s, %s]. "+
					"Position control would fault immediately. Turn the shaft back inside by hand, "+
					"re-reference the output while disarmed, or set IgnorePositionBounds.",
				units.Position(s.telemetry.Position), units.Position(lower), units.Position(upper)))
		}

		units := motor.Render(s.units)
		message := fmt.Sprintf(
			"Controller limits motion to [%s, %s]; targets are clamped to it.",
			units.Position(lower), units.Position(upper))
		if span := upper - lower; span < 0.1 {
			message += fmt.Sprintf(
				" That is only %s of travel — if you expected a free axis, "+
					"widen the controller's bounds or set IgnorePositionBounds.", units.Span(span))
		}
		advisory = append(advisory, message)
	} else if s.identity != nil && !hasBounds {
		advisory = append(advisory,
			"Controller reports no travel bounds; only the host envelope applies.")
	}

	if s.params.IgnorePositionBounds {
		advisory = append(advisory, "The controller's own position bounds are being bypassed.")
	}
	if s.params.DeferTorqueToController {
		advisory = append(advisory,
			"The torque limit is handed to the controller; the host torque ceiling does not apply.")
	}
	if s.estop {
		blocking = append([]string{"Emergency stop is latched — reset it first."}, blocking...)
	}
	if s.stats.consecutiveTimeouts > 0 {
		blocking = append(blocking, "The controller is not replying reliably.")
	}

	s.armBlockers, s.armAdvisories = blocking, advisory
	return blocking, advisory
}

// Arm permits motion.
//
// It refuses while anything is blocking, and — unless the Session was built
// with RequireAcknowledgement disabled — it refuses until the caller confirms
// the operator has seen the advisories.
func (s *Session) Arm(acknowledged bool) error {
	s.mu.Lock()
	blocking, advisory := s.preflightLocked()
	if len(blocking) > 0 {
		s.mu.Unlock()
		s.log(LevelWarn, "Arming refused", join(blocking))
		return fmt.Errorf("session: cannot arm: %s", blocking[0])
	}
	if s.requireAck && !acknowledged {
		s.mu.Unlock()
		return errors.New("session: safety checklist not acknowledged")
	}

	s.mode = motor.CtrlStop
	if s.stopBurst < 3 {
		s.stopBurst = 3
	}
	s.paramsDirty = true
	s.monitor.Reset()
	s.armed = true
	s.heartbeatAt = time.Now()
	maxTorque, maxVelocity := s.safety.MaxTorque, s.safety.MaxVelocity
	s.mu.Unlock()

	units := motor.Render(s.units)
	s.log(LevelOK, "Armed", fmt.Sprintf(
		"torque limit %s · velocity limit %s", units.Torque(maxTorque), units.Velocity(maxVelocity)))
	if len(advisory) > 0 {
		s.log(LevelWarn, "Advisories at arm time", join(advisory))
	}
	s.signal()
	return nil
}

// Disarm forbids motion and commands a stop.
func (s *Session) Disarm(reason string) {
	s.mu.Lock()
	wasArmed := s.armed
	s.stopShuttleLocked("axis disarmed", LevelInfo)
	s.stopTrajectoryLocked("axis disarmed", LevelInfo)
	s.armed = false
	s.mode = motor.CtrlStop
	s.stopBurst = 8
	s.mu.Unlock()

	if wasArmed {
		s.log(LevelInfo, "Disarmed", reason)
	}
	s.signal()
}

// EStop latches an emergency stop.  It is idempotent: the first call logs, the
// rest only refresh the reason.
func (s *Session) EStop(code, message, detail string) {
	s.mu.Lock()
	first := !s.estop
	s.stopShuttleLocked("emergency stop", LevelWarn)
	s.stopTrajectoryLocked("emergency stop", LevelWarn)
	s.estop = true
	s.estopCode, s.estopReason, s.estopDetail = code, message, detail
	s.armed = false
	s.mode = motor.CtrlStop
	// A burst rather than a single stop: the one command that matters most is
	// the one most worth sending several times.
	s.stopBurst = 20
	s.mu.Unlock()

	if first {
		s.log(LevelError, "E-STOP · "+message, detail)
	}
	s.signal()
}

// ResetEStop clears a latched stop.  The axis stays disarmed afterwards, so
// clearing a stop can never by itself put a machine back in motion.
func (s *Session) ResetEStop() error {
	s.mu.Lock()
	if !s.estop {
		s.mu.Unlock()
		return nil
	}
	s.estop = false
	s.estopCode, s.estopReason, s.estopDetail = "", "", ""
	s.latchedFault = 0
	s.monitor.Reset()
	s.paramsDirty = true
	s.mu.Unlock()

	s.log(LevelOK, "Emergency stop reset", "The axis stays disarmed.")
	return nil
}

// SetMode selects what the loop commands.
func (s *Session) SetMode(mode motor.ControlMode) error {
	s.mu.Lock()
	if s.link != nil && !s.link.Capabilities().Supports(mode) {
		s.mu.Unlock()
		return fmt.Errorf("session: %s does not support %s mode", s.link.Kind(), mode)
	}
	if mode == s.mode {
		// Nothing to change.  Returning early also protects the setpoint: the
		// "adopt the current position" step below is there to stop a mode
		// change from becoming a move, and re-running it would drag a live
		// target to wherever the axis happens to be.
		s.mu.Unlock()
		return nil
	}
	if mode != motor.CtrlPosition {
		s.stopShuttleLocked("control mode changed", LevelInfo)
		s.stopTrajectoryLocked("control mode changed", LevelInfo)
	}
	if mode != motor.CtrlStop && !s.armed {
		s.mu.Unlock()
		return errors.New("session: arm the axis first")
	}
	if s.estop {
		s.mu.Unlock()
		return errors.New("session: emergency stop is latched")
	}

	// Adopt the current position rather than servoing to a stale target: a
	// mode change must never itself be a move command.
	if motor.Valid(s.telemetry.Position) {
		switch mode {
		case motor.CtrlPosition:
			s.setpoint.Position = s.telemetry.Position
		case motor.CtrlStayWithin:
			span := max(0.1, (s.setpoint.WithinUpper-s.setpoint.WithinLower)/2)
			s.setpoint.WithinLower = s.telemetry.Position - span
			s.setpoint.WithinUpper = s.telemetry.Position + span
		}
	}
	if mode == motor.CtrlVelocity {
		s.setpoint.Velocity = 0
	}

	s.mode = mode
	s.paramsDirty = true
	if mode == motor.CtrlStop && s.stopBurst < 3 {
		s.stopBurst = 3
	}
	s.mu.Unlock()

	s.log(LevelInfo, "Mode: "+mode.String(), "")
	s.signal()
	return nil
}

// SetPosition sets the position target, clamped to the reachable range.
func (s *Session) SetPosition(rev float64) float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.setpoint.Position, _ = s.clampTargetLocked(rev)
	s.signal()
	return s.setpoint.Position
}

// SetVelocity sets the velocity target, clamped to the velocity ceiling.
func (s *Session) SetVelocity(revPerSec float64) float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.setpoint.Velocity, _ = s.safety.ClampVelocity(revPerSec)
	s.signal()
	return s.setpoint.Velocity
}

// SetWithin sets the stay-within bounds.
func (s *Session) SetWithin(lower, upper float64) (float64, float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.setpoint.WithinLower, _ = s.safety.ClampPosition(lower)
	s.setpoint.WithinUpper, _ = s.safety.ClampPosition(upper)
	s.paramsDirty = true
	s.signal()
	return s.setpoint.WithinLower, s.setpoint.WithinUpper
}

// SetParams updates the control parameters, clamped to the safety envelope.
func (s *Session) SetParams(params Params) Params {
	s.mu.Lock()
	previouslyDeferred := s.params.DeferTorqueToController
	previouslyBypassed := s.params.IgnorePositionBounds

	params.MaxTorque = motor.Clamp(params.MaxTorque, 0, s.safety.MaxTorque)
	params.VelocityLimit = motor.Clamp(params.VelocityLimit, 0, s.safety.MaxVelocity)
	params.AccelLimit = motor.Clamp(params.AccelLimit, 0.01, max(0.01, s.safety.AccelLimit))
	params.KpScale = motor.Clamp(params.KpScale, 0, s.safety.MaxKpScale)
	params.KdScale = motor.Clamp(params.KdScale, 0, s.safety.MaxKdScale)
	params.ILimitScale = motor.Clamp(params.ILimitScale, 0, 1.0)
	params.Feedforward = motor.Clamp(params.Feedforward, -s.safety.MaxTorque, s.safety.MaxTorque)

	s.params = params
	s.paramsDirty = true
	s.mu.Unlock()

	if params.DeferTorqueToController && !previouslyDeferred {
		s.log(LevelWarn, "Torque limit handed to the controller",
			"The host torque ceiling no longer applies. The current, stall and runaway guards still do.")
	}
	if params.IgnorePositionBounds && !previouslyBypassed {
		s.log(LevelWarn, "Controller position bounds bypassed",
			"The firmware travel envelope is ignored. Only the host envelope is left.")
	}
	s.signal()
	return params
}

// Params returns the current control parameters.
func (s *Session) Params() Params {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.params
}

// SetSafety replaces the safety envelope and reports what was adjusted.
func (s *Session) SetSafety(config safety.Config) (safety.Config, []string) {
	s.mu.Lock()
	var envelope *motor.Envelope
	if s.link != nil {
		envelope = envelopeFor(s.link)
	}
	notes := config.Normalise(envelope)
	s.safety = config
	s.preset = "custom"
	params := s.params
	s.mu.Unlock()

	// Re-apply the (possibly tighter) envelope to the live parameters, so the
	// reported values are the ones that will actually be commanded.
	s.SetParams(params)

	if len(notes) > 0 {
		s.log(LevelWarn, "Safety limits adjusted", join(notes))
	} else {
		s.log(LevelInfo, "Safety limits updated", "")
	}
	s.Preflight()

	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.safety, notes
}

// Safety returns the current envelope.
func (s *Session) Safety() safety.Config {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.safety
}

// SetPreset applies a motion-trip sensitivity preset.
func (s *Session) SetPreset(name string) error {
	preset, ok := safety.Presets[name]
	if !ok {
		return fmt.Errorf("session: unknown safety preset %q", name)
	}
	s.mu.Lock()
	preset.Apply(&s.safety)
	s.safety.Normalise(nil)
	s.preset = name
	s.mu.Unlock()

	s.log(LevelInfo, "Safety sensitivity: "+preset.Label, preset.Description)
	return nil
}

// SetRate changes the control rate.
func (s *Session) SetRate(hz int) int {
	s.mu.Lock()
	rate := clampRate(hz)
	changed := rate != s.rateHz
	s.rateHz = rate
	if changed {
		s.paramsDirty = true
		s.stats.rateWindow = s.stats.rateWindow[:0]
	}
	watchdog := s.safety.WatchdogTimeout(1.0 / float64(rate))
	s.mu.Unlock()

	if changed {
		s.log(LevelInfo, fmt.Sprintf("Control rate set to %d Hz", rate),
			fmt.Sprintf("watchdog timeout %.0f ms", watchdog*1e3))
		s.signal()
	}
	return rate
}

// ZeroOutput re-references the output position.
//
// It is refused while armed.  Re-defining where the axis thinks it is while it
// is being driven would move the machine by the size of the correction.
func (s *Session) ZeroOutput(position float64) error {
	s.mu.Lock()
	if s.armed {
		s.mu.Unlock()
		return errors.New("session: disarm before re-referencing the output")
	}
	if s.connection != Connected {
		s.mu.Unlock()
		return errors.New("session: not connected")
	}
	cmd := motor.NewCommand(motor.CtrlSetOutput)
	cmd.Position = position
	s.pendingSpecial = &cmd
	s.mu.Unlock()

	s.log(LevelInfo, fmt.Sprintf("Output position re-referenced to %s",
		motor.Render(s.units).Position(position)),
		"Runtime only — the controller's stored configuration is untouched.")
	s.signal()
	return nil
}

// Heartbeat tells the session that its supervising client is still alive.
// Without one within safety.DeadmanTimeout, the loop latches a stop.
func (s *Session) Heartbeat() {
	s.mu.Lock()
	s.heartbeatAt = time.Now()
	s.mu.Unlock()
}

// clampTargetLocked constrains a position target to what is actually
// reachable: the host envelope intersected with the controller's own bounds.
func (s *Session) clampTargetLocked(value float64) (float64, bool) {
	lower, upper := s.effectiveLimitsLocked()
	return motor.ClampChanged(value, lower, upper)
}

// effectiveLimitsLocked returns the range a position command can reach.
func (s *Session) effectiveLimitsLocked() (lower, upper float64) {
	lower, upper = -1e6, 1e6
	if s.safety.PositionLimitEnabled {
		lower, upper = s.safety.PositionMin, s.safety.PositionMax
	}

	boundLower, boundUpper, ok := s.identity.Bounds()
	if ok && !s.params.IgnorePositionBounds {
		inset := min(boundsInset, (boundUpper-boundLower)*0.01)
		lower = max(lower, boundLower+inset)
		upper = min(upper, boundUpper-inset)
		if upper < lower {
			middle := (boundLower + boundUpper) / 2
			lower, upper = middle, middle
		}
	}
	return lower, upper
}

// signal nudges the control loop to run its next cycle immediately, so an
// operator action is not delayed by up to a full period.
func (s *Session) signal() {
	select {
	case s.wake <- struct{}{}:
	default:
	}
}

func clampRate(hz int) int {
	if hz < MinRateHz {
		return MinRateHz
	}
	if hz > MaxRateHz {
		return MaxRateHz
	}
	return hz
}

// revisionSuffix adds the family and revision to a message, but only when the
// device actually reported them — "(family -1, hwrev -1)" tells nobody
// anything except that the driver has no such concept.
func revisionSuffix(identity *motor.Identity) string {
	if identity.Family < 0 && identity.HWRev < 0 {
		return ""
	}
	return fmt.Sprintf(" (family %d, hwrev %d)", identity.Family, identity.HWRev)
}

func orAuto(port string) string {
	if port == "" {
		return "auto"
	}
	return port
}

func join(items []string) string {
	out := ""
	for i, item := range items {
		if i > 0 {
			out += " · "
		}
		out += item
	}
	return out
}
