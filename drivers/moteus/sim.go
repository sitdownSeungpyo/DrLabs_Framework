package moteus

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	"drlabs/motor"
)

// SimLink is a software model of a moteus.
//
// It exists so that the control loop, the state machine and every safety trip
// can be exercised with nothing plugged in.  The model is small but honest:
// a trapezoidal trajectory generator feeding a torque-limited PD controller,
// driving an inertia with viscous friction, plus thermal rise and supply sag.
// That is enough for the interesting failures — overshoot, stall, thermal
// runaway, position-envelope escape — to actually happen rather than being
// faked.
//
// It always identifies itself as simulated, so nothing downstream can mistake
// it for hardware.
type SimLink struct {
	canID uint8

	// Physical parameters.
	Inertia        float64 // kg·m², rotor plus load
	Friction       float64 // Nm per rev/s
	TorqueConstant float64 // Nm/A

	// SubstepSeconds is the model's own integration step.  It is deliberately
	// finer than any host control period, so the physics stays stable whether
	// the caller runs at 20 Hz or 500 Hz.
	SubstepSeconds float64

	// DefaultTorque stands in for the controller's own configured limit, used
	// when a command defers the limit rather than setting one.
	DefaultTorque float64

	position float64
	velocity float64

	// The controller's internal trajectory state, distinct from the measured
	// axis: the difference between them is what generates torque.
	controlPosition float64
	controlVelocity float64

	boardTemp float64
	motorTemp float64
	voltage   float64
	mode      Mode
	fault     int

	last   time.Time
	params simParams
	open   bool
}

type simParams struct {
	maxTorque       float64
	velocityLimit   float64
	accelLimit      float64
	kpScale         float64
	kdScale         float64
	watchdogTimeout float64
}

// newSimLink builds a simulated moteus from a Config.
//
// Recognised options let a caller model a specific machine:
//
//	inertia          kg·m² (default 0.006)
//	friction         Nm per rev/s (default 0.02)
//	torque_constant  Nm/A (default 0.06)
func newSimLink(cfg motor.Config) (motor.Link, error) {
	if err := checkOptions(cfg, "inertia", "friction", "torque_constant"); err != nil {
		return nil, err
	}
	id := cfg.ID
	if id == 0 {
		id = 1
	}

	inertia, err := floatOption(cfg, "inertia", 0.006)
	if err != nil {
		return nil, err
	}
	friction, err := floatOption(cfg, "friction", 0.02)
	if err != nil {
		return nil, err
	}
	torqueConstant, err := floatOption(cfg, "torque_constant", 0.06)
	if err != nil {
		return nil, err
	}
	if inertia <= 0 || torqueConstant <= 0 {
		return nil, fmt.Errorf("moteus-sim: inertia and torque_constant must be positive")
	}

	return NewSimLink(uint8(id), inertia, friction, torqueConstant), nil
}

// NewSimLink builds a simulator directly.
func NewSimLink(canID uint8, inertia, friction, torqueConstant float64) *SimLink {
	return &SimLink{
		canID:          canID,
		Inertia:        inertia,
		Friction:       friction,
		TorqueConstant: torqueConstant,
		SubstepSeconds: 0.0005, // 2 kHz
		DefaultTorque:  1.0,
		boardTemp:      24.0,
		motorTemp:      24.0,
		voltage:        24.0,
		mode:           ModeStopped,
		params: simParams{
			velocityLimit:   1.0,
			accelLimit:      5.0,
			kpScale:         1.0,
			kdScale:         1.0,
			watchdogTimeout: 0.1,
		},
	}
}

// Kind identifies the driver.
func (s *SimLink) Kind() string { return "moteus-sim" }

// Describe reports the simulated connection.
func (s *SimLink) Describe() map[string]string {
	return map[string]string{
		"driver": "moteus-sim",
		"can_id": strconv.Itoa(int(s.canID)),
	}
}

// Open starts the model's clock.
func (s *SimLink) Open(context.Context) error {
	s.last = time.Now()
	s.open = true
	return nil
}

// Close stops the simulated axis.
func (s *SimLink) Close() error {
	s.mode = ModeStopped
	s.open = false
	return nil
}

// Identify reports a simulated r4.11.
func (s *SimLink) Identify(context.Context) (*motor.Identity, error) {
	identity := motor.NewIdentity("moteus")
	identity.Family = FamilyR4
	identity.HWRev = 8
	identity.Firmware = "simulated"
	identity.SerialNumber = "SIMULATOR"
	identity.Board = BoardName(FamilyR4, 8) + " (simulated)"
	identity.Status = BoardStatus(FamilyR4, 8)
	identity.Supported = true
	identity.Known = true
	identity.Simulated = true
	return identity, nil
}

// Capabilities match the real driver, so a simulated session exercises the
// same code paths a real one does.
func (s *SimLink) Capabilities() motor.Capabilities {
	return (&Link{}).Capabilities()
}

// Envelope reports the same ratings as the real driver, so a simulated session
// exercises the same safety arithmetic.
func (s *SimLink) Envelope() motor.Envelope { return (&Link{}).Envelope() }

// Cycle advances the model by the elapsed wall time and returns telemetry.
func (s *SimLink) Cycle(_ context.Context, cmd motor.Command, profile motor.QueryProfile) (*motor.Telemetry, error) {
	if !s.open {
		return nil, motor.ErrClosed
	}

	now := time.Now()
	dt := 0.001
	if !s.last.IsZero() {
		dt = math.Min(math.Max(now.Sub(s.last).Seconds(), 1e-4), 0.05)
	}
	s.last = now

	s.applyParams(cmd)

	maxTorque := s.params.maxTorque
	if cmd.DeferTorqueToController || math.IsNaN(maxTorque) {
		// NaN means "use the controller's own limit"; model that as the
		// firmware default rather than letting NaN into the physics, where it
		// would poison every value downstream of it.
		maxTorque = s.DefaultTorque
	}

	if cmd.Mode == motor.CtrlSetOutput {
		target := cmd.Position
		if math.IsNaN(target) {
			target = 0
		}
		offset := target - s.position
		s.position += offset
		s.controlPosition = s.position
	}

	steps := int(math.Max(1, math.Round(dt/s.SubstepSeconds)))
	h := dt / float64(steps)
	torque := 0.0

	for i := 0; i < steps; i++ {
		torque = s.step(cmd, maxTorque, h)
	}

	current := torque / s.TorqueConstant
	s.motorTemp += (0.6*current*current - 0.05*(s.motorTemp-24.0)) * dt
	s.boardTemp += (0.3*current*current - 0.04*(s.boardTemp-24.0)) * dt
	s.voltage = 24.0 - 0.05*math.Abs(current) + 0.02*math.Sin(float64(now.UnixNano())/1e9)

	return s.telemetry(torque, current, profile), nil
}

// applyParams records the limits carried by this command.  A NaN field means
// the command did not mention that parameter, so the previous value stands —
// exactly the sticky-register behaviour of the real controller.
func (s *SimLink) applyParams(cmd motor.Command) {
	if !math.IsNaN(cmd.MaxTorque) {
		s.params.maxTorque = cmd.MaxTorque
	}
	if cmd.DeferTorqueToController {
		s.params.maxTorque = math.NaN()
	}
	if !math.IsNaN(cmd.VelocityLimit) {
		s.params.velocityLimit = cmd.VelocityLimit
	}
	if !math.IsNaN(cmd.AccelLimit) {
		s.params.accelLimit = cmd.AccelLimit
	}
	if !math.IsNaN(cmd.KpScale) {
		s.params.kpScale = cmd.KpScale
	}
	if !math.IsNaN(cmd.KdScale) {
		s.params.kdScale = cmd.KdScale
	}
	if !math.IsNaN(cmd.WatchdogTimeout) {
		s.params.watchdogTimeout = cmd.WatchdogTimeout
	}
}

// step integrates one substep and returns the torque applied.
func (s *SimLink) step(cmd motor.Command, maxTorque, h float64) float64 {
	torque := 0.0

	switch cmd.Mode {
	case motor.CtrlPosition, motor.CtrlVelocity, motor.CtrlStayWithin, motor.CtrlTorque:
		if cmd.Mode == motor.CtrlStayWithin {
			s.mode = ModeStayWithin
		} else {
			s.mode = ModePosition
		}

		velocityLimit := math.Max(0.001, s.params.velocityLimit)
		accelLimit := math.Max(0.01, s.params.accelLimit)

		target := cmd.Position
		targetVelocity := cmd.Velocity
		if math.IsNaN(targetVelocity) {
			targetVelocity = 0
		}

		var desiredVelocity float64
		if math.IsNaN(target) {
			desiredVelocity = math.Max(math.Min(targetVelocity, velocityLimit), -velocityLimit)
		} else {
			err := target - s.controlPosition
			// The fastest we may travel and still brake to a stop exactly on
			// the target under the acceleration limit.
			stopVelocity := math.Sqrt(math.Max(0, 2*accelLimit*math.Abs(err)))
			desiredVelocity = math.Copysign(math.Min(velocityLimit, stopVelocity), err)
			if math.Abs(err) < 1e-4 {
				desiredVelocity = 0
			}
		}

		dv := math.Max(math.Min(desiredVelocity-s.controlVelocity, accelLimit*h), -accelLimit*h)
		s.controlVelocity += dv
		s.controlPosition += s.controlVelocity * h

		kp := 120.0 * s.params.kpScale
		kd := 6.0 * s.params.kdScale
		torque = kp*(s.controlPosition-s.position) + kd*(s.controlVelocity-s.velocity)
		if !math.IsNaN(cmd.FeedforwardTorque) {
			torque += cmd.FeedforwardTorque
		}
		torque = math.Max(math.Min(torque, maxTorque), -maxTorque)

	case motor.CtrlHold:
		s.mode = ModeZeroVelocity
		torque = math.Max(math.Min(-6.0*s.velocity, maxTorque), -maxTorque)
		s.controlPosition = s.position
		s.controlVelocity = 0

	case motor.CtrlBrake:
		s.mode = ModeBrake
		torque = -0.2 * s.velocity
		s.controlPosition = s.position
		s.controlVelocity = 0

	default:
		s.mode = ModeStopped
		torque = 0
		s.controlPosition = s.position
		s.controlVelocity = 0
	}

	accel := (torque - s.Friction*s.velocity) / s.Inertia
	s.velocity += accel * h
	s.position += s.velocity * h
	return torque
}

func (s *SimLink) telemetry(torque, current float64, profile motor.QueryProfile) *motor.Telemetry {
	out := motor.NewTelemetry()
	out.Mode = int(s.mode)
	out.ModeName = ModeName(out.Mode)
	out.Fault = s.fault
	fault := FaultText(s.fault)
	out.FaultName = fault.Name
	out.FaultDetail = fault.Detail
	out.HardFault = IsHardFault(s.fault)

	out.Position = s.position
	out.Velocity = s.velocity
	out.Torque = torque
	out.QCurrent = current
	out.Voltage = s.voltage
	out.Temperature = s.boardTemp

	switch profile {
	case motor.ProfileFull:
		out.DCurrent = 0
		out.Power = math.Abs(current) * s.voltage * 0.5
		out.MotorTemperature = s.motorTemp
		out.TrajectoryComplete = 0
		if math.Abs(s.controlVelocity) < 1e-6 {
			out.TrajectoryComplete = 1
		}
		out.HomeState = 0
	case motor.ProfileVerify:
		out.ActiveMaxTorque = s.params.maxTorque
		out.ActiveTimeout = s.params.watchdogTimeout
		out.ActiveVelocityLimit = s.params.velocityLimit
		out.ActiveAccelLimit = s.params.accelLimit
	}
	return &out
}

// InjectFault forces the simulator into a fault, so that fault handling can be
// tested without damaging anything.
func (s *SimLink) InjectFault(code int) {
	s.fault = code
	if IsHardFault(code) {
		s.mode = ModeFault
	}
}

func floatOption(cfg motor.Config, key string, def float64) (float64, error) {
	text := cfg.Option(key, "")
	if text == "" {
		return def, nil
	}
	value, err := strconv.ParseFloat(text, 64)
	if err != nil {
		return 0, fmt.Errorf("moteus-sim: option %s=%q is not a number", key, text)
	}
	return value, nil
}
