package robstride

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	"drlabs/motor"
)

// SimLink is a software model of a RobStride actuator.
//
// It models the controller as the manual describes it — torque is
// Kp·(p−p̂) + Kd·(v−v̂) + t_ff, bounded by limit_torque — driving an inertia
// with viscous friction, plus thermal rise.  That is enough for the
// interesting failures to actually happen: overshoot, stall, thermal rise,
// escaping a position envelope.
//
// It also enforces the parts of the protocol that are easy to get wrong, so
// that a mistake shows up in a test rather than on a bench: commands are
// ignored until the motor is enabled, and the run mode decides which setpoint
// is in charge.
type SimLink struct {
	canID uint8
	model Model

	// Physical parameters.
	Inertia        float64 // kg·m²
	Friction       float64 // Nm per rad/s
	SubstepSeconds float64

	// kp and kd are what a gain scale of 1.0 maps to, as in the real Link.
	kp float64
	kd float64

	position float64 // rad, continuous
	velocity float64 // rad/s
	temp     float64 // °C
	vbus     float64 // V

	runMode RunMode
	enabled bool
	flags   uint16

	// Parameter table state that the driver actually uses.
	limitTorque float64
	spdRef      float64
	locRef      float64
	limitSpd    float64

	// The CSP profile generator's internal state, distinct from the measured
	// axis: the difference between them is what generates torque.
	controlPosition float64
	controlVelocity float64
	haveControl     bool

	positionMode RunMode

	last time.Time
	open bool

	turns        float64
	lastRawAngle float64
	haveRaw      bool
}

// newSimLink builds a simulated RobStride from a Config.
//
// It takes the same required model option as the real driver, because the
// point of the simulator is to exercise the same scaling and the same limits.
func newSimLink(cfg motor.Config) (motor.Link, error) {
	if err := checkOptions(cfg, "model", "inertia", "friction", "kp", "kd", "host_id", "timeout_ms"); err != nil {
		return nil, err
	}
	model, err := requireModel(cfg)
	if err != nil {
		return nil, err
	}

	id := cfg.ID
	if id == 0 {
		id = 1
	}
	inertia, err := floatOption(cfg, "inertia", defaultInertia(model), 1e-6, 1e3)
	if err != nil {
		return nil, err
	}
	friction, err := floatOption(cfg, "friction", 0.02, 0, 1e3)
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

	sim := NewSimLink(uint8(id), model, inertia, friction)
	sim.kp, sim.kd = kp, kd
	return sim, nil
}

// defaultInertia scales with the model's torque rating, so that a simulated
// RS04 does not accelerate like a simulated RS05.
func defaultInertia(model Model) float64 {
	return 0.002 + model.TorqueRange*0.0005
}

// NewSimLink builds a simulator directly.
func NewSimLink(canID uint8, model Model, inertia, friction float64) *SimLink {
	return &SimLink{
		canID:          canID,
		model:          model,
		Inertia:        inertia,
		Friction:       friction,
		SubstepSeconds: 0.0005, // 2 kHz, independent of the host rate
		kp:             model.DefaultKp,
		kd:             model.DefaultKd,
		temp:           24,
		vbus:           48,
		limitTorque:    model.TorqueLimitMax,
		limitSpd:       model.SpeedLimitMax,
		positionMode:   RunPositionCSP,
	}
}

// Kind identifies the driver.
func (s *SimLink) Kind() string { return "robstride-sim" }

// Describe reports the simulated connection.
func (s *SimLink) Describe() map[string]string {
	return map[string]string{
		"driver": "robstride-sim",
		"model":  s.model.Name,
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
	s.enabled = false
	s.open = false
	return nil
}

// Identify reports the configured model, marked as unverified exactly as the
// real driver does — a simulator that claimed more certainty than the hardware
// can provide would hide the gap rather than model it.
func (s *SimLink) Identify(context.Context) (*motor.Identity, error) {
	identity := motor.NewIdentity("robstride")
	identity.Board = s.model.Name + " (simulated)"
	identity.SerialNumber = "SIMULATOR"
	identity.Status = motor.BoardDeclared
	identity.Known = true
	identity.Simulated = true
	return identity, nil
}

// Capabilities match the real driver.
func (s *SimLink) Capabilities() motor.Capabilities {
	return (&Link{model: s.model}).Capabilities()
}

// Envelope matches the real driver.
func (s *SimLink) Envelope() motor.Envelope {
	return (&Link{model: s.model}).Envelope()
}

// InjectFault sets the summary fault bits, so fault handling can be tested
// without damaging anything.
func (s *SimLink) InjectFault(flags uint16) {
	s.flags = flags
	if flags != 0 {
		s.enabled = false
	}
}

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

	control, err := s.applyCommand(cmd)
	if err != nil {
		return nil, err
	}

	steps := int(math.Max(1, math.Round(dt/s.SubstepSeconds)))
	h := dt / float64(steps)
	torque := 0.0
	for i := 0; i < steps; i++ {
		torque = s.step(control, h)
	}

	// The thermal model is crude but monotonic in the thing that matters:
	// sustained torque heats the actuator.
	normalised := torque / math.Max(s.model.TorqueRange, 1e-6)
	s.temp += (40*normalised*normalised - 0.05*(s.temp-24)) * dt
	s.vbus = 48 - 2*math.Abs(normalised)

	telemetry := s.telemetry(torque, profile)
	return &telemetry, nil
}

// applyCommand updates the simulated controller's state and returns the
// operation-mode command in force, if any.
func (s *SimLink) applyCommand(cmd motor.Command) (*ControlCommand, error) {
	if motor.Valid(cmd.MaxTorque) {
		s.limitTorque = motor.Clamp(cmd.MaxTorque, 0, s.model.TorqueLimitMax)
	}

	switch cmd.Mode {
	case motor.CtrlStop, motor.CtrlQuery:
		s.enabled = false
		return nil, nil

	case motor.CtrlSetOutput:
		if motor.Valid(cmd.Position) && math.Abs(cmd.Position) > 1e-9 {
			return nil, fmt.Errorf(
				"robstride: the output can only be re-referenced to zero, not to %.4f rev", cmd.Position)
		}
		if s.enabled {
			return nil, fmt.Errorf("robstride: stop the motor before re-referencing the output")
		}
		s.position = 0
		s.turns, s.haveRaw = 0, false
		return nil, nil

	case motor.CtrlBrake, motor.CtrlStayWithin:
		return nil, motor.ErrUnsupportedMode

	case motor.CtrlVelocity:
		s.runMode = RunVelocity
		s.enabled = true
		target := 0.0
		if motor.Valid(cmd.Velocity) {
			target = motor.RevPerSecToRadPerSec(cmd.Velocity)
		}
		s.spdRef = motor.Clamp(target, -s.model.SpeedRefMax, s.model.SpeedRefMax)
		return nil, nil

	case motor.CtrlPosition:
		if s.positionMode == RunPositionCSP {
			s.runMode = RunPositionCSP
			s.enabled = true
			if motor.Valid(cmd.VelocityLimit) {
				s.limitSpd = motor.Clamp(motor.RevPerSecToRadPerSec(cmd.VelocityLimit),
					0, s.model.SpeedLimitMax)
			}
			s.locRef = motor.RevToRad(valueOr(cmd.Position, 0))
			if !s.haveControl {
				s.controlPosition = s.position
				s.haveControl = true
			}
			return nil, nil
		}
		fallthrough

	case motor.CtrlTorque, motor.CtrlHold:
		s.runMode = RunOperation
		s.enabled = true
		control := (&Link{model: s.model, kp: s.kp, kd: s.kd}).controlCommand(cmd)
		return &control, nil
	}

	return nil, motor.ErrUnsupportedMode
}

// step integrates one substep and returns the torque applied.
func (s *SimLink) step(control *ControlCommand, h float64) float64 {
	torque := 0.0

	if s.enabled && s.flags == 0 {
		switch s.runMode {
		case RunVelocity:
			// The dedicated velocity loop: a PI controller, modelled here as
			// a stiff proportional term, which is why it holds a setpoint
			// under load where the operation mode's damping term alone sags.
			torque = 4.0 * s.model.TorqueRange / math.Max(s.model.VelocityRange, 1e-6) *
				(s.spdRef - s.velocity)

		case RunPositionCSP:
			// The controller's own profile generator: ramp a setpoint towards
			// the target at no more than limit_spd, then servo the shaft to
			// it.  This is what makes a step change in loc_ref a bounded move
			// rather than a full-torque lunge.
			limit := math.Max(0.01, s.limitSpd)
			err := s.locRef - s.controlPosition
			desired := math.Copysign(math.Min(limit, math.Abs(err)/math.Max(h, 1e-6)), err)
			if math.Abs(err) < 1e-6 {
				desired = 0
			}
			s.controlVelocity = desired
			s.controlPosition += s.controlVelocity * h

			torque = s.model.DefaultKp*(s.controlPosition-s.position) +
				s.model.DefaultKd*(s.controlVelocity-s.velocity)

		case RunOperation:
			if control != nil {
				// The controller's own formula, from the manual.
				target := unwrapTowards(control.Angle, s.position)
				torque = control.Kp*(target-s.position) +
					control.Kd*(control.Velocity-s.velocity) +
					control.Torque
			}
		}
	}

	torque = motor.Clamp(torque, -s.limitTorque, s.limitTorque)

	accel := (torque - s.Friction*s.velocity) / s.Inertia
	s.velocity += accel * h
	s.position += s.velocity * h
	return torque
}

// unwrapTowards resolves a wrapped ±4π target to the equivalent angle nearest
// the current position, which is what the real controller does internally and
// what stops a commanded wrap from looking like a full-speed unwind.
func unwrapTowards(target, current float64) float64 {
	window := 2 * PositionRange
	for target-current > PositionRange {
		target -= window
	}
	for current-target > PositionRange {
		target += window
	}
	return target
}

func (s *SimLink) telemetry(torque float64, profile motor.QueryProfile) motor.Telemetry {
	out := motor.NewTelemetry()

	// The simulated wire path goes through the same codec the real one does,
	// so a scaling mistake shows up here too rather than being bypassed.
	raw := Feedback{
		MotorID:     s.canID,
		Flags:       s.flags,
		Mode:        ModeReset,
		Angle:       decodeUint16(encodeUint16(wrapAngle(s.position), PositionRange), PositionRange),
		Velocity:    decodeUint16(encodeUint16(s.velocity, s.model.VelocityRange), s.model.VelocityRange),
		Torque:      decodeUint16(encodeUint16(torque, s.model.TorqueRange), s.model.TorqueRange),
		Temperature: math.Round(s.temp*10) / 10,
	}
	if s.enabled {
		raw.Mode = ModeRun
	}

	name, detail := raw.FaultText()
	out.Mode = int(raw.Mode)
	out.ModeName = ModeName(out.Mode)
	out.Fault = int(raw.Flags)
	out.FaultName = name
	out.FaultDetail = detail
	out.HardFault = raw.Faulted()

	out.Position = motor.RadToRev(s.unwrap(raw.Angle))
	out.Velocity = motor.RadPerSecToRevPerSec(raw.Velocity)
	out.Torque = raw.Torque
	out.Temperature = raw.Temperature
	out.MotorTemperature = raw.Temperature

	if profile != motor.ProfileFast {
		out.Voltage = s.vbus
		// The current the actuator would be drawing for this torque, using the
		// model's rated current at rated torque as the constant.
		out.QCurrent = torque / s.model.TorqueRange * s.model.CurrentRefMax
	}
	if profile == motor.ProfileVerify {
		out.ActiveMaxTorque = s.limitTorque
		// Only report limit_spd while the mode it governs is selected, as the
		// real driver does — otherwise the host verifies its velocity limit
		// against a parameter that is not in charge and warns for ever.
		if s.runMode == RunPositionCSP {
			out.ActiveVelocityLimit = motor.RadPerSecToRevPerSec(s.limitSpd)
		}
	}
	return out
}

// unwrap mirrors the real Link's turn counting, so the simulator reports a
// continuous multi-turn position the same way.
func (s *SimLink) unwrap(raw float64) float64 {
	if !s.haveRaw {
		s.lastRawAngle = raw
		s.haveRaw = true
		return raw
	}
	delta := raw - s.lastRawAngle
	window := 2 * PositionRange
	switch {
	case delta > PositionRange:
		s.turns -= window
	case delta < -PositionRange:
		s.turns += window
	}
	s.lastRawAngle = raw
	return raw + s.turns
}
