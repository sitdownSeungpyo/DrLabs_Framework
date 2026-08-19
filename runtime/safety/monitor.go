package safety

import (
	"fmt"
	"time"

	"drlabs/motor"
)

// Trip is a latched reason to stop.
type Trip struct {
	// Code is a stable identifier, suitable for switching on.
	Code string
	// Message is one sentence stating what happened.
	Message string
	// Detail explains why it happened and what to do about it.  It is the
	// difference between an operator who reaches for the manual and one who
	// fixes the machine.
	Detail string
}

func (t *Trip) Error() string { return t.Message }

// timer measures how long a condition has held continuously, so that a
// momentary excursion does not latch a stop.
type timer struct {
	since time.Time
}

// check reports whether condition has been true without interruption for
// duration.
func (t *timer) check(condition bool, now time.Time, duration float64) bool {
	if !condition {
		t.since = time.Time{}
		return false
	}
	if t.since.IsZero() {
		t.since = now
	}
	return now.Sub(t.since).Seconds() >= duration
}

// Monitor evaluates telemetry against a Config and decides whether the machine
// must be stopped.
//
// It holds no lock: the control loop owns it and is the only thing that calls
// Evaluate.  Warnings are read by the status path, which copies them.
type Monitor struct {
	Config *Config

	// Units renders the positions, speeds and torques that appear in the
	// messages this monitor produces.  A nil value means motor space, which
	// is right for a session driving one controller directly; the robot layer
	// supplies a joint renderer so that a limit written in radians is not
	// reported back in revolutions.
	Units motor.Units

	current timer
	runaway timer
	stall   timer

	// Warnings are the non-fatal observations from the last evaluation.
	Warnings []string
}

// NewMonitor returns a Monitor bound to a Config.  The Config is held by
// pointer so that live edits take effect on the next cycle.
func NewMonitor(config *Config) *Monitor {
	return &Monitor{Config: config}
}

// Reset clears the trip timers.  Called on arm and after an emergency stop is
// cleared, so that time spent stopped does not count towards a trip.
func (m *Monitor) Reset() {
	m.current = timer{}
	m.runaway = timer{}
	m.stall = timer{}
	m.Warnings = nil
}

// Preflight is the check performed once, before the operator may arm.
//
// It returns two lists: blocking reasons that must be cleared first, and
// advisories the operator should read but may proceed past.  Keeping them
// apart matters — a system that treats every observation as a blocker teaches
// people to bypass it.
func (m *Monitor) Preflight(identity *motor.Identity, telemetry motor.Telemetry, connected bool) (blocking, advisory []string) {
	config := m.Config
	units := motor.Render(m.Units)

	if !connected {
		blocking = append(blocking, "Not connected.")
	}

	switch {
	case identity == nil:
		advisory = append(advisory, "Board identity could not be read; verify manually what is attached before arming.")
	case identity.Status == motor.BoardRejected:
		blocking = append(blocking, fmt.Sprintf(
			"Connected board is %q, which this build knows to be outside the supported range.", identity.Board))
	case identity.Status == motor.BoardUnknown:
		advisory = append(advisory, fmt.Sprintf(
			"Board %q was not positively identified; its electrical limits are assumed, not verified.", identity.Board))
	case identity.Status == motor.BoardDeclared:
		// This one never resolves: the device has no identity register, so the
		// declaration stays an assumption for as long as it is connected.  If
		// the wrong model is configured, every torque command is scaled by the
		// wrong factor and nothing on the bus will say so.
		advisory = append(advisory, fmt.Sprintf(
			"Board %q comes from configuration, not from the motor — it cannot report its own model. "+
				"Check the label on the actuator: a wrong model mis-scales torque and velocity.",
			identity.Board))
	}

	if telemetry.Mode < 0 && !motor.Valid(telemetry.Position) {
		blocking = append(blocking, "No telemetry received from the controller.")
		return blocking, advisory
	}

	if !motor.Valid(telemetry.Voltage) {
		advisory = append(advisory, "Bus voltage is not being reported.")
	} else {
		switch {
		case telemetry.Voltage < config.BusVoltageMin:
			blocking = append(blocking, fmt.Sprintf(
				"Bus voltage %.1f V is below the configured minimum %.1f V.",
				telemetry.Voltage, config.BusVoltageMin))
		case telemetry.Voltage > config.BusVoltageMax:
			blocking = append(blocking, fmt.Sprintf(
				"Bus voltage %.1f V is above the configured maximum %.1f V.",
				telemetry.Voltage, config.BusVoltageMax))
		}
	}

	if telemetry.HardFault {
		blocking = append(blocking, fmt.Sprintf(
			"Controller reports fault %d (%s). %s",
			telemetry.Fault, telemetry.FaultName, telemetry.FaultDetail))
	}

	if !motor.Valid(telemetry.Position) {
		blocking = append(blocking,
			"Output position is invalid — the encoder or the motor calibration is not usable.")
	} else if config.PositionLimitEnabled {
		// The same margin the running check allows, so that the two agree:
		// an axis may be armed wherever it would be allowed to keep running.
		//
		// Without it the arm check is stricter than the trip check, and the
		// gap is not theoretical — a target clamped to the envelope limit is
		// commanded exactly at the bound, and the axis settles a fraction of
		// a degree outside it through ordinary tracking error.  That is a
		// position this layer itself asked for, and it left the axis unable
		// to re-arm while the running check would only have warned.
		margin := config.PositionTripMargin
		if telemetry.Position < config.PositionMin-margin || telemetry.Position > config.PositionMax+margin {
			blocking = append(blocking, fmt.Sprintf(
				"Current position %s is outside the host safety envelope [%s, %s] plus %s of margin. "+
					"Widen the envelope, or move the axis back inside — the drive is off, so the shaft is free.",
				units.Position(telemetry.Position),
				units.Position(config.PositionMin), units.Position(config.PositionMax),
				units.Span(margin)))
		}
	}

	if motor.Valid(telemetry.Temperature) && telemetry.Temperature > config.BoardTempWarn {
		advisory = append(advisory, fmt.Sprintf("Board is already warm (%.0f °C).", telemetry.Temperature))
	}
	if config.MaxTorque > 2.0 {
		advisory = append(advisory, fmt.Sprintf(
			"Torque limit is set to %.2f Nm. Start low with an unfamiliar motor.", config.MaxTorque))
	}

	return blocking, advisory
}

// Evaluate examines one cycle's telemetry and returns a Trip when the machine
// must be stopped, or nil when it may continue.  It also refreshes Warnings.
//
// The armed flag matters more than it looks.  Several conditions — being
// outside the position envelope in particular — are a reason to refuse arming
// but not a reason to latch while disarmed: with the drive off, the operator's
// only way to fix it is to move the axis by hand, and tripping on that would
// make the fault unrecoverable.
func (m *Monitor) Evaluate(telemetry motor.Telemetry, commanded motor.Command, armed bool, now time.Time) *Trip {
	units := motor.Render(m.Units)
	config := m.Config
	var warnings []string
	var trip *Trip

	setTrip := func(candidate *Trip) {
		if trip == nil {
			trip = candidate
		}
	}

	// --- controller-reported problems ------------------------------------
	switch {
	case telemetry.HardFault:
		setTrip(&Trip{
			Code:    "controller_fault",
			Message: fmt.Sprintf("Controller fault %d: %s", telemetry.Fault, telemetry.FaultName),
			Detail:  telemetry.FaultDetail,
		})
	case telemetry.Fault > 0:
		// A limiting state, not a fault.  It is the single most useful thing
		// to surface when a motor accepts commands but does not move.
		warnings = append(warnings, telemetry.FaultName)
	}

	// --- electrical --------------------------------------------------------
	if motor.Valid(telemetry.Voltage) {
		switch {
		case telemetry.Voltage > config.BusVoltageMax:
			setTrip(&Trip{
				Code:    "over_voltage",
				Message: fmt.Sprintf("Bus voltage %.1f V above limit %.1f V", telemetry.Voltage, config.BusVoltageMax),
				Detail:  "Regeneration with no battery on the bus is the usual cause.",
			})
		case telemetry.Voltage < config.BusVoltageMin && armed:
			setTrip(&Trip{
				Code:    "under_voltage",
				Message: fmt.Sprintf("Bus voltage %.1f V below limit %.1f V", telemetry.Voltage, config.BusVoltageMin),
				Detail:  "The supply is sagging under load.",
			})
		case telemetry.Voltage > config.BusVoltageMax*0.95:
			warnings = append(warnings, "bus voltage near limit")
		}
	}

	if motor.Valid(telemetry.QCurrent) {
		over := abs(telemetry.QCurrent) > config.CurrentTrip
		if m.current.check(over, now, config.CurrentTripTime) {
			setTrip(&Trip{
				Code:    "over_current",
				Message: fmt.Sprintf("Q current %.1f A above limit %.1f A", telemetry.QCurrent, config.CurrentTrip),
				Detail:  fmt.Sprintf("Sustained for more than %.0f ms.", config.CurrentTripTime*1e3),
			})
		} else if over {
			warnings = append(warnings, "current above limit")
		}
	}

	// --- thermal -----------------------------------------------------------
	if motor.Valid(telemetry.Temperature) {
		switch {
		case telemetry.Temperature > config.BoardTempTrip:
			setTrip(&Trip{
				Code: "board_over_temperature",
				Message: fmt.Sprintf("Board temperature %.0f °C above limit %.0f °C",
					telemetry.Temperature, config.BoardTempTrip),
			})
		case telemetry.Temperature > config.BoardTempWarn:
			warnings = append(warnings, "board temperature high")
		}
	}
	if motor.Valid(telemetry.MotorTemperature) {
		switch {
		case telemetry.MotorTemperature > config.MotorTempTrip:
			setTrip(&Trip{
				Code: "motor_over_temperature",
				Message: fmt.Sprintf("Motor temperature %.0f °C above limit %.0f °C",
					telemetry.MotorTemperature, config.MotorTempTrip),
			})
		case telemetry.MotorTemperature > config.MotorTempWarn:
			warnings = append(warnings, "motor temperature high")
		}
	}

	// --- motion ------------------------------------------------------------
	if motor.Valid(telemetry.Position) && config.PositionLimitEnabled {
		margin := config.PositionTripMargin
		outside := telemetry.Position > config.PositionMax+margin ||
			telemetry.Position < config.PositionMin-margin

		switch {
		case outside && !armed:
			warnings = append(warnings, "outside soft position limits")
		case outside:
			setTrip(&Trip{
				Code:    "position_envelope",
				Message: fmt.Sprintf("Position %s left the soft envelope", units.Position(telemetry.Position)),
				Detail: fmt.Sprintf("Envelope [%s, %s] plus %s margin.",
					units.Position(config.PositionMin), units.Position(config.PositionMax),
					units.Span(margin)),
			})
		case telemetry.Position > config.PositionMax || telemetry.Position < config.PositionMin:
			warnings = append(warnings, "outside soft position limits")
		}
	}

	if armed && motor.Valid(telemetry.Velocity) {
		ceiling := max(0.05, config.MaxVelocity*config.RunawayVelocityFactor)
		if m.runaway.check(abs(telemetry.Velocity) > ceiling, now, config.RunawayTime) {
			setTrip(&Trip{
				Code:    "velocity_runaway",
				Message: fmt.Sprintf("Velocity %s exceeded %s",
					units.Velocity(telemetry.Velocity), units.Velocity(ceiling)),
				Detail: fmt.Sprintf("Held for %.1f s. The ceiling is the velocity limit %s × %.1f. "+
					"Raise the velocity ceiling, lower kp, or pick a looser preset if this was normal overshoot.",
					config.RunawayTime, units.Velocity(config.MaxVelocity), config.RunawayVelocityFactor),
			})
		}

		if config.StallDetectEnabled {
			// A stall is only meaningful against an intent to move: current
			// with no motion is exactly what holding a load looks like.
			wantsMotion := abs(commanded.VelocityTarget) > config.StallVelocity
			stalled := wantsMotion &&
				abs(telemetry.Velocity) < config.StallVelocity &&
				motor.Valid(telemetry.QCurrent) &&
				abs(telemetry.QCurrent) > config.StallCurrent

			if m.stall.check(stalled, now, config.StallTime) {
				setTrip(&Trip{
					Code:    "stall",
					Message: fmt.Sprintf("Stall detected: %.1f A with no motion", telemetry.QCurrent),
					Detail: fmt.Sprintf("Motion was commanded but the axis moved less than %s "+
						"while drawing more than %.1f A for %.1f s.",
						units.Velocity(config.StallVelocity), config.StallCurrent, config.StallTime),
				})
			}
		}
	}

	m.Warnings = warnings
	return trip
}

// ClampCommand applies every command clamp and returns the notes describing
// what was reduced.
//
// It is the last thing between an application's intent and the bus, so it runs
// on every cycle rather than only when a setpoint changes: a limit tightened
// mid-motion has to take effect immediately, not at the next user action.
func (m *Monitor) ClampCommand(cmd motor.Command) (motor.Command, []string) {
	config := m.Config
	var notes []string

	if position, changed := config.ClampPosition(cmd.Position); changed {
		cmd.Position = position
		notes = append(notes, "position clamped to the soft envelope")
	} else {
		cmd.Position = position
	}

	if velocity, changed := config.ClampVelocity(cmd.Velocity); changed {
		cmd.Velocity = velocity
		notes = append(notes, "velocity clamped to the velocity ceiling")
	} else {
		cmd.Velocity = velocity
	}

	// The torque ceiling is bypassed on purpose when the controller's own
	// limit has been requested; the current, stall and runaway guards are what
	// protect the machine in that mode.
	if !cmd.DeferTorqueToController {
		if torque, changed := config.ClampTorque(cmd.MaxTorque); changed {
			cmd.MaxTorque = torque
			notes = append(notes, "torque limit clamped to the torque ceiling")
		} else {
			cmd.MaxTorque = torque
		}
	}

	if motor.Valid(cmd.VelocityLimit) {
		cmd.VelocityLimit = motor.Clamp(cmd.VelocityLimit, 0, config.MaxVelocity)
	}
	if motor.Valid(cmd.AccelLimit) {
		cmd.AccelLimit = motor.Clamp(cmd.AccelLimit, 0.01, max(0.01, config.AccelLimit))
	}
	if motor.Valid(cmd.KpScale) {
		cmd.KpScale = motor.Clamp(cmd.KpScale, 0, config.MaxKpScale)
	}
	if motor.Valid(cmd.KdScale) {
		cmd.KdScale = motor.Clamp(cmd.KdScale, 0, config.MaxKdScale)
	}
	if motor.Valid(cmd.ILimitScale) {
		cmd.ILimitScale = motor.Clamp(cmd.ILimitScale, 0, 1.0)
	}
	if motor.Valid(cmd.FeedforwardTorque) {
		cmd.FeedforwardTorque = motor.Clamp(cmd.FeedforwardTorque, -config.MaxTorque, config.MaxTorque)
	}
	if motor.Valid(cmd.LowerBound) {
		cmd.LowerBound, _ = config.ClampPosition(cmd.LowerBound)
	}
	if motor.Valid(cmd.UpperBound) {
		cmd.UpperBound, _ = config.ClampPosition(cmd.UpperBound)
	}

	return cmd, notes
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}
