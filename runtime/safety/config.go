// Package safety is the host-side protection layer.
//
// Motor controllers carry their own protections — over and under voltage, over
// temperature, position bounds, command watchdog.  Those protect the *board*.
//
// This package adds a second layer that protects the *machine*.  The motor,
// the gearbox and whatever they are bolted to are not the controller
// manufacturer's products, so their envelope has to be declared by the
// operator and enforced on the host, before anything reaches the bus.
//
// There are two kinds of check:
//
//	clamps  applied to every outgoing command; they limit values silently
//	        because a command that has been reduced is still a command
//	trips   conditions observed in telemetry that latch an emergency stop
//
// Nothing here is ever written to a controller's persistent configuration.
package safety

import (
	"fmt"
	"math"

	"drlabs/motor"
)

// Config is the operator-declared envelope.
//
// Defaults are deliberately timid — one turn of travel, one newton-metre, two
// revolutions per second — because the first power-up of an unfamiliar motor
// is exactly when a generous default does damage.
type Config struct {
	// --- command clamps ---------------------------------------------------

	MaxTorque   float64 // Nm
	MaxVelocity float64 // rev/s
	AccelLimit  float64 // rev/s²
	MaxKpScale  float64 // 0..1
	MaxKdScale  float64 // 0..1

	// --- soft position envelope -------------------------------------------

	PositionLimitEnabled bool
	PositionMin          float64 // rev
	PositionMax          float64 // rev
	// PositionTripMargin is how far past the envelope the axis may travel
	// before the stop latches.  Without it, ordinary overshoot at the edge of
	// the range would trip on every approach.
	PositionTripMargin float64 // rev

	// --- electrical trips --------------------------------------------------

	BusVoltageMin   float64 // V
	BusVoltageMax   float64 // V
	CurrentTrip     float64 // A
	CurrentTripTime float64 // s the current must stay over the limit

	// --- thermal trips ------------------------------------------------------

	BoardTempWarn float64 // °C
	BoardTempTrip float64 // °C
	MotorTempWarn float64 // °C
	MotorTempTrip float64 // °C

	// --- motion trips -------------------------------------------------------

	// RunawayVelocityFactor multiplies MaxVelocity to get the ceiling above
	// which motion counts as a runaway.  A factor rather than an absolute
	// value so that raising the velocity limit raises the trip with it.
	RunawayVelocityFactor float64
	RunawayTime           float64 // s

	StallDetectEnabled bool
	StallCurrent       float64 // A
	StallVelocity      float64 // rev/s below which the axis counts as still
	StallTime          float64 // s

	// --- communication ------------------------------------------------------

	MaxConsecutiveTimeouts int

	// WatchdogPeriods sets the controller's command timeout as a multiple of
	// the control period, so it scales with the loop rate instead of needing
	// to be retuned every time the rate changes.
	WatchdogPeriods float64
	// WatchdogMin is a floor chosen from measured USB behaviour: a USB-CAN
	// transaction is well under a millisecond typically but can reach ~20 ms
	// on a loaded Windows host, and a watchdog below that would fault the
	// controller on an ordinary scheduling hiccup.
	WatchdogMin float64 // s
	WatchdogMax float64 // s

	// DeadmanTimeout stops the axis when the supervising client stops talking
	// to the session.  Zero disables it.
	DeadmanTimeout float64 // s

	// --- operator confirmations ---------------------------------------------

	// LargeMove is the move size beyond which a UI should ask for
	// confirmation.  It is advisory: the runtime does not enforce it.
	LargeMove float64 // rev
}

// DefaultConfig returns a conservative envelope suitable for a first
// power-up: half a turn either side of zero, 1 Nm, 2 rev/s.
func DefaultConfig() Config {
	return Config{
		MaxTorque:   1.00,
		MaxVelocity: 2.0,
		AccelLimit:  5.0,
		MaxKpScale:  1.0,
		MaxKdScale:  1.0,

		PositionLimitEnabled: true,
		PositionMin:          -0.5, // -180°
		PositionMax:          0.5,  // +180°
		PositionTripMargin:   0.05, // 18° of overshoot

		BusVoltageMin:   11.0,
		BusVoltageMax:   42.0,
		CurrentTrip:     12.0,
		CurrentTripTime: 0.40,

		BoardTempWarn: 60.0,
		BoardTempTrip: 80.0,
		MotorTempWarn: 70.0,
		MotorTempTrip: 100.0,

		RunawayVelocityFactor: 2.5,
		RunawayTime:           0.50,
		StallDetectEnabled:    true,
		StallCurrent:          8.0,
		StallVelocity:         0.05,
		StallTime:             2.00,

		MaxConsecutiveTimeouts: 5,
		WatchdogPeriods:        4.0,
		WatchdogMin:            0.08,
		WatchdogMax:            0.50,
		DeadmanTimeout:         2.0,

		LargeMove: 0.25, // 90°
	}
}

// Normalise brings the configuration into a self-consistent, physically
// possible state and reports every adjustment it made.
//
// It returns notes rather than silently correcting, because an operator who
// asked for a limit and got a different one needs to know — the whole point of
// this layer is that the declared envelope is the one in force.
func (c *Config) Normalise(envelope *motor.Envelope) []string {
	var notes []string

	if envelope != nil {
		if envelope.VelocityMax > 0 && c.MaxVelocity > envelope.VelocityMax {
			c.MaxVelocity = envelope.VelocityMax
			notes = append(notes, "velocity limit clamped to the controller's documented maximum")
		}
		if envelope.PeakCurrent > 0 && c.CurrentTrip > envelope.PeakCurrent {
			c.CurrentTrip = envelope.PeakCurrent
			notes = append(notes, "current trip clamped to the controller's peak phase current")
		}
		if envelope.VoltageMin > 0 && c.BusVoltageMin < envelope.VoltageMin {
			c.BusVoltageMin = envelope.VoltageMin
			notes = append(notes, "bus voltage minimum raised to the controller's minimum")
		}
		if envelope.VoltageMax > 0 && c.BusVoltageMax > envelope.VoltageMax {
			c.BusVoltageMax = envelope.VoltageMax
			notes = append(notes, "bus voltage maximum lowered to the controller's maximum")
		}
	}

	c.MaxVelocity = math.Max(0, c.MaxVelocity)
	c.MaxTorque = math.Max(0, c.MaxTorque)
	c.AccelLimit = math.Max(0, c.AccelLimit)
	c.MaxKpScale = math.Min(math.Max(0, c.MaxKpScale), 1.0)
	c.MaxKdScale = math.Min(math.Max(0, c.MaxKdScale), 1.0)

	if c.BusVoltageMax <= c.BusVoltageMin {
		c.BusVoltageMax = c.BusVoltageMin + 1.0
		notes = append(notes, "bus voltage maximum raised above the minimum")
	}
	if c.PositionMax < c.PositionMin {
		c.PositionMin, c.PositionMax = c.PositionMax, c.PositionMin
		notes = append(notes, "position limits were swapped into order")
	}

	if c.MaxConsecutiveTimeouts < 1 {
		c.MaxConsecutiveTimeouts = 1
	}
	c.WatchdogPeriods = math.Min(math.Max(2.0, c.WatchdogPeriods), 50.0)
	if c.WatchdogMin <= 0 {
		c.WatchdogMin = 0.08
	}
	if c.WatchdogMax < c.WatchdogMin {
		c.WatchdogMax = c.WatchdogMin
	}
	if c.DeadmanTimeout > 0 {
		c.DeadmanTimeout = math.Min(math.Max(0.5, c.DeadmanTimeout), 30.0)
	}

	return notes
}

// FitVoltageTo widens the bus voltage trip band when it does not contain the
// supply the connected hardware is rated for, and reports what it changed.
//
// This is the one place the safety layer relaxes a limit rather than tightening
// it, and it is deliberately narrow.  The band's purpose is to catch a supply
// that has sagged or a bus that is regenerating; a band that excludes the
// motor's own rated voltage catches neither, it just trips the instant the
// machine is powered.  A 48 V actuator against a band inherited from a 24 V
// controller is not a protected machine, it is one that cannot be armed — and
// an operator who meets that on every connect learns to widen the band by hand,
// past where they would have stopped.
//
// A band that already contains the rated supply is left exactly as declared,
// margins and all.
func (c *Config) FitVoltageTo(envelope motor.Envelope) []string {
	rated := envelope.VoltageRated
	if rated <= 0 || envelope.VoltageMin <= 0 || envelope.VoltageMax <= 0 {
		return nil
	}
	if rated >= c.BusVoltageMin && rated <= c.BusVoltageMax {
		return nil
	}

	before := [2]float64{c.BusVoltageMin, c.BusVoltageMax}
	c.BusVoltageMin = envelope.VoltageMin
	c.BusVoltageMax = envelope.VoltageMax

	return []string{fmt.Sprintf(
		"bus voltage band widened from [%g, %g] V to the hardware's rated range [%g, %g] V, "+
			"because the declared band excluded this motor's %g V rated supply",
		before[0], before[1], c.BusVoltageMin, c.BusVoltageMax, rated)}
}

// WatchdogTimeout is the command timeout to send the controller for a loop
// running at the given period.
func (c Config) WatchdogTimeout(period float64) float64 {
	return math.Min(math.Max(c.WatchdogPeriods*period, c.WatchdogMin), c.WatchdogMax)
}

// ClampPosition constrains a position target to the soft envelope.
func (c Config) ClampPosition(value float64) (float64, bool) {
	if !c.PositionLimitEnabled || !motor.Valid(value) {
		return value, false
	}
	return motor.ClampChanged(value, c.PositionMin, c.PositionMax)
}

// ClampVelocity constrains a velocity command to ±MaxVelocity.
func (c Config) ClampVelocity(value float64) (float64, bool) {
	if !motor.Valid(value) {
		return value, false
	}
	return motor.ClampChanged(value, -c.MaxVelocity, c.MaxVelocity)
}

// ClampTorque constrains a torque limit to the operator's ceiling.  An unset
// value becomes the ceiling itself, so a command that forgets to state a limit
// still gets one.
func (c Config) ClampTorque(value float64) (float64, bool) {
	if !motor.Valid(value) {
		return c.MaxTorque, false
	}
	return motor.ClampChanged(value, 0, c.MaxTorque)
}

// Preset is a one-click set of motion-trip sensitivities.
type Preset struct {
	Name        string
	Label       string
	Description string
	Apply       func(*Config)
}

// Presets tighten or loosen every motion trip together.  Standard is the
// default; Conservative is for a first power-up with an unknown motor, and
// Relaxed for a machine that is known to overshoot or run stiff.
var Presets = map[string]Preset{
	"conservative": {
		Name:        "conservative",
		Label:       "Conservative",
		Description: "First power-up with an unknown motor. Trips early.",
		Apply: func(c *Config) {
			c.RunawayVelocityFactor, c.RunawayTime = 1.5, 0.30
			c.StallCurrent, c.StallTime = 4.0, 1.0
			c.CurrentTrip, c.CurrentTripTime = 6.0, 0.20
			c.BoardTempTrip, c.MotorTempTrip = 70.0, 90.0
		},
	},
	"standard": {
		Name:        "standard",
		Label:       "Standard",
		Description: "Normal use. Tolerates overshoot and stiff joints.",
		Apply: func(c *Config) {
			c.RunawayVelocityFactor, c.RunawayTime = 2.5, 0.50
			c.StallCurrent, c.StallTime = 8.0, 2.0
			c.CurrentTrip, c.CurrentTripTime = 12.0, 0.40
			c.BoardTempTrip, c.MotorTempTrip = 80.0, 100.0
		},
	},
	"relaxed": {
		Name:        "relaxed",
		Label:       "Relaxed",
		Description: "Only clear faults stop the axis. Watch it closely.",
		Apply: func(c *Config) {
			c.RunawayVelocityFactor, c.RunawayTime = 4.0, 1.00
			c.StallCurrent, c.StallTime = 15.0, 4.0
			c.CurrentTrip, c.CurrentTripTime = 20.0, 1.00
			c.BoardTempTrip, c.MotorTempTrip = 90.0, 110.0
		},
	},
}
