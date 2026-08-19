package robot

import (
	"math"

	"drlabs/motor"
	"drlabs/runtime/safety"
)

// The joint-to-motor mapping.
//
// Everything above this layer is in joint space: radians at the output, torque
// at the mechanism.  Everything below is in motor space: revolutions at the
// shaft, torque at the rotor.  A gearbox, a sign and a zero offset separate
// them, and getting any of the three wrong produces a machine that moves — just
// not the way anyone asked.
//
// The conversions are collected here rather than spread through the runtime so
// that there is exactly one place to check when an axis goes the wrong way.

// Mapping is one joint's relationship to its motor.
type Mapping struct {
	// GearRatio is motor turns per joint turn.
	GearRatio float64
	// Direction is +1 or -1.
	Direction float64
	// OffsetRad is where the motor sits, in joint radians, when the joint
	// reads zero.
	OffsetRad float64
}

func mappingOf(joint JointConfig) Mapping {
	return Mapping{
		GearRatio: joint.GearRatio,
		Direction: joint.Direction,
		OffsetRad: joint.OffsetRad,
	}
}

// MotorRevolutions converts a joint angle to a motor position.
func (m Mapping) MotorRevolutions(jointRad float64) float64 {
	if !motor.Valid(jointRad) {
		return jointRad
	}
	return motor.RadToRev((jointRad+m.OffsetRad)*m.Direction) * m.GearRatio
}

// JointRadians converts a motor position back to a joint angle.
func (m Mapping) JointRadians(motorRev float64) float64 {
	if !motor.Valid(motorRev) {
		return motorRev
	}
	return motor.RevToRad(motorRev/m.GearRatio)*m.Direction - m.OffsetRad
}

// MotorRevPerSecond converts a joint velocity.  The offset does not apply to a
// rate; only the gearing and the sign do.
func (m Mapping) MotorRevPerSecond(jointRadS float64) float64 {
	if !motor.Valid(jointRadS) {
		return jointRadS
	}
	return motor.RadPerSecToRevPerSec(jointRadS*m.Direction) * m.GearRatio
}

// JointRadPerSecond converts a motor velocity back to joint units.
func (m Mapping) JointRadPerSecond(motorRevS float64) float64 {
	if !motor.Valid(motorRevS) {
		return motorRevS
	}
	return motor.RevPerSecToRadPerSec(motorRevS/m.GearRatio) * m.Direction
}

// MotorTorque converts a joint torque to what the motor must produce.
//
// The gearbox multiplies torque as it divides speed, so a joint that needs
// 6 Nm through a 6:1 reduction needs 1 Nm at the rotor.  Getting this
// backwards is the difference between a limit that protects the mechanism and
// one that is thirty-six times too generous.
func (m Mapping) MotorTorque(jointNm float64) float64 {
	if !motor.Valid(jointNm) {
		return jointNm
	}
	return jointNm / m.GearRatio
}

// JointTorque converts a reported motor torque to joint torque.
func (m Mapping) JointTorque(motorNm float64) float64 {
	if !motor.Valid(motorNm) {
		return motorNm
	}
	return motorNm * m.GearRatio
}

// MotorAccel converts a joint acceleration limit.
func (m Mapping) MotorAccel(jointRadS2 float64) float64 {
	if !motor.Valid(jointRadS2) {
		return jointRadS2
	}
	return motor.RadPerSecToRevPerSec(jointRadS2) * m.GearRatio
}

// safetyConfig turns a joint's declared envelope into the motor-space envelope
// the session enforces.
//
// The travel limits are converted as a pair and then ordered, because a joint
// with direction -1 maps its minimum to the larger motor position: leaving
// them unswapped would produce an envelope that excludes everything.
func (m Mapping) safetyConfig(limits JointLimits, base safety.Config) safety.Config {
	lower := m.MotorRevolutions(limits.MinRad)
	upper := m.MotorRevolutions(limits.MaxRad)
	if upper < lower {
		lower, upper = upper, lower
	}

	config := base
	config.PositionLimitEnabled = true
	config.PositionMin = lower
	config.PositionMax = upper
	config.MaxTorque = m.MotorTorque(limits.MaxTorqueNm)
	config.MaxVelocity = math.Abs(m.MotorRevPerSecond(limits.MaxVelocityRadS))
	if limits.MaxAccelRadS2 > 0 {
		config.AccelLimit = math.Abs(m.MotorAccel(limits.MaxAccelRadS2))
	}

	// A robot's joints are commanded by a program, not by a browser, so the
	// operator-facing deadman has no meaning here.  What protects the machine
	// if the upper layer dies is the controller's own watchdog, which the
	// session sends every cycle.
	config.DeadmanTimeout = 0
	return config
}
