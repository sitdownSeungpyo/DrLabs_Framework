package robot

import (
	"fmt"

	"drlabs/motor"
)

// Reporting a joint in the units its limits were written in.
//
// A joint's description is authored in joint space — radians at the output,
// newton metres at the mechanism — and everything below the joint works in
// motor space.  Both are correct, and they are different numbers: a 6:1 joint
// at 0.5 rad is 3.0 rev at the shaft, and a 6 Nm limit at the mechanism is
// 1 Nm at the rotor.
//
// So a message that reaches whoever wrote the description has to be in their
// units.  Reporting "0.477 rev is outside [0.477, 0.875] rev" to somebody who
// wrote min_rad 3.0 is not wrong arithmetic, it is an answer to a question
// nobody asked — and it hides the cause, because the two numbers agree to the
// digits printed while differing where it mattered.

// jointUnits renders motor-space quantities in the joint's own units.
type jointUnits struct{ mapping Mapping }

// Position is a place, so it carries the offset and the direction.
func (u jointUnits) Position(rev float64) string {
	if !motor.Valid(rev) {
		return "unreported"
	}
	return fmt.Sprintf("%.4f rad", u.mapping.JointRadians(rev))
}

// Span is a distance, so it carries neither.  Running it through JointRadians
// would add the joint's zero offset to a margin, which is how a 0.05 rev
// margin would come out as a number nowhere near 0.05 of anything.
func (u jointUnits) Span(rev float64) string {
	if !motor.Valid(rev) {
		return "unreported"
	}
	return fmt.Sprintf("%.4f rad", motor.RevToRad(rev/u.mapping.GearRatio))
}

func (u jointUnits) Velocity(revPerSec float64) string {
	if !motor.Valid(revPerSec) {
		return "unreported"
	}
	return fmt.Sprintf("%.2f rad/s", u.mapping.JointRadPerSecond(revPerSec))
}

func (u jointUnits) Accel(revPerSecSquared float64) string {
	if !motor.Valid(revPerSecSquared) {
		return "unreported"
	}
	return fmt.Sprintf("%.2f rad/s²", motor.RevPerSecToRadPerSec(revPerSecSquared/u.mapping.GearRatio))
}

func (u jointUnits) Torque(nm float64) string {
	if !motor.Valid(nm) {
		return "unreported"
	}
	return fmt.Sprintf("%.2f Nm", u.mapping.JointTorque(nm))
}
