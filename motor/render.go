package motor

import "fmt"

// Saying a number back in the space the reader is thinking in.
//
// Everything below the robot layer works in motor space: revolutions at the
// shaft, torque at the rotor.  Everything above it is authored in joint space:
// radians at the output, torque at the mechanism.  A gearbox, a sign and a zero
// offset separate the two, so the same axis at the same instant is 0.477 rev
// and 3.000 rad — and 0.10 Nm at the joint is 0.017 Nm at the rotor through a
// six to one reduction.
//
// Both numbers are correct.  What is not correct is answering a question posed
// in one space with a number from the other, which is what a message built at
// the safety layer does when it reaches somebody who wrote a joint limit in a
// config file.  The message is where the space has to be decided, because by
// the time it is a string it is too late.
//
// This is presentation only.  Nothing here is used in a calculation, on a wire,
// or in a limit.

// Units renders motor-space quantities as text.
//
// A nil Units means motor space, which is what a single-axis session speaks: an
// operator driving one controller directly is thinking in revolutions, and
// there is no joint for them to think in instead.
type Units interface {
	// Position is a place on the axis, so it carries the zero offset and the
	// sign.
	Position(rev float64) string

	// Span is a distance along the axis — a margin, a range, a remaining
	// travel.  It is deliberately separate from Position: a distance has no
	// zero to be offset from and no direction to be reversed, and rendering
	// one through Position would add the joint's offset to a number that is
	// not a place.
	Span(rev float64) string

	Velocity(revPerSec float64) string
	Accel(revPerSecSquared float64) string
	Torque(nm float64) string
}

// MotorUnits renders motor space unchanged: revolutions and rotor torque.
type MotorUnits struct{}

func (MotorUnits) Position(rev float64) string       { return fmt.Sprintf("%.4f rev", rev) }
func (MotorUnits) Span(rev float64) string           { return fmt.Sprintf("%.4f rev", rev) }
func (MotorUnits) Velocity(revPerSec float64) string { return fmt.Sprintf("%.2f rev/s", revPerSec) }
func (MotorUnits) Accel(revPerSecSquared float64) string {
	return fmt.Sprintf("%.2f rev/s²", revPerSecSquared)
}
func (MotorUnits) Torque(nm float64) string { return fmt.Sprintf("%.2f Nm", nm) }

// Render returns units, or motor space when none was supplied.
func Render(units Units) Units {
	if units == nil {
		return MotorUnits{}
	}
	return units
}
