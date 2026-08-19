package motor

import "math"

// The framework speaks one set of physical units on every interface that is
// shared between drivers.  Each driver converts to and from its own wire
// encoding internally, so application code never has to know whether it is
// talking to a Dynamixel (encoder ticks) or a moteus (revolutions).
//
//	position     revolutions      (rev)
//	velocity     revolutions/sec  (rev/s)
//	torque       newton-metres    (Nm)
//	current      amperes          (A)
//	voltage      volts            (V)
//	temperature  degrees Celsius  (°C)
//	time         seconds          (s)
//
// A float64 field carries NaN when the value is unknown or was not reported.
// Use Valid to test before doing arithmetic on one.

// NaN is the "not reported" / "leave unchanged" marker used across the API.
func NaN() float64 { return math.NaN() }

// Valid reports whether v carries a usable measurement.
func Valid(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// Angle conversions.
func DegToRev(deg float64) float64 { return deg / 360.0 }
func RevToDeg(rev float64) float64 { return rev * 360.0 }
func RadToRev(rad float64) float64 { return rad / (2 * math.Pi) }
func RevToRad(rev float64) float64 { return rev * (2 * math.Pi) }

// Velocity conversions.
func RPMToRevPerSec(rpm float64) float64     { return rpm / 60.0 }
func RevPerSecToRPM(rps float64) float64     { return rps * 60.0 }
func RadPerSecToRevPerSec(r float64) float64 { return r / (2 * math.Pi) }
func RevPerSecToRadPerSec(r float64) float64 { return r * (2 * math.Pi) }

// Clamp constrains v to [lo, hi].  A NaN input is returned unchanged, so that
// "leave this parameter alone" survives a clamping pass instead of collapsing
// to a bound — min/max on NaN would silently produce 0 and command no torque.
func Clamp(v, lo, hi float64) float64 {
	if !Valid(v) {
		return v
	}
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// ClampChanged is Clamp plus a flag reporting whether the value was altered,
// so callers can tell the operator that their request was limited.
func ClampChanged(v, lo, hi float64) (float64, bool) {
	out := Clamp(v, lo, hi)
	return out, Valid(v) && out != v
}
