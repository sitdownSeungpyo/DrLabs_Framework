package robot

import (
	"strings"
	"testing"

	"drlabs/motor"
)

// A geared, reversed, offset joint: the case where motor space and joint space
// disagree about every number.
func geared() jointUnits {
	return jointUnits{mapping: Mapping{GearRatio: 6.0, Direction: -1, OffsetRad: 0.25}}
}

// A place carries the gearing, the sign and the zero offset.
func TestPositionIsRenderedInJointRadians(t *testing.T) {
	u := geared()
	mapping := u.mapping

	// Whatever the joint calls 1.0 rad, the renderer has to call it that too
	// when handed the motor position it maps to.
	rev := mapping.MotorRevolutions(1.0)
	got := u.Position(rev)
	if !strings.HasSuffix(got, "rad") {
		t.Fatalf("position rendered as %q, want joint radians", got)
	}
	if !strings.HasPrefix(got, "1.0000") {
		t.Errorf("1.0 rad at the joint is %.4f rev at the motor and rendered as %q", rev, got)
	}
}

// A distance carries the gearing but neither the offset nor the sign.
//
// This is the distinction that makes Span a separate method: run a margin
// through Position and the joint's zero offset is added to a number that is
// not a place, which turns a 0.05 rev margin into a reading nowhere near it.
func TestSpanCarriesGearingButNotTheOffset(t *testing.T) {
	u := geared()

	// 6 motor revolutions is one joint revolution: 2π rad of travel.
	got := u.Span(6.0)
	if !strings.HasPrefix(got, "6.2832") {
		t.Errorf("6 rev through a 6:1 reduction is 2π rad of travel, rendered as %q", got)
	}

	// The offset would show up here if Span used Position.
	place := u.Position(6.0)
	if place == got {
		t.Errorf("Span and Position agree (%q); the offset is not being applied to the place", got)
	}
}

// Torque crosses the gearbox the other way.
func TestTorqueIsRenderedAtTheMechanism(t *testing.T) {
	u := geared()
	// 1 Nm at the rotor is 6 Nm at the joint through a 6:1 reduction.
	if got := u.Torque(1.0); !strings.HasPrefix(got, "6.00") {
		t.Errorf("1 Nm at the rotor rendered as %q, want 6.00 Nm at the joint", got)
	}
}

func TestVelocityAndAccelAreRenderedInJointUnits(t *testing.T) {
	u := geared()
	if got := u.Velocity(6.0); !strings.HasSuffix(got, "rad/s") {
		t.Errorf("velocity rendered as %q", got)
	}
	if got := u.Accel(6.0); !strings.HasSuffix(got, "rad/s²") {
		t.Errorf("acceleration rendered as %q", got)
	}
}

// An unreported value must not be rendered as a number.
func TestUnreportedValuesSaySo(t *testing.T) {
	u := geared()
	nan := motor.NaN()
	for name, got := range map[string]string{
		"position": u.Position(nan),
		"span":     u.Span(nan),
		"velocity": u.Velocity(nan),
		"accel":    u.Accel(nan),
		"torque":   u.Torque(nan),
	} {
		if got != "unreported" {
			t.Errorf("%s of NaN rendered as %q", name, got)
		}
	}
}

// Without a renderer the framework keeps speaking motor space, which is what a
// single-axis session means.
func TestMotorUnitsAreTheDefault(t *testing.T) {
	u := motor.Render(nil)
	if got := u.Position(0.5); got != "0.5000 rev" {
		t.Errorf("default position rendering is %q", got)
	}
	if got := u.Torque(0.25); got != "0.25 Nm" {
		t.Errorf("default torque rendering is %q", got)
	}
}
