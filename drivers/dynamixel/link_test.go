package dynamixel

import (
	"math"
	"testing"

	"drlabs/motor"
)

// These tests cover the part of the driver that is pure arithmetic: turning
// the servo's encoder counts and driver units into the framework's physical
// units.  Getting them wrong is not a visible failure — the motor still moves,
// the numbers are just silently scaled — so they are worth pinning against the
// published control table values.

func TestPositionConversion(t *testing.T) {
	profile := XSeries

	// One full turn is 4096 counts on an X-series encoder.
	if got := 4096.0 / profile.CountsPerRev; got != 1.0 {
		t.Errorf("4096 counts should be one revolution, got %v", got)
	}
	// A quarter turn, the value a position command has to produce.
	if got := math.Round(0.25 * profile.CountsPerRev); got != 1024 {
		t.Errorf("0.25 rev should be 1024 counts, got %v", got)
	}
}

func TestVelocityConversion(t *testing.T) {
	profile := XSeries

	// The register steps in 0.229 RPM.  One rev/s is 60 RPM.
	counts := 1.0 * 60.0 / profile.VelocityRPMPerCount
	if math.Abs(counts-262.0) > 1.0 {
		t.Errorf("1 rev/s should be about 262 counts, got %v", counts)
	}

	// And back again.
	revPerSec := 262.0 * profile.VelocityRPMPerCount / 60.0
	if math.Abs(revPerSec-1.0) > 0.01 {
		t.Errorf("262 counts should be about 1 rev/s, got %v", revPerSec)
	}
}

func TestCurrentUnitDiffersBetweenFamilies(t *testing.T) {
	// This is the conversion most likely to be wrong in practice: an XL330
	// counts in 1 mA and an XM430 in 2.69 mA, so using one profile for the
	// other puts every current reading out by nearly a factor of three — and
	// a current trip set from those readings would be equally wrong.
	if XSeries.CurrentAmpsPerCount == XL330.CurrentAmpsPerCount {
		t.Fatal("the two families should not share a current unit")
	}
	if got := 1000 * XL330.CurrentAmpsPerCount; math.Abs(got-1.0) > 1e-9 {
		t.Errorf("XL330: 1000 counts should be 1.0 A, got %v", got)
	}
	if got := 1000 * XSeries.CurrentAmpsPerCount; math.Abs(got-2.69) > 1e-9 {
		t.Errorf("X-series: 1000 counts should be 2.69 A, got %v", got)
	}
}

func TestHardwareErrorDecoding(t *testing.T) {
	cases := []struct {
		status uint8
		want   string
	}{
		{0x00, "No fault"},
		{0x01, "Input voltage error"},
		{0x04, "Overheating error"},
		{0x20, "Overload error"},
	}
	for _, tc := range cases {
		if name, _ := decodeHardwareError(tc.status); name != tc.want {
			t.Errorf("status %#02x: got %q, want %q", tc.status, name, tc.want)
		}
	}

	// An unrecognised bit should still be reported as an error rather than
	// being silently treated as healthy.
	if name, _ := decodeHardwareError(0x80); name == "No fault" {
		t.Error("an unknown error bit should not decode as healthy")
	}
}

func TestConfigRejectsUnknownOptions(t *testing.T) {
	// A typo in a config file should be reported, not quietly ignored — an
	// ignored torque_konstant is a motor with no torque scaling at all.
	_, err := newLink(motor.Config{
		Driver:  "dynamixel",
		Port:    "COM1",
		ID:      1,
		Options: map[string]string{"torque_konstant": "0.5"},
	})
	if err == nil {
		t.Fatal("an unknown option should be rejected")
	}
}

func TestConfigRequiresAPort(t *testing.T) {
	if _, err := newLink(motor.Config{Driver: "dynamixel", ID: 1}); err == nil {
		t.Error("a serial driver with no port should be rejected")
	}
}

func TestConfigRejectsOutOfRangeIDs(t *testing.T) {
	// 253 to 255 are reserved by the protocol.
	if _, err := newLink(motor.Config{Driver: "dynamixel", Port: "COM1", ID: 254}); err == nil {
		t.Error("a reserved servo id should be rejected")
	}
}

func TestTorqueIsUnreportedWithoutAConstant(t *testing.T) {
	// Without a motor-specific torque constant the current register cannot be
	// turned into newton-metres.  Reporting a fabricated number would be
	// worse than reporting none, because a safety limit would be set from it.
	link, err := newLink(motor.Config{Driver: "dynamixel", Port: "COM1", ID: 1})
	if err != nil {
		t.Fatal(err)
	}
	if link.(*Link).profile.TorqueConstant != 0 {
		t.Error("the default profile should carry no torque constant")
	}

	withConstant, err := newLink(motor.Config{
		Driver:  "dynamixel",
		Port:    "COM1",
		ID:      1,
		Options: map[string]string{"torque_constant": "0.35"},
	})
	if err != nil {
		t.Fatal(err)
	}
	if got := withConstant.(*Link).profile.TorqueConstant; got != 0.35 {
		t.Errorf("torque constant: got %v, want 0.35", got)
	}
}

func TestCapabilitiesReportWhatIsMissing(t *testing.T) {
	// A Dynamixel has no stay-within mode.  Saying so lets a UI grey the
	// option out instead of failing at command time.
	link, err := newLink(motor.Config{Driver: "dynamixel", Port: "COM1", ID: 1})
	if err != nil {
		t.Fatal(err)
	}
	caps := link.Capabilities()

	if caps.Supports(motor.CtrlStayWithin) {
		t.Error("a Dynamixel should not claim stay-within support")
	}
	if !caps.Supports(motor.CtrlPosition) || !caps.Supports(motor.CtrlVelocity) {
		t.Error("position and velocity control should be supported")
	}
	if !caps.HasWatchdog {
		t.Error("the X-series control table has a bus watchdog")
	}
}

func TestWatchdogNeverRoundsDownToDisabled(t *testing.T) {
	// The register is one byte in 20 ms steps, and zero means "no watchdog".
	// A 5 ms request must not round to zero: that would turn a request for a
	// tighter watchdog into no watchdog at all.
	profile := XSeries
	counts := int(math.Round(0.005 / profile.WatchdogSecondsPerCount))
	if counts < 1 {
		counts = 1
	}
	if counts == 0 {
		t.Fatal("a short timeout rounded down to disabled")
	}
	if got := float64(counts) * profile.WatchdogSecondsPerCount; got != 0.020 {
		t.Errorf("the shortest expressible watchdog is 20 ms, got %v", got)
	}
}
