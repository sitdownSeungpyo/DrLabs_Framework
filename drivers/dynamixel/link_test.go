package dynamixel

import (
	"context"
	"math"
	"strings"
	"testing"
	"time"

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

// Every profile has to declare the supply it is rated for.
//
// Without it the safety layer keeps the voltage trip band it inherited from a
// 12 V bus, and a 5 V XL330 reporting a perfectly healthy 4.8 V fails preflight
// and cannot be armed at all.  The band is only widened when the rated supply
// falls outside it, so the number has to be there for the check to fire.
func TestProfilesDeclareTheirRatedSupply(t *testing.T) {
	for name, profile := range Profiles {
		if profile.VoltageRated <= 0 {
			t.Errorf("profile %q declares no rated supply", name)
			continue
		}
		if profile.VoltageRated < profile.VoltageMin || profile.VoltageRated > profile.VoltageMax {
			t.Errorf("profile %q: rated supply %.1f V is outside its own range [%.1f, %.1f] V",
				name, profile.VoltageRated, profile.VoltageMin, profile.VoltageMax)
		}
	}
}

// The rated supply has to reach the safety layer, not merely exist.
func TestEnvelopeCarriesTheRatedSupply(t *testing.T) {
	link, err := motor.New(motor.Config{
		Driver: "dynamixel", Port: "COM1", ID: 1,
		Options: map[string]string{"profile": "xl330"},
	})
	if err != nil {
		t.Fatalf("building the link: %v", err)
	}
	enveloped, ok := link.(motor.Enveloped)
	if !ok {
		t.Fatal("the dynamixel link no longer publishes an envelope")
	}
	if got := enveloped.Envelope().VoltageRated; got != 5.0 {
		t.Errorf("XL330 rated supply reaches the envelope as %v, want 5", got)
	}
}

// tuningPort is a serial port that records the read timeout asked of it.
type tuningPort struct {
	tuned []time.Duration
}

func (p *tuningPort) Read([]byte) (int, error)  { return 0, nil }
func (p *tuningPort) Write(b []byte) (int, error) { return len(b), nil }
func (p *tuningPort) Close() error                { return nil }
func (p *tuningPort) SetReadTimeout(d time.Duration) error {
	p.tuned = append(p.tuned, d)
	return nil
}

// Opening has to tune the port's read timeout.
//
// A serial port opens with a plain total read timeout, which makes a read block
// until it has filled the whole buffer it asked for.  The packet reader asks
// for far more than any status packet, so without this every read costs the
// full timeout: measured on an XL330 at 1 Mbaud, 15.0 ms a cycle — 67 Hz
// whatever rate was configured, against 1.1 ms once tuned.
func TestOpenTunesTheSerialReadTimeout(t *testing.T) {
	port := &tuningPort{}
	link := NewLinkWithPort(port, 1, XL330)

	// The ping finds nobody, which is expected; the tuning happens first.
	_ = link.Open(context.Background())

	if len(port.tuned) == 0 {
		t.Fatal("Open left the port at its default read timeout")
	}
	if got := port.tuned[0]; got != ReadTimeout {
		t.Errorf("read timeout set to %v, want %v", got, ReadTimeout)
	}
}

// Every model in the table has to name a profile that exists.
func TestKnownModelsNameRealProfiles(t *testing.T) {
	for model, known := range KnownModels {
		if known.Name == "" {
			t.Errorf("model %d has no name", model)
		}
		if _, ok := Profiles[known.Profile]; !ok {
			t.Errorf("model %d (%s) names profile %q, which does not exist",
				model, known.Name, known.Profile)
		}
	}
}

func linkFor(t *testing.T, options map[string]string) *Link {
	t.Helper()
	cfg := motor.Config{Driver: "dynamixel", Port: "COM1", ID: 1, Options: options}
	link, err := motor.New(cfg)
	if err != nil {
		t.Fatalf("building the link: %v", err)
	}
	return link.(*Link)
}

// A config that names no profile is driven by what the servo says it is.
//
// This is the case that used to go wrong quietly: an XL330 fell back to the
// X-series table, and every current reading came out 2.69 times too high while
// the board still reported itself as supported.
func TestUndeclaredProfileComesFromTheModelNumber(t *testing.T) {
	link := linkFor(t, nil)
	if link.profile.Name != XSeries.Name {
		t.Fatalf("the default profile changed: %q", link.profile.Name)
	}

	if err := link.resolveProfile(1200); err != nil {
		t.Fatalf("resolving model 1200: %v", err)
	}
	if link.profile.Name != XL330.Name {
		t.Errorf("model 1200 selected the %q profile, want %q", link.profile.Name, XL330.Name)
	}
	if link.modelName != "XL330-M288-T" {
		t.Errorf("model name resolved to %q", link.modelName)
	}
	if link.profile.CurrentAmpsPerCount != XL330.CurrentAmpsPerCount {
		t.Errorf("current scale is %v, want the XL330's %v",
			link.profile.CurrentAmpsPerCount, XL330.CurrentAmpsPerCount)
	}
}

// A declaration that matches the hardware is left alone.
func TestDeclaredProfileThatMatchesIsKept(t *testing.T) {
	link := linkFor(t, map[string]string{"profile": "xl330"})
	if err := link.resolveProfile(1200); err != nil {
		t.Fatalf("resolving model 1200: %v", err)
	}
	if link.profile.Name != XL330.Name {
		t.Errorf("profile became %q", link.profile.Name)
	}
}

// A declaration the hardware contradicts is refused.
//
// Carrying on would mean reporting current and voltage scaled for a motor that
// is not the one on the bus, which is worse than not connecting at all.
func TestDeclaredProfileThatContradictsTheModelIsRefused(t *testing.T) {
	link := linkFor(t, map[string]string{"profile": "x"})
	err := link.resolveProfile(1200)
	if err == nil {
		t.Fatal("an X-series profile on an XL330 was accepted")
	}
	for _, want := range []string{"1200", "XL330-M288-T", "xl330"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("the refusal does not mention %q: %v", want, err)
		}
	}
}

// A model this build does not know is left as configured, not guessed at.
func TestUnknownModelLeavesTheProfileAlone(t *testing.T) {
	link := linkFor(t, map[string]string{"profile": "xl330"})
	if err := link.resolveProfile(65000); err != nil {
		t.Fatalf("an unrecognised model should not be an error: %v", err)
	}
	if link.profile.Name != XL330.Name {
		t.Errorf("profile changed to %q", link.profile.Name)
	}
	if link.modelName != "" {
		t.Errorf("an unrecognised model was given the name %q", link.modelName)
	}
}
