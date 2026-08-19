package safety

import (
	"math"
	"testing"
	"time"

	"drlabs/motor"
)

// healthy is telemetry from a machine that is behaving.
func healthy() motor.Telemetry {
	telemetry := motor.NewTelemetry()
	telemetry.Mode = 10
	telemetry.Fault = 0
	telemetry.Position = 0.0
	telemetry.Velocity = 0.0
	telemetry.QCurrent = 0.5
	telemetry.Voltage = 24.0
	telemetry.Temperature = 30.0
	telemetry.MotorTemperature = 30.0
	return telemetry
}

func newTestMonitor() (*Monitor, *Config) {
	config := DefaultConfig()
	config.Normalise(nil)
	return NewMonitor(&config), &config
}

func TestHealthyTelemetryDoesNotTrip(t *testing.T) {
	monitor, _ := newTestMonitor()
	if trip := monitor.Evaluate(healthy(), motor.NewCommand(motor.CtrlPosition), true, time.Now()); trip != nil {
		t.Fatalf("healthy telemetry tripped: %s", trip.Message)
	}
}

func TestHardFaultTripsImmediately(t *testing.T) {
	monitor, _ := newTestMonitor()
	telemetry := healthy()
	telemetry.Fault = 39
	telemetry.FaultName = "Outside limit"
	telemetry.HardFault = true

	trip := monitor.Evaluate(telemetry, motor.NewCommand(motor.CtrlPosition), true, time.Now())
	if trip == nil {
		t.Fatal("a hard fault should trip")
	}
	if trip.Code != "controller_fault" {
		t.Errorf("code: got %q, want controller_fault", trip.Code)
	}
}

func TestLimitingStateWarnsButDoesNotTrip(t *testing.T) {
	// "The output is being limited" is information the operator needs, not a
	// reason to stop the machine.  Treating it as a fault would make a
	// correctly torque-limited axis unusable.
	monitor, _ := newTestMonitor()
	telemetry := healthy()
	telemetry.Fault = 102
	telemetry.FaultName = "Limiting: max torque"
	telemetry.HardFault = false

	trip := monitor.Evaluate(telemetry, motor.NewCommand(motor.CtrlPosition), true, time.Now())
	if trip != nil {
		t.Fatalf("a limiting state should not trip, got %s", trip.Message)
	}
	if len(monitor.Warnings) == 0 {
		t.Error("a limiting state should produce a warning")
	}
}

func TestOverCurrentNeedsToBeSustained(t *testing.T) {
	// A momentary current spike is what acceleration looks like.  Only a
	// sustained excursion is a fault.
	monitor, config := newTestMonitor()
	telemetry := healthy()
	telemetry.QCurrent = config.CurrentTrip * 2

	start := time.Now()
	cmd := motor.NewCommand(motor.CtrlPosition)

	if trip := monitor.Evaluate(telemetry, cmd, true, start); trip != nil {
		t.Fatal("a single over-current sample should not trip")
	}
	halfway := start.Add(time.Duration(config.CurrentTripTime / 2 * float64(time.Second)))
	if trip := monitor.Evaluate(telemetry, cmd, true, halfway); trip != nil {
		t.Fatal("an excursion shorter than the trip time should not trip")
	}
	past := start.Add(time.Duration((config.CurrentTripTime + 0.01) * float64(time.Second)))
	trip := monitor.Evaluate(telemetry, cmd, true, past)
	if trip == nil {
		t.Fatal("a sustained over-current should trip")
	}
	if trip.Code != "over_current" {
		t.Errorf("code: got %q, want over_current", trip.Code)
	}
}

func TestBriefOverCurrentResetsTheTimer(t *testing.T) {
	monitor, config := newTestMonitor()
	start := time.Now()
	cmd := motor.NewCommand(motor.CtrlPosition)

	over := healthy()
	over.QCurrent = config.CurrentTrip * 2

	monitor.Evaluate(over, cmd, true, start)
	// One sample back inside the limit clears the accumulated time.
	monitor.Evaluate(healthy(), cmd, true, start.Add(100*time.Millisecond))

	past := start.Add(time.Duration((config.CurrentTripTime + 0.01) * float64(time.Second)))
	if trip := monitor.Evaluate(over, cmd, true, past); trip != nil {
		t.Fatalf("the timer should have restarted, got %s", trip.Message)
	}
}

func TestPositionEnvelopeOnlyTripsWhileArmed(t *testing.T) {
	// While disarmed the drive is off, so the operator's only way back inside
	// the envelope is to move the axis by hand.  Latching on that would make
	// the condition unrecoverable.
	monitor, config := newTestMonitor()
	telemetry := healthy()
	telemetry.Position = config.PositionMax + config.PositionTripMargin + 0.1
	cmd := motor.NewCommand(motor.CtrlPosition)

	if trip := monitor.Evaluate(telemetry, cmd, false, time.Now()); trip != nil {
		t.Fatalf("a disarmed axis outside the envelope should not trip, got %s", trip.Message)
	}
	if len(monitor.Warnings) == 0 {
		t.Error("it should still warn while disarmed")
	}

	trip := monitor.Evaluate(telemetry, cmd, true, time.Now())
	if trip == nil {
		t.Fatal("an armed axis outside the envelope should trip")
	}
	if trip.Code != "position_envelope" {
		t.Errorf("code: got %q, want position_envelope", trip.Code)
	}
}

func TestPositionTripMarginToleratesOvershoot(t *testing.T) {
	monitor, config := newTestMonitor()
	telemetry := healthy()
	// Just past the limit, but inside the margin: ordinary overshoot.
	telemetry.Position = config.PositionMax + config.PositionTripMargin/2

	if trip := monitor.Evaluate(telemetry, motor.NewCommand(motor.CtrlPosition), true, time.Now()); trip != nil {
		t.Fatalf("overshoot inside the margin should not trip, got %s", trip.Message)
	}
}

func TestStallNeedsAnIntentToMove(t *testing.T) {
	// Current with no motion is exactly what holding a load looks like.  A
	// stall is only meaningful against a command that asked for motion.
	monitor, config := newTestMonitor()
	telemetry := healthy()
	// Above the stall threshold but below the over-current trip, so that the
	// stall logic is what is under test rather than the current guard.
	telemetry.QCurrent = (config.StallCurrent + config.CurrentTrip) / 2
	telemetry.Velocity = 0

	start := time.Now()
	past := start.Add(time.Duration((config.StallTime + 0.1) * float64(time.Second)))

	holding := motor.NewCommand(motor.CtrlPosition)
	holding.VelocityTarget = 0
	monitor.Evaluate(telemetry, holding, true, start)
	if trip := monitor.Evaluate(telemetry, holding, true, past); trip != nil && trip.Code == "stall" {
		t.Fatal("holding a load should not be reported as a stall")
	}

	monitor.Reset()
	moving := motor.NewCommand(motor.CtrlPosition)
	moving.VelocityTarget = 1.0
	monitor.Evaluate(telemetry, moving, true, start)
	trip := monitor.Evaluate(telemetry, moving, true, past)
	if trip == nil || trip.Code != "stall" {
		t.Fatalf("a commanded move with no motion and high current should stall-trip, got %v", trip)
	}
}

func TestVelocityRunawayScalesWithTheLimit(t *testing.T) {
	monitor, config := newTestMonitor()
	telemetry := healthy()
	telemetry.Velocity = config.MaxVelocity * config.RunawayVelocityFactor * 1.5

	start := time.Now()
	past := start.Add(time.Duration((config.RunawayTime + 0.1) * float64(time.Second)))
	cmd := motor.NewCommand(motor.CtrlVelocity)

	monitor.Evaluate(telemetry, cmd, true, start)
	trip := monitor.Evaluate(telemetry, cmd, true, past)
	if trip == nil || trip.Code != "velocity_runaway" {
		t.Fatalf("sustained excess velocity should trip, got %v", trip)
	}
}

func TestUnmeasuredFieldsDoNotTrip(t *testing.T) {
	// A fast-profile cycle reports no motor temperature.  An absent sensor
	// must not be read as a cold motor, nor as a hot one.
	monitor, _ := newTestMonitor()
	telemetry := healthy()
	telemetry.MotorTemperature = math.NaN()
	telemetry.Voltage = math.NaN()
	telemetry.QCurrent = math.NaN()

	if trip := monitor.Evaluate(telemetry, motor.NewCommand(motor.CtrlPosition), true, time.Now()); trip != nil {
		t.Fatalf("unmeasured fields should not trip, got %s", trip.Message)
	}
}

func TestNormaliseClampsToTheHardwareEnvelope(t *testing.T) {
	config := DefaultConfig()
	config.MaxVelocity = 10000
	config.CurrentTrip = 10000
	config.BusVoltageMin = 1
	config.BusVoltageMax = 1000

	envelope := motor.Envelope{
		VoltageMin: 10, VoltageMax: 44, PeakCurrent: 100, VelocityMax: 467,
	}
	notes := config.Normalise(&envelope)

	if config.MaxVelocity != 467 {
		t.Errorf("velocity: got %v, want 467", config.MaxVelocity)
	}
	if config.CurrentTrip != 100 {
		t.Errorf("current trip: got %v, want 100", config.CurrentTrip)
	}
	if config.BusVoltageMin != 10 || config.BusVoltageMax != 44 {
		t.Errorf("voltage: got [%v, %v], want [10, 44]", config.BusVoltageMin, config.BusVoltageMax)
	}
	if len(notes) != 4 {
		t.Errorf("every adjustment should be reported; got %d notes: %v", len(notes), notes)
	}
}

func TestNormaliseSwapsInvertedPositionLimits(t *testing.T) {
	config := DefaultConfig()
	config.PositionMin, config.PositionMax = 1.0, -1.0

	notes := config.Normalise(nil)
	if config.PositionMin != -1.0 || config.PositionMax != 1.0 {
		t.Errorf("got [%v, %v], want [-1, 1]", config.PositionMin, config.PositionMax)
	}
	if len(notes) == 0 {
		t.Error("swapping the limits should be reported, not done silently")
	}
}

func TestWatchdogTimeoutStaysWithinItsBounds(t *testing.T) {
	config := DefaultConfig()
	config.Normalise(nil)

	// A fast loop would otherwise ask for a watchdog shorter than a USB
	// transaction can reliably be, which would fault the controller on an
	// ordinary scheduling hiccup.
	if got := config.WatchdogTimeout(1.0 / 500); got < config.WatchdogMin {
		t.Errorf("500 Hz: got %v, below the floor %v", got, config.WatchdogMin)
	}
	// A very slow loop must not be allowed an unboundedly long one.
	if got := config.WatchdogTimeout(1.0 / 10); got > config.WatchdogMax {
		t.Errorf("10 Hz: got %v, above the ceiling %v", got, config.WatchdogMax)
	}
}

func TestClampCommandRespectsTheTorqueCeiling(t *testing.T) {
	monitor, config := newTestMonitor()
	config.MaxTorque = 0.5

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.MaxTorque = 5.0
	cmd.Position = 100.0
	cmd.Velocity = 100.0

	clamped, notes := monitor.ClampCommand(cmd)
	if clamped.MaxTorque != 0.5 {
		t.Errorf("torque: got %v, want 0.5", clamped.MaxTorque)
	}
	if clamped.Position != config.PositionMax {
		t.Errorf("position: got %v, want %v", clamped.Position, config.PositionMax)
	}
	if clamped.Velocity != config.MaxVelocity {
		t.Errorf("velocity: got %v, want %v", clamped.Velocity, config.MaxVelocity)
	}
	if len(notes) != 3 {
		t.Errorf("every clamp should be reported; got %v", notes)
	}
}

func TestClampCommandLeavesDeferredTorqueAlone(t *testing.T) {
	// When the controller's own limit has been requested, running the NaN
	// through a clamp would collapse it to a number and command zero torque.
	monitor, _ := newTestMonitor()

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.DeferTorqueToController = true

	clamped, _ := monitor.ClampCommand(cmd)
	if !math.IsNaN(clamped.MaxTorque) {
		t.Errorf("a deferred torque limit should stay unset, got %v", clamped.MaxTorque)
	}
}

func TestPreflightBlocksOutsideTheEnvelope(t *testing.T) {
	monitor, config := newTestMonitor()
	telemetry := healthy()
	telemetry.Position = config.PositionMax + 1.0

	blocking, _ := monitor.Preflight(nil, telemetry, true)
	if len(blocking) == 0 {
		t.Fatal("arming outside the envelope should be blocked")
	}
}

func TestPreflightSeparatesAdvisoriesFromBlockers(t *testing.T) {
	// A system that treats every observation as a blocker teaches people to
	// bypass it, so a warm board is an advisory and nothing more.
	monitor, config := newTestMonitor()
	telemetry := healthy()
	telemetry.Temperature = config.BoardTempWarn + 5

	blocking, advisory := monitor.Preflight(nil, telemetry, true)
	if len(blocking) != 0 {
		t.Errorf("a warm board should not block arming, got %v", blocking)
	}
	if len(advisory) == 0 {
		t.Error("a warm board should be advised")
	}
}

func TestPresetsTightenAndLoosenTogether(t *testing.T) {
	conservative := DefaultConfig()
	Presets["conservative"].Apply(&conservative)

	relaxed := DefaultConfig()
	Presets["relaxed"].Apply(&relaxed)

	if conservative.CurrentTrip >= relaxed.CurrentTrip {
		t.Error("the conservative preset should trip at a lower current")
	}
	if conservative.RunawayTime >= relaxed.RunawayTime {
		t.Error("the conservative preset should trip sooner")
	}
	if conservative.BoardTempTrip >= relaxed.BoardTempTrip {
		t.Error("the conservative preset should trip at a lower temperature")
	}
}
