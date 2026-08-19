package robstride

import (
	"context"
	"math"
	"testing"
	"time"

	"drlabs/motor"
)

// step drives the simulator for the given wall time, returning the last
// telemetry.  The simulator integrates against the real clock, so the tests
// below run for tenths of a second rather than using a virtual one.
func step(t *testing.T, sim *SimLink, cmd motor.Command, duration time.Duration) *motor.Telemetry {
	t.Helper()

	var last *motor.Telemetry
	deadline := time.Now().Add(duration)
	for time.Now().Before(deadline) {
		telemetry, err := sim.Cycle(context.Background(), cmd, motor.ProfileFull)
		if err != nil {
			t.Fatalf("Cycle: %v", err)
		}
		last = telemetry
		time.Sleep(2 * time.Millisecond)
	}
	if last == nil {
		t.Fatal("the simulator produced no telemetry")
	}
	return last
}

func newSim(t *testing.T, model Model) *SimLink {
	t.Helper()
	sim := NewSimLink(1, model, 0.01, 0.02)
	if err := sim.Open(context.Background()); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = sim.Close() })
	return sim
}

func TestSimulatorReachesAPositionTarget(t *testing.T) {
	sim := newSim(t, RS01)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0.25
	cmd.MaxTorque = 5
	cmd.VelocityLimit = 2

	telemetry := step(t, sim, cmd, 1500*time.Millisecond)
	if math.Abs(telemetry.Position-0.25) > 0.01 {
		t.Errorf("position: got %.4f rev, want 0.25", telemetry.Position)
	}
}

func TestSimulatorRespectsTheVelocityLimit(t *testing.T) {
	// The controller's profile generator is what bounds the approach speed, so
	// a simulator that ignored the limit would let a driver bug through.
	sim := newSim(t, RS01)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 3.0 // far enough to spend the whole run at the limit
	cmd.MaxTorque = 10
	cmd.VelocityLimit = 0.5 // rev/s

	peak := 0.0
	deadline := time.Now().Add(700 * time.Millisecond)
	for time.Now().Before(deadline) {
		telemetry, err := sim.Cycle(context.Background(), cmd, motor.ProfileFast)
		if err != nil {
			t.Fatal(err)
		}
		if v := math.Abs(telemetry.Velocity); v > peak {
			peak = v
		}
		time.Sleep(2 * time.Millisecond)
	}

	if peak > 0.5*1.2 {
		t.Errorf("peak velocity %.3f rev/s exceeded the 0.5 rev/s limit", peak)
	}
	if peak < 0.5*0.6 {
		t.Errorf("peak velocity %.3f rev/s never approached the limit; the axis is not moving", peak)
	}
}

func TestSimulatorHoldsAVelocity(t *testing.T) {
	sim := newSim(t, RS01)

	cmd := motor.NewCommand(motor.CtrlVelocity)
	cmd.Velocity = 1.0
	cmd.MaxTorque = 5

	telemetry := step(t, sim, cmd, 700*time.Millisecond)
	if math.Abs(telemetry.Velocity-1.0) > 0.1 {
		t.Errorf("velocity: got %.3f rev/s, want 1.0", telemetry.Velocity)
	}
}

func TestSimulatorTorqueIsBoundedByTheLimit(t *testing.T) {
	// The torque limit is what actually protects the machine, whatever the
	// gains are.  If it did not bind here, a runaway gain in the driver would
	// look fine in every test.
	sim := newSim(t, RS03)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 5.0 // a large error, so the position loop saturates
	cmd.MaxTorque = 2.0
	cmd.VelocityLimit = 5

	deadline := time.Now().Add(400 * time.Millisecond)
	for time.Now().Before(deadline) {
		telemetry, err := sim.Cycle(context.Background(), cmd, motor.ProfileFast)
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(telemetry.Torque) > 2.0*1.05 {
			t.Fatalf("torque reached %.3f Nm against a 2.0 Nm limit", telemetry.Torque)
		}
		time.Sleep(2 * time.Millisecond)
	}
}

func TestSimulatorIsReleasedUntilCommanded(t *testing.T) {
	// A stop or a query must leave the output free.  A simulator that drove
	// while disarmed would hide exactly the bug that matters most.
	sim := newSim(t, RS01)

	// Get it moving first.
	move := motor.NewCommand(motor.CtrlPosition)
	move.Position = 0.5
	move.MaxTorque = 5
	move.VelocityLimit = 2
	step(t, sim, move, 300*time.Millisecond)

	telemetry := step(t, sim, motor.NewCommand(motor.CtrlStop), 400*time.Millisecond)
	if math.Abs(telemetry.Torque) > oneCount(RS01.TorqueRange) {
		t.Errorf("a stopped axis produced %.4f Nm", telemetry.Torque)
	}
	if telemetry.ModeName != "Reset" {
		t.Errorf("mode after stop: got %q, want Reset", telemetry.ModeName)
	}
}

func TestSimulatorReportsFaultsAndStopsDriving(t *testing.T) {
	sim := newSim(t, RS01)

	move := motor.NewCommand(motor.CtrlPosition)
	move.Position = 0.5
	move.MaxTorque = 5
	move.VelocityLimit = 2
	step(t, sim, move, 200*time.Millisecond)

	sim.InjectFault(flagOvertemperature)
	telemetry := step(t, sim, move, 200*time.Millisecond)

	if !telemetry.HardFault {
		t.Error("an injected fault should be reported")
	}
	if telemetry.FaultName != "Over temperature" {
		t.Errorf("fault: got %q", telemetry.FaultName)
	}
	if math.Abs(telemetry.Torque) > oneCount(RS01.TorqueRange) {
		t.Errorf("a faulted motor produced %.4f Nm", telemetry.Torque)
	}
}

func TestSimulatorRefusesToRezeroWhileDriving(t *testing.T) {
	sim := newSim(t, RS01)

	move := motor.NewCommand(motor.CtrlPosition)
	move.Position = 0.2
	move.MaxTorque = 5
	move.VelocityLimit = 2
	step(t, sim, move, 100*time.Millisecond)

	zero := motor.NewCommand(motor.CtrlSetOutput)
	zero.Position = 0
	if _, err := sim.Cycle(context.Background(), zero, motor.ProfileFast); err == nil {
		t.Error("re-referencing while driving should be refused")
	}
}

// oneCount is the resolution of a symmetric 16-bit field of the given range.
//
// It also bounds how close to zero the protocol can get.  A symmetric range
// spread over 65536 codes has no code at its exact centre, so "zero torque" is
// really "the code nearest zero", half a count away.  On an RS04 that is under
// two milli-newton-metres, but a test that demands an exact zero will fail on
// every model — which is why these compare against a count, not against 0.
func oneCount(rangeValue float64) float64 { return 2 * rangeValue / 65535 }

func TestSimulatorTelemetryGoesThroughTheCodec(t *testing.T) {
	// The simulated wire path uses the same 16-bit codec as the real one, so a
	// per-model scaling mistake shows up here instead of being bypassed.  A
	// command below one count of the model's range is indistinguishable from
	// no command at all.
	sim := newSim(t, RS04)

	cmd := motor.NewCommand(motor.CtrlTorque)
	// One count of RS04's ±120 Nm range is about 3.7 milli-newton-metres.
	cmd.FeedforwardTorque = 0.001
	cmd.MaxTorque = 100

	telemetry, err := sim.Cycle(context.Background(), cmd, motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}
	if encodeUint16(0.001, RS04.TorqueRange) != encodeUint16(0, RS04.TorqueRange) {
		t.Fatal("the test premise is wrong: 0.001 Nm is more than one count on an RS04")
	}
	if math.Abs(telemetry.Torque) > oneCount(RS04.TorqueRange) {
		t.Errorf("a sub-count torque produced %.6f Nm, more than one count", telemetry.Torque)
	}
}

func TestSimulatorAppliesPerModelScaling(t *testing.T) {
	// The same commanded torque must produce the same physical torque whatever
	// the model, even though the wire encoding differs by a factor of twenty.
	for _, model := range []Model{RS00, RS01, RS02, RS03, RS04, RS05, RS06} {
		t.Run(model.Name, func(t *testing.T) {
			sim := newSim(t, model)

			cmd := motor.NewCommand(motor.CtrlTorque)
			cmd.FeedforwardTorque = model.TorqueRange / 4
			cmd.MaxTorque = model.TorqueLimitMax

			telemetry, err := sim.Cycle(context.Background(), cmd, motor.ProfileFast)
			if err != nil {
				t.Fatal(err)
			}

			want := math.Min(model.TorqueRange/4, model.TorqueLimitMax)
			tolerance := math.Max(2*model.TorqueRange/65535, 1e-3)
			if math.Abs(telemetry.Torque-want) > tolerance {
				t.Errorf("torque: got %.4f Nm, want %.4f (tolerance %.4f)",
					telemetry.Torque, want, tolerance)
			}
		})
	}
}

func TestSimulatorRejectsUnsupportedModes(t *testing.T) {
	sim := newSim(t, RS01)
	for _, mode := range []motor.ControlMode{motor.CtrlBrake, motor.CtrlStayWithin} {
		if _, err := sim.Cycle(context.Background(), motor.NewCommand(mode), motor.ProfileFast); err != motor.ErrUnsupportedMode {
			t.Errorf("%s: got %v, want ErrUnsupportedMode", mode, err)
		}
	}
}

func TestSimulatorRequiresAModel(t *testing.T) {
	if _, err := newSimLink(motor.Config{Driver: "robstride-sim", ID: 1}); err == nil {
		t.Error("the simulator should require a model, exactly as the driver does")
	}
}
