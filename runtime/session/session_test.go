package session

import (
	"context"
	"math"
	"testing"
	"time"

	"drlabs/drivers/moteus"
	"drlabs/motor"
	"drlabs/runtime/safety"
	// The simulated moteus registers itself, which is what lets these tests
	// exercise the whole stack — state machine, control loop, safety monitor
	// and driver — with no hardware attached.
)

func simConfig() motor.Config {
	return motor.Config{Driver: "moteus-sim", ID: 1}
}

// newConnected returns a running session against the simulator, already past
// the initial stop burst.
func newConnected(t *testing.T) *Session {
	t.Helper()

	s := New(Options{RateHz: 200})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := s.Connect(ctx, simConfig(), false); err != nil {
		t.Fatalf("Connect: %v", err)
	}
	t.Cleanup(func() { _ = s.Disconnect() })

	waitFor(t, time.Second, func() bool {
		return motor.Valid(s.Status().Telemetry.Position)
	}, "telemetry to arrive")
	return s
}

// waitFor polls until condition holds, or fails the test.
func waitFor(t *testing.T, limit time.Duration, condition func() bool, what string) {
	t.Helper()
	deadline := time.Now().Add(limit)
	for time.Now().Before(deadline) {
		if condition() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatalf("timed out after %v waiting for %s", limit, what)
}

func TestConnectStartsTheLoopAndReportsIdentity(t *testing.T) {
	s := newConnected(t)
	status := s.Status()

	if status.Connection != Connected {
		t.Errorf("connection: got %v, want connected", status.Connection)
	}
	if status.Identity == nil || !status.Identity.Simulated {
		t.Error("the simulator should identify itself as simulated")
	}
	if status.Armed {
		t.Error("a fresh connection must not be armed")
	}
	if status.Stats.Cycles == 0 {
		t.Error("the control loop should be running")
	}
}

func TestLoopAchievesItsRequestedRate(t *testing.T) {
	s := newConnected(t)
	// Give the rate window enough samples to be meaningful.
	time.Sleep(600 * time.Millisecond)

	stats := s.Status().Stats
	if stats.AchievedHz < float64(stats.RequestedHz)*0.85 {
		t.Errorf("achieved %.1f Hz against a requested %d Hz", stats.AchievedHz, stats.RequestedHz)
	}
	if stats.ReplyRate < 0.99 {
		t.Errorf("reply rate %.2f against a simulator that always answers", stats.ReplyRate)
	}
}

func TestMotionRequiresArming(t *testing.T) {
	s := newConnected(t)

	if err := s.SetMode(motor.CtrlPosition); err == nil {
		t.Fatal("a disarmed session should refuse a motion mode")
	}

	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		t.Fatalf("SetMode after arming: %v", err)
	}
}

func TestArmRequiresAcknowledgement(t *testing.T) {
	// An arm that cannot be refused is not a checklist.
	s := newConnected(t)
	if err := s.Arm(false); err == nil {
		t.Fatal("arming without acknowledgement should be refused")
	}
}

func TestPositionCommandMovesTheAxis(t *testing.T) {
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		t.Fatalf("SetMode: %v", err)
	}
	s.SetPosition(0.2)

	waitFor(t, 5*time.Second, func() bool {
		return math.Abs(s.Status().Telemetry.Position-0.2) < 0.01
	}, "the axis to reach 0.2 rev")
}

func TestModeChangeDoesNotItselfCommandAMove(t *testing.T) {
	// Entering position mode adopts the current position as the target.  If
	// it kept a stale setpoint, selecting a mode would fling the axis across
	// its travel — which is the single most dangerous thing a UI can do.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		t.Fatalf("SetMode: %v", err)
	}
	s.SetPosition(0.3)
	waitFor(t, 5*time.Second, func() bool {
		return math.Abs(s.Status().Telemetry.Position-0.3) < 0.01
	}, "the axis to reach 0.3 rev")

	// Leave position mode and come back.
	if err := s.SetMode(motor.CtrlStop); err != nil {
		t.Fatalf("SetMode(stop): %v", err)
	}
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		t.Fatalf("SetMode(position): %v", err)
	}

	setpoint := s.Status().Setpoint.Position
	if math.Abs(setpoint-0.3) > 0.02 {
		t.Errorf("re-entering position mode adopted %.4f rev instead of the axis position 0.3", setpoint)
	}
}

func TestSetpointIsClampedToTheEnvelope(t *testing.T) {
	s := newConnected(t)
	safety := s.Safety()

	got := s.SetPosition(safety.PositionMax + 10)
	if got != safety.PositionMax {
		t.Errorf("got %v, want the envelope maximum %v", got, safety.PositionMax)
	}
}

func TestEStopLatchesAndDisarms(t *testing.T) {
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	s.EStop("test", "Test stop", "raised by a test")

	status := s.Status()
	if !status.EStop.Latched {
		t.Error("the stop should be latched")
	}
	if status.Armed {
		t.Error("an emergency stop must disarm")
	}
	if err := s.Arm(true); err == nil {
		t.Error("arming should be refused while a stop is latched")
	}
}

func TestEStopResetLeavesTheAxisDisarmed(t *testing.T) {
	// Clearing a stop must never by itself put a machine back in motion.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	s.EStop("test", "Test stop", "")

	if err := s.ResetEStop(); err != nil {
		t.Fatalf("ResetEStop: %v", err)
	}
	status := s.Status()
	if status.EStop.Latched {
		t.Error("the stop should be cleared")
	}
	if status.Armed {
		t.Error("the axis must stay disarmed after a reset")
	}
}

func TestControllerFaultStopsTheSession(t *testing.T) {
	// ConnectLink lets the test hold the simulator, so a controller fault can
	// be injected the way real hardware would raise one.
	sim := moteus.NewSimLink(1, 0.006, 0.02, 0.06)

	s := New(Options{RateHz: 200})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := s.ConnectLink(ctx, sim, false); err != nil {
		t.Fatalf("ConnectLink: %v", err)
	}
	defer s.Disconnect()

	waitFor(t, time.Second, func() bool {
		return motor.Valid(s.Status().Telemetry.Position)
	}, "telemetry")
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	// Fault 39: position control started outside the controller's own bounds.
	sim.InjectFault(39)

	waitFor(t, 2*time.Second, func() bool {
		return s.Status().EStop.Latched
	}, "the controller fault to stop the session")

	status := s.Status()
	if status.EStop.Code != "controller_fault" {
		t.Errorf("stop code: got %q, want controller_fault", status.EStop.Code)
	}
	if status.Armed {
		t.Error("a fault must disarm the axis")
	}
	if status.LatchedFault != 39 {
		t.Errorf("the fault should stay visible after the stop, got %d", status.LatchedFault)
	}
}

func TestLimitingStateDoesNotStopTheSession(t *testing.T) {
	// A controller reporting "output is being limited" is doing its job.
	// Stopping on it would make a correctly torque-limited axis unusable.
	sim := moteus.NewSimLink(1, 0.006, 0.02, 0.06)

	s := New(Options{RateHz: 200})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := s.ConnectLink(ctx, sim, false); err != nil {
		t.Fatalf("ConnectLink: %v", err)
	}
	defer s.Disconnect()

	waitFor(t, time.Second, func() bool {
		return motor.Valid(s.Status().Telemetry.Position)
	}, "telemetry")
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	sim.InjectFault(102) // limiting: max torque
	time.Sleep(300 * time.Millisecond)

	status := s.Status()
	if status.EStop.Latched {
		t.Fatalf("a limiting state stopped the session: %s", status.EStop.Reason)
	}
	if len(status.Warnings) == 0 {
		t.Error("a limiting state should surface as a warning")
	}
}

func TestSafetyEnvelopeIsClampedToTheHardware(t *testing.T) {
	s := newConnected(t)

	config := s.Safety()
	config.MaxVelocity = 100000
	updated, notes := s.SetSafety(config)

	if updated.MaxVelocity >= 100000 {
		t.Errorf("velocity %v was not clamped to the controller's rating", updated.MaxVelocity)
	}
	if len(notes) == 0 {
		t.Error("the adjustment should be reported rather than made silently")
	}
}

func TestParamsAreClampedToTheSafetyEnvelope(t *testing.T) {
	s := newConnected(t)
	safety := s.Safety()

	params := DefaultParams()
	params.MaxTorque = safety.MaxTorque * 10
	applied := s.SetParams(params)

	if applied.MaxTorque != safety.MaxTorque {
		t.Errorf("torque: got %v, want the ceiling %v", applied.MaxTorque, safety.MaxTorque)
	}
}

func TestShuttleRunsAndStops(t *testing.T) {
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	params := DefaultParams()
	params.MaxTorque = 0.8
	params.VelocityLimit = 2.0
	s.SetParams(params)

	s.ConfigureShuttle(-0.15, 0.15, 0.05, 1)
	if err := s.StartShuttle(); err != nil {
		t.Fatalf("StartShuttle: %v", err)
	}

	waitFor(t, 20*time.Second, func() bool {
		return !s.Status().Shuttle.Running
	}, "the shuttle to finish one cycle")

	if completed := s.Status().Shuttle.Completed; completed != 1 {
		t.Errorf("completed %d cycles, want 1", completed)
	}
}

func TestShuttleStopsWhenTheAxisIsDisarmed(t *testing.T) {
	// The sequencer is evaluated by the loop, so it stops the instant the
	// axis leaves armed position mode rather than waiting for a supervisor.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	s.ConfigureShuttle(-0.2, 0.2, 0.5, 0)
	if err := s.StartShuttle(); err != nil {
		t.Fatalf("StartShuttle: %v", err)
	}

	s.Disarm("test")
	waitFor(t, time.Second, func() bool {
		return !s.Status().Shuttle.Running
	}, "the shuttle to stop with the axis")
}

func TestZeroOutputIsRefusedWhileArmed(t *testing.T) {
	// Re-defining where the axis thinks it is while it is being driven would
	// move the machine by the size of the correction.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.ZeroOutput(0); err == nil {
		t.Error("re-referencing should be refused while armed")
	}

	s.Disarm("test")
	if err := s.ZeroOutput(0); err != nil {
		t.Errorf("re-referencing should be allowed while disarmed: %v", err)
	}
}

func TestDeadmanStopsAnUnattendedAxis(t *testing.T) {
	config := safety.DefaultConfig()
	config.DeadmanTimeout = 0.5

	s := New(Options{RateHz: 100, Safety: &config})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := s.Connect(ctx, simConfig(), false); err != nil {
		t.Fatalf("Connect: %v", err)
	}
	defer s.Disconnect()

	waitFor(t, time.Second, func() bool {
		return motor.Valid(s.Status().Telemetry.Position)
	}, "telemetry")

	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	// Deliberately stop sending heartbeats.
	waitFor(t, 3*time.Second, func() bool {
		return s.Status().EStop.Code == "deadman"
	}, "the deadman to stop the axis")
}

func TestDisconnectStopsTheLoop(t *testing.T) {
	s := newConnected(t)
	before := s.Status().Stats.Cycles

	if err := s.Disconnect(); err != nil {
		t.Fatalf("Disconnect: %v", err)
	}
	time.Sleep(100 * time.Millisecond)

	status := s.Status()
	if status.Connection != Disconnected {
		t.Errorf("connection: got %v, want disconnected", status.Connection)
	}
	if status.Stats.Cycles != before && status.Stats.Cycles > before+2 {
		t.Errorf("the loop kept running after disconnect: %d -> %d", before, status.Stats.Cycles)
	}
}

func TestEventsAreOrderedAndIncremental(t *testing.T) {
	s := newConnected(t)

	all := s.Events(0)
	if len(all) == 0 {
		t.Fatal("connecting should have logged events")
	}
	for i := 1; i < len(all); i++ {
		if all[i].Seq <= all[i-1].Seq {
			t.Fatal("events should be in increasing sequence order")
		}
	}
	if rest := s.Events(all[len(all)-1].Seq); len(rest) != 0 {
		t.Errorf("asking past the newest event returned %d entries", len(rest))
	}
}

func TestSamplesAreRecordedForPlotting(t *testing.T) {
	s := newConnected(t)
	time.Sleep(200 * time.Millisecond)

	samples := s.Samples(0, 50)
	if len(samples) == 0 {
		t.Fatal("the loop should be recording telemetry samples")
	}
	if len(samples) > 50 {
		t.Errorf("the limit was not honoured: got %d samples", len(samples))
	}
	for i := 1; i < len(samples); i++ {
		if samples[i].Seq <= samples[i-1].Seq {
			t.Fatal("samples should be in increasing sequence order")
		}
	}
}

func TestTrajectoryProfilesAMove(t *testing.T) {
	// A host-profiled move should arrive at the target and, unlike a bare step
	// setpoint, should get there by way of a bounded velocity.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	params := DefaultParams()
	params.MaxTorque = 0.8
	params.VelocityLimit = 2.0
	s.SetParams(params)

	if err := s.StartTrajectory(0.3, 0.5, 2.0); err != nil {
		t.Fatalf("StartTrajectory: %v", err)
	}

	peak := 0.0
	waitFor(t, 10*time.Second, func() bool {
		status := s.Status()
		if v := math.Abs(status.Telemetry.Velocity); v > peak {
			peak = v
		}
		return !status.Trajectory.Running
	}, "the profile to finish")

	if peak > 0.5*1.5 {
		t.Errorf("peak velocity %.3f rev/s exceeded the profile's 0.5 rev/s limit", peak)
	}

	waitFor(t, 5*time.Second, func() bool {
		return math.Abs(s.Status().Telemetry.Position-0.3) < 0.01
	}, "the axis to settle on the target")
}

func TestTrajectoryStopsWithTheAxis(t *testing.T) {
	// The sequencer runs inside the loop, so disarming must end it without any
	// supervisor having to notice.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.StartTrajectory(0.4, 0.2, 1.0); err != nil {
		t.Fatalf("StartTrajectory: %v", err)
	}

	s.Disarm("test")
	waitFor(t, time.Second, func() bool {
		return !s.Status().Trajectory.Running
	}, "the trajectory to stop with the axis")
}

func TestTrajectoryAndShuttleAreMutuallyExclusive(t *testing.T) {
	// Both own the position setpoint; running them together would have them
	// fight over it every cycle.
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}
	if err := s.StartTrajectory(0.4, 0.2, 1.0); err != nil {
		t.Fatalf("StartTrajectory: %v", err)
	}

	s.ConfigureShuttle(-0.2, 0.2, 0.1, 1)
	if err := s.StartShuttle(); err != nil {
		t.Fatalf("StartShuttle: %v", err)
	}
	if s.Status().Trajectory.Running {
		t.Error("starting the shuttle should have stopped the profiled move")
	}
	s.StopShuttle()
}

func TestTrajectoryRespectsTheSafetyEnvelope(t *testing.T) {
	s := newConnected(t)
	if err := s.Arm(true); err != nil {
		t.Fatalf("Arm: %v", err)
	}

	envelope := s.Safety()
	if err := s.StartTrajectory(envelope.PositionMax+5, 1.0, 2.0); err != nil {
		t.Fatalf("StartTrajectory: %v", err)
	}
	if target := s.Status().Trajectory.Target; target != envelope.PositionMax {
		t.Errorf("target: got %v, want the envelope maximum %v", target, envelope.PositionMax)
	}
}

func TestTrajectoryRequiresAnArmedAxis(t *testing.T) {
	s := newConnected(t)
	if err := s.StartTrajectory(0.2, 0.5, 2.0); err == nil {
		t.Error("a disarmed session should refuse a profiled move")
	}
}

// newServo returns a session in the servo role: energised on connect, no
// operator, waiting for commands.
func newServo(t *testing.T) *Session {
	t.Helper()

	config := safety.DefaultConfig()
	// A servo waiting with no client attached is the normal state, not a
	// fault, so the deadman has no place in this role.
	config.DeadmanTimeout = 0

	s := New(Options{RateHz: 200, Safety: &config, AutoArm: true})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := s.Connect(ctx, simConfig(), false); err != nil {
		t.Fatalf("Connect: %v", err)
	}
	t.Cleanup(func() { _ = s.Disconnect() })
	return s
}

func TestAutoArmEnergisesWithoutAnOperator(t *testing.T) {
	// The framework's job in this role is to be live and waiting.  An axis
	// that needs a human to press Arm cannot serve an upper layer.
	s := newServo(t)

	waitFor(t, 3*time.Second, func() bool {
		return s.Status().Armed
	}, "the session to arm itself")

	status := s.Status()
	if status.Mode != motor.CtrlPosition {
		t.Errorf("mode: got %s, want position", status.Mode)
	}
	// Energising must not itself be a move: the setpoint has to adopt where
	// the axis already is.
	if math.Abs(status.Setpoint.Position-status.Telemetry.Position) > 0.01 {
		t.Errorf("setpoint %.4f does not match the measured position %.4f",
			status.Setpoint.Position, status.Telemetry.Position)
	}
}

func TestAutoArmNeedsNoAcknowledgement(t *testing.T) {
	// There is nobody to acknowledge a checklist in this role, so requiring
	// one would mean the axis never arms.
	s := newServo(t)
	waitFor(t, 3*time.Second, func() bool { return s.Status().Armed }, "the session to arm itself")

	s.Disarm("test")
	if err := s.Arm(false); err != nil {
		t.Errorf("an auto-arming session should not demand acknowledgement: %v", err)
	}
}

func TestAutoArmWillNotDriveThroughALatchedStop(t *testing.T) {
	// A stop that undid itself would be worse than no stop at all.
	s := newServo(t)
	waitFor(t, 3*time.Second, func() bool { return s.Status().Armed }, "the session to arm itself")

	s.EStop("test", "Test stop", "")
	time.Sleep(500 * time.Millisecond)

	if s.Status().Armed {
		t.Fatal("auto-arm re-energised through a latched emergency stop")
	}

	// Clearing it is an explicit act, and only then may the axis come back.
	if err := s.ResetEStop(); err != nil {
		t.Fatal(err)
	}
	waitFor(t, 3*time.Second, func() bool {
		return s.Status().Armed
	}, "the session to re-arm after the stop was cleared")
}

func TestAutoArmRetriesWhileBlocked(t *testing.T) {
	// The reasons an axis cannot arm are usually transient.  A servo that gave
	// up on the first attempt would need a human to notice — the thing this
	// role exists to avoid.
	config := safety.DefaultConfig()
	config.DeadmanTimeout = 0
	// An envelope the axis starts outside of, so preflight blocks at first.
	config.PositionMin, config.PositionMax = 5.0, 6.0

	s := New(Options{RateHz: 200, Safety: &config, AutoArm: true})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := s.Connect(ctx, simConfig(), false); err != nil {
		t.Fatalf("Connect: %v", err)
	}
	defer s.Disconnect()

	time.Sleep(500 * time.Millisecond)
	if s.Status().Armed {
		t.Fatal("armed despite being outside the position envelope")
	}

	// Widen the envelope: the blocker clears, and nothing else happens.
	widened := s.Safety()
	widened.PositionMin, widened.PositionMax = -1, 1
	s.SetSafety(widened)

	waitFor(t, 3*time.Second, func() bool {
		return s.Status().Armed
	}, "the session to arm once the blocker cleared")
}

func TestManualSessionsStillRequireArming(t *testing.T) {
	// Auto-arm is opt-in.  The operator-facing path must not have quietly
	// become live-on-connect.
	s := newConnected(t)
	time.Sleep(300 * time.Millisecond)
	if s.Status().Armed {
		t.Error("a session without AutoArm should stay disarmed")
	}
	if err := s.Arm(false); err == nil {
		t.Error("it should still demand acknowledgement")
	}
}
