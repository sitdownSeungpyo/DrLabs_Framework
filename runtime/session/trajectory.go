package session

import (
	"errors"
	"fmt"
	"math"
	"time"

	"drlabs/motor"
	"drlabs/trajectory"
)

// Host-side trajectory execution.
//
// Both supported controller families can profile a move themselves — moteus
// from its velocity and acceleration limits, RobStride from its CSP speed
// limit — and when they can, letting them do it is better: the profile is
// regenerated at the firmware's own rate rather than at whatever the host
// manages.
//
// This exists for the cases where that is not enough:
//
//   - the controller has no profile generator in the mode being used, which is
//     true of every operation/MIT-mode command;
//   - the move has to be repeatable and known in advance, so that a test can
//     compare what the axis did against what it was told;
//   - the host wants the velocity feedforward that comes with knowing where on
//     the profile it is, which a bare position setpoint cannot express.
//
// Like the shuttle, the sequencer runs inside the control loop rather than in
// an application on top of it, so its timing does not depend on a caller
// staying responsive and it stops the instant the axis is disarmed or faulted.

// TrajectoryStatus reports the state of a host-side profiled move.
type TrajectoryStatus struct {
	Running bool

	Start  float64 // rev, where the move began
	Target float64 // rev
	// MaxVelocity and Accel are the profile's own limits, which may be tighter
	// than the session's parameters but never looser.
	MaxVelocity float64 // rev/s
	Accel       float64 // rev/s²

	Elapsed  float64 // s
	Total    float64 // s, the profile's duration
	Progress float64 // 0..1

	// Setpoint and Feedforward are the profile's current output: where the
	// axis should be now, and how fast it should be going.
	Setpoint    float64 // rev
	Feedforward float64 // rev/s

	Status string
}

// StartTrajectory begins a host-profiled move from the current position to
// target.
//
// The move starts from where the axis actually is, not from the last setpoint.
// Starting from a stale setpoint would put a step at the beginning of a profile
// whose whole purpose is to have none.
func (s *Session) StartTrajectory(target, maxVelocity, accel float64) error {
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		return err
	}

	s.mu.Lock()
	if s.connection != Connected {
		s.mu.Unlock()
		return errors.New("session: not connected")
	}
	if !s.armed {
		s.mu.Unlock()
		return errors.New("session: arm the axis first")
	}
	if s.estop {
		s.mu.Unlock()
		return errors.New("session: emergency stop is latched")
	}
	if !motor.Valid(s.telemetry.Position) {
		s.mu.Unlock()
		return errors.New("session: the axis position is not being reported")
	}

	// The shuttle and a profiled move both own the position setpoint; running
	// them together would have them fight over it every cycle.
	s.stopShuttleLocked("a trajectory was started", LevelInfo)

	start := s.telemetry.Position
	clamped, _ := s.clampTargetLocked(target)

	// The profile may not exceed the operator's envelope, whatever was asked
	// for here.
	maxVelocity = motor.Clamp(maxVelocity, 0, math.Min(s.params.VelocityLimit, s.safety.MaxVelocity))
	accel = motor.Clamp(accel, 0, math.Max(0.01, s.safety.AccelLimit))
	if maxVelocity <= 0 || accel <= 0 {
		s.mu.Unlock()
		return errors.New("session: the velocity and acceleration limits leave no room to move")
	}

	profile, err := trajectory.NewTrapezoidalProfile(start, clamped, maxVelocity, accel)
	if err != nil {
		s.mu.Unlock()
		return fmt.Errorf("session: %w", err)
	}

	s.trajectory = TrajectoryStatus{
		Running:     true,
		Start:       start,
		Target:      clamped,
		MaxVelocity: maxVelocity,
		Accel:       accel,
		Total:       profile.TotalTime(),
		Setpoint:    start,
		Status:      "running",
	}
	s.profile = profile
	s.trajectoryStartedAt = time.Now()
	s.mu.Unlock()

	units := motor.Render(s.units)
	s.log(LevelOK, "Trajectory started", fmt.Sprintf(
		"%s → %s · %s · %s · %.2f s",
		units.Position(start), units.Position(clamped),
		units.Velocity(maxVelocity), units.Accel(accel), profile.TotalTime()))
	s.signal()
	return nil
}

// StopTrajectory abandons a profiled move, leaving the axis holding wherever
// the profile had reached.
func (s *Session) StopTrajectory() {
	s.mu.Lock()
	s.stopTrajectoryLocked("stopped by operator", LevelInfo)
	s.mu.Unlock()
}

func (s *Session) stopTrajectoryLocked(reason string, level Level) {
	if !s.trajectory.Running {
		return
	}
	progress := s.trajectory.Progress
	s.trajectory.Running = false
	s.trajectory.Status = "idle"
	s.profile = nil
	s.logLocked(level, "Trajectory stopped",
		fmt.Sprintf("%s · %.0f%% complete", reason, progress*100))
}

// Trajectory returns a copy of the profiled-move state.
func (s *Session) Trajectory() TrajectoryStatus {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.trajectory
}

// advanceTrajectoryLocked samples the profile for this cycle and returns the
// position to command and the velocity feedforward that goes with it.
func (s *Session) advanceTrajectoryLocked(now time.Time) (position, feedforward float64, ok bool) {
	if !s.trajectory.Running || s.profile == nil {
		return 0, 0, false
	}
	// A profiled move has no business continuing unless the axis is armed, in
	// position mode and not stopped.
	if s.estop || !s.armed || s.mode != motor.CtrlPosition {
		s.stopTrajectoryLocked("axis left armed position mode", LevelWarn)
		return 0, 0, false
	}

	elapsed := now.Sub(s.trajectoryStartedAt).Seconds()
	point := s.profile.Sample(elapsed)

	s.trajectory.Elapsed = elapsed
	s.trajectory.Setpoint = point.Position
	s.trajectory.Feedforward = point.Velocity
	if s.trajectory.Total > 0 {
		s.trajectory.Progress = math.Min(1, elapsed/s.trajectory.Total)
	} else {
		s.trajectory.Progress = 1
	}
	s.trajectory.Status = fmt.Sprintf("%.0f%% · %s",
		s.trajectory.Progress*100, motor.Render(s.units).Position(point.Position))

	if elapsed >= s.trajectory.Total {
		// Hold the target rather than releasing it: the move is finished, the
		// axis is not.
		s.trajectory.Setpoint = s.trajectory.Target
		s.trajectory.Feedforward = 0
		s.stopTrajectoryLocked("completed", LevelOK)
		return s.trajectory.Target, 0, true
	}

	return point.Position, point.Velocity, true
}
