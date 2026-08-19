package session

import (
	"errors"
	"fmt"
	"math"
	"time"

	"drlabs/motor"
)

// The A↔B shuttle: an unattended back-and-forth between two positions, for
// burn-in, endurance runs and mechanism testing.
//
// The sequencer lives inside the control loop rather than in an application
// on top of it.  That is the whole point: its timing does not depend on a UI
// staying responsive, and it stops the instant the axis is disarmed, faulted
// or emergency-stopped, because it is evaluated by the same code that decides
// whether to command anything at all.

// Shuttle is the A↔B sequencer's state.
type Shuttle struct {
	Running bool

	A         float64 // rev
	B         float64 // rev
	Dwell     float64 // s to hold at each end
	Cycles    int     // 0 runs until stopped
	Completed int     // full A→B→A cycles finished

	// Phase is one of "idle", "approach", "moving", "dwell".  The approach
	// phase is the initial run to A, which is not counted as a cycle.
	Phase  string
	Target string // "a" or "b"
	Status string // human readable progress

	wasApproach  bool
	arrivedAt    time.Time
	legStartedAt time.Time
	legs         int
}

// Tolerances for calling a leg finished.  Both matter: position alone would
// count a fast pass through the target as an arrival.
const (
	shuttleTolerance = 0.0028 // rev, about one degree
	shuttleStillness = 0.05   // rev/s
)

func defaultShuttle() Shuttle {
	return Shuttle{
		A:      -0.25, // -90°
		B:      0.25,  // +90°
		Dwell:  1.0,
		Phase:  "idle",
		Target: "a",
	}
}

// Shuttle returns a copy of the sequencer state.
func (s *Session) Shuttle() Shuttle {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.shuttle
}

// ConfigureShuttle updates the endpoints and timing without starting it.
func (s *Session) ConfigureShuttle(a, b, dwell float64, cycles int) Shuttle {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.shuttle.A, _ = s.clampTargetLocked(a)
	s.shuttle.B, _ = s.clampTargetLocked(b)
	s.shuttle.Dwell = motor.Clamp(dwell, 0, 600)
	if cycles < 0 {
		cycles = 0
	}
	s.shuttle.Cycles = cycles
	return s.shuttle
}

// StartShuttle begins the A↔B sequence.  It requires an armed axis in position
// mode, and switches into position mode if it is not there already.
func (s *Session) StartShuttle() error {
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
	if math.Abs(s.shuttle.B-s.shuttle.A) < 1e-4 {
		s.mu.Unlock()
		return errors.New("session: A and B are the same position")
	}

	s.stopTrajectoryLocked("the shuttle was started", LevelInfo)
	s.shuttle.Running = true
	s.shuttle.Completed = 0
	s.shuttle.Phase = "approach"
	s.shuttle.Target = "a"
	s.shuttle.legs = 0
	s.shuttle.legStartedAt = time.Now()
	a, b, dwell, cycles := s.shuttle.A, s.shuttle.B, s.shuttle.Dwell, s.shuttle.Cycles
	s.mu.Unlock()

	limit := "until stopped"
	if cycles > 0 {
		limit = fmt.Sprintf("%d cycles", cycles)
	}
	s.log(LevelOK, "Shuttle started",
		fmt.Sprintf("A %.4f rev ↔ B %.4f rev · dwell %.2f s · %s", a, b, dwell, limit))
	s.signal()
	return nil
}

// StopShuttle ends the sequence and leaves the axis where it is.
func (s *Session) StopShuttle() {
	s.mu.Lock()
	s.stopShuttleLocked("stopped by operator", LevelInfo)
	s.mu.Unlock()
}

func (s *Session) stopShuttleLocked(reason string, level Level) {
	if !s.shuttle.Running {
		return
	}
	completed := s.shuttle.Completed
	s.shuttle.Running = false
	s.shuttle.Phase = "idle"
	s.shuttle.Status = ""
	s.logLocked(level, "Shuttle stopped",
		fmt.Sprintf("%s · %d cycle(s) completed", reason, completed))
}

// legTimeout is how long one leg may take before the axis is called stuck.
//
// It is derived from the distance and the velocity limit rather than being a
// fixed number, so a slow long move is not mistaken for a jam — with a
// generous multiplier and a floor, because acceleration limits and dwell make
// the ideal time an underestimate.
func (s *Session) legTimeout() time.Duration {
	span := math.Abs(s.shuttle.B - s.shuttle.A)
	speed := math.Max(0.01, math.Min(s.params.VelocityLimit, s.safety.MaxVelocity))
	seconds := math.Max(5.0, (span/speed)*3.0+5.0)
	return time.Duration(seconds * float64(time.Second))
}

// advanceShuttleLocked runs one step of the sequencer and returns the position
// to command.  The second return is false when the shuttle has nothing to say
// this cycle, in which case the existing setpoint stands.
func (s *Session) advanceShuttleLocked(now time.Time) (float64, bool) {
	if !s.shuttle.Running {
		return 0, false
	}
	// The sequence has no business running unless the axis is armed, in
	// position mode and not stopped.  Checking here rather than relying on the
	// caller means every path that leaves that state also ends the sequence.
	if s.estop || !s.armed || s.mode != motor.CtrlPosition {
		s.stopShuttleLocked("axis left armed position mode", LevelWarn)
		return 0, false
	}

	target := s.shuttle.A
	if s.shuttle.Target == "b" {
		target = s.shuttle.B
	}
	target, _ = s.clampTargetLocked(target)

	if !motor.Valid(s.telemetry.Position) {
		return target, true
	}
	velocity := s.telemetry.Velocity
	if !motor.Valid(velocity) {
		velocity = 0
	}

	label := "A"
	if s.shuttle.Target == "b" {
		label = "B"
	}
	arrived := math.Abs(s.telemetry.Position-target) <= shuttleTolerance &&
		math.Abs(velocity) <= shuttleStillness

	switch s.shuttle.Phase {
	case "approach", "moving":
		s.shuttle.Status = "moving to " + label
		switch {
		case arrived:
			s.shuttle.wasApproach = s.shuttle.Phase == "approach"
			s.shuttle.Phase = "dwell"
			s.shuttle.arrivedAt = now
		case now.Sub(s.shuttle.legStartedAt) > s.legTimeout():
			s.stopShuttleLocked(fmt.Sprintf("axis did not reach %s within %.0f s",
				label, s.legTimeout().Seconds()), LevelWarn)
			return 0, false
		}

	case "dwell":
		remaining := s.shuttle.Dwell - now.Sub(s.shuttle.arrivedAt).Seconds()
		s.shuttle.Status = fmt.Sprintf("holding at %s (%.1f s)", label, math.Max(0, remaining))
		if remaining > 0 {
			break
		}

		// The first leg only brings the axis to A; counting starts once the
		// real A→B→A shuttle begins.
		if s.shuttle.wasApproach {
			s.shuttle.wasApproach = false
		} else {
			s.shuttle.legs++
			s.shuttle.Completed = s.shuttle.legs / 2
			if s.shuttle.Cycles > 0 && s.shuttle.Target == "a" &&
				s.shuttle.Completed >= s.shuttle.Cycles {
				s.stopShuttleLocked(
					fmt.Sprintf("%d cycle(s) finished", s.shuttle.Cycles), LevelOK)
				return target, true
			}
		}

		if s.shuttle.Target == "a" {
			s.shuttle.Target = "b"
			target = s.shuttle.B
		} else {
			s.shuttle.Target = "a"
			target = s.shuttle.A
		}
		s.shuttle.Phase = "moving"
		s.shuttle.legStartedAt = now
		target, _ = s.clampTargetLocked(target)
	}

	return target, true
}
