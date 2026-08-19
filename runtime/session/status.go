package session

import (
	"math"
	"time"

	"drlabs/motor"
	"drlabs/runtime/safety"
	"drlabs/runtime/timing"
)

// Status is a consistent snapshot of everything a supervisor needs to display.
//
// It is a deep copy.  The control loop mutates its state on every period, and
// a UI reading a shared map while that happens would see a torn view — or, in
// Go, would race.  Copying a few hundred bytes at whatever rate a UI refreshes
// is not a cost worth optimising away.
type Status struct {
	Connection   Connection
	ConnectError string
	Link         map[string]string
	Identity     *motor.Identity

	Armed         bool
	Mode          motor.ControlMode
	ArmBlockers   []string
	ArmAdvisories []string

	EStop EStopStatus

	Setpoint   Setpoint
	Params     Params
	Shuttle    Shuttle
	Trajectory TrajectoryStatus
	Safety     safety.Config
	Preset     string

	// ControllerBounds is the travel envelope stored in the controller, and
	// EffectiveLimits is what a command can actually reach once the host
	// envelope is intersected with it.  They are reported separately because
	// "the target is being clamped" and "by which of the two limits" are
	// different questions, and only the second one tells you what to change.
	ControllerBounds *[2]float64
	EffectiveLimits  [2]float64

	Telemetry motor.Telemetry
	Warnings  []string

	// LatchedFault keeps the fault that stopped the axis visible after the
	// controller's own register has been cleared by a stop.
	LatchedFault int

	Stats Stats
	At    time.Time
}

// EStopStatus describes a latched emergency stop.
type EStopStatus struct {
	Latched bool
	Code    string
	Reason  string
	Detail  string
}

// Stats reports how the control loop is actually performing, as opposed to
// how it was configured.
type Stats struct {
	RequestedHz int

	// AchievedHz is measured, not assumed.  A loop that cannot keep up is the
	// most common cause of surprising motor behaviour, and the only way to
	// know is to count cycles against the clock.
	AchievedHz float64

	// JitterMS is the standard deviation of the interval between cycles.
	JitterMS float64

	// Transaction is how long the bus round-trip takes.  A p95 far above the
	// mean is the signature of an adapter that mostly keeps up but stalls.
	Transaction timing.Statistics

	Cycles              uint64
	Replies             uint64
	Timeouts            uint64
	ConsecutiveTimeouts int
	ReplyRate           float64

	// WatchdogMS is the command timeout currently being sent, so an operator
	// can see how long the controller will keep driving if the host stops.
	WatchdogMS float64
}

// Status returns a snapshot.
func (s *Session) Status() Status {
	s.mu.Lock()
	// Keep the arm checklist live while disarmed, so an operator can see
	// exactly what stands between them and an armed axis without polling a
	// separate call for it.
	if s.connection == Connected && !s.armed {
		s.preflightLocked()
	}

	status := Status{
		Connection:    s.connection,
		ConnectError:  s.connectError,
		Identity:      s.identity,
		Armed:         s.armed,
		Mode:          s.mode,
		ArmBlockers:   copyStrings(s.armBlockers),
		ArmAdvisories: copyStrings(s.armAdvisories),
		EStop: EStopStatus{
			Latched: s.estop,
			Code:    s.estopCode,
			Reason:  s.estopReason,
			Detail:  s.estopDetail,
		},
		Setpoint:     s.setpoint,
		Params:       s.params,
		Shuttle:      s.shuttle,
		Trajectory:   s.trajectory,
		Safety:       s.safety,
		Preset:       s.preset,
		Telemetry:    s.telemetry,
		Warnings:     copyStrings(s.monitor.Warnings),
		LatchedFault: s.latchedFault,
		Stats:        s.statsLocked(),
		At:           time.Now(),
	}

	if s.connection != Disconnected && s.linkDesc != nil {
		status.Link = make(map[string]string, len(s.linkDesc))
		for key, value := range s.linkDesc {
			status.Link[key] = value
		}
	}

	if lower, upper, ok := s.identity.Bounds(); ok {
		status.ControllerBounds = &[2]float64{lower, upper}
	}
	lower, upper := s.effectiveLimitsLocked()
	status.EffectiveLimits = [2]float64{lower, upper}

	s.mu.Unlock()
	return status
}

func (s *Session) statsLocked() Stats {
	stats := Stats{
		RequestedHz:         s.rateHz,
		Transaction:         timing.Summarise(s.stats.durations),
		Cycles:              s.stats.cycles,
		Replies:             s.stats.replies,
		Timeouts:            s.stats.timeouts,
		ConsecutiveTimeouts: s.stats.consecutiveTimeouts,
		WatchdogMS:          s.safety.WatchdogTimeout(1.0/float64(s.rateHz)) * 1e3,
	}

	if len(s.stats.rateWindow) > 5 {
		span := s.stats.rateWindow[len(s.stats.rateWindow)-1].
			Sub(s.stats.rateWindow[0]).Seconds()
		if span > 0 {
			stats.AchievedHz = float64(len(s.stats.rateWindow)-1) / span
		}
	}
	if len(s.stats.periods) > 5 {
		stats.JitterMS = standardDeviationMS(s.stats.periods)
	}
	if s.stats.cycles > 0 {
		stats.ReplyRate = float64(s.stats.replies) / float64(s.stats.cycles)
	}
	return stats
}

func standardDeviationMS(samples []time.Duration) float64 {
	var total float64
	for _, sample := range samples {
		total += sample.Seconds()
	}
	mean := total / float64(len(samples))

	var variance float64
	for _, sample := range samples {
		delta := sample.Seconds() - mean
		variance += delta * delta
	}
	variance /= float64(len(samples))

	return math.Sqrt(variance) * 1e3
}

func copyStrings(items []string) []string {
	if len(items) == 0 {
		return nil
	}
	out := make([]string, len(items))
	copy(out, items)
	return out
}
