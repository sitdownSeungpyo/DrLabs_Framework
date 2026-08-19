package session

import (
	"context"
	"fmt"
	"math"
	"time"

	"drlabs/motor"
	"drlabs/runtime/safety"
	"drlabs/runtime/timing"
)

// loopStats is the loop's own bookkeeping.  It lives on the Session but is
// written only by the loop goroutine.
type loopStats struct {
	cycles              uint64
	replies             uint64
	timeouts            uint64
	consecutiveTimeouts int

	durations  []time.Duration // transaction time
	periods    []time.Duration // measured interval between cycles
	rateWindow []time.Time

	lastCycleAt time.Time
	rateWarnAt  time.Time
}

const statsWindow = 400

func (l *loopStats) record(sample []time.Duration, value time.Duration) []time.Duration {
	if len(sample) >= statsWindow {
		copy(sample, sample[1:])
		sample = sample[:statsWindow-1]
	}
	return append(sample, value)
}

// run is the control loop.  It is the only goroutine that touches the Link.
func (s *Session) run(ctx context.Context) {
	defer close(s.done)

	// A coarse OS timer turns a 100 Hz loop into a 64 Hz one; ask for
	// something finer for as long as we are running.
	_, releaseTimer := timing.RaiseResolution()
	defer releaseTimer()

	period := s.currentPeriod()
	next := time.Now().Add(period)

	for {
		period = s.currentPeriod()
		spin := time.Duration(math.Min(float64(timing.DefaultSpin), float64(period)/4))
		timing.SleepUntil(ctx, next, s.wake, spin)

		if ctx.Err() != nil {
			return
		}

		next = next.Add(period)
		// If we have fallen more than a period behind — a long GC pause, a
		// laptop resuming from sleep — resynchronise rather than trying to
		// catch up with a burst of back-to-back cycles.
		if next.Before(time.Now().Add(-period)) {
			next = time.Now().Add(period)
		}

		s.cycle(ctx, period)
	}
}

func (s *Session) currentPeriod() time.Duration {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return time.Duration(float64(time.Second) / float64(s.rateHz))
}

// cycle runs one control period: build a command, transact, evaluate.
func (s *Session) cycle(ctx context.Context, period time.Duration) {
	s.mu.Lock()
	link := s.link
	if link == nil {
		s.mu.Unlock()
		return
	}

	started := time.Now()
	if !s.stats.lastCycleAt.IsZero() {
		s.stats.periods = s.stats.record(s.stats.periods, started.Sub(s.stats.lastCycleAt))
	}
	s.stats.lastCycleAt = started

	cmd := s.buildCommandLocked(period)
	profile := s.profileForLocked(cmd)
	wasFull := cmd.Full
	s.mu.Unlock()

	telemetry, err := link.Cycle(ctx, cmd, profile)

	s.mu.Lock()
	s.stats.cycles++
	s.stats.durations = s.stats.record(s.stats.durations, time.Since(started))
	s.stats.rateWindow = appendCapped(s.stats.rateWindow, started, 200)

	var trip *safety.Trip
	var estopArgs *estopRequest

	switch {
	case err != nil:
		estopArgs = &estopRequest{"link_error", "Transport error", err.Error()}

	case telemetry == nil:
		s.stats.timeouts++
		s.stats.consecutiveTimeouts++
		if s.stats.consecutiveTimeouts >= s.safety.MaxConsecutiveTimeouts && !s.estop {
			estopArgs = &estopRequest{
				"comm_loss",
				"No reply from the controller",
				fmt.Sprintf("%d consecutive cycles without a response. The controller's own "+
					"watchdog will stop the motor after %.0f ms.",
					s.stats.consecutiveTimeouts, s.safety.WatchdogTimeout(period.Seconds())*1e3),
			}
		}

	default:
		if wasFull {
			s.paramsDirty = false
			s.paramsSentAt = started
		}
		s.stats.consecutiveTimeouts = 0
		s.stats.replies++
		s.lastReplyAt = time.Now()
		s.telemetry.Merge(*telemetry)

		s.samples.append(time.Since(s.t0), s.telemetry)

		if s.telemetry.HardFault {
			s.latchedFault = s.telemetry.Fault
		}
		if profile == motor.ProfileVerify {
			s.checkActiveLimitsLocked(*telemetry, period)
		}
		trip = s.monitor.Evaluate(s.telemetry, cmd, s.armed, time.Now())
	}

	// Deadman: the supervising client has to keep talking to us.
	if estopArgs == nil && s.armed && s.safety.DeadmanTimeout > 0 {
		if idle := time.Since(s.heartbeatAt).Seconds(); idle > s.safety.DeadmanTimeout {
			estopArgs = &estopRequest{
				"deadman",
				"Lost contact with the supervising client",
				fmt.Sprintf("No heartbeat for %.1f s.", idle),
			}
		}
	}

	// Energise as soon as the axis is fit to be energised.  In the servo role
	// this is what "the framework is running" means.
	s.tryAutoArmLocked()

	// Re-assert the full parameter set periodically, so a controller that
	// restarted does not keep running on defaults we never chose.
	if s.armed && !s.paramsDirty && started.Sub(s.paramsSentAt) > paramRefresh {
		s.paramsDirty = true
	}

	s.cycleCount++
	rateWarning := s.checkAchievedRateLocked(started, period)
	alreadyStopped := s.estop
	s.mu.Unlock()

	if rateWarning != nil {
		s.log(LevelWarn, rateWarning.message, rateWarning.detail)
	}
	if trip != nil && !alreadyStopped {
		s.EStop(trip.Code, trip.Message, trip.Detail)
	}
	if estopArgs != nil && !alreadyStopped {
		s.EStop(estopArgs.code, estopArgs.message, estopArgs.detail)
	}
}

type estopRequest struct{ code, message, detail string }

// buildCommandLocked translates the state machine into one command.
func (s *Session) buildCommandLocked(period time.Duration) motor.Command {
	// A pending one-shot — re-referencing the output, say — takes this cycle
	// and is then forgotten.
	if s.pendingSpecial != nil {
		cmd := *s.pendingSpecial
		s.pendingSpecial = nil
		return cmd
	}

	if s.estop || s.stopBurst > 0 {
		if s.stopBurst > 0 {
			s.stopBurst--
		}
		return motor.NewCommand(motor.CtrlStop)
	}
	if !s.armed {
		// Disarmed: watch the axis, do not drive it.
		return motor.NewCommand(motor.CtrlQuery)
	}

	cmd := motor.NewCommand(s.mode)
	cmd.WatchdogTimeout = s.safety.WatchdogTimeout(period.Seconds())
	cmd.VelocityLimit = min(s.params.VelocityLimit, s.safety.MaxVelocity)
	cmd.AccelLimit = s.params.AccelLimit
	cmd.KpScale = s.params.KpScale
	cmd.KdScale = s.params.KdScale
	cmd.ILimitScale = s.params.ILimitScale
	cmd.FeedforwardTorque = s.params.Feedforward
	cmd.IgnorePositionBounds = s.params.IgnorePositionBounds
	cmd.DeferTorqueToController = s.params.DeferTorqueToController
	cmd.Full = s.paramsDirty

	if !s.params.DeferTorqueToController {
		cmd.MaxTorque = min(s.params.MaxTorque, s.safety.MaxTorque)
	}

	switch s.mode {
	case motor.CtrlStop, motor.CtrlBrake, motor.CtrlHold:
		// Nothing further: the mode is the command.

	case motor.CtrlStayWithin:
		cmd.LowerBound, _ = s.safety.ClampPosition(s.setpoint.WithinLower)
		cmd.UpperBound, _ = s.safety.ClampPosition(s.setpoint.WithinUpper)
		// The bounds are the whole point of this mode, so they go out every
		// cycle rather than only when the parameter set is refreshed.
		cmd.Full = true

	case motor.CtrlPosition:
		// A host-profiled move and the shuttle both own the setpoint; only one
		// of them can be running, which StartTrajectory and StartShuttle
		// enforce between them.
		feedforward := 0.0
		if s.trajectory.Running {
			if position, velocity, ok := s.advanceTrajectoryLocked(time.Now()); ok {
				s.setpoint.Position = position
				feedforward = velocity
			}
		} else if s.shuttle.Running {
			if target, ok := s.advanceShuttleLocked(time.Now()); ok {
				s.setpoint.Position = target
			}
		}
		target, clamped := s.clampTargetLocked(s.setpoint.Position)
		if clamped {
			s.setpoint.Position = target
		}
		cmd.Position = target
		// The profile knows how fast the axis should be going, and saying so
		// is the difference between the controller chasing a moving target and
		// tracking one.  Without a profile there is nothing to feed forward.
		cmd.Velocity = feedforward

		// Tell the safety monitor whether motion is expected, so that holding
		// against a load is not mistaken for a stall.
		if motor.Valid(s.telemetry.Position) &&
			math.Abs(target-s.telemetry.Position) > max(0.02, s.safety.StallVelocity) {
			cmd.VelocityTarget = cmd.VelocityLimit
		}

	case motor.CtrlVelocity:
		velocity, clamped := s.safety.ClampVelocity(s.setpoint.Velocity)
		if clamped {
			s.setpoint.Velocity = velocity
		}
		cmd.Velocity = velocity
		cmd.VelocityTarget = velocity

	case motor.CtrlTorque:
		cmd.VelocityTarget = 0
	}

	return cmd
}

// profileForLocked picks this cycle's query profile.
func (s *Session) profileForLocked(cmd motor.Command) motor.QueryProfile {
	profile := motor.ProfileFast
	switch {
	case s.cycleCount%verifyQueryDivisor == 0:
		profile = motor.ProfileVerify
	case s.cycleCount%fullQueryDivisor == 0:
		profile = motor.ProfileFull
	}

	// A full parameter frame plus a wide query would approach the CAN-FD
	// payload limit; keep them on separate cycles.
	if cmd.Full && profile != motor.ProfileFast {
		profile = motor.ProfileFast
	}
	return profile
}

// checkActiveLimitsLocked compares the controller's live command registers
// with what this session believes it commanded.
//
// A mismatch is not a theoretical worry.  Command registers are sticky and
// survive things the host does not see — a brownout that resets the
// controller, a dropped frame, another program on the same bus.  When they
// disagree, the parameter frame is re-sent immediately rather than waiting for
// the next periodic refresh.
func (s *Session) checkActiveLimitsLocked(telemetry motor.Telemetry, period time.Duration) {
	if !s.armed {
		return
	}
	switch s.mode {
	case motor.CtrlPosition, motor.CtrlVelocity, motor.CtrlStayWithin:
	default:
		return
	}

	type check struct {
		name     string
		expected float64
		actual   float64
	}
	checks := []check{
		{"velocity limit", min(s.params.VelocityLimit, s.safety.MaxVelocity), telemetry.ActiveVelocityLimit},
		{"acceleration limit", s.params.AccelLimit, telemetry.ActiveAccelLimit},
		{"watchdog timeout", s.safety.WatchdogTimeout(period.Seconds()), telemetry.ActiveTimeout},
	}
	if !s.params.DeferTorqueToController {
		checks = append(checks, check{
			"max torque", min(s.params.MaxTorque, s.safety.MaxTorque), telemetry.ActiveMaxTorque})
	}

	for _, item := range checks {
		if !motor.Valid(item.actual) {
			continue
		}
		tolerance := max(0.02, math.Abs(item.expected)*0.05)
		if math.Abs(item.actual-item.expected) > tolerance {
			s.paramsDirty = true
			s.logLocked(LevelWarn,
				"Controller limit mismatch ("+item.name+")",
				fmt.Sprintf("expected %.3f, controller reports %.3f — resending parameters",
					item.expected, item.actual))
			return
		}
	}
}

type rateWarning struct{ message, detail string }

// checkAchievedRateLocked tells the operator when the requested rate is not
// being achieved, instead of quietly running slower than the status claims.
func (s *Session) checkAchievedRateLocked(now time.Time, period time.Duration) *rateWarning {
	if s.cycleCount%200 != 0 || len(s.stats.rateWindow) <= 50 {
		return nil
	}
	span := s.stats.rateWindow[len(s.stats.rateWindow)-1].Sub(s.stats.rateWindow[0]).Seconds()
	if span <= 0 {
		return nil
	}

	achieved := float64(len(s.stats.rateWindow)-1) / span
	if achieved >= float64(s.rateHz)*0.85 {
		return nil
	}
	if now.Sub(s.stats.rateWarnAt) <= 30*time.Second {
		return nil
	}
	s.stats.rateWarnAt = now

	return &rateWarning{
		message: fmt.Sprintf("Control loop is running at %.0f Hz, not %d Hz", achieved, s.rateHz),
		detail: fmt.Sprintf("The transport cannot complete a transaction every %.1f ms. "+
			"Lower the control rate.", period.Seconds()*1e3),
	}
}

func appendCapped[T any](slice []T, value T, limit int) []T {
	if len(slice) >= limit {
		copy(slice, slice[1:])
		slice = slice[:limit-1]
	}
	return append(slice, value)
}

// envelopeFor extracts a Link's hardware ratings, if it publishes any.
func envelopeFor(link motor.Link) *motor.Envelope {
	if enveloped, ok := link.(motor.Enveloped); ok {
		envelope := enveloped.Envelope()
		return &envelope
	}
	return nil
}

// tryAutoArmLocked energises the axis when it is configured to run
// unattended and nothing is standing in the way.
//
// It is called every cycle rather than once at connect, because the reasons an
// axis cannot arm are usually transient — a supply still settling, a
// controller still reporting the fault it powered up with — and a servo that
// gave up on the first attempt would need a human to notice, which is the
// thing this role exists to avoid.
//
// The one condition it will not wait out is a latched emergency stop.  That
// has to be cleared deliberately, by whatever decided to raise it.
func (s *Session) tryAutoArmLocked() {
	if !s.autoArm || s.armed || s.estop || s.connection != Connected {
		return
	}
	// Nothing has been heard from the controller yet; there is nothing to
	// check preflight against.
	if s.telemetry.Mode < 0 && !motor.Valid(s.telemetry.Position) {
		return
	}

	blocking, _ := s.preflightLocked()
	if len(blocking) > 0 {
		// Say why, but once a second rather than at the control rate.
		if time.Since(s.autoArmReportedAt) > time.Second {
			s.autoArmReportedAt = time.Now()
			s.logLocked(LevelWarn, "Waiting to arm", join(blocking))
		}
		return
	}

	s.monitor.Reset()
	s.armed = true
	s.heartbeatAt = time.Now()
	s.paramsDirty = true
	s.autoArmReportedAt = time.Time{}

	// Adopt the position the axis is already at, so energising is not itself a
	// move.  This is the whole difference between "torque on, holding" and
	// "torque on, and now travelling to wherever the last setpoint was".
	s.mode = s.initialMode
	if motor.Valid(s.telemetry.Position) {
		switch s.mode {
		case motor.CtrlPosition:
			s.setpoint.Position = s.telemetry.Position
		case motor.CtrlStayWithin:
			span := max(0.1, (s.setpoint.WithinUpper-s.setpoint.WithinLower)/2)
			s.setpoint.WithinLower = s.telemetry.Position - span
			s.setpoint.WithinUpper = s.telemetry.Position + span
		}
	}
	if s.mode == motor.CtrlVelocity {
		s.setpoint.Velocity = 0
	}

	s.logLocked(LevelOK, "Armed automatically", fmt.Sprintf(
		"holding %s at %.4f rev · torque limit %.2f Nm",
		s.mode, s.setpoint.Position, min(s.params.MaxTorque, s.safety.MaxTorque)))
}
