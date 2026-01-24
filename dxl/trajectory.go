package dxl

import (
	"fmt"
	"math"
	"time"
)

// TrapezoidalProfile represents a trapezoidal velocity profile for motion planning.
// It generates smooth motion with constant acceleration, constant velocity, and constant deceleration phases.
type TrapezoidalProfile struct {
	StartPos     float64       // Starting position
	TargetPos    float64       // Target position
	MaxVelocity  float64       // Maximum velocity (units/sec)
	Acceleration float64       // Acceleration (units/sec^2)

	// Calculated profile parameters
	totalTime    float64       // Total time for the motion
	accelTime    float64       // Time for acceleration phase
	decelTime    float64       // Time for deceleration phase
	cruiseTime   float64       // Time for constant velocity phase
	cruiseVel    float64       // Actual cruise velocity (may be < MaxVelocity)
	distance     float64       // Total distance to travel
}

// TrajectoryPoint represents a single point in the trajectory
type TrajectoryPoint struct {
	Time     float64 // Time from start (seconds)
	Position float64 // Position at this time
	Velocity float64 // Velocity at this time
	Accel    float64 // Acceleration at this time
}

// NewTrapezoidalProfile creates a new trapezoidal velocity profile.
// Parameters:
//   - startPos: Starting position (motor units, e.g., encoder ticks)
//   - targetPos: Target position (motor units)
//   - maxVel: Maximum velocity (units/second)
//   - accel: Acceleration (units/second^2)
func NewTrapezoidalProfile(startPos, targetPos, maxVel, accel float64) (*TrapezoidalProfile, error) {
	if maxVel <= 0 {
		return nil, fmt.Errorf("max velocity must be positive")
	}
	if accel <= 0 {
		return nil, fmt.Errorf("acceleration must be positive")
	}

	profile := &TrapezoidalProfile{
		StartPos:     startPos,
		TargetPos:    targetPos,
		MaxVelocity:  maxVel,
		Acceleration: accel,
	}

	profile.calculate()
	return profile, nil
}

// calculate computes the profile timing and parameters
func (p *TrapezoidalProfile) calculate() {
	p.distance = math.Abs(p.TargetPos - p.StartPos)

	if p.distance == 0 {
		// No movement needed
		p.totalTime = 0
		p.accelTime = 0
		p.decelTime = 0
		p.cruiseTime = 0
		p.cruiseVel = 0
		return
	}

	// Time to reach max velocity
	timeToMaxVel := p.MaxVelocity / p.Acceleration

	// Distance traveled during acceleration and deceleration
	distanceAccelDecel := p.MaxVelocity * timeToMaxVel

	if distanceAccelDecel > p.distance {
		// Triangular profile - never reaches max velocity
		p.cruiseVel = math.Sqrt(p.Acceleration * p.distance)
		p.accelTime = p.cruiseVel / p.Acceleration
		p.decelTime = p.accelTime
		p.cruiseTime = 0
	} else {
		// Trapezoidal profile - reaches max velocity
		p.cruiseVel = p.MaxVelocity
		p.accelTime = timeToMaxVel
		p.decelTime = timeToMaxVel
		p.cruiseTime = (p.distance - distanceAccelDecel) / p.MaxVelocity
	}

	p.totalTime = p.accelTime + p.cruiseTime + p.decelTime
}

// Sample returns the trajectory point at a given time.
// Time should be in seconds from the start of motion.
func (p *TrapezoidalProfile) Sample(t float64) TrajectoryPoint {
	if t <= 0 {
		return TrajectoryPoint{
			Time:     0,
			Position: p.StartPos,
			Velocity: 0,
			Accel:    0,
		}
	}

	if t >= p.totalTime {
		return TrajectoryPoint{
			Time:     p.totalTime,
			Position: p.TargetPos,
			Velocity: 0,
			Accel:    0,
		}
	}

	direction := 1.0
	if p.TargetPos < p.StartPos {
		direction = -1.0
	}

	var pos, vel, accel float64

	if t <= p.accelTime {
		// Acceleration phase
		accel = p.Acceleration
		vel = accel * t
		pos = 0.5 * accel * t * t
	} else if t <= p.accelTime + p.cruiseTime {
		// Constant velocity (cruise) phase
		accel = 0
		vel = p.cruiseVel
		tCruise := t - p.accelTime
		posCruiseStart := 0.5 * p.Acceleration * p.accelTime * p.accelTime
		pos = posCruiseStart + vel * tCruise
	} else {
		// Deceleration phase
		accel = -p.Acceleration
		tDecel := t - p.accelTime - p.cruiseTime
		velDecelStart := p.cruiseVel
		vel = velDecelStart - p.Acceleration * tDecel
		posCruiseStart := 0.5 * p.Acceleration * p.accelTime * p.accelTime
		posCruiseEnd := posCruiseStart + p.cruiseVel * p.cruiseTime
		pos = posCruiseEnd + velDecelStart * tDecel - 0.5 * p.Acceleration * tDecel * tDecel
	}

	return TrajectoryPoint{
		Time:     t,
		Position: p.StartPos + direction * pos,
		Velocity: direction * vel,
		Accel:    direction * accel,
	}
}

// Generate creates a complete trajectory with points sampled at the given rate.
// sampleRate is in Hz (samples per second).
func (p *TrapezoidalProfile) Generate(sampleRate float64) []TrajectoryPoint {
	if p.totalTime == 0 {
		return []TrajectoryPoint{{
			Time:     0,
			Position: p.StartPos,
			Velocity: 0,
			Accel:    0,
		}}
	}

	dt := 1.0 / sampleRate
	numPoints := int(math.Ceil(p.totalTime * sampleRate)) + 1

	points := make([]TrajectoryPoint, 0, numPoints)

	for i := 0; i < numPoints; i++ {
		t := float64(i) * dt
		if t > p.totalTime {
			t = p.totalTime
		}
		points = append(points, p.Sample(t))
	}

	return points
}

// Duration returns the total duration of the trajectory in seconds
func (p *TrapezoidalProfile) Duration() time.Duration {
	return time.Duration(p.totalTime * float64(time.Second))
}

// TotalTime returns the total time in seconds
func (p *TrapezoidalProfile) TotalTime() float64 {
	return p.totalTime
}

// TrajectoryExecutor executes a trajectory on a motor using the controller
type TrajectoryExecutor struct {
	controller *Controller
	motorID    uint8
}

// NewTrajectoryExecutor creates a new trajectory executor
func NewTrajectoryExecutor(controller *Controller, motorID uint8) *TrajectoryExecutor {
	return &TrajectoryExecutor{
		controller: controller,
		motorID:    motorID,
	}
}

// Execute runs the trajectory on the motor.
// This is a blocking call that sends position commands at the specified rate.
func (e *TrajectoryExecutor) Execute(profile *TrapezoidalProfile, updateRate float64) error {
	points := profile.Generate(updateRate)

	if len(points) == 0 {
		return fmt.Errorf("empty trajectory")
	}

	ticker := time.NewTicker(time.Duration(float64(time.Second) / updateRate))
	defer ticker.Stop()

	for i, point := range points {
		position := uint32(point.Position)

		// Send command to motor
		e.controller.CommandChan <- []Command{
			{ID: e.motorID, Value: position},
		}

		// Wait for next update (except for last point)
		if i < len(points)-1 {
			<-ticker.C
		}
	}

	return nil
}

// ExecuteAsync runs the trajectory asynchronously.
// Returns a channel that will be closed when the trajectory is complete.
func (e *TrajectoryExecutor) ExecuteAsync(profile *TrapezoidalProfile, updateRate float64) (<-chan error, error) {
	points := profile.Generate(updateRate)

	if len(points) == 0 {
		return nil, fmt.Errorf("empty trajectory")
	}

	errChan := make(chan error, 1)

	go func() {
		defer close(errChan)

		ticker := time.NewTicker(time.Duration(float64(time.Second) / updateRate))
		defer ticker.Stop()

		for i, point := range points {
			position := uint32(point.Position)

			// Send command to motor
			select {
			case e.controller.CommandChan <- []Command{
				{ID: e.motorID, Value: position},
			}:
			default:
				errChan <- fmt.Errorf("command channel full")
				return
			}

			// Wait for next update (except for last point)
			if i < len(points)-1 {
				<-ticker.C
			}
		}
	}()

	return errChan, nil
}
