package trajectory

import (
	"fmt"
	"math"
	"time"
)

// TrapezoidalProfile generates smooth motion with constant acceleration,
// constant velocity, and constant deceleration phases.
type TrapezoidalProfile struct {
	StartPos     float64 // Starting position
	TargetPos    float64 // Target position
	MaxVelocity  float64 // Maximum velocity (units/sec)
	Acceleration float64 // Acceleration (units/sec^2)

	totalTime  float64
	accelTime  float64
	decelTime  float64
	cruiseTime float64
	cruiseVel  float64
	distance   float64
}

// NewTrapezoidalProfile creates a new trapezoidal velocity profile.
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

func (p *TrapezoidalProfile) calculate() {
	p.distance = math.Abs(p.TargetPos - p.StartPos)

	if p.distance == 0 {
		p.totalTime = 0
		p.accelTime = 0
		p.decelTime = 0
		p.cruiseTime = 0
		p.cruiseVel = 0
		return
	}

	timeToMaxVel := p.MaxVelocity / p.Acceleration
	distanceAccelDecel := p.MaxVelocity * timeToMaxVel

	if distanceAccelDecel > p.distance {
		// Triangular profile
		p.cruiseVel = math.Sqrt(p.Acceleration * p.distance)
		p.accelTime = p.cruiseVel / p.Acceleration
		p.decelTime = p.accelTime
		p.cruiseTime = 0
	} else {
		// Trapezoidal profile
		p.cruiseVel = p.MaxVelocity
		p.accelTime = timeToMaxVel
		p.decelTime = timeToMaxVel
		p.cruiseTime = (p.distance - distanceAccelDecel) / p.MaxVelocity
	}

	p.totalTime = p.accelTime + p.cruiseTime + p.decelTime
}

// Sample returns the trajectory point at time t.
func (p *TrapezoidalProfile) Sample(t float64) Point {
	if t <= 0 {
		return Point{Time: 0, Position: p.StartPos}
	}

	if t >= p.totalTime {
		return Point{Time: p.totalTime, Position: p.TargetPos}
	}

	direction := 1.0
	if p.TargetPos < p.StartPos {
		direction = -1.0
	}

	var pos, vel, accel float64

	if t <= p.accelTime {
		accel = p.Acceleration
		vel = accel * t
		pos = 0.5 * accel * t * t
	} else if t <= p.accelTime+p.cruiseTime {
		accel = 0
		vel = p.cruiseVel
		tCruise := t - p.accelTime
		posCruiseStart := 0.5 * p.Acceleration * p.accelTime * p.accelTime
		pos = posCruiseStart + vel*tCruise
	} else {
		accel = -p.Acceleration
		tDecel := t - p.accelTime - p.cruiseTime
		velDecelStart := p.cruiseVel
		vel = velDecelStart - p.Acceleration*tDecel
		posCruiseStart := 0.5 * p.Acceleration * p.accelTime * p.accelTime
		posCruiseEnd := posCruiseStart + p.cruiseVel*p.cruiseTime
		pos = posCruiseEnd + velDecelStart*tDecel - 0.5*p.Acceleration*tDecel*tDecel
	}

	return Point{
		Time:     t,
		Position: p.StartPos + direction*pos,
		Velocity: direction * vel,
		Accel:    direction * accel,
	}
}

// Generate creates a complete trajectory sampled at the given rate (Hz).
func (p *TrapezoidalProfile) Generate(sampleRate float64) []Point {
	if sampleRate <= 0 {
		return nil
	}
	if p.totalTime == 0 {
		return []Point{{Position: p.StartPos}}
	}

	dt := 1.0 / sampleRate
	numPoints := int(math.Ceil(p.totalTime*sampleRate)) + 1
	points := make([]Point, 0, numPoints)

	for i := 0; i < numPoints; i++ {
		t := float64(i) * dt
		if t > p.totalTime {
			t = p.totalTime
		}
		points = append(points, p.Sample(t))
	}

	return points
}

// TotalTime returns the total duration in seconds.
func (p *TrapezoidalProfile) TotalTime() float64 {
	return p.totalTime
}

// Duration returns the total duration as time.Duration.
func (p *TrapezoidalProfile) Duration() time.Duration {
	return time.Duration(p.totalTime * float64(time.Second))
}

// CruiseTime returns the cruise phase duration (exported for testing).
func (p *TrapezoidalProfile) CruiseTime() float64 {
	return p.cruiseTime
}

// CruiseVelocity returns the actual cruise velocity (exported for testing).
func (p *TrapezoidalProfile) CruiseVelocity() float64 {
	return p.cruiseVel
}
