// Package trajectory provides motor-agnostic motion planning profiles.
package trajectory

import "time"

// Point represents a single point in a trajectory.
type Point struct {
	Time     float64 // Time from start (seconds)
	Position float64 // Position at this time
	Velocity float64 // Velocity at this time
	Accel    float64 // Acceleration at this time
}

// Profile generates trajectory points for smooth motor motion.
type Profile interface {
	// Sample returns the trajectory point at time t (seconds from start).
	Sample(t float64) Point

	// Generate creates a complete trajectory sampled at the given rate (Hz).
	Generate(sampleRate float64) []Point

	// TotalTime returns the total duration in seconds.
	TotalTime() float64

	// Duration returns the total duration as time.Duration.
	Duration() time.Duration
}
