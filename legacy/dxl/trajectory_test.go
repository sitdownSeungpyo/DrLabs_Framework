package dxl

import (
	"math"
	"testing"
)

func TestNewTrapezoidalProfile(t *testing.T) {
	tests := []struct {
		name      string
		startPos  float64
		targetPos float64
		maxVel    float64
		accel     float64
		wantErr   bool
	}{
		{
			name:      "valid profile",
			startPos:  0,
			targetPos: 1000,
			maxVel:    500,
			accel:     1000,
			wantErr:   false,
		},
		{
			name:      "zero velocity",
			startPos:  0,
			targetPos: 1000,
			maxVel:    0,
			accel:     1000,
			wantErr:   true,
		},
		{
			name:      "negative velocity",
			startPos:  0,
			targetPos: 1000,
			maxVel:    -100,
			accel:     1000,
			wantErr:   true,
		},
		{
			name:      "zero acceleration",
			startPos:  0,
			targetPos: 1000,
			maxVel:    500,
			accel:     0,
			wantErr:   true,
		},
		{
			name:      "negative acceleration",
			startPos:  0,
			targetPos: 1000,
			maxVel:    500,
			accel:     -1000,
			wantErr:   true,
		},
		{
			name:      "same start and target",
			startPos:  500,
			targetPos: 500,
			maxVel:    100,
			accel:     200,
			wantErr:   false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			profile, err := NewTrapezoidalProfile(tt.startPos, tt.targetPos, tt.maxVel, tt.accel)
			if (err != nil) != tt.wantErr {
				t.Errorf("NewTrapezoidalProfile() error = %v, wantErr %v", err, tt.wantErr)
				return
			}
			if !tt.wantErr && profile == nil {
				t.Error("NewTrapezoidalProfile() returned nil profile without error")
			}
		})
	}
}

func TestTrapezoidalProfileCalculate(t *testing.T) {
	t.Run("trapezoidal profile (reaches max velocity)", func(t *testing.T) {
		profile, err := NewTrapezoidalProfile(0, 2000, 500, 1000)
		if err != nil {
			t.Fatalf("Failed to create profile: %v", err)
		}

		// With accel=1000, to reach maxVel=500 takes 0.5s
		// Distance during accel = 0.5 * 1000 * 0.5^2 = 125
		// Distance during accel+decel = 250
		// Cruise distance = 2000 - 250 = 1750
		// Cruise time = 1750 / 500 = 3.5s
		// Total time = 0.5 + 3.5 + 0.5 = 4.5s

		expectedTotalTime := 4.5
		if math.Abs(profile.TotalTime()-expectedTotalTime) > 0.001 {
			t.Errorf("TotalTime() = %v, want %v", profile.TotalTime(), expectedTotalTime)
		}
	})

	t.Run("triangular profile (doesn't reach max velocity)", func(t *testing.T) {
		profile, err := NewTrapezoidalProfile(0, 100, 1000, 1000)
		if err != nil {
			t.Fatalf("Failed to create profile: %v", err)
		}

		// Distance too short to reach maxVel
		// cruiseVel = sqrt(accel * distance) = sqrt(1000 * 100) = 316.23
		// accelTime = 316.23 / 1000 = 0.316s
		// Total time = 0.632s (no cruise phase)

		if profile.cruiseTime != 0 {
			t.Errorf("Expected cruiseTime=0 for triangular profile, got %v", profile.cruiseTime)
		}
		if profile.cruiseVel >= profile.MaxVelocity {
			t.Errorf("Expected cruiseVel < MaxVelocity for triangular profile")
		}
	})

	t.Run("zero distance", func(t *testing.T) {
		profile, err := NewTrapezoidalProfile(500, 500, 100, 200)
		if err != nil {
			t.Fatalf("Failed to create profile: %v", err)
		}

		if profile.TotalTime() != 0 {
			t.Errorf("TotalTime() = %v, want 0 for zero distance", profile.TotalTime())
		}
	})
}

func TestTrapezoidalProfileSample(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 1000, 500, 1000)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	t.Run("sample at start", func(t *testing.T) {
		point := profile.Sample(0)
		if point.Position != 0 {
			t.Errorf("Position at t=0: got %v, want 0", point.Position)
		}
		if point.Velocity != 0 {
			t.Errorf("Velocity at t=0: got %v, want 0", point.Velocity)
		}
	})

	t.Run("sample at end", func(t *testing.T) {
		point := profile.Sample(profile.TotalTime())
		if math.Abs(point.Position-1000) > 0.001 {
			t.Errorf("Position at end: got %v, want 1000", point.Position)
		}
		if point.Velocity != 0 {
			t.Errorf("Velocity at end: got %v, want 0", point.Velocity)
		}
	})

	t.Run("sample beyond end", func(t *testing.T) {
		point := profile.Sample(profile.TotalTime() + 10)
		if math.Abs(point.Position-1000) > 0.001 {
			t.Errorf("Position beyond end: got %v, want 1000", point.Position)
		}
	})

	t.Run("sample at negative time", func(t *testing.T) {
		point := profile.Sample(-1)
		if point.Position != 0 {
			t.Errorf("Position at t<0: got %v, want 0", point.Position)
		}
	})

	t.Run("velocity profile shape", func(t *testing.T) {
		// Check acceleration phase - velocity should increase
		p1 := profile.Sample(0.1)
		p2 := profile.Sample(0.2)
		if p2.Velocity <= p1.Velocity {
			t.Error("Velocity should increase during acceleration phase")
		}
		if p1.Accel <= 0 {
			t.Error("Acceleration should be positive during accel phase")
		}

		// Check deceleration phase - velocity should decrease
		endTime := profile.TotalTime()
		p3 := profile.Sample(endTime - 0.2)
		p4 := profile.Sample(endTime - 0.1)
		if p4.Velocity >= p3.Velocity {
			t.Error("Velocity should decrease during deceleration phase")
		}
		if p4.Accel >= 0 {
			t.Error("Acceleration should be negative during decel phase")
		}
	})
}

func TestTrapezoidalProfileNegativeDirection(t *testing.T) {
	profile, err := NewTrapezoidalProfile(1000, 0, 500, 1000)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	t.Run("start position", func(t *testing.T) {
		point := profile.Sample(0)
		if point.Position != 1000 {
			t.Errorf("Start position: got %v, want 1000", point.Position)
		}
	})

	t.Run("end position", func(t *testing.T) {
		point := profile.Sample(profile.TotalTime())
		if math.Abs(point.Position-0) > 0.001 {
			t.Errorf("End position: got %v, want 0", point.Position)
		}
	})

	t.Run("negative velocity direction", func(t *testing.T) {
		midPoint := profile.Sample(profile.TotalTime() / 2)
		if midPoint.Velocity >= 0 {
			t.Errorf("Velocity should be negative for reverse motion, got %v", midPoint.Velocity)
		}
	})
}

func TestTrapezoidalProfileGenerate(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 1000, 500, 1000)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	t.Run("generate at 100Hz", func(t *testing.T) {
		points := profile.Generate(100)
		expectedPoints := int(math.Ceil(profile.TotalTime()*100)) + 1

		if len(points) != expectedPoints {
			t.Errorf("Generated %d points, expected %d", len(points), expectedPoints)
		}

		// First point should be at start
		if points[0].Position != 0 {
			t.Errorf("First point position: got %v, want 0", points[0].Position)
		}

		// Last point should be at target
		lastPoint := points[len(points)-1]
		if math.Abs(lastPoint.Position-1000) > 0.001 {
			t.Errorf("Last point position: got %v, want 1000", lastPoint.Position)
		}
	})

	t.Run("generate zero distance", func(t *testing.T) {
		zeroProfile, _ := NewTrapezoidalProfile(500, 500, 100, 200)
		points := zeroProfile.Generate(100)

		if len(points) != 1 {
			t.Errorf("Zero distance should generate 1 point, got %d", len(points))
		}
		if points[0].Position != 500 {
			t.Errorf("Zero distance point position: got %v, want 500", points[0].Position)
		}
	})
}

func TestTrapezoidalProfileDuration(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 1000, 500, 1000)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	duration := profile.Duration()
	expectedDuration := profile.TotalTime() * 1e9 // Convert to nanoseconds

	if math.Abs(float64(duration.Nanoseconds())-expectedDuration) > 1000 {
		t.Errorf("Duration() = %v, want %v ns", duration, expectedDuration)
	}
}

func TestTrajectoryPointContinuity(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 2000, 500, 1000)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	// Generate high-resolution trajectory
	points := profile.Generate(1000) // 1kHz

	maxPositionJump := 0.0
	maxVelocityJump := 0.0

	for i := 1; i < len(points); i++ {
		posJump := math.Abs(points[i].Position - points[i-1].Position)
		velJump := math.Abs(points[i].Velocity - points[i-1].Velocity)

		if posJump > maxPositionJump {
			maxPositionJump = posJump
		}
		if velJump > maxVelocityJump {
			maxVelocityJump = velJump
		}
	}

	// At 1kHz with max velocity 500, max position change per step should be ~0.5
	if maxPositionJump > 1.0 {
		t.Errorf("Position discontinuity detected: max jump = %v", maxPositionJump)
	}

	// Velocity change should be smooth (accel * dt = 1000 * 0.001 = 1.0)
	if maxVelocityJump > 2.0 {
		t.Errorf("Velocity discontinuity detected: max jump = %v", maxVelocityJump)
	}
}

func BenchmarkTrapezoidalProfileSample(b *testing.B) {
	profile, _ := NewTrapezoidalProfile(0, 4096, 1000, 5000)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		t := float64(i%100) / 100.0 * profile.TotalTime()
		profile.Sample(t)
	}
}

func BenchmarkTrapezoidalProfileGenerate(b *testing.B) {
	profile, _ := NewTrapezoidalProfile(0, 4096, 1000, 5000)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		profile.Generate(100) // 100Hz
	}
}
