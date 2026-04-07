package trajectory

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
		{"valid profile", 0, 1000, 500, 1000, false},
		{"zero velocity", 0, 1000, 0, 1000, true},
		{"negative velocity", 0, 1000, -100, 1000, true},
		{"zero acceleration", 0, 1000, 500, 0, true},
		{"negative acceleration", 0, 1000, 500, -1000, true},
		{"same start and target", 500, 500, 100, 200, false},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			profile, err := NewTrapezoidalProfile(tt.startPos, tt.targetPos, tt.maxVel, tt.accel)
			if (err != nil) != tt.wantErr {
				t.Errorf("error = %v, wantErr %v", err, tt.wantErr)
				return
			}
			if !tt.wantErr && profile == nil {
				t.Error("returned nil profile without error")
			}
		})
	}
}

func TestTrapezoidalProfileCalculate(t *testing.T) {
	t.Run("trapezoidal", func(t *testing.T) {
		profile, _ := NewTrapezoidalProfile(0, 2000, 500, 1000)
		if math.Abs(profile.TotalTime()-4.5) > 0.001 {
			t.Errorf("TotalTime() = %v, want 4.5", profile.TotalTime())
		}
	})

	t.Run("triangular", func(t *testing.T) {
		profile, _ := NewTrapezoidalProfile(0, 100, 1000, 1000)
		if profile.CruiseTime() != 0 {
			t.Errorf("CruiseTime = %v, want 0", profile.CruiseTime())
		}
		if profile.CruiseVelocity() >= profile.MaxVelocity {
			t.Error("CruiseVelocity should be < MaxVelocity")
		}
	})

	t.Run("zero distance", func(t *testing.T) {
		profile, _ := NewTrapezoidalProfile(500, 500, 100, 200)
		if profile.TotalTime() != 0 {
			t.Errorf("TotalTime() = %v, want 0", profile.TotalTime())
		}
	})
}

func TestTrapezoidalProfileSample(t *testing.T) {
	profile, _ := NewTrapezoidalProfile(0, 1000, 500, 1000)

	t.Run("start", func(t *testing.T) {
		p := profile.Sample(0)
		if p.Position != 0 || p.Velocity != 0 {
			t.Errorf("at t=0: pos=%v vel=%v", p.Position, p.Velocity)
		}
	})

	t.Run("end", func(t *testing.T) {
		p := profile.Sample(profile.TotalTime())
		if math.Abs(p.Position-1000) > 0.001 || p.Velocity != 0 {
			t.Errorf("at end: pos=%v vel=%v", p.Position, p.Velocity)
		}
	})

	t.Run("beyond end", func(t *testing.T) {
		p := profile.Sample(profile.TotalTime() + 10)
		if math.Abs(p.Position-1000) > 0.001 {
			t.Errorf("beyond end: pos=%v", p.Position)
		}
	})

	t.Run("negative time", func(t *testing.T) {
		p := profile.Sample(-1)
		if p.Position != 0 {
			t.Errorf("at t<0: pos=%v", p.Position)
		}
	})
}

func TestTrapezoidalProfileNegativeDirection(t *testing.T) {
	profile, _ := NewTrapezoidalProfile(1000, 0, 500, 1000)

	p := profile.Sample(profile.TotalTime())
	if math.Abs(p.Position) > 0.001 {
		t.Errorf("end pos=%v, want 0", p.Position)
	}

	mid := profile.Sample(profile.TotalTime() / 2)
	if mid.Velocity >= 0 {
		t.Errorf("velocity should be negative, got %v", mid.Velocity)
	}
}

func TestTrapezoidalProfileGenerate(t *testing.T) {
	profile, _ := NewTrapezoidalProfile(0, 1000, 500, 1000)
	points := profile.Generate(100)

	expected := int(math.Ceil(profile.TotalTime()*100)) + 1
	if len(points) != expected {
		t.Errorf("got %d points, want %d", len(points), expected)
	}

	if points[0].Position != 0 {
		t.Errorf("first pos=%v, want 0", points[0].Position)
	}
	last := points[len(points)-1]
	if math.Abs(last.Position-1000) > 0.001 {
		t.Errorf("last pos=%v, want 1000", last.Position)
	}
}

func TestTrajectoryPointContinuity(t *testing.T) {
	profile, _ := NewTrapezoidalProfile(0, 2000, 500, 1000)
	points := profile.Generate(1000)

	maxPosJump := 0.0
	maxVelJump := 0.0
	for i := 1; i < len(points); i++ {
		pj := math.Abs(points[i].Position - points[i-1].Position)
		vj := math.Abs(points[i].Velocity - points[i-1].Velocity)
		if pj > maxPosJump {
			maxPosJump = pj
		}
		if vj > maxVelJump {
			maxVelJump = vj
		}
	}

	if maxPosJump > 1.0 {
		t.Errorf("position discontinuity: max jump = %v", maxPosJump)
	}
	if maxVelJump > 2.0 {
		t.Errorf("velocity discontinuity: max jump = %v", maxVelJump)
	}
}

func BenchmarkSample(b *testing.B) {
	profile, _ := NewTrapezoidalProfile(0, 4096, 1000, 5000)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		t := float64(i%100) / 100.0 * profile.TotalTime()
		profile.Sample(t)
	}
}

func BenchmarkGenerate(b *testing.B) {
	profile, _ := NewTrapezoidalProfile(0, 4096, 1000, 5000)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		profile.Generate(100)
	}
}
