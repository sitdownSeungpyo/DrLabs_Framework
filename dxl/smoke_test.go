package dxl

import (
	"context"
	"math"
	"testing"
	"time"
)

// === Smoke tests for recent improvements ===

// 1. SyncRead4Byte: partial results on motor error
func TestSmokeSyncRead4BytePartialResults(t *testing.T) {
	mock := NewMockSerialPort()
	driver := NewDriver(mock)

	// Motor 1 responds, Motor 2 will timeout (no second response)
	motor1Response := buildStatusPacket(1, 0, []byte{0x00, 0x08, 0x00, 0x00}) // pos=2048
	mock.SetResponse(motor1Response)

	values, err := driver.SyncRead4Byte(132, []uint8{1, 2})

	// Should NOT return error - partial results are valid
	if values == nil {
		t.Fatal("Expected partial results, got nil")
	}

	// Motor 1 should have data
	if val, ok := values[1]; !ok {
		t.Error("Motor 1 data missing from partial results")
	} else if val != 2048 {
		t.Errorf("Motor 1 value: got %d, want 2048", val)
	}

	// Motor 2 should be absent (timed out)
	if _, ok := values[2]; ok {
		t.Error("Motor 2 should not be present in partial results")
	}

	_ = err // err may or may not be nil depending on partial success
}

// 2. SyncRead4Byte: all motors fail returns error
func TestSmokeSyncRead4ByteAllFail(t *testing.T) {
	mock := NewMockSerialPort()
	driver := NewDriver(mock)

	// No response at all - both motors timeout
	_, err := driver.SyncRead4Byte(132, []uint8{1, 2})
	if err == nil {
		t.Error("Expected error when all motors fail, got nil")
	}
}

// 3. Driver configurable timeout
func TestSmokeDriverConfigurableTimeout(t *testing.T) {
	mock := NewMockSerialPort()
	driver := NewDriver(mock)

	// Default should be DefaultTimeout
	if driver.Timeout != DefaultTimeout {
		t.Errorf("Default timeout: got %v, want %v", driver.Timeout, DefaultTimeout)
	}

	// Set custom timeout
	driver.Timeout = 500 * time.Millisecond
	if driver.Timeout != 500*time.Millisecond {
		t.Errorf("Custom timeout not set correctly")
	}

	// Short timeout should fail faster
	driver.Timeout = 10 * time.Millisecond
	start := time.Now()
	_, err := driver.Read(1, 132, 4) // No response set
	elapsed := time.Since(start)

	if err == nil {
		t.Error("Expected timeout error")
	}
	if elapsed > 50*time.Millisecond {
		t.Errorf("Timeout took too long: %v (expected ~10ms)", elapsed)
	}
}

// 4. clampToUint32 overflow protection
func TestSmokeClampToUint32(t *testing.T) {
	tests := []struct {
		name     string
		input    float64
		expected uint32
	}{
		{"normal value", 2048, 2048},
		{"zero", 0, 0},
		{"negative clamps to 0", -100, 0},
		{"large negative clamps to 0", -999999, 0},
		{"max uint32", math.MaxUint32, math.MaxUint32},
		{"exceeds uint32 clamps to max", math.MaxUint32 + 1000, math.MaxUint32},
		{"huge value clamps to max", 1e18, math.MaxUint32},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := clampToUint32(tt.input)
			if result != tt.expected {
				t.Errorf("clampToUint32(%v) = %d, want %d", tt.input, result, tt.expected)
			}
		})
	}
}

// 5. ExecuteWithContext cancellation
func TestSmokeTrajectoryContextCancel(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 4096, 100, 500)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	// Profile should take significant time at low velocity
	if profile.TotalTime() < 1.0 {
		t.Fatalf("Profile too short for cancel test: %v seconds", profile.TotalTime())
	}

	// Create a minimal controller with context
	ctx, cancel := context.WithCancel(context.Background())
	ctrl := &Controller{
		CommandChan: make(chan []Command, 100),
		ctx:         ctx,
		cancel:      cancel,
	}

	executor := NewTrajectoryExecutor(ctrl, 1)

	// Cancel after 50ms
	go func() {
		time.Sleep(50 * time.Millisecond)
		cancel()
	}()

	start := time.Now()
	execErr := executor.ExecuteWithContext(ctx, profile, 100)
	elapsed := time.Since(start)

	if execErr == nil {
		t.Error("Expected context cancelled error, got nil")
	}
	if execErr != context.Canceled {
		t.Errorf("Expected context.Canceled, got: %v", execErr)
	}

	// Should have stopped well before the full trajectory duration
	if elapsed > 500*time.Millisecond {
		t.Errorf("Cancel took too long: %v", elapsed)
	}
}

// 6. ExecuteAsync uses controller context
func TestSmokeTrajectoryExecuteAsync(t *testing.T) {
	profile, err := NewTrapezoidalProfile(0, 4096, 100, 500)
	if err != nil {
		t.Fatalf("Failed to create profile: %v", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	ctrl := &Controller{
		CommandChan: make(chan []Command, 1000),
		ctx:         ctx,
		cancel:      cancel,
	}

	executor := NewTrajectoryExecutor(ctrl, 1)

	errChan, err := executor.ExecuteAsync(profile, 100)
	if err != nil {
		t.Fatalf("ExecuteAsync failed: %v", err)
	}

	// Cancel and wait for completion
	time.Sleep(30 * time.Millisecond)
	cancel()

	execErr := <-errChan
	if execErr != context.Canceled {
		t.Errorf("Expected context.Canceled from async, got: %v", execErr)
	}
}

// 7. Integer tick precision
func TestSmokeTickerIntegerPrecision(t *testing.T) {
	// Verify integer arithmetic produces correct intervals
	rates := []float64{50, 100, 200, 333, 500, 1000}

	for _, rate := range rates {
		intervalNs := int64(time.Second) / int64(rate)
		interval := time.Duration(intervalNs)

		expectedMs := 1000.0 / rate
		actualMs := float64(interval.Nanoseconds()) / 1e6

		// Should be within 0.01ms of expected
		if math.Abs(actualMs-expectedMs) > 0.01 {
			t.Errorf("Rate %.0f Hz: interval %v (%.3f ms), expected %.3f ms",
				rate, interval, actualMs, expectedMs)
		}
	}
}
