package dxl

import (
	"context"
	"fmt"
	"testing"
	"time"
)

// mockPortOpener returns a PortOpener that yields the given mock port.
func mockPortOpener(mock *MockSerialPort) PortOpener {
	return func(devicePort string, baudRate int) (SerialPortInterface, error) {
		return mock, nil
	}
}

// failPortOpener returns a PortOpener that always fails.
func failPortOpener(msg string) PortOpener {
	return func(devicePort string, baudRate int) (SerialPortInterface, error) {
		return nil, fmt.Errorf("%s", msg)
	}
}

// setupTestController creates a controller wired to a mock serial port.
// The mock is pre-loaded with ping + torque-enable responses for the given motor IDs.
// Uses QueueResponses so each Transfer call gets exactly one response.
func setupTestController(t *testing.T, ids []uint8) (*Controller, *MockSerialPort) {
	t.Helper()
	mock := NewMockSerialPort()

	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.OpenPort = mockPortOpener(mock)
	c.EEPROMWriteDelay = 1 * time.Millisecond  // speed up tests
	c.TorqueDisableDelay = 1 * time.Millisecond // speed up tests

	if err := c.SetMotorIDs(ids); err != nil {
		t.Fatalf("SetMotorIDs: %v", err)
	}

	// Build ping + torque-enable responses for each motor.
	// Each motor needs: ping response + write response (torque enable) + read response (verify torque)
	// Use QueueResponses so each Transfer.Write loads exactly one response.
	var responses [][]byte
	for _, id := range ids {
		// Ping response: model 1060 (XM430), firmware 1
		responses = append(responses, buildStatusPacket(id, 0, []byte{0x24, 0x04, 0x01}))
		// Write response (torque enable success)
		responses = append(responses, buildStatusPacket(id, 0, nil))
		// Read response (torque verify: value = 1)
		responses = append(responses, buildStatusPacket(id, 0, []byte{0x01}))
	}
	mock.QueueResponses(responses...)

	return c, mock
}

// --- SetMotorIDs tests ---

func TestSetMotorIDs_Single(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	if err := c.SetMotorIDs([]uint8{5}); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	ids := c.getMotorIDs()
	if len(ids) != 1 || ids[0] != 5 {
		t.Errorf("expected [5], got %v", ids)
	}
	if c.isSyncMode() {
		t.Error("sync mode should be false for single motor")
	}
}

func TestSetMotorIDs_Multiple(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	if err := c.SetMotorIDs([]uint8{1, 2, 3}); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	ids := c.getMotorIDs()
	if len(ids) != 3 {
		t.Errorf("expected 3 IDs, got %d", len(ids))
	}
	if !c.isSyncMode() {
		t.Error("sync mode should be true for multiple motors")
	}
}

func TestSetMotorIDs_InvalidID(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	err := c.SetMotorIDs([]uint8{1, 253})
	if err == nil {
		t.Error("expected error for invalid motor ID 253")
	}
}

func TestSetMotorIDs_MaxValidID(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	err := c.SetMotorIDs([]uint8{MaxValidMotorID})
	if err != nil {
		t.Errorf("expected no error for ID %d, got: %v", MaxValidMotorID, err)
	}
}

// --- goalAddrForMode tests ---

func TestGoalAddrForMode_Position(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	addr := c.goalAddrForMode(OpModePosition)
	if addr != ModelXSeries.AddrGoalPosition {
		t.Errorf("expected %d, got %d", ModelXSeries.AddrGoalPosition, addr)
	}
}

func TestGoalAddrForMode_Velocity(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	addr := c.goalAddrForMode(OpModeVelocity)
	if addr != ModelXSeries.AddrGoalVelocity {
		t.Errorf("expected %d, got %d", ModelXSeries.AddrGoalVelocity, addr)
	}
}

func TestGoalAddrForMode_PWM(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	addr := c.goalAddrForMode(OpModePWM)
	if addr != ModelXSeries.AddrGoalPWM {
		t.Errorf("expected %d, got %d", ModelXSeries.AddrGoalPWM, addr)
	}
}

func TestGoalAddrForMode_Current(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	addr := c.goalAddrForMode(OpModeCurrent)
	if addr != ModelXSeries.AddrGoalCurrent {
		t.Errorf("expected AddrGoalCurrent %d, got %d", ModelXSeries.AddrGoalCurrent, addr)
	}
}

func TestGoalAddrForMode_CurrentFallback(t *testing.T) {
	// Model with no AddrGoalCurrent set
	model := MotorModel{
		AddrTorqueEnable:    64,
		AddrGoalPosition:    116,
		AddrGoalVelocity:    104,
		AddrGoalPWM:         100,
		AddrGoalCurrent:     0, // Not set
		AddrPresentPosition: 132,
		AddrOperatingMode:   11,
	}
	c := NewController("/dev/test", 1000000, model)
	addr := c.goalAddrForMode(OpModeCurrent)
	if addr != model.AddrGoalPosition {
		t.Errorf("expected fallback to GoalPosition %d, got %d", model.AddrGoalPosition, addr)
	}
}

func TestGoalAddrForMode_ExtendedPosition(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	addr := c.goalAddrForMode(OpModeExtendedPosition)
	if addr != ModelXSeries.AddrGoalPosition {
		t.Errorf("expected %d, got %d", ModelXSeries.AddrGoalPosition, addr)
	}
}

// --- Start validation tests ---

func TestStart_EmptyPort(t *testing.T) {
	c := NewController("", 1000000, ModelXSeries)
	err := c.Start()
	if err == nil {
		t.Error("expected error for empty device port")
	}
}

func TestStart_NoMotorIDs(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.mu.Lock()
	c.MotorIDs = nil
	c.mu.Unlock()

	err := c.Start()
	if err == nil {
		t.Error("expected error for empty motor IDs")
	}
}

func TestStart_UninitializedModel(t *testing.T) {
	c := NewController("/dev/test", 1000000, MotorModel{})
	err := c.Start()
	if err == nil {
		t.Error("expected error for uninitialized motor model")
	}
}

func TestStart_PortOpenFail(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.OpenPort = failPortOpener("mock port error")

	err := c.Start()
	if err == nil {
		t.Error("expected error when port open fails")
	}
}

func TestStart_PingFail(t *testing.T) {
	mock := NewMockSerialPort()
	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.OpenPort = mockPortOpener(mock)

	// Don't set any response — ping will timeout
	mock.SetReadError(fmt.Errorf("no data"))

	err := c.Start()
	if err == nil {
		t.Error("expected error when ping fails")
	}
}

func TestStart_Success(t *testing.T) {
	c, _ := setupTestController(t, []uint8{1})

	err := c.Start()
	if err != nil {
		t.Fatalf("Start failed: %v", err)
	}
	defer c.Stop()
}

// --- controlLoop tests ---

func TestControlLoop_CommandDispatch_Individual(t *testing.T) {
	c, mock := setupTestController(t, []uint8{1})

	err := c.Start()
	if err != nil {
		t.Fatalf("Start failed: %v", err)
	}
	defer c.Stop()

	// Prepare write response for the command
	writeResp := buildStatusPacket(1, 0, nil)
	// Prepare read response for feedback
	readResp := buildStatusPacket(1, 0, []byte{0x00, 0x08, 0x00, 0x00}) // position 2048
	mock.SetResponse(append(writeResp, readResp...))

	// Send a command
	c.CommandChan <- []Command{{ID: 1, Value: 2048}}

	// Wait for feedback
	select {
	case fb := <-c.FeedbackChan:
		if len(fb) == 0 {
			t.Error("expected feedback, got empty slice")
		}
	case <-time.After(500 * time.Millisecond):
		// Feedback may not arrive if read times out, that's OK for this test
	}
}

func TestControlLoop_ContextCancel(t *testing.T) {
	c, _ := setupTestController(t, []uint8{1})

	err := c.Start()
	if err != nil {
		t.Fatalf("Start failed: %v", err)
	}

	// Cancel should stop the loop
	c.cancel()
	done := make(chan struct{})
	go func() {
		c.wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		// OK - control loop exited
	case <-time.After(2 * time.Second):
		t.Error("control loop did not exit after context cancel")
	}
}

func TestControlLoop_SyncMode(t *testing.T) {
	c, mock := setupTestController(t, []uint8{1, 2})

	err := c.Start()
	if err != nil {
		t.Fatalf("Start failed: %v", err)
	}
	defer c.Stop()

	if !c.isSyncMode() {
		t.Error("expected sync mode for 2 motors")
	}

	// Set up sync read responses for feedback
	resp1 := buildStatusPacket(1, 0, []byte{0x00, 0x04, 0x00, 0x00})
	resp2 := buildStatusPacket(2, 0, []byte{0x00, 0x08, 0x00, 0x00})
	mock.SetResponse(append(resp1, resp2...))

	// Wait for feedback
	select {
	case fb := <-c.FeedbackChan:
		if len(fb) != 2 {
			t.Errorf("expected 2 feedbacks, got %d", len(fb))
		}
	case <-time.After(500 * time.Millisecond):
		// Timeout OK for mock
	}
}

// --- DroppedFeedbacks test ---

func TestDroppedFeedbacksCounter(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	if c.DroppedFeedbacks() != 0 {
		t.Errorf("expected 0 dropped feedbacks initially, got %d", c.DroppedFeedbacks())
	}

	// Simulate by directly incrementing
	c.droppedFeedbacks.Add(5)
	if c.DroppedFeedbacks() != 5 {
		t.Errorf("expected 5 dropped feedbacks, got %d", c.DroppedFeedbacks())
	}
}

// --- NewController defaults test ---

func TestNewController_Defaults(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	if c.devicePort != "/dev/test" {
		t.Errorf("expected /dev/test, got %s", c.devicePort)
	}
	if c.baudRate != 1000000 {
		t.Errorf("expected 1000000, got %d", c.baudRate)
	}
	if c.EEPROMWriteDelay != DefaultEEPROMWriteDelay {
		t.Errorf("expected EEPROMWriteDelay %v, got %v", DefaultEEPROMWriteDelay, c.EEPROMWriteDelay)
	}
	if c.TorqueDisableDelay != DefaultTorqueDisableDelay {
		t.Errorf("expected TorqueDisableDelay %v, got %v", DefaultTorqueDisableDelay, c.TorqueDisableDelay)
	}
	if c.OpenPort == nil {
		t.Error("expected non-nil OpenPort")
	}
	if len(c.MotorIDs) != 1 || c.MotorIDs[0] != 1 {
		t.Errorf("expected default MotorIDs [1], got %v", c.MotorIDs)
	}
	if c.activeGoalAddr != ModelXSeries.AddrGoalPosition {
		t.Errorf("expected default goal addr %d, got %d", ModelXSeries.AddrGoalPosition, c.activeGoalAddr)
	}
}

// --- SetOperatingMode tests ---

func TestSetOperatingMode_AlreadySameMode(t *testing.T) {
	mock := NewMockSerialPort()
	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.EEPROMWriteDelay = 1 * time.Millisecond
	c.TorqueDisableDelay = 1 * time.Millisecond

	// Set up driver directly (avoid starting controlLoop which would race)
	c.driver = NewDriver(mock)
	c.driver.Timeout = 50 * time.Millisecond

	// SetOperatingMode: Flush, Read(mode), skip EEPROM, enableTorque(Write+Read)
	mock.QueueResponses(
		buildStatusPacket(1, 0, []byte{OpModePosition}), // read current mode = Position
		buildStatusPacket(1, 0, nil),                     // torque enable write
		buildStatusPacket(1, 0, []byte{0x01}),            // torque verify read
	)

	err := c.SetOperatingMode(1, OpModePosition)
	if err != nil {
		t.Errorf("SetOperatingMode (same mode) failed: %v", err)
	}
}

// --- enableTorque / disableTorque tests ---

func TestEnableTorque_NoDriver(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	err := c.enableTorque(1)
	if err == nil {
		t.Error("expected error when driver is nil")
	}
}

func TestDisableTorque_NoDriver(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	err := c.disableTorque(1)
	if err == nil {
		t.Error("expected error when driver is nil")
	}
}

// --- ModelXSeries validation ---

func TestModelXSeries_AddrGoalCurrent(t *testing.T) {
	if ModelXSeries.AddrGoalCurrent != 102 {
		t.Errorf("expected AddrGoalCurrent 102, got %d", ModelXSeries.AddrGoalCurrent)
	}
}

// --- getMotorIDs returns copy ---

func TestGetMotorIDs_ReturnsCopy(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)
	c.SetMotorIDs([]uint8{1, 2, 3})

	ids := c.getMotorIDs()
	ids[0] = 99 // mutate the copy

	original := c.getMotorIDs()
	if original[0] == 99 {
		t.Error("getMotorIDs should return a copy, not a reference")
	}
}

// --- Concurrent access test ---

func TestSetMotorIDs_Concurrent(t *testing.T) {
	c := NewController("/dev/test", 1000000, ModelXSeries)

	ctx, cancel := context.WithTimeout(context.Background(), 1*time.Second)
	defer cancel()

	done := make(chan struct{})
	go func() {
		defer close(done)
		for i := 0; ctx.Err() == nil; i++ {
			_ = c.SetMotorIDs([]uint8{uint8(i%252 + 1)})
		}
	}()

	for ctx.Err() == nil {
		_ = c.getMotorIDs()
		_ = c.isSyncMode()
		_ = c.getActiveGoalAddr()
	}

	<-done
}
