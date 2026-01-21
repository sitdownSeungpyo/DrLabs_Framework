package dxl

import (
	"context"
	"fmt"
	"runtime"
	"sync"
	"time"
)

// Controller manages the Dynamixel communication loop
type Controller struct {
	driver     *Driver
	devicePort string
	baudRate   int

	// Channels for communication with the control loop
	CommandChan  chan []Command
	FeedbackChan chan []Feedback

	// Context for graceful shutdown
	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup

	// Configuration
	Model    MotorModel
	MotorIDs []uint8 // List of motor IDs to control

	// Internal State
	mu               sync.RWMutex // Protects shared state
	activeGoalAddr   uint16
	useSyncReadWrite bool // Enable sync read/write for better performance
}

// MotorModel defines the Control Table addresses for a specific motor type
type MotorModel struct {
	AddrTorqueEnable    uint16
	AddrGoalPosition    uint16
	AddrGoalVelocity    uint16
	AddrGoalPWM         uint16
	AddrPresentPosition uint16
	AddrOperatingMode   uint16
}

// Command represents a write command to a motor
type Command struct {
	ID    uint8
	Value uint32
}

// Feedback represents a read value from a motor
type Feedback struct {
	ID    uint8
	Value uint32
	Error error
}

// Common Motor Models (Protocol 2.0 examples)
var (
	// X-Series (XM430, XC430, etc.) & MX-Series (Protocol 2.0 firmware)
	ModelXSeries = MotorModel{
		AddrTorqueEnable:    64,
		AddrGoalPosition:    116,
		AddrGoalVelocity:    104,
		AddrGoalPWM:         100,
		AddrPresentPosition: 132,
		AddrOperatingMode:   11,
	}
	// Pro-Series (H54, H42, etc.)
	ModelProSeries = MotorModel{
		AddrTorqueEnable:    562, // Example, verify for specific PRO model
		AddrGoalPosition:    596,
		AddrGoalVelocity:    600, // Check Manual
		AddrGoalPWM:         584, // Check Manual
		AddrPresentPosition: 611,
		AddrOperatingMode:   11, // PRO Series often shares 11 too, need check
	}
	// PRO+ Series usually similar to X-Series layout or specific
)

const (
	OpModeCurrent          = 0
	OpModeVelocity         = 1
	OpModePosition         = 3
	OpModeExtendedPosition = 4
	OpModeCurrentBasedPos  = 5
	OpModePWM              = 16
)

func NewController(devicePort string, baudRate int, model MotorModel) *Controller {
	ctx, cancel := context.WithCancel(context.Background())
	return &Controller{
		devicePort:       devicePort,
		baudRate:         baudRate,
		CommandChan:      make(chan []Command, 1),
		FeedbackChan:     make(chan []Feedback, 100),
		ctx:              ctx,
		cancel:           cancel,
		Model:            model,
		MotorIDs:         []uint8{1}, // Default single motor
		activeGoalAddr:   model.AddrGoalPosition, // Default Address
		useSyncReadWrite: false, // Default to individual commands for single motor
	}
}

// SetMotorIDs configures which motors to control
// Automatically enables sync read/write if multiple motors
// Thread-safe: can be called while control loop is running
func (c *Controller) SetMotorIDs(ids []uint8) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.MotorIDs = ids
	c.useSyncReadWrite = len(ids) > 1
}

// getMotorIDs returns a copy of motor IDs (thread-safe)
func (c *Controller) getMotorIDs() []uint8 {
	c.mu.RLock()
	defer c.mu.RUnlock()
	ids := make([]uint8, len(c.MotorIDs))
	copy(ids, c.MotorIDs)
	return ids
}

// isSyncMode returns whether sync read/write is enabled (thread-safe)
func (c *Controller) isSyncMode() bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.useSyncReadWrite
}

// getActiveGoalAddr returns the active goal address (thread-safe)
func (c *Controller) getActiveGoalAddr() uint16 {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.activeGoalAddr
}

// Start spawns the control loop goroutine
func (c *Controller) Start() error {
	// 1. Open Serial Port (Native Windows)
	sp, err := OpenSerial(c.devicePort, c.baudRate)
	if err != nil {
		return fmt.Errorf("failed to open serial port: %v", err)
	}

	c.driver = NewDriver(sp)

	// 2. Ping Motor 1 Check
	fmt.Println("Pinging Motor ID 1...")
	model, err := c.driver.Ping(1)
	if err != nil {
		sp.Close()
		return fmt.Errorf("ping failed for ID 1: %v. Check Power/ID/Baudrate", err)
	}
	fmt.Printf("Motor ID 1 Found! Model Number: %d\n", model)

	// 3. Enable Torque
	if err := c.enableTorque(1); err != nil {
		sp.Close()
		return fmt.Errorf("failed to enable torque: %v", err)
	}

	// Start the control loop in a separate goroutine
	c.wg.Add(1)
	go c.controlLoop()

	return nil
}

func (c *Controller) enableTorque(id uint8) error {
	// Write 1 to proper address
	fmt.Printf("Enabling Torque for ID %d at address %d...\n", id, c.Model.AddrTorqueEnable)
	err := c.driver.Write(id, c.Model.AddrTorqueEnable, []byte{1})
	if err != nil {
		return err
	}

	// Small delay to let motor process the command
	time.Sleep(50 * time.Millisecond)

	// Verify (optional - can be disabled if causing issues)
	// Read 1 Byte
	data, err := c.driver.Read(id, c.Model.AddrTorqueEnable, 1)
	if err != nil {
		// If read fails, assume write succeeded (some motors don't respond well to rapid read after write)
		fmt.Printf("Warning: Could not verify torque enable (read error: %v), assuming success\n", err)
		return nil
	}
	if len(data) == 0 {
		fmt.Printf("Warning: Empty response when verifying torque enable, assuming success\n")
		return nil
	}
	if data[0] != 1 {
		// Print warning but don't fail - the write command succeeded
		fmt.Printf("Warning: Torque enable readback mismatch (expected 1, got %d), but write succeeded\n", data[0])
	}
	return nil
}

func (c *Controller) disableTorque(id uint8) error {
	fmt.Printf("Disabling Torque for ID %d...\n", id)
	return c.driver.Write(id, c.Model.AddrTorqueEnable, []byte{0})
}

// SetOperatingMode changes the control mode (Torque Disable -> Set Mode -> Torque Enable)
// Common Modes: 1 (Velocity), 3 (Position), 16 (PWM)
func (c *Controller) SetOperatingMode(id uint8, mode uint8) error {
	// 1. Disable Torque
	if err := c.disableTorque(id); err != nil {
		return fmt.Errorf("failed to disable torque: %v", err)
	}

	// 2. Set Mode
	fmt.Printf("Setting Operating Mode to %d for ID %d...\n", mode, id)
	if err := c.driver.Write(id, c.Model.AddrOperatingMode, []byte{mode}); err != nil {
		return fmt.Errorf("failed to set operating mode: %v", err)
	}

	// EEPROM Write Delay: Operating Mode change requires reboot/flash time.
	// This happens ONLY ONCE at startup, so it does not affect control loop performance.
	// Increased delay to ensure motor is fully ready after mode change
	time.Sleep(1000 * time.Millisecond)

	// Update Active Goal Address (thread-safe)
	c.mu.Lock()
	switch mode {
	case OpModeVelocity:
		c.activeGoalAddr = c.Model.AddrGoalVelocity
	case OpModePWM:
		c.activeGoalAddr = c.Model.AddrGoalPWM
	case OpModePosition, OpModeExtendedPosition, OpModeCurrentBasedPos:
		c.activeGoalAddr = c.Model.AddrGoalPosition
	case OpModeCurrent:
		// c.activeGoalAddr = c.Model.AddrGoalCurrent // Need to add if supported
		fmt.Printf("Warning: Current mode not fully supported, using position address\n")
		c.activeGoalAddr = c.Model.AddrGoalPosition
	}
	c.mu.Unlock()

	// 3. Re-Enable Torque
	if err := c.enableTorque(id); err != nil {
		return fmt.Errorf("failed to enable torque: %v", err)
	}

	return nil
}

// Stop signals the control loop to exit and waits for it to finish
func (c *Controller) Stop() {
	c.cancel()
	c.wg.Wait()
}

func (c *Controller) controlLoop() {
	defer c.wg.Done()

	// 1. Lock OS Thread to reduce scheduler jitter
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	defer c.driver.port.Close()

	for {
		select {
		case <-c.ctx.Done():
			return
		// 1. Process Commands (Prioritized)
		case cmds := <-c.CommandChan:
			goalAddr := c.getActiveGoalAddr()
			if c.isSyncMode() {
				// Use Sync Write for multiple motors (more efficient)
				values := make(map[uint8]uint32)
				for _, cmd := range cmds {
					values[cmd.ID] = cmd.Value
				}
				if err := c.driver.SyncWrite4Byte(goalAddr, values); err != nil {
					fmt.Printf("SyncWrite error: %v\n", err)
				}
			} else {
				// Individual writes for single motor or legacy mode
				for _, cmd := range cmds {
					if err := c.driver.Write4Byte(cmd.ID, goalAddr, cmd.Value); err != nil {
						fmt.Printf("Write error for motor %d: %v\n", cmd.ID, err)
					}
				}
			}
		default:
			// No commands, continue to reads
		}

		// 2. Read Feedback
		var feedbacks []Feedback
		motorIDs := c.getMotorIDs()

		if c.isSyncMode() {
			// Use Sync Read for multiple motors (more efficient)
			values, err := c.driver.SyncRead4Byte(c.Model.AddrPresentPosition, motorIDs)
			if err != nil {
				// Error reading all motors, create error feedback for each
				for _, id := range motorIDs {
					feedbacks = append(feedbacks, Feedback{ID: id, Value: 0, Error: err})
				}
			} else {
				// Success, create feedback for each motor
				for _, id := range motorIDs {
					if val, ok := values[id]; ok {
						feedbacks = append(feedbacks, Feedback{ID: id, Value: val, Error: nil})
					} else {
						feedbacks = append(feedbacks, Feedback{ID: id, Value: 0, Error: fmt.Errorf("no data for motor %d", id)})
					}
				}
			}
		} else {
			// Individual reads for single motor
			for _, id := range motorIDs {
				val, err := c.driver.Read4Byte(id, c.Model.AddrPresentPosition)
				feedbacks = append(feedbacks, Feedback{ID: id, Value: val, Error: err})
			}
		}

		// Send feedback (non-blocking)
		select {
		case c.FeedbackChan <- feedbacks:
		default:
			// Channel full, drop oldest feedback
		}
	}
}
