package dxl

import (
	"fmt"
	"runtime"
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
	StopChan     chan struct{}

	// Configuration
	LoopPeriod time.Duration
	Model      MotorModel

	// Internal State
	activeGoalAddr uint16
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

func NewController(devicePort string, baudRate int, loopHz int, model MotorModel) *Controller {
	return &Controller{
		devicePort:     devicePort,
		baudRate:       baudRate,
		CommandChan:    make(chan []Command, 1),
		FeedbackChan:   make(chan []Feedback, 100),
		StopChan:       make(chan struct{}),
		LoopPeriod:     time.Second / time.Duration(loopHz),
		Model:          model,
		activeGoalAddr: model.AddrGoalPosition, // Default Address
	}
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
	// Verify
	// Read 1 Byte
	data, err := c.driver.Read(id, c.Model.AddrTorqueEnable, 1)
	if err != nil {
		return err
	}
	if len(data) > 0 && data[0] != 1 {
		return fmt.Errorf("torque enable mismatch. Expected 1, got %d", data[0])
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

	// EEPROM Write Delay: Changing Operating Mode writes to EEPROM.
	// Give the motor some time to process before sending next command.
	time.Sleep(200 * time.Millisecond)

	// Update Active Goal Address
	switch mode {
	case OpModeVelocity:
		c.activeGoalAddr = c.Model.AddrGoalVelocity
	case OpModePWM:
		c.activeGoalAddr = c.Model.AddrGoalPWM
	case OpModePosition, OpModeExtendedPosition, OpModeCurrentBasedPos:
		c.activeGoalAddr = c.Model.AddrGoalPosition
	case OpModeCurrent:
		// c.activeGoalAddr = c.Model.AddrGoalCurrent // Need to add if supported
		// Fallback or warning?
	}

	// 3. Re-Enable Torque
	if err := c.enableTorque(id); err != nil {
		return fmt.Errorf("failed to enable torque: %v", err)
	}

	return nil
}

// Stop signals the control loop to exit
func (c *Controller) Stop() {
	close(c.StopChan)
}

func (c *Controller) controlLoop() {
	// 1. Lock OS Thread to reduce scheduler jitter
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	defer c.driver.port.Close()

	ticker := time.NewTicker(c.LoopPeriod)
	defer ticker.Stop()

	for {
		select {
		case <-c.StopChan:
			return
		case <-ticker.C:
			// 1. Process Commands
			select {
			case cmds := <-c.CommandChan:
				for _, cmd := range cmds {
					// Use Active Goal Address
					// Note: This assumes all motors in loop use same mode, or commands are consistent.
					// Since Controller is simple, we assume single-mode operation for now.
					err := c.driver.Write4Byte(cmd.ID, c.activeGoalAddr, cmd.Value)
					if err != nil {
						// In real app, maybe log or count errors
					}
				}
			default:
			}

			// 2. Read Feedback (Example: ID 1)
			val, err := c.driver.Read4Byte(1, c.Model.AddrPresentPosition)
			f := Feedback{ID: 1, Value: val, Error: err}

			select {
			case c.FeedbackChan <- []Feedback{f}:
			default:
			}
		}
	}
}
