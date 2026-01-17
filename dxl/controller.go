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
}

// MotorModel defines the Control Table addresses for a specific motor type
type MotorModel struct {
	AddrTorqueEnable    uint16
	AddrGoalPosition    uint16
	AddrPresentPosition uint16
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
		AddrPresentPosition: 132,
	}
	// Pro-Series (H54, H42, etc.)
	ModelProSeries = MotorModel{
		AddrTorqueEnable:    562, // Example, verify for specific PRO model
		AddrGoalPosition:    596,
		AddrPresentPosition: 611,
	}
	// PRO+ Series usually similar to X-Series layout or specific
)

func NewController(devicePort string, baudRate int, loopHz int, model MotorModel) *Controller {
	return &Controller{
		devicePort:   devicePort,
		baudRate:     baudRate,
		CommandChan:  make(chan []Command, 1),
		FeedbackChan: make(chan []Feedback, 100),
		StopChan:     make(chan struct{}),
		LoopPeriod:   time.Second / time.Duration(loopHz),
		Model:        model,
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
					// Use Driver Write
					err := c.driver.Write4Byte(cmd.ID, c.Model.AddrGoalPosition, cmd.Value)
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
