package dynamixel

import (
	"context"
	"fmt"
	"log"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"go_dxl/transport/serial"
)

// Channel buffer size defaults
const (
	DefaultCommandChanSize  = 1
	DefaultFeedbackChanSize = 100
)

// Timing defaults
const (
	DefaultEEPROMWriteDelay   = 1000 * time.Millisecond
	DefaultTorqueDisableDelay = 150 * time.Millisecond
)

// PortOpener is a function type that opens a serial port.
type PortOpener func(devicePort string, baudRate int) (serial.Port, error)

// Controller manages the Dynamixel communication loop.
type Controller struct {
	driver     *Driver
	devicePort string
	baudRate   int

	CommandChan  chan []Command
	FeedbackChan chan []Feedback

	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup

	Model    MotorModel
	MotorIDs []uint8

	EEPROMWriteDelay   time.Duration
	TorqueDisableDelay time.Duration

	OpenPort PortOpener

	mu               sync.RWMutex
	activeGoalAddr   uint16
	useSyncReadWrite bool

	droppedFeedbacks atomic.Int64
}

// Command represents a write command to a motor.
type Command struct {
	ID    uint8
	Value uint32
}

// Feedback represents a read value from a motor.
type Feedback struct {
	ID    uint8
	Value uint32
	Error error
}

// NewController creates a new Dynamixel controller.
func NewController(devicePort string, baudRate int, model MotorModel) *Controller {
	ctx, cancel := context.WithCancel(context.Background())
	return &Controller{
		devicePort:         devicePort,
		baudRate:           baudRate,
		CommandChan:        make(chan []Command, DefaultCommandChanSize),
		FeedbackChan:       make(chan []Feedback, DefaultFeedbackChanSize),
		ctx:                ctx,
		cancel:             cancel,
		Model:              model,
		MotorIDs:           []uint8{1},
		activeGoalAddr:     model.AddrGoalPosition,
		useSyncReadWrite:   false,
		EEPROMWriteDelay:   DefaultEEPROMWriteDelay,
		TorqueDisableDelay: DefaultTorqueDisableDelay,
		OpenPort:           defaultOpenPort,
	}
}

func defaultOpenPort(devicePort string, baudRate int) (serial.Port, error) {
	return serial.Open(devicePort, baudRate)
}

// SetMotorIDs configures which motors to control.
// Automatically enables sync read/write for multiple motors.
func (c *Controller) SetMotorIDs(ids []uint8) error {
	for _, id := range ids {
		if id > MaxValidMotorID {
			return fmt.Errorf("invalid motor ID %d: must be 0-%d", id, MaxValidMotorID)
		}
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	c.MotorIDs = ids
	c.useSyncReadWrite = len(ids) > 1
	return nil
}

func (c *Controller) getMotorIDs() []uint8 {
	c.mu.RLock()
	defer c.mu.RUnlock()
	ids := make([]uint8, len(c.MotorIDs))
	copy(ids, c.MotorIDs)
	return ids
}

func (c *Controller) isSyncMode() bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.useSyncReadWrite
}

func (c *Controller) getActiveGoalAddr() uint16 {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.activeGoalAddr
}

// Start begins the control loop.
func (c *Controller) Start() error {
	if c.devicePort == "" {
		return fmt.Errorf("device port must not be empty")
	}
	motorIDs := c.getMotorIDs()
	if len(motorIDs) == 0 {
		return fmt.Errorf("no motor IDs configured")
	}
	if c.Model.AddrTorqueEnable == 0 || c.Model.AddrPresentPosition == 0 || c.Model.AddrOperatingMode == 0 {
		return fmt.Errorf("motor model has uninitialized addresses")
	}

	opener := c.OpenPort
	if opener == nil {
		opener = defaultOpenPort
	}
	sp, err := opener(c.devicePort, c.baudRate)
	if err != nil {
		return fmt.Errorf("failed to open serial port: %v", err)
	}

	c.driver = NewDriver(sp)

	for _, id := range motorIDs {
		fmt.Printf("Pinging Motor ID %d...\n", id)
		model, err := c.driver.Ping(id)
		if err != nil {
			sp.Close()
			return fmt.Errorf("ping failed for ID %d: %v", id, err)
		}
		fmt.Printf("Motor ID %d Found! Model Number: %d\n", id, model)

		if err := c.enableTorque(id); err != nil {
			sp.Close()
			return fmt.Errorf("failed to enable torque for ID %d: %v", id, err)
		}
	}

	c.wg.Add(1)
	go c.controlLoop()
	return nil
}

func (c *Controller) enableTorque(id uint8) error {
	if c.driver == nil {
		return fmt.Errorf("controller not started")
	}
	fmt.Printf("Enabling Torque for ID %d at address %d...\n", id, c.Model.AddrTorqueEnable)
	if err := c.driver.Write(id, c.Model.AddrTorqueEnable, []byte{1}); err != nil {
		return err
	}

	time.Sleep(50 * time.Millisecond)

	data, err := c.driver.Read(id, c.Model.AddrTorqueEnable, 1)
	if err != nil {
		fmt.Printf("Warning: Could not verify torque enable (read error: %v), assuming success\n", err)
		return nil
	}
	if len(data) == 0 {
		fmt.Printf("Warning: Empty response when verifying torque enable, assuming success\n")
		return nil
	}
	if data[0] != 1 {
		fmt.Printf("Warning: Torque enable readback mismatch (expected 1, got %d), but write succeeded\n", data[0])
	}
	return nil
}

func (c *Controller) disableTorque(id uint8) error {
	if c.driver == nil {
		return fmt.Errorf("controller not started")
	}
	fmt.Printf("Disabling Torque for ID %d...\n", id)
	return c.driver.Write(id, c.Model.AddrTorqueEnable, []byte{0})
}

// SetOperatingMode changes the control mode.
func (c *Controller) SetOperatingMode(id uint8, mode uint8) error {
	originalTimeout := c.driver.Timeout
	c.driver.Timeout = 500 * time.Millisecond
	defer func() { c.driver.Timeout = originalTimeout }()

	c.driver.Flush()
	time.Sleep(100 * time.Millisecond)

	currentMode, err := c.driver.Read(id, c.Model.AddrOperatingMode, 1)
	if err != nil {
		fmt.Printf("Warning: could not read current mode for motor %d: %v\n", id, err)
	} else if len(currentMode) > 0 {
		fmt.Printf("Motor %d current mode: %d (target: %d)\n", id, currentMode[0], mode)
		if currentMode[0] == mode {
			fmt.Printf("Motor %d already in mode %d, skipping EEPROM write...\n", id, mode)
			c.mu.Lock()
			c.activeGoalAddr = c.goalAddrForMode(mode)
			c.mu.Unlock()
			return c.enableTorque(id)
		}
	}

	c.driver.Flush()
	time.Sleep(10 * time.Millisecond)

	if err := c.disableTorque(id); err != nil {
		return fmt.Errorf("failed to disable torque: %v", err)
	}

	time.Sleep(c.TorqueDisableDelay)

	c.driver.Flush()
	data, err := c.driver.Read(id, c.Model.AddrTorqueEnable, 1)
	if err != nil {
		fmt.Printf("Warning: could not verify torque disable: %v\n", err)
		time.Sleep(100 * time.Millisecond)
	} else if len(data) > 0 && data[0] != 0 {
		fmt.Printf("Torque still enabled, retrying...\n")
		if err := c.disableTorque(id); err != nil {
			return fmt.Errorf("failed to disable torque (retry): %v", err)
		}
		time.Sleep(100 * time.Millisecond)
	}

	fmt.Printf("Setting Operating Mode to %d for ID %d...\n", mode, id)
	if err := c.driver.Write(id, c.Model.AddrOperatingMode, []byte{mode}); err != nil {
		return fmt.Errorf("failed to set operating mode: %v", err)
	}

	time.Sleep(c.EEPROMWriteDelay)

	data, err = c.driver.Read(id, c.Model.AddrOperatingMode, 1)
	if err != nil {
		fmt.Printf("Warning: could not verify operating mode (read error: %v)\n", err)
	} else if len(data) > 0 && data[0] != mode {
		return fmt.Errorf("operating mode verification failed: wrote %d, read back %d", mode, data[0])
	}

	c.mu.Lock()
	c.activeGoalAddr = c.goalAddrForMode(mode)
	c.mu.Unlock()

	if err := c.enableTorque(id); err != nil {
		return fmt.Errorf("failed to enable torque: %v", err)
	}
	return nil
}

func (c *Controller) goalAddrForMode(mode uint8) uint16 {
	switch mode {
	case OpModeVelocity:
		return c.Model.AddrGoalVelocity
	case OpModePWM:
		return c.Model.AddrGoalPWM
	case OpModeCurrent:
		if c.Model.AddrGoalCurrent != 0 {
			return c.Model.AddrGoalCurrent
		}
		log.Printf("Warning: AddrGoalCurrent not set in model, falling back to GoalPosition")
		return c.Model.AddrGoalPosition
	case OpModePosition, OpModeExtendedPosition, OpModeCurrentBasedPos:
		return c.Model.AddrGoalPosition
	default:
		return c.Model.AddrGoalPosition
	}
}

// Stop signals the control loop to exit and waits.
func (c *Controller) Stop() {
	c.cancel()
	c.wg.Wait()
}

func (c *Controller) controlLoop() {
	defer c.wg.Done()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	defer c.driver.port.Close()

	for {
		select {
		case <-c.ctx.Done():
			return
		case cmds := <-c.CommandChan:
			goalAddr := c.getActiveGoalAddr()
			if c.isSyncMode() {
				values := make(map[uint8]uint32)
				for _, cmd := range cmds {
					values[cmd.ID] = cmd.Value
				}
				if err := c.driver.SyncWrite4Byte(goalAddr, values); err != nil {
					fmt.Printf("SyncWrite error: %v\n", err)
				}
			} else {
				for _, cmd := range cmds {
					if err := c.driver.Write4Byte(cmd.ID, goalAddr, cmd.Value); err != nil {
						fmt.Printf("Write error for motor %d: %v\n", cmd.ID, err)
					}
				}
			}
		default:
		}

		var feedbacks []Feedback
		motorIDs := c.getMotorIDs()

		if c.isSyncMode() {
			values, syncErr := c.driver.SyncRead4Byte(c.Model.AddrPresentPosition, motorIDs)
			if syncErr != nil {
				log.Printf("SyncRead error: %v", syncErr)
			}
			for _, id := range motorIDs {
				if val, ok := values[id]; ok {
					feedbacks = append(feedbacks, Feedback{ID: id, Value: val})
				} else {
					feedbacks = append(feedbacks, Feedback{ID: id, Error: fmt.Errorf("no response from motor %d", id)})
				}
			}
		} else {
			for _, id := range motorIDs {
				val, err := c.driver.Read4Byte(id, c.Model.AddrPresentPosition)
				feedbacks = append(feedbacks, Feedback{ID: id, Value: val, Error: err})
			}
		}

		select {
		case c.FeedbackChan <- feedbacks:
		default:
			dropped := c.droppedFeedbacks.Add(1)
			if dropped == 1 || dropped%1000 == 0 {
				log.Printf("Warning: feedback channel full, dropped %d total feedbacks", dropped)
			}
		}
	}
}

// DroppedFeedbacks returns the count of dropped feedback messages.
func (c *Controller) DroppedFeedbacks() int64 {
	return c.droppedFeedbacks.Load()
}

// Driver returns the underlying Driver (for advanced use).
func (c *Controller) GetDriver() *Driver {
	return c.driver
}
