// Package motor defines universal interfaces for robot motor control.
// All motor types (Dynamixel, Robstride, ODrive, etc.) implement these interfaces.
package motor

// OperatingMode represents the motor control mode.
type OperatingMode uint8

const (
	ModePosition OperatingMode = iota
	ModeVelocity
	ModeCurrent
	ModePWM
	ModeExtendedPosition
	ModeCurrentBasedPosition
)

// ModelInfo contains motor identification data returned by Ping.
type ModelInfo struct {
	ModelNumber uint16
	Firmware    uint8
}

// Motor is the core interface that all motor drivers implement.
type Motor interface {
	// ID returns the motor's bus address.
	ID() uint8

	// Ping verifies communication and returns model information.
	Ping() (ModelInfo, error)

	// SetTorque enables or disables motor torque output.
	SetTorque(enabled bool) error

	// SetOperatingMode changes the motor's control mode.
	// Typically requires torque to be disabled first.
	SetOperatingMode(mode OperatingMode) error

	// Position returns the current position in raw encoder units.
	Position() (int32, error)

	// SetPosition commands the motor to move to the specified position.
	SetPosition(pos int32) error

	// Velocity returns the current velocity.
	Velocity() (int32, error)

	// SetVelocity commands the motor to spin at the specified velocity.
	SetVelocity(vel int32) error
}

// Goal represents a control command for a single motor.
type Goal struct {
	MotorID uint8
	Value   int32
}

// State represents feedback from a single motor.
type State struct {
	MotorID  uint8
	Position int32
	Velocity int32
	Error    error
}
