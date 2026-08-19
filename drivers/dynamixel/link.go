package dynamixel

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"strconv"
	"time"

	"drlabs/motor"
	"drlabs/transport/serial"
)

// Link adapts a Dynamixel servo to the framework's motor.Link.
//
// The interesting work is unit translation.  A Dynamixel speaks encoder counts,
// 0.229 RPM steps and milliamp-scale current units; the framework speaks
// revolutions, rev/s and amperes.  Doing that conversion here — rather than
// leaving it to applications — is what lets one control loop, one safety
// monitor and one UI drive a Dynamixel and a moteus without knowing which is
// which.
//
// What a Dynamixel cannot do is also part of the contract.  It has no
// stay-within mode, and without a motor-specific torque constant it cannot
// report torque in newton-metres, so those are reported as unsupported and
// unmeasured rather than approximated.
type Link struct {
	config  motor.Config
	profile Profile

	port   serial.Port
	driver *Driver
	id     uint8

	// ownsPort records whether this Link opened the serial port.  On a daisy
	// chain it did not, and closing it would take every other servo with it.
	ownsPort bool

	// currentMode is the operating mode believed to be in the motor's EEPROM.
	// Changing it requires torque off, an EEPROM write and a settling delay,
	// so it is tracked rather than written every cycle.
	currentMode int

	torqueEnabled bool

	// lastGoal remembers what was last commanded, so unchanged parameters are
	// not rewritten every period.  Profile velocity and acceleration live in
	// EEPROM-adjacent RAM and rewriting them at the control rate wastes most
	// of the serial bandwidth the loop has.
	lastVelocityLimit float64
	lastAccelLimit    float64
	lastWatchdog      float64
}

func init() {
	motor.Register("dynamixel", "ROBOTIS Dynamixel servo over Protocol 2.0 serial", newLink)
}

// newLink builds a Dynamixel Link from a Config.
//
// Recognised options:
//
//	profile          control table family: "x" (default) or "xl330"
//	torque_constant  Nm/A, so that torque can be reported (default unset)
//	counts_per_rev   override the encoder resolution
func newLink(cfg motor.Config) (motor.Link, error) {
	if cfg.ID < 0 || cfg.ID > MaxValidMotorID {
		return nil, fmt.Errorf("dynamixel: servo id %d is out of range (0..%d)", cfg.ID, MaxValidMotorID)
	}
	if cfg.Port == "" {
		return nil, fmt.Errorf("dynamixel: a serial port must be given")
	}

	for key := range cfg.Options {
		switch key {
		case "profile", "torque_constant", "counts_per_rev":
		default:
			return nil, fmt.Errorf("dynamixel: unknown option %q", key)
		}
	}

	name := cfg.Option("profile", "x")
	profile, ok := Profiles[name]
	if !ok {
		return nil, fmt.Errorf("dynamixel: unknown profile %q", name)
	}

	if text := cfg.Option("torque_constant", ""); text != "" {
		value, err := strconv.ParseFloat(text, 64)
		if err != nil || value <= 0 {
			return nil, fmt.Errorf("dynamixel: torque_constant=%q must be a positive number", text)
		}
		profile.TorqueConstant = value
	}
	if text := cfg.Option("counts_per_rev", ""); text != "" {
		value, err := strconv.ParseFloat(text, 64)
		if err != nil || value <= 0 {
			return nil, fmt.Errorf("dynamixel: counts_per_rev=%q must be a positive number", text)
		}
		profile.CountsPerRev = value
	}

	baud := cfg.Baud
	if baud == 0 {
		baud = 1000000
	}
	cfg.Baud = baud

	return &Link{
		config:            cfg,
		profile:           profile,
		id:                uint8(cfg.ID),
		currentMode:       -1,
		lastVelocityLimit: math.NaN(),
		lastAccelLimit:    math.NaN(),
		lastWatchdog:      math.NaN(),
	}, nil
}

// NewLinkWithPort builds a Link on a serial port the caller already owns.
//
// This is what a daisy chain needs: one port opened once, wrapped so that
// transactions cannot interleave, and one Link per servo id on it.  Close does
// not close the port — whoever opened it decides when the chain goes away.
func NewLinkWithPort(port serial.Port, id uint8, profile Profile) *Link {
	return &Link{
		config:            motor.Config{Driver: "dynamixel", ID: int(id)},
		profile:           profile,
		port:              port,
		driver:            NewDriver(port),
		id:                id,
		ownsPort:          false,
		currentMode:       -1,
		lastVelocityLimit: math.NaN(),
		lastAccelLimit:    math.NaN(),
		lastWatchdog:      math.NaN(),
	}
}

// Kind identifies the driver.
func (l *Link) Kind() string { return "dynamixel" }

// Describe reports the connection.
func (l *Link) Describe() map[string]string {
	return map[string]string{
		"driver":  "dynamixel",
		"port":    l.config.Port,
		"baud":    strconv.Itoa(l.config.Baud),
		"id":      strconv.Itoa(int(l.id)),
		"profile": l.profile.Name,
	}
}

// Capabilities report what a Dynamixel can be asked to do.
//
// Torque reporting is conditional: without a motor-specific torque constant
// the current register cannot be turned into newton-metres, and a safety
// monitor is better told "not measured" than given a fabricated number.
func (l *Link) Capabilities() motor.Capabilities {
	return motor.Capabilities{
		Modes: []motor.ControlMode{
			motor.CtrlStop, motor.CtrlQuery, motor.CtrlPosition,
			motor.CtrlVelocity, motor.CtrlTorque, motor.CtrlHold,
		},
		// A Protocol 2.0 bus at 1 Mbit/s completes a write-plus-read for one
		// servo well inside a millisecond, but the serial round trip and the
		// host's scheduling put the practical ceiling far below that.
		MaxRateHz:          200,
		HasWatchdog:        l.profile.AddrBusWatchdog != 0,
		ReportsTemperature: true,
		ReportsCurrent:     true,
		ReportsVoltage:     true,
	}
}

// Envelope reports the family's supply and speed ratings.
func (l *Link) Envelope() motor.Envelope {
	return motor.Envelope{
		VoltageMin:  l.profile.VoltageMin,
		VoltageMax:  l.profile.VoltageMax,
		VelocityMax: l.profile.MaxVelocityRevPerSec,
	}
}

// Open acquires the serial port and confirms the servo answers.
func (l *Link) Open(context.Context) error {
	if l.port == nil {
		port, err := serial.Open(l.config.Port, l.config.Baud)
		if err != nil {
			return &motor.LinkError{Op: "open", Err: err}
		}
		l.port = port
		l.driver = NewDriver(port)
		l.ownsPort = true
	}

	if _, err := l.driver.Ping(l.id); err != nil {
		l.release()
		return &motor.LinkError{
			Op: "open",
			Err: fmt.Errorf("no reply from servo id %d on %s at %d baud: %w",
				l.id, l.config.Port, l.config.Baud, err),
		}
	}

	// Leave the motor free until the caller arms it, matching every other
	// driver: connecting is not a reason for a machine to become rigid.
	if err := l.setTorque(false); err != nil {
		l.release()
		return err
	}
	return nil
}

// release drops this Link's hold on the port, closing it only if this Link
// opened it.
func (l *Link) release() {
	if l.ownsPort && l.port != nil {
		_ = l.port.Close()
	}
	l.port, l.driver = nil, nil
}

// Close disables torque and releases the port.
func (l *Link) Close() error {
	if l.port == nil {
		return nil
	}
	_ = l.setTorque(false)

	var err error
	if l.ownsPort {
		err = l.port.Close()
	}
	l.port, l.driver = nil, nil
	return err
}

// Identify reads the model and firmware numbers.
func (l *Link) Identify(context.Context) (*motor.Identity, error) {
	if l.driver == nil {
		return nil, motor.ErrClosed
	}
	identity := motor.NewIdentity("dynamixel")

	model, err := l.driver.Ping(l.id)
	if err != nil {
		return nil, err
	}
	identity.ModelNumber = uint32(model)
	identity.Board = fmt.Sprintf("Dynamixel model %d (%s control table)", model, l.profile.Name)

	if data, err := l.driver.Read(l.id, l.profile.AddrFirmwareVersion, 1); err == nil && len(data) == 1 {
		identity.Firmware = strconv.Itoa(int(data[0]))
	}

	// A Dynamixel has no configurable travel envelope equivalent to the one a
	// moteus enforces — position limits exist but only apply in the
	// single-turn modes — so no bounds are reported and the host envelope is
	// the only one in force.
	identity.Status = motor.BoardOK
	identity.Supported = true
	identity.Known = true
	return identity, nil
}

// Cycle transmits one command and reads telemetry back.
func (l *Link) Cycle(_ context.Context, cmd motor.Command, profile motor.QueryProfile) (*motor.Telemetry, error) {
	if l.driver == nil {
		return nil, motor.ErrClosed
	}

	if err := l.applyCommand(cmd); err != nil {
		return nil, err
	}
	return l.readTelemetry(profile)
}

// applyCommand writes whatever this command changes.
func (l *Link) applyCommand(cmd motor.Command) error {
	switch cmd.Mode {
	case motor.CtrlStop:
		// Torque off is the Dynamixel equivalent of releasing the bridge.
		return l.setTorque(false)

	case motor.CtrlQuery:
		return nil

	case motor.CtrlBrake, motor.CtrlStayWithin, motor.CtrlSetOutput:
		return motor.ErrUnsupportedMode
	}

	operatingMode, err := l.operatingModeFor(cmd.Mode)
	if err != nil {
		return err
	}
	if err := l.ensureMode(operatingMode); err != nil {
		return err
	}
	if err := l.applyLimits(cmd); err != nil {
		return err
	}
	if err := l.setTorque(true); err != nil {
		return err
	}

	switch cmd.Mode {
	case motor.CtrlPosition:
		if !motor.Valid(cmd.Position) {
			return nil
		}
		counts := int32(math.Round(cmd.Position * l.profile.CountsPerRev))
		return l.driver.Write4Byte(l.id, l.profile.AddrGoalPosition, uint32(counts))

	case motor.CtrlVelocity:
		if !motor.Valid(cmd.Velocity) {
			return nil
		}
		counts := int32(math.Round(cmd.Velocity * 60.0 / l.profile.VelocityRPMPerCount))
		return l.driver.Write4Byte(l.id, l.profile.AddrGoalVelocity, uint32(counts))

	case motor.CtrlHold:
		// There is no zero-velocity mode; commanding zero velocity in
		// velocity mode is the closest equivalent and behaves the same way.
		return l.driver.Write4Byte(l.id, l.profile.AddrGoalVelocity, 0)

	case motor.CtrlTorque:
		if !motor.Valid(cmd.FeedforwardTorque) {
			return nil
		}
		amps, err := l.torqueToAmps(cmd.FeedforwardTorque)
		if err != nil {
			return err
		}
		counts := int16(math.Round(amps / l.profile.CurrentAmpsPerCount))
		return l.driver.Write(l.id, l.profile.AddrGoalCurrent,
			binary.LittleEndian.AppendUint16(nil, uint16(counts)))
	}
	return nil
}

func (l *Link) operatingModeFor(mode motor.ControlMode) (int, error) {
	switch mode {
	case motor.CtrlPosition:
		// Extended position mode, so that a multi-turn target is not silently
		// wrapped into a single revolution.  The framework's position unit is
		// revolutions with no implied range, and the single-turn mode would
		// quietly break that contract past ±0.5 rev.
		return OpModeExtendedPosition, nil
	case motor.CtrlVelocity, motor.CtrlHold:
		return OpModeVelocity, nil
	case motor.CtrlTorque:
		return OpModeCurrent, nil
	}
	return 0, motor.ErrUnsupportedMode
}

// ensureMode switches the operating mode if needed.
//
// The mode register is in EEPROM and is rejected while torque is enabled, so
// the switch is torque-off, write, settle.  It is slow — hundreds of
// milliseconds — which is exactly why it is done only when the mode actually
// changes.
func (l *Link) ensureMode(mode int) error {
	if l.currentMode == mode {
		return nil
	}
	if err := l.setTorque(false); err != nil {
		return err
	}
	if err := l.driver.Write(l.id, l.profile.AddrOperatingMode, []byte{byte(mode)}); err != nil {
		return err
	}
	// An EEPROM write needs to settle before the next command, or the motor
	// answers with a stale mode and the first goal goes to the wrong register.
	time.Sleep(200 * time.Millisecond)
	l.currentMode = mode

	// A mode change invalidates the profile registers, so they are re-sent.
	l.lastVelocityLimit = math.NaN()
	l.lastAccelLimit = math.NaN()
	return nil
}

// applyLimits writes the profile velocity, acceleration and bus watchdog, but
// only when they change or when the caller asked for a full refresh.
func (l *Link) applyLimits(cmd motor.Command) error {
	if motor.Valid(cmd.VelocityLimit) && (cmd.Full || cmd.VelocityLimit != l.lastVelocityLimit) {
		counts := uint32(math.Round(cmd.VelocityLimit * 60.0 / l.profile.VelocityRPMPerCount))
		if err := l.driver.Write4Byte(l.id, l.profile.AddrProfileVelocity, counts); err != nil {
			return err
		}
		l.lastVelocityLimit = cmd.VelocityLimit
	}

	if motor.Valid(cmd.AccelLimit) && (cmd.Full || cmd.AccelLimit != l.lastAccelLimit) {
		// rev/s² to rev/min², then into the register's own step.
		revPerMin2 := cmd.AccelLimit * 3600.0
		counts := uint32(math.Round(revPerMin2 / l.profile.AccelRevPerMin2PerCount))
		if err := l.driver.Write4Byte(l.id, l.profile.AddrProfileAccel, counts); err != nil {
			return err
		}
		l.lastAccelLimit = cmd.AccelLimit
	}

	if l.profile.AddrBusWatchdog != 0 && motor.Valid(cmd.WatchdogTimeout) &&
		(cmd.Full || cmd.WatchdogTimeout != l.lastWatchdog) {
		counts := int(math.Round(cmd.WatchdogTimeout / l.profile.WatchdogSecondsPerCount))
		// The register is one byte and rounds down to zero for short timeouts,
		// where zero means "no watchdog at all" — the opposite of what a short
		// timeout is asking for.  Clamp into the representable range instead.
		if counts < 1 {
			counts = 1
		}
		if counts > l.profile.WatchdogMaxCount {
			counts = l.profile.WatchdogMaxCount
		}
		if err := l.driver.Write(l.id, l.profile.AddrBusWatchdog, []byte{byte(counts)}); err != nil {
			return err
		}
		l.lastWatchdog = cmd.WatchdogTimeout
	}
	return nil
}

func (l *Link) setTorque(enabled bool) error {
	if l.torqueEnabled == enabled && l.currentMode >= 0 {
		return nil
	}
	value := byte(0)
	if enabled {
		value = 1
	}
	if err := l.driver.Write(l.id, l.profile.AddrTorqueEnable, []byte{value}); err != nil {
		return err
	}
	l.torqueEnabled = enabled
	return nil
}

func (l *Link) torqueToAmps(torque float64) (float64, error) {
	if l.profile.TorqueConstant <= 0 {
		return 0, fmt.Errorf(
			"dynamixel: torque control needs a torque constant; set the torque_constant option (Nm/A)")
	}
	return torque / l.profile.TorqueConstant, nil
}

// readTelemetry reads the present-value block in as few transactions as the
// profile allows.
//
// The registers from present current through present temperature are
// contiguous, so the wide read is one round trip rather than five — which on a
// serial bus is the difference between a 200 Hz loop and a 40 Hz one.
func (l *Link) readTelemetry(profile motor.QueryProfile) (*motor.Telemetry, error) {
	telemetry := motor.NewTelemetry()

	// Present current (2), velocity (4), position (4) = 10 bytes from 126.
	length := uint16(10)
	if profile != motor.ProfileFast {
		// …through present temperature at 146 = 21 bytes.
		length = l.profile.AddrPresentTemp + 1 - l.profile.AddrPresentCurrent
	}

	data, err := l.driver.Read(l.id, l.profile.AddrPresentCurrent, length)
	if err != nil {
		// A single missed reply is a dropped cycle, not a broken link: the
		// loop counts it and carries on, and only a run of them is fatal.
		if _, isTimeout := err.(*motor.TimeoutError); isTimeout {
			return nil, nil
		}
		return nil, err
	}
	if len(data) < 10 {
		return nil, nil
	}

	current := float64(int16(binary.LittleEndian.Uint16(data[0:2]))) * l.profile.CurrentAmpsPerCount
	velocity := float64(int32(binary.LittleEndian.Uint32(data[2:6]))) *
		l.profile.VelocityRPMPerCount / 60.0
	position := float64(int32(binary.LittleEndian.Uint32(data[6:10]))) / l.profile.CountsPerRev

	telemetry.QCurrent = current
	telemetry.Velocity = velocity
	telemetry.Position = position
	if l.profile.TorqueConstant > 0 {
		telemetry.Torque = current * l.profile.TorqueConstant
	}

	// The motor has no mode register in the moteus sense; report whether the
	// bridge is driving, which is what the safety layer actually asks.
	telemetry.Mode = 0
	telemetry.ModeName = "Stopped"
	if l.torqueEnabled {
		telemetry.Mode = 1
		telemetry.ModeName = "Driving"
	}

	if length > 10 && len(data) >= int(length) {
		voltageOffset := l.profile.AddrPresentVoltage - l.profile.AddrPresentCurrent
		tempOffset := l.profile.AddrPresentTemp - l.profile.AddrPresentCurrent
		telemetry.Voltage = float64(binary.LittleEndian.Uint16(
			data[voltageOffset:voltageOffset+2])) * l.profile.VoltageVoltsPerCount
		telemetry.MotorTemperature = float64(data[tempOffset])
		// A Dynamixel is an integrated servo: the only temperature it has is
		// the one inside the case, and it stands for both board and motor.
		telemetry.Temperature = telemetry.MotorTemperature
	}

	if profile != motor.ProfileFast {
		if status, err := l.driver.Read(l.id, l.profile.AddrHardwareError, 1); err == nil && len(status) == 1 {
			telemetry.Fault = int(status[0])
			telemetry.FaultName, telemetry.FaultDetail = decodeHardwareError(status[0])
			telemetry.HardFault = status[0] != 0
		}
	}

	return &telemetry, nil
}
