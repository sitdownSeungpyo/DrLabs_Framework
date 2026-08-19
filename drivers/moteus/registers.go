package moteus

// Register numbers, taken from the moteus reference documentation
// (docs/reference.md, "A.2.b Registers").
type Register uint32

const (
	RegMode               Register = 0x000
	RegPosition           Register = 0x001
	RegVelocity           Register = 0x002
	RegTorque             Register = 0x003
	RegQCurrent           Register = 0x004
	RegDCurrent           Register = 0x005
	RegAbsPosition        Register = 0x006
	RegPower              Register = 0x007
	RegMotorTemperature   Register = 0x00a
	RegTrajectoryComplete Register = 0x00b
	// RegHomeState shares its number with the older "rezero state".
	RegHomeState   Register = 0x00c
	RegVoltage     Register = 0x00d
	RegTemperature Register = 0x00e
	RegFault       Register = 0x00f

	RegPWMPhaseA Register = 0x010
	RegPWMPhaseB Register = 0x011
	RegPWMPhaseC Register = 0x012

	RegVoltagePhaseA Register = 0x014
	RegVoltagePhaseB Register = 0x015
	RegVoltagePhaseC Register = 0x016

	RegVFOCTheta     Register = 0x018
	RegVFOCVoltage   Register = 0x019
	RegVoltageDQD    Register = 0x01a
	RegVoltageDQQ    Register = 0x01b
	RegCommandQCurr  Register = 0x01c
	RegCommandDCurr  Register = 0x01d
	RegVFOCThetaRate Register = 0x01e

	// Command registers.  These are sticky: a value written here stays in
	// force until it is written again, which is why the per-cycle frame only
	// needs to carry the handful that actually change every period.
	RegCommandPosition             Register = 0x020
	RegCommandVelocity             Register = 0x021
	RegCommandFeedforwardTorque    Register = 0x022
	RegCommandKpScale              Register = 0x023
	RegCommandKdScale              Register = 0x024
	RegCommandPositionMaxTorque    Register = 0x025
	RegCommandStopPosition         Register = 0x026
	RegCommandTimeout              Register = 0x027
	RegCommandVelocityLimit        Register = 0x028
	RegCommandAccelLimit           Register = 0x029
	RegCommandFixedVoltageOverride Register = 0x02a
	RegCommandILimitScale          Register = 0x02b
	RegCommandFixedCurrentOverride Register = 0x02c
	RegCommandIgnorePositionBounds Register = 0x02d

	RegPositionKp          Register = 0x030
	RegPositionKi          Register = 0x031
	RegPositionKd          Register = 0x032
	RegPositionFeedforward Register = 0x033
	RegPositionCommand     Register = 0x034

	RegControlPosition Register = 0x038
	RegControlVelocity Register = 0x039
	RegControlTorque   Register = 0x03a
	RegPositionError   Register = 0x03b
	RegVelocityError   Register = 0x03c
	RegTorqueError     Register = 0x03d

	RegWithinLowerBound           Register = 0x040
	RegWithinUpperBound           Register = 0x041
	RegWithinFeedforwardTorque    Register = 0x042
	RegWithinKpScale              Register = 0x043
	RegWithinKdScale              Register = 0x044
	RegWithinMaxTorque            Register = 0x045
	RegWithinTimeout              Register = 0x046
	RegWithinILimitScale          Register = 0x047
	RegWithinIgnorePositionBounds Register = 0x048

	RegEncoder0Position Register = 0x050
	RegEncoder0Velocity Register = 0x051
	RegEncoder1Position Register = 0x052
	RegEncoder1Velocity Register = 0x053
	RegEncoder2Position Register = 0x054
	RegEncoder2Velocity Register = 0x055
	RegEncoderValidity  Register = 0x058

	RegAux1GPIOCommand Register = 0x05c
	RegAux2GPIOCommand Register = 0x05d
	RegAux1GPIOStatus  Register = 0x05e
	RegAux2GPIOStatus  Register = 0x05f

	RegMillisecondCounter Register = 0x070
	RegClockTrim          Register = 0x071

	RegAux1PWMInputPeriod    Register = 0x072
	RegAux1PWMInputDutyCycle Register = 0x073
	RegAux2PWMInputPeriod    Register = 0x074
	RegAux2PWMInputDutyCycle Register = 0x075

	RegModelNumber        Register = 0x100
	RegFirmwareVersion    Register = 0x101
	RegRegisterMapVersion Register = 0x102
	RegMultiplexID        Register = 0x110

	RegSerialNumber1 Register = 0x120
	RegSerialNumber2 Register = 0x121
	RegSerialNumber3 Register = 0x122

	// RegSetOutputNearest re-references the output to the nearest equivalent
	// of the given position; RegSetOutputExact sets it verbatim.  Both are
	// runtime only — neither touches stored configuration.
	RegSetOutputNearest          Register = 0x130
	RegSetOutputExact            Register = 0x131
	RegRequireReindex            Register = 0x132
	RegRecapturePositionVelocity Register = 0x133

	RegDriverFault1 Register = 0x140
	RegDriverFault2 Register = 0x141
)

// Mode is the value of register 0x000.
type Mode int

const (
	ModeStopped      Mode = 0
	ModeFault        Mode = 1
	ModePWM          Mode = 5
	ModeVoltage      Mode = 6
	ModeVoltageFOC   Mode = 7
	ModeVoltageDQ    Mode = 8
	ModeCurrent      Mode = 9
	ModePosition     Mode = 10
	ModeTimeout      Mode = 11
	ModeZeroVelocity Mode = 12
	ModeStayWithin   Mode = 13
	ModeMeasureInd   Mode = 14
	ModeBrake        Mode = 15
)

var modeNames = map[Mode]string{
	ModeStopped:      "Stopped",
	ModeFault:        "Fault",
	2:                "Preparing",
	3:                "Preparing",
	4:                "Preparing",
	ModePWM:          "PWM",
	ModeVoltage:      "Voltage",
	ModeVoltageFOC:   "Voltage FOC",
	ModeVoltageDQ:    "Voltage DQ",
	ModeCurrent:      "Current",
	ModePosition:     "Position",
	ModeTimeout:      "Timeout",
	ModeZeroVelocity: "Zero velocity",
	ModeStayWithin:   "Stay within",
	ModeMeasureInd:   "Measure inductance",
	ModeBrake:        "Brake",
}

// ModeName decodes register 0x000 for display.
func ModeName(mode int) string {
	if name, ok := modeNames[Mode(mode)]; ok {
		return name
	}
	return "unknown"
}

// activeModes are the modes in which the bridge is driving the motor.  Arming
// while the controller is already in one of them is worth warning about: the
// axis is live before the operator has consented to it.
var activeModes = map[Mode]bool{
	ModePWM: true, ModeVoltage: true, ModeVoltageFOC: true, ModeVoltageDQ: true,
	ModeCurrent: true, ModePosition: true, ModeZeroVelocity: true,
	ModeStayWithin: true, ModeBrake: true,
}

// IsActiveMode reports whether the controller is driving the motor.
func IsActiveMode(mode int) bool { return activeModes[Mode(mode)] }

// Scale factors for each register family, indexed by resolution
// (int8, int16, int32).  F32 registers carry the physical value directly.
var (
	positionScales    = [3]float64{0.01, 0.0001, 0.00001} // revolutions
	velocityScales    = [3]float64{0.1, 0.00025, 0.00001} // rev/s
	accelScales       = [3]float64{0.05, 0.001, 0.00001}  // rev/s^2
	torqueScales      = [3]float64{0.5, 0.01, 0.001}      // Nm
	pwmScales         = [3]float64{1.0 / 127.0, 1.0 / 32767.0, 1.0 / 2147483647.0}
	voltageScales     = [3]float64{0.5, 0.1, 0.001}       // V
	temperatureScales = [3]float64{1.0, 0.1, 0.001}       // °C
	timeScales        = [3]float64{0.01, 0.001, 0.000001} // s
	currentScales     = [3]float64{1.0, 0.1, 0.001}       // A
	powerScales       = [3]float64{10.0, 0.05, 0.0001}    // W
)

// registerScales maps a register to the scale table its value uses.  A
// register absent from the map is an integer whose raw value is its meaning
// (mode, fault, trajectory-complete, home state).
var registerScales = map[Register]*[3]float64{
	RegPosition:         &positionScales,
	RegVelocity:         &velocityScales,
	RegTorque:           &torqueScales,
	RegQCurrent:         &currentScales,
	RegDCurrent:         &currentScales,
	RegAbsPosition:      &positionScales,
	RegPower:            &powerScales,
	RegMotorTemperature: &temperatureScales,
	RegVoltage:          &voltageScales,
	RegTemperature:      &temperatureScales,

	RegCommandPosition:          &positionScales,
	RegCommandVelocity:          &velocityScales,
	RegCommandFeedforwardTorque: &torqueScales,
	RegCommandKpScale:           &pwmScales,
	RegCommandKdScale:           &pwmScales,
	RegCommandPositionMaxTorque: &torqueScales,
	RegCommandStopPosition:      &positionScales,
	RegCommandTimeout:           &timeScales,
	RegCommandVelocityLimit:     &velocityScales,
	RegCommandAccelLimit:        &accelScales,
	RegCommandILimitScale:       &pwmScales,

	RegWithinLowerBound:        &positionScales,
	RegWithinUpperBound:        &positionScales,
	RegWithinFeedforwardTorque: &torqueScales,
	RegWithinMaxTorque:         &torqueScales,
	RegWithinTimeout:           &timeScales,

	RegPositionKp:          &torqueScales,
	RegPositionKi:          &torqueScales,
	RegPositionKd:          &torqueScales,
	RegPositionFeedforward: &torqueScales,
	RegPositionCommand:     &torqueScales,

	RegControlPosition: &positionScales,
	RegControlVelocity: &velocityScales,
	RegControlTorque:   &torqueScales,
	RegPositionError:   &positionScales,
	RegVelocityError:   &velocityScales,
	RegTorqueError:     &torqueScales,

	RegEncoder0Position: &positionScales,
	RegEncoder0Velocity: &velocityScales,
	RegEncoder1Position: &positionScales,
	RegEncoder1Velocity: &velocityScales,
	RegEncoder2Position: &positionScales,
	RegEncoder2Velocity: &velocityScales,
}

// scaleRegister converts a raw register value into physical units.
func scaleRegister(register Register, value regValue) float64 {
	scales, ok := registerScales[register]
	if !ok || value.res == F32 {
		return value.raw
	}
	return value.raw * scales[value.res]
}
