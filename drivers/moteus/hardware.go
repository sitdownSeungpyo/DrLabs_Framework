package moteus

import (
	"fmt"

	"drlabs/motor"
)

// Hardware facts for the moteus controller family.
//
// Every number here comes from the mjbots/moteus reference documentation
// (docs/reference.md and the hardware pages).  They live in one file so that
// the driver, the safety monitor and any UI agree on the envelope rather than
// each carrying its own copy.

// Board families, as reported by the firmware telemetry channel.
var boardFamilies = map[int]string{
	0: "moteus (r4 / r5 series)",
	1: "moteus-n1",
	2: "moteus-c1",
	3: "moteus-x1",
}

// FamilyR4 is family 0, the only family whose electrical envelope this build
// carries.  Boards from the other families are recognised by name but are not
// characterised here, so they are reported as unknown rather than accepted.
const FamilyR4 = 0

// Family 0 hardware revisions, from the firmware's own table.  Revisions 0-2
// are pre-r4 prototypes; 3-8 are the r4 boards.
var r4RevisionNames = map[int]string{
	0: "r1",
	1: "r2",
	2: "r3",
	3: "r4.1",
	4: "r4.2/r4.3",
	5: "r4.4",
	6: "r4.5",
	7: "r4.5b-r4.8",
	8: "r4.11",
}

const (
	r4RevisionMin = 3
	r4RevisionMax = 8
)

// Envelope is the electrical and motion envelope of a controller family.
type Envelope struct {
	InputVoltageMin     float64 // V
	InputVoltageMax     float64 // V
	InputVoltageNominal float64 // V
	PeakPhaseCurrent    float64 // A
	PWMRateHz           float64 // switching frequency, equal to the control rate
	PositionAbsMax      float64 // rev
	VelocityMax         float64 // rev/s
}

// R4Envelope is the documented envelope of the moteus r4.
var R4Envelope = Envelope{
	InputVoltageMin:     10.0, // "10V to 44V (<= 10S LiPo)"
	InputVoltageMax:     44.0,
	InputVoltageNominal: 24.0,
	PeakPhaseCurrent:    100.0,
	PWMRateHz:           30000.0,
	PositionAbsMax:      32768.0,
	VelocityMax:         467.0,
}

// CAN bus parameters for moteus.
const (
	CANNominalBitrate = 1_000_000
	CANDataBitrate    = 5_000_000
	// CANDocumentedMaxCycleHz is the rate the documentation calls out as
	// achievable.  Whether it is achievable *here* depends on the adapter and
	// the host's scheduling, which is why the session measures the rate it
	// actually attains rather than trusting this number.
	CANDocumentedMaxCycleHz = 1000
)

// BoardName is a best-effort human readable name for a family/revision pair.
func BoardName(family, hwrev int) string {
	if family < 0 {
		return "unknown"
	}
	if family == FamilyR4 {
		if name, ok := r4RevisionNames[hwrev]; ok {
			return "moteus " + name
		}
		if hwrev < 0 {
			return "moteus (family 0, revision not reported)"
		}
		return fmt.Sprintf("moteus family 0, hwrev %d", hwrev)
	}
	if name, ok := boardFamilies[family]; ok {
		return name
	}
	return fmt.Sprintf("family %d", family)
}

// BoardStatus decides whether this build knows enough about a board to drive
// it safely.
//
// Only family 0 revisions 3-8 are positively characterised.  Anything else is
// reported as unknown rather than rejected outright: a revision table always
// lags new hardware, and a stale table must not lock a genuine controller out.
// The pre-r4 prototypes are the one hard rejection, because their electrical
// limits are known to differ.
func BoardStatus(family, hwrev int) motor.BoardStatus {
	if family < 0 {
		return motor.BoardUnknown
	}
	if family != FamilyR4 {
		return motor.BoardUnknown
	}
	if hwrev < 0 {
		return motor.BoardUnknown
	}
	if hwrev >= r4RevisionMin && hwrev <= r4RevisionMax {
		return motor.BoardOK
	}
	if hwrev < r4RevisionMin {
		return motor.BoardRejected // r1/r2/r3 prototypes
	}
	return motor.BoardUnknown // newer than this build knows about
}

// Fault describes one value of register 0x00f.
type Fault struct {
	Name   string
	Detail string
}

// faults are the controller's fault and limiting codes.  Codes at or above
// LimitingFaultMin are not faults at all: they say "the output is being
// limited, and here is by what", which is the single most useful thing to
// show an operator whose motor is accepting commands but not moving.
var faults = map[int]Fault{
	0:  {"No fault", ""},
	32: {"Calibration fault", "The encoder could not sense the magnet during calibration."},
	33: {"Motor driver fault", "Usually undervoltage, or a supply that cannot deliver the demanded current."},
	34: {"Over voltage", "Bus voltage exceeded servo.max_voltage — most often regeneration with no battery on the bus."},
	35: {"Encoder fault", "The encoder reported inconsistent readings."},
	36: {"Motor not configured", "moteus_tool --calibrate has not been run for this motor."},
	37: {"PWM cycle overrun", "Firmware level timing error."},
	38: {"Over temperature", "The board exceeded its configured fault temperature."},
	39: {"Outside limit", "Position control was started while the axis was already outside servopos.position_min/max. " +
		"Turn the shaft back inside by hand (the driver is off, so it is free), re-reference with a set-output while " +
		"disarmed, or set IgnorePositionBounds on the command."},
	40: {"Under voltage", "Bus voltage dropped below the configured limit."},
	41: {"Config changed", "A configuration value changed that requires a stop."},
	42: {"Theta invalid", "No valid commutation encoder."},
	43: {"Position invalid", "No valid output encoder."},
	44: {"Driver enable fault", "The gate driver failed to enable."},
	45: {"Stop position deprecated", "Use velocity and acceleration limits instead of stop position."},
	46: {"Timing violation", "An internal timing constraint was violated."},
	47: {"BEMF feedforward without accel limit", "An acceleration limit is required for this feature."},
	48: {"Invalid limits", "Configured position bounds are outside the valid range."},
	49: {"Position control error", "Position error exceeded the configured threshold."},
	50: {"Velocity control error", "Velocity error exceeded the configured threshold."},

	96:  {"Limiting: max velocity", "The trajectory is capped by servo.max_velocity or the commanded velocity limit."},
	97:  {"Limiting: max power", "Output is capped by servo.max_power_W."},
	98:  {"Limiting: max voltage", "Output is capped because the bus voltage is at its limit."},
	99:  {"Limiting: max current", "Output is capped by servo.max_current_A."},
	100: {"Limiting: FET temperature", "Output derated because the board is hot."},
	101: {"Limiting: motor temperature", "Output derated because the motor is hot."},
	102: {"Limiting: max torque", "The commanded maximum torque is the active limit. If the axis will not move, " +
		"raise the torque limit — this is the most common reason a moteus sits still while accepting commands."},
	103: {"Limiting: position bounds", "The target is outside servopos.position_min/max, so the firmware clamps it."},
	104: {"Limiting: flux braking", "Flux braking is active to absorb regenerated energy."},
	105: {"Limiting: field weakening", "Field weakening is active at high speed."},
}

// LimitingFaultMin is the first code that reports limiting rather than a
// latching fault.
const LimitingFaultMin = 96

// FaultText decodes a fault code into a name and an explanation.
func FaultText(code int) Fault {
	if fault, ok := faults[code]; ok {
		return fault
	}
	return Fault{Name: fmt.Sprintf("Fault %d", code), Detail: ""}
}

// IsHardFault reports whether a code is a latching fault, as opposed to an
// informational limiting state or no fault at all.
func IsHardFault(code int) bool {
	return code > 0 && code < LimitingFaultMin
}

// Home state values for register 0x00c.
var homeStates = map[int]string{0: "relative", 1: "rotor", 2: "output"}

// HomeStateName decodes register 0x00c.
func HomeStateName(state int) string {
	if name, ok := homeStates[state]; ok {
		return name
	}
	return ""
}
