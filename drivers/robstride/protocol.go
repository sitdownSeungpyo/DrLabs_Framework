// Package robstride implements the RobStride (灵足时代) quasi-direct-drive
// actuator family in pure Go: RS00, RS01, RS02, RS03, RS04, RS05 and RS06.
//
// The wire protocol is CAN 2.0B at 1 Mbit/s with 29-bit extended identifiers
// and an 8-byte payload.  Every detail below is taken from the official
// RobStride user manuals (Product_Information repository, "User Manual260713"
// for each model); the per-model scaling table in models.go is the part that
// differs between them, and getting it wrong is the difference between
// commanding 17 Nm and commanding 120.
//
//	robstride.Link  ->  robstride.Controller  ->  can.Bus  ->  fdcanusb
package robstride

import (
	"encoding/binary"
	"fmt"
	"math"
)

// CommunicationType is the 5-bit function code in bits 28..24 of the
// arbitration identifier.
type CommunicationType uint8

const (
	// TypeGetDeviceID asks for the 64-bit MCU unique identifier.
	TypeGetDeviceID CommunicationType = 0
	// TypeControl is the operation-control ("MIT") command: position,
	// velocity, Kp, Kd in the payload and feedforward torque in the id.
	TypeControl CommunicationType = 1
	// TypeFeedback is the motor's reply to almost every command.
	TypeFeedback CommunicationType = 2
	// TypeEnable puts the motor into run mode.
	TypeEnable CommunicationType = 3
	// TypeStop takes it out of run mode, optionally clearing faults.
	TypeStop CommunicationType = 4
	// TypeSetZero declares the current shaft position to be mechanical zero.
	TypeSetZero CommunicationType = 6
	// TypeSetCANID changes the motor's own address, effective immediately.
	TypeSetCANID CommunicationType = 7
	// TypeReadParam and TypeWriteParam access the 0x70xx parameter table.
	TypeReadParam  CommunicationType = 17
	TypeWriteParam CommunicationType = 18
	// TypeFaultFeedback carries the detailed fault and warning words.
	TypeFaultFeedback CommunicationType = 21
	// TypeSaveParams commits parameters that survive a power cycle.
	TypeSaveParams CommunicationType = 22
	// TypeSetBaudRate and TypeSetReporting take effect on the next power-up.
	TypeSetBaudRate  CommunicationType = 23
	TypeSetReporting CommunicationType = 24
)

// Arbitration identifier layout:
//
//	bits 28..24   communication type
//	bits 23..8    data area 2 — usually the host id in bits 15..8, but the
//	              control command puts the whole feedforward torque there
//	bits  7..0    destination address (the target motor's CAN id)
//
// Replies invert the addressing: the motor's id lands in bits 15..8 and the
// host's in bits 7..0.
const (
	typeShift      = 24
	typeMask       = 0x1f
	dataArea2Shift = 8
	dataArea2Mask  = 0xffff
	addressMask    = 0xff
)

// arbitrationID builds the identifier for a frame sent to a motor.
func arbitrationID(commType CommunicationType, dataArea2 uint16, target uint8) uint32 {
	return uint32(commType&typeMask)<<typeShift |
		uint32(dataArea2)<<dataArea2Shift |
		uint32(target)
}

// parseArbitrationID splits a received identifier.
//
// In a reply, data area 2 carries the responding motor's id in its low byte
// and — for a feedback frame — the fault and mode bits above it.
func parseArbitrationID(id uint32) (commType CommunicationType, dataArea2 uint16, address uint8) {
	return CommunicationType((id >> typeShift) & typeMask),
		uint16((id >> dataArea2Shift) & dataArea2Mask),
		uint8(id & addressMask)
}

// DefaultHostID is the address this host answers to.  0xFD is the value the
// manuals use in their worked examples; any value the bus does not otherwise
// use will do.
const DefaultHostID uint8 = 0xFD

// broadcastReplyAddress is the fixed address a device-id reply is sent to.
const broadcastReplyAddress uint8 = 0xFE

// PositionRange is the full-scale range of the position field, ±4π radians,
// on every model.  The controller counts turns within it and wraps, so a
// multi-turn axis has to track revolutions on the host.
const PositionRange = 4 * math.Pi

// Mode is the motor's own state, reported in bits 23..22 of a feedback frame.
type Mode uint8

const (
	ModeReset       Mode = 0
	ModeCalibration Mode = 1
	ModeRun         Mode = 2
)

var modeNames = map[Mode]string{
	ModeReset:       "Reset",
	ModeCalibration: "Calibration",
	ModeRun:         "Run",
}

// ModeName decodes the mode field for display.
func ModeName(mode int) string {
	if name, ok := modeNames[Mode(mode)]; ok {
		return name
	}
	return "unknown"
}

// Feedback flag bits, in bits 21..16 of a feedback frame's identifier.
//
// These are a summary; the detailed fault and warning words come from a fault
// feedback frame (type 21).
const (
	flagUndervoltage    = 1 << 0 // bit 16
	flagOvercurrent     = 1 << 1 // bit 17, three-phase
	flagOvertemperature = 1 << 2 // bit 18
	flagMagneticEncoder = 1 << 3 // bit 19
	flagOverloadStall   = 1 << 4 // bit 20
	flagUncalibrated    = 1 << 5 // bit 21
)

// summaryFaults maps each summary bit to a description, most severe first so
// that the reported cause is the one worth acting on.
var summaryFaults = []struct {
	bit    uint16
	name   string
	detail string
}{
	{flagUncalibrated, "Uncalibrated",
		"The motor's encoder has not been calibrated. It cannot be driven until it is."},
	{flagMagneticEncoder, "Magnetic encoder fault",
		"The encoder is not reporting usable positions. Check the magnet and the shaft coupling."},
	{flagOvertemperature, "Over temperature",
		"The motor exceeded its temperature limit and has derated or stopped."},
	{flagOvercurrent, "Three-phase over current",
		"The phase current exceeded the hardware limit — usually a short, a stall, or too high a torque command."},
	{flagUndervoltage, "Under voltage",
		"The bus voltage dropped below the protection threshold. Check the supply and its current capability."},
	{flagOverloadStall, "Stall overload protection",
		"The motor's own stall-overload algorithm tripped: it has been drawing high current without turning."},
}

// Feedback is a decoded type-2 frame, in the motor's native units.
type Feedback struct {
	MotorID uint8

	// Flags is the raw summary word from identifier bits 21..16.
	Flags uint16

	Mode Mode

	Angle       float64 // rad, within ±4π
	Velocity    float64 // rad/s
	Torque      float64 // Nm
	Temperature float64 // °C
}

// Faulted reports whether any summary fault bit is set.
func (f Feedback) Faulted() bool { return f.Flags != 0 }

// FaultText describes the most significant fault present.
func (f Feedback) FaultText() (name, detail string) {
	for _, fault := range summaryFaults {
		if f.Flags&fault.bit != 0 {
			return fault.name, fault.detail
		}
	}
	return "No fault", ""
}

// encodeUint16 maps a physical value onto the protocol's 0..65535 field.
//
// Every field in the control and feedback frames is a 16-bit linear map over a
// symmetric range, and the range is per-model — which is why this takes it as
// an argument rather than baking one in.  Out-of-range values are clamped, not
// wrapped: commanding more torque than the codec can express must not come out
// the other side as full reverse.
//
// One consequence is worth knowing before it surprises someone. A symmetric
// range spread over 65536 codes has no code at its exact centre, so zero is
// not representable: it encodes to the nearest code and decodes back half a
// count away.  On an RS01 that is 0.26 milli-newton-metres and on an RS04 it
// is 1.8, far below anything that matters physically — but a comparison
// against an exact zero will never hold, so telemetry should be compared
// against a count rather than against 0.
func encodeUint16(value, limit float64) uint16 {
	if math.IsNaN(value) {
		value = 0
	}
	scaled := (value + limit) / (2 * limit) * 65535.0
	switch {
	case scaled <= 0:
		return 0
	case scaled >= 65535:
		return 65535
	default:
		return uint16(math.Round(scaled))
	}
}

// decodeUint16 is the inverse of encodeUint16.
func decodeUint16(raw uint16, limit float64) float64 {
	return float64(raw)/65535.0*(2*limit) - limit
}

// ControlCommand is one operation-control ("MIT") frame, in the motor's native
// units.
//
// The controller computes torque as
//
//	t = Kp·(angle − measured) + Kd·(velocity − measured) + torque
//
// so this one frame expresses position control, velocity control, pure torque
// and pure damping, depending on which gains are non-zero.
type ControlCommand struct {
	Angle    float64 // rad
	Velocity float64 // rad/s
	Kp       float64 // Nm/rad
	Kd       float64 // Nm/(rad/s)
	Torque   float64 // Nm, feedforward
}

// encodeControl builds the identifier and payload for a control command.
//
// The payload is big-endian — "the high byte is in front" in the manuals —
// while the parameter frames are little-endian.  Mixing the two up produces
// positions that look like noise, so the two encoders are kept apart.
func encodeControl(cmd ControlCommand, model Model, target uint8) (uint32, []byte) {
	id := arbitrationID(TypeControl, encodeUint16(cmd.Torque, model.TorqueRange), target)

	data := make([]byte, 8)
	binary.BigEndian.PutUint16(data[0:2], encodeUint16(cmd.Angle, PositionRange))
	binary.BigEndian.PutUint16(data[2:4], encodeUint16(cmd.Velocity, model.VelocityRange))
	binary.BigEndian.PutUint16(data[4:6], encodeGain(cmd.Kp, model.KpMax))
	binary.BigEndian.PutUint16(data[6:8], encodeGain(cmd.Kd, model.KdMax))
	return id, data
}

// encodeGain maps a gain onto its 0..max field.  Gains are one-sided, so this
// is a plain scale rather than the symmetric map the other fields use.
func encodeGain(value, max float64) uint16 {
	if math.IsNaN(value) || value <= 0 {
		return 0
	}
	scaled := value / max * 65535.0
	if scaled >= 65535 {
		return 65535
	}
	return uint16(math.Round(scaled))
}

// decodeFeedback parses a type-2 frame.
func decodeFeedback(id uint32, data []byte, model Model) (Feedback, error) {
	if len(data) < 8 {
		return Feedback{}, fmt.Errorf("robstride: feedback frame carries %d bytes, want 8", len(data))
	}
	_, dataArea2, _ := parseArbitrationID(id)

	return Feedback{
		MotorID: uint8(dataArea2 & 0xff),
		// Identifier bits 21..16 are bits 13..8 of data area 2.
		Flags: (dataArea2 >> 8) & 0x3f,
		// Identifier bits 23..22 are bits 15..14 of data area 2.
		Mode:     Mode((dataArea2 >> 14) & 0x03),
		Angle:    decodeUint16(binary.BigEndian.Uint16(data[0:2]), PositionRange),
		Velocity: decodeUint16(binary.BigEndian.Uint16(data[2:4]), model.VelocityRange),
		Torque:   decodeUint16(binary.BigEndian.Uint16(data[4:6]), model.TorqueRange),
		// Temperature is a plain fixed-point value, not a ranged map, and is
		// read as signed so that a below-zero ambient does not come back as
		// several thousand degrees.
		Temperature: float64(int16(binary.BigEndian.Uint16(data[6:8]))) / 10.0,
	}, nil
}

// FaultReport is a decoded type-21 frame: the full fault and warning words,
// which carry more detail than the six summary bits in a feedback frame.
type FaultReport struct {
	Fault   uint32
	Warning uint32
}

// detailedFaults decode the type-21 fault word.
var detailedFaults = []struct {
	bit    uint32
	name   string
	detail string
}{
	{1 << 16, "A-phase current sampling over current", "The A-phase current sensor read beyond its range."},
	{1 << 14, "Stall overload protection", "The motor's stall-overload algorithm tripped."},
	{1 << 9, "Position initialisation fault", "The controller could not establish a starting position."},
	{1 << 8, "Hardware identification fault", "The controller did not recognise its own hardware."},
	{1 << 7, "Encoder uncalibrated", "The motor encoder has not been calibrated."},
	{1 << 5, "C-phase current sampling over current", "The C-phase current sensor read beyond its range."},
	{1 << 4, "B-phase current sampling over current", "The B-phase current sensor read beyond its range."},
	{1 << 3, "Over voltage", "The bus voltage exceeded the protection threshold."},
	{1 << 2, "Under voltage", "The bus voltage fell below the protection threshold."},
	{1 << 1, "Driver chip fault", "The gate driver reported a fault."},
	{1 << 0, "Motor over temperature", "The motor exceeded its temperature fault threshold."},
}

// Text describes the most significant fault in the report.
func (r FaultReport) Text() (name, detail string) {
	for _, fault := range detailedFaults {
		if r.Fault&fault.bit != 0 {
			return fault.name, fault.detail
		}
	}
	if r.Warning&1 != 0 {
		return "Motor over temperature warning", "The motor is above its warning threshold but below the fault threshold."
	}
	return "No fault", ""
}

// decodeFaultReport parses a type-21 frame.
func decodeFaultReport(data []byte) (FaultReport, error) {
	if len(data) < 8 {
		return FaultReport{}, fmt.Errorf("robstride: fault frame carries %d bytes, want 8", len(data))
	}
	return FaultReport{
		Fault:   binary.LittleEndian.Uint32(data[0:4]),
		Warning: binary.LittleEndian.Uint32(data[4:8]),
	}, nil
}
