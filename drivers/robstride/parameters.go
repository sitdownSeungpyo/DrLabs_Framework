package robstride

import (
	"encoding/binary"
	"fmt"
	"math"
	"sort"
)

// The 0x70xx parameter table, read with communication type 17 and written with
// type 18.  A type-18 write is volatile — it is lost on power-down unless a
// save frame (type 22) commits it.
//
// Parameter frames are little-endian, unlike the control and feedback frames.

// Parameter is an index into the motor's parameter table.
type Parameter uint16

const (
	ParamRunMode     Parameter = 0x7005 // uint8, see RunMode
	ParamIqRef       Parameter = 0x7006 // float, A — current-mode setpoint
	ParamSpdRef      Parameter = 0x700A // float, rad/s — velocity-mode setpoint
	ParamLimitTorque Parameter = 0x700B // float, Nm — torque ceiling in every mode
	ParamCurKp       Parameter = 0x7010 // float — current loop Kp
	ParamCurKi       Parameter = 0x7011 // float — current loop Ki
	ParamCurFiltGain Parameter = 0x7014 // float, 0..1
	ParamLocRef      Parameter = 0x7016 // float, rad — position-mode setpoint
	ParamLimitSpd    Parameter = 0x7017 // float, rad/s — CSP speed limit
	ParamLimitCur    Parameter = 0x7018 // float, A — velocity/position current limit
	ParamMechPos     Parameter = 0x7019 // float, rad — read only
	ParamIqFilt      Parameter = 0x701A // float, A — read only, filtered q current
	ParamMechVel     Parameter = 0x701B // float, rad/s — read only
	ParamVBus        Parameter = 0x701C // float, V — read only
	ParamLocKp       Parameter = 0x701E // float — position loop Kp
	ParamSpdKp       Parameter = 0x701F // float — velocity loop Kp
	ParamSpdKi       Parameter = 0x7020 // float — velocity loop Ki
	ParamSpdFiltGain Parameter = 0x7021 // float
	ParamAccRad      Parameter = 0x7022 // float, rad/s² — velocity-mode acceleration
	ParamVelMax      Parameter = 0x7024 // float, rad/s — PP-mode speed
	ParamAccSet      Parameter = 0x7025 // float, rad/s² — PP-mode acceleration
	ParamReportTime  Parameter = 0x7026 // uint16 — active reporting interval
	ParamCANTimeout  Parameter = 0x7028 // uint32 — command timeout, 20000 == 1 s
)

// ParamKind is the on-wire type of a parameter's value.
type ParamKind uint8

const (
	KindFloat ParamKind = iota
	KindUint8
	KindUint16
	KindUint32
)

// ParamInfo describes one parameter.
type ParamInfo struct {
	Index    Parameter
	Name     string
	Kind     ParamKind
	Writable bool
	Unit     string
}

var paramTable = []ParamInfo{
	{ParamRunMode, "run_mode", KindUint8, true, ""},
	{ParamIqRef, "iq_ref", KindFloat, true, "A"},
	{ParamSpdRef, "spd_ref", KindFloat, true, "rad/s"},
	{ParamLimitTorque, "limit_torque", KindFloat, true, "Nm"},
	{ParamCurKp, "cur_kp", KindFloat, true, ""},
	{ParamCurKi, "cur_ki", KindFloat, true, ""},
	{ParamCurFiltGain, "cur_filt_gain", KindFloat, true, ""},
	{ParamLocRef, "loc_ref", KindFloat, true, "rad"},
	{ParamLimitSpd, "limit_spd", KindFloat, true, "rad/s"},
	{ParamLimitCur, "limit_cur", KindFloat, true, "A"},
	{ParamMechPos, "mechPos", KindFloat, false, "rad"},
	{ParamIqFilt, "iqf", KindFloat, false, "A"},
	{ParamMechVel, "mechVel", KindFloat, false, "rad/s"},
	{ParamVBus, "VBUS", KindFloat, false, "V"},
	{ParamLocKp, "loc_kp", KindFloat, true, ""},
	{ParamSpdKp, "spd_kp", KindFloat, true, ""},
	{ParamSpdKi, "spd_ki", KindFloat, true, ""},
	{ParamSpdFiltGain, "spd_filt_gain", KindFloat, true, ""},
	{ParamAccRad, "acc_rad", KindFloat, true, "rad/s^2"},
	{ParamVelMax, "vel_max", KindFloat, true, "rad/s"},
	{ParamAccSet, "acc_set", KindFloat, true, "rad/s^2"},
	{ParamReportTime, "EPScan_time", KindUint16, true, ""},
	{ParamCANTimeout, "canTimeout", KindUint32, true, ""},
}

var (
	paramsByIndex = map[Parameter]ParamInfo{}
	paramsByName  = map[string]ParamInfo{}
)

func init() {
	for _, info := range paramTable {
		paramsByIndex[info.Index] = info
		paramsByName[info.Name] = info
	}
}

// LookupParam finds a parameter by its documented name.
func LookupParam(name string) (ParamInfo, bool) {
	info, ok := paramsByName[name]
	return info, ok
}

// ParamNames lists the known parameter names, sorted.
func ParamNames() []string {
	names := make([]string, 0, len(paramsByName))
	for name := range paramsByName {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// Info returns what is known about a parameter index.
func (p Parameter) Info() (ParamInfo, bool) {
	info, ok := paramsByIndex[p]
	return info, ok
}

func (p Parameter) String() string {
	if info, ok := p.Info(); ok {
		return info.Name
	}
	return fmt.Sprintf("0x%04X", uint16(p))
}

// RunMode selects which of the controller's loops is in charge.
type RunMode uint8

const (
	// RunOperation is the "MIT" mode: one frame carries position, velocity,
	// both gains and a feedforward torque, and the controller computes
	// torque = Kp·(p−p̂) + Kd·(v−v̂) + t_ff.  It is the mode a host control
	// loop wants, because a whole command fits in one frame.
	RunOperation RunMode = 0
	// RunPositionPP is a trapezoidal profile move to loc_ref.  Speed and
	// acceleration cannot be changed while it runs.
	RunPositionPP RunMode = 1
	// RunVelocity is a closed-loop velocity controller tracking spd_ref.
	RunVelocity RunMode = 2
	// RunCurrent commands iq_ref directly.
	RunCurrent RunMode = 3
	// RunPositionCSP is cyclic synchronous position: loc_ref is a setpoint
	// the host updates every cycle, limited by limit_spd.
	RunPositionCSP RunMode = 5
)

var runModeNames = map[RunMode]string{
	RunOperation:   "operation",
	RunPositionPP:  "position-pp",
	RunVelocity:    "velocity",
	RunCurrent:     "current",
	RunPositionCSP: "position-csp",
}

func (m RunMode) String() string {
	if name, ok := runModeNames[m]; ok {
		return name
	}
	return fmt.Sprintf("run_mode(%d)", uint8(m))
}

// encodeParamWrite builds the payload of a type-18 write.
//
//	bytes 0..1  parameter index, little-endian
//	bytes 2..3  zero
//	bytes 4..7  value, little-endian
func encodeParamWrite(index Parameter, value float64) ([]byte, error) {
	data := make([]byte, 8)
	binary.LittleEndian.PutUint16(data[0:2], uint16(index))

	info, known := paramsByIndex[index]
	if !known {
		// An index this build does not know is still writable — the table
		// lags firmware — but it can only be sent as a float.
		info = ParamInfo{Index: index, Kind: KindFloat, Writable: true}
	}
	if !info.Writable {
		return nil, fmt.Errorf("robstride: parameter %s is read only", index)
	}
	if math.IsNaN(value) || math.IsInf(value, 0) {
		return nil, fmt.Errorf("robstride: parameter %s cannot be set to %v", index, value)
	}

	switch info.Kind {
	case KindUint8:
		data[4] = byte(uint8(value))
	case KindUint16:
		binary.LittleEndian.PutUint16(data[4:8], uint16(value))
	case KindUint32:
		binary.LittleEndian.PutUint32(data[4:8], uint32(value))
	default:
		binary.LittleEndian.PutUint32(data[4:8], math.Float32bits(float32(value)))
	}
	return data, nil
}

// encodeParamRead builds the payload of a type-17 read.
func encodeParamRead(index Parameter) []byte {
	data := make([]byte, 8)
	binary.LittleEndian.PutUint16(data[0:2], uint16(index))
	return data
}

// decodeParamReply parses the reply to a type-17 read.
//
// It returns the index the motor answered with as well as the value, because a
// reply that does not match the request means the bus delivered someone else's
// answer — and quietly using it would put another parameter's value into this
// one's variable.
func decodeParamReply(data []byte) (Parameter, float64, error) {
	if len(data) < 8 {
		return 0, 0, fmt.Errorf("robstride: parameter reply carries %d bytes, want 8", len(data))
	}
	index := Parameter(binary.LittleEndian.Uint16(data[0:2]))

	info, known := paramsByIndex[index]
	if !known {
		info = ParamInfo{Index: index, Kind: KindFloat}
	}

	switch info.Kind {
	case KindUint8:
		return index, float64(data[4]), nil
	case KindUint16:
		return index, float64(binary.LittleEndian.Uint16(data[4:8])), nil
	case KindUint32:
		return index, float64(binary.LittleEndian.Uint32(data[4:8])), nil
	default:
		return index, float64(math.Float32frombits(binary.LittleEndian.Uint32(data[4:8]))), nil
	}
}
