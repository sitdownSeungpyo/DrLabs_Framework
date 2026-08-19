package robstride

import (
	"encoding/binary"
	"encoding/hex"
	"math"
	"strings"
	"testing"
)

// The expected values here come from the official RobStride user manuals
// (RobStride/Product_Information, "<model>User Manual260713.pdf"), section 4.1.
// Where the manual gives a worked example, it is reproduced verbatim, because
// a codec that is merely self-consistent will happily talk to nothing.

func TestArbitrationIDLayout(t *testing.T) {
	// Type 17 read of loc_kp, from the manual's worked example: type 0x11,
	// host id 0xFD in data area 2, target motor 0x7F.
	id := arbitrationID(TypeReadParam, 0x00FD, 0x7F)
	if id != 0x1100FD7F {
		t.Errorf("got %#08x, want 0x1100FD7F", id)
	}

	commType, dataArea2, address := parseArbitrationID(id)
	if commType != TypeReadParam {
		t.Errorf("type: got %d, want %d", commType, TypeReadParam)
	}
	if dataArea2 != 0x00FD {
		t.Errorf("data area 2: got %#04x, want 0x00FD", dataArea2)
	}
	if address != 0x7F {
		t.Errorf("address: got %#02x, want 0x7F", address)
	}
}

func TestReplyAddressingIsInverted(t *testing.T) {
	// The manual's reply to that read: type 17, motor id 0x7F in data area 2,
	// host id 0xFD in the address.  Getting this backwards means every reply
	// on a multi-motor bus is attributed to the wrong actuator.
	commType, dataArea2, address := parseArbitrationID(0x11007FFD)
	if commType != TypeReadParam {
		t.Errorf("type: got %d, want 17", commType)
	}
	if uint8(dataArea2&0xff) != 0x7F {
		t.Errorf("motor id: got %#02x, want 0x7F", uint8(dataArea2&0xff))
	}
	if address != 0xFD {
		t.Errorf("host id: got %#02x, want 0xFD", address)
	}
}

func TestParameterReadMatchesTheManualExample(t *testing.T) {
	// "Byte0~1: index, corresponding to loc_kp" — 1E 70 00 00 00 00 00 00.
	data := encodeParamRead(ParamLocKp)
	const want = "1E70000000000000"
	if got := strings.ToUpper(hex.EncodeToString(data)); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
}

func TestParameterReplyMatchesTheManualExample(t *testing.T) {
	// The manual's reply payload: 1E 70 00 00 00 00 F0 41, which is loc_kp
	// = 30.0 as an IEEE-754 single, low byte first.
	data, err := hex.DecodeString("1E7000000000F041")
	if err != nil {
		t.Fatal(err)
	}

	index, value, err := decodeParamReply(data)
	if err != nil {
		t.Fatalf("decodeParamReply: %v", err)
	}
	if index != ParamLocKp {
		t.Errorf("index: got %s, want loc_kp", index)
	}
	if math.Abs(value-30.0) > 1e-6 {
		t.Errorf("value: got %v, want 30", value)
	}
}

func TestParameterWriteIsLittleEndian(t *testing.T) {
	// Parameter frames are little-endian while control frames are big-endian.
	// Mixing them up is the single easiest way to produce a driver that talks
	// convincing nonsense, so the byte order is pinned.
	data, err := encodeParamWrite(ParamLocRef, 1.0)
	if err != nil {
		t.Fatal(err)
	}
	const want = "16700000" + "0000803F" // index 0x7016, then 1.0f
	if got := strings.ToUpper(hex.EncodeToString(data)); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
}

func TestRunModeIsWrittenAsASingleByte(t *testing.T) {
	// run_mode is a uint8, not a float.  Sending it as a float would set the
	// mode to whatever the first byte of the mantissa happens to be.
	data, err := encodeParamWrite(ParamRunMode, float64(RunVelocity))
	if err != nil {
		t.Fatal(err)
	}
	const want = "05700000" + "02000000"
	if got := strings.ToUpper(hex.EncodeToString(data)); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
}

func TestReadOnlyParametersAreRefused(t *testing.T) {
	if _, err := encodeParamWrite(ParamVBus, 48); err == nil {
		t.Error("writing to a read-only parameter should be refused")
	}
}

func TestControlFrameLayout(t *testing.T) {
	// Position at zero, velocity at zero, no gains, no torque should encode
	// as the exact midpoint of every symmetric field.
	id, data := encodeControl(ControlCommand{}, RS01, 0x7F)

	commType, torqueRaw, address := parseArbitrationID(id)
	if commType != TypeControl {
		t.Errorf("type: got %d, want 1", commType)
	}
	if address != 0x7F {
		t.Errorf("address: got %#02x, want 0x7F", address)
	}
	// Zero torque is the midpoint of ±17 Nm: 32768 after rounding.
	if torqueRaw != 32768 {
		t.Errorf("torque field: got %d, want 32768", torqueRaw)
	}

	if got := binary.BigEndian.Uint16(data[0:2]); got != 32768 {
		t.Errorf("angle field: got %d, want 32768", got)
	}
	if got := binary.BigEndian.Uint16(data[2:4]); got != 32768 {
		t.Errorf("velocity field: got %d, want 32768", got)
	}
	// Gains are one-sided, so zero is zero rather than mid-scale.
	if got := binary.BigEndian.Uint16(data[4:6]); got != 0 {
		t.Errorf("kp field: got %d, want 0", got)
	}
	if got := binary.BigEndian.Uint16(data[6:8]); got != 0 {
		t.Errorf("kd field: got %d, want 0", got)
	}
}

func TestControlFrameIsBigEndian(t *testing.T) {
	// A full-scale positive angle must appear as FFFF, not as FFFF read the
	// other way — the distinction only shows up on an asymmetric value.
	_, data := encodeControl(ControlCommand{Angle: PositionRange / 2}, RS01, 1)
	raw := binary.BigEndian.Uint16(data[0:2])

	// +2π of a ±4π range is three quarters of full scale: 0xBFFF.
	if raw < 49000 || raw > 49200 {
		t.Errorf("angle field: got %d, want about 49151", raw)
	}
	// "the high byte is in front": byte 0 must be the most significant one.
	if data[0] != byte(raw>>8) || data[1] != byte(raw) {
		t.Errorf("payload is not big-endian: % X for raw %#04x", data[:2], raw)
	}
}

func TestCodecRoundTripsAcrossEveryModel(t *testing.T) {
	// A round trip through the 16-bit codec should return the original value
	// to within one count of its range.
	for _, model := range []Model{RS00, RS01, RS02, RS03, RS04, RS05, RS06} {
		t.Run(model.Name, func(t *testing.T) {
			cases := []struct {
				name  string
				value float64
				limit float64
			}{
				{"zero torque", 0, model.TorqueRange},
				{"full torque", model.TorqueRange, model.TorqueRange},
				{"reverse torque", -model.TorqueRange, model.TorqueRange},
				{"half torque", model.TorqueRange / 2, model.TorqueRange},
				{"velocity", model.VelocityRange / 3, model.VelocityRange},
				{"position", PositionRange / 2, PositionRange},
			}
			for _, tc := range cases {
				raw := encodeUint16(tc.value, tc.limit)
				back := decodeUint16(raw, tc.limit)
				tolerance := 2 * tc.limit / 65535
				if math.Abs(back-tc.value) > tolerance {
					t.Errorf("%s: %v round-tripped to %v (tolerance %v)",
						tc.name, tc.value, back, tolerance)
				}
			}
		})
	}
}

func TestCodecClampsRatherThanWrapping(t *testing.T) {
	// Commanding more than the codec can express must saturate.  Wrapping
	// would turn "more torque than possible" into full reverse, which is the
	// worst possible failure mode for this particular field.
	if got := encodeUint16(1000, RS01.TorqueRange); got != 65535 {
		t.Errorf("over-range: got %d, want 65535", got)
	}
	if got := encodeUint16(-1000, RS01.TorqueRange); got != 0 {
		t.Errorf("under-range: got %d, want 0", got)
	}
	if got := encodeGain(1e9, RS01.KpMax); got != 65535 {
		t.Errorf("over-range gain: got %d, want 65535", got)
	}
	if got := encodeGain(-5, RS01.KpMax); got != 0 {
		t.Errorf("negative gain: got %d, want 0", got)
	}
}

func TestModelScalingIsDistinct(t *testing.T) {
	// The same wire value means very different torques on different models.
	// This is the reason the model is a required setting, so it is worth a
	// test that fails loudly if the table is ever flattened.
	const raw = 65535 // full scale

	rs01 := decodeUint16(raw, RS01.TorqueRange)
	rs04 := decodeUint16(raw, RS04.TorqueRange)

	if math.Abs(rs01-17) > 0.01 {
		t.Errorf("RS01 full scale: got %v Nm, want 17", rs01)
	}
	if math.Abs(rs04-120) > 0.01 {
		t.Errorf("RS04 full scale: got %v Nm, want 120", rs04)
	}
	if rs04/rs01 < 6 {
		t.Error("the model table has lost its per-model distinction")
	}
}

func TestFeedbackDecoding(t *testing.T) {
	// Motor 0x7F, no faults, run mode, with mid-scale angle/velocity/torque
	// and 31.5 °C.
	id := uint32(TypeFeedback)<<24 | uint32(ModeRun)<<22 | 0x7F<<8 | 0xFD

	data := make([]byte, 8)
	binary.BigEndian.PutUint16(data[0:2], 32768)
	binary.BigEndian.PutUint16(data[2:4], 32768)
	binary.BigEndian.PutUint16(data[4:6], 32768)
	binary.BigEndian.PutUint16(data[6:8], 315)

	feedback, err := decodeFeedback(id, data, RS01)
	if err != nil {
		t.Fatalf("decodeFeedback: %v", err)
	}
	if feedback.MotorID != 0x7F {
		t.Errorf("motor id: got %#02x, want 0x7F", feedback.MotorID)
	}
	if feedback.Mode != ModeRun {
		t.Errorf("mode: got %v, want run", feedback.Mode)
	}
	if feedback.Faulted() {
		t.Error("no fault bits were set, but the frame decoded as faulted")
	}
	if math.Abs(feedback.Angle) > 1e-3 {
		t.Errorf("angle: got %v, want about 0", feedback.Angle)
	}
	if math.Abs(feedback.Temperature-31.5) > 1e-6 {
		t.Errorf("temperature: got %v, want 31.5", feedback.Temperature)
	}
}

func TestFeedbackDecodesNegativeTemperature(t *testing.T) {
	// A sub-zero ambient is reported as a negative fixed-point value.  Reading
	// it unsigned would produce several thousand degrees and trip a thermal
	// guard on a cold morning.
	id := uint32(TypeFeedback)<<24 | 0x01<<8 | 0xFD
	data := make([]byte, 8)
	binary.BigEndian.PutUint16(data[0:2], 32768)
	binary.BigEndian.PutUint16(data[2:4], 32768)
	binary.BigEndian.PutUint16(data[4:6], 32768)
	belowZero := int16(-105) // -10.5 °C, in tenths of a degree
	binary.BigEndian.PutUint16(data[6:8], uint16(belowZero))

	feedback, err := decodeFeedback(id, data, RS01)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(feedback.Temperature+10.5) > 1e-6 {
		t.Errorf("temperature: got %v, want -10.5", feedback.Temperature)
	}
}

func TestFeedbackFaultBits(t *testing.T) {
	// Identifier bits 21..16 carry the summary flags, and bits 23..22 the
	// mode.  An off-by-one here would report an over-temperature as an
	// encoder fault, or lose the mode entirely.
	cases := []struct {
		bit  uint32
		want string
	}{
		{16, "Under voltage"},
		{17, "Three-phase over current"},
		{18, "Over temperature"},
		{19, "Magnetic encoder fault"},
		{20, "Stall overload protection"},
		{21, "Uncalibrated"},
	}
	for _, tc := range cases {
		id := uint32(TypeFeedback)<<24 | 1<<tc.bit | 0x01<<8 | 0xFD
		feedback, err := decodeFeedback(id, make([]byte, 8), RS01)
		if err != nil {
			t.Fatal(err)
		}
		if !feedback.Faulted() {
			t.Errorf("bit %d: not reported as a fault", tc.bit)
			continue
		}
		if name, _ := feedback.FaultText(); name != tc.want {
			t.Errorf("bit %d: got %q, want %q", tc.bit, name, tc.want)
		}
	}
}

func TestFeedbackModeBits(t *testing.T) {
	for _, mode := range []Mode{ModeReset, ModeCalibration, ModeRun} {
		id := uint32(TypeFeedback)<<24 | uint32(mode)<<22 | 0x01<<8 | 0xFD
		feedback, err := decodeFeedback(id, make([]byte, 8), RS01)
		if err != nil {
			t.Fatal(err)
		}
		if feedback.Mode != mode {
			t.Errorf("mode %v decoded as %v", mode, feedback.Mode)
		}
	}
}

func TestFaultReportDecoding(t *testing.T) {
	// Bit 0 of the fault word is motor over-temperature; bit 2 is
	// under-voltage.  The report names the most significant one present.
	data := make([]byte, 8)
	binary.LittleEndian.PutUint32(data[0:4], 1<<2)

	report, err := decodeFaultReport(data)
	if err != nil {
		t.Fatal(err)
	}
	if name, _ := report.Text(); name != "Under voltage" {
		t.Errorf("got %q, want %q", name, "Under voltage")
	}
}

func TestLookupModelTolerantOfSpelling(t *testing.T) {
	for _, name := range []string{"RS03", "rs03", "RS-03", "rs_03", "03", "3", " RS03 "} {
		model, err := LookupModel(name)
		if err != nil {
			t.Errorf("%q: %v", name, err)
			continue
		}
		if model.Name != "RS03" {
			t.Errorf("%q resolved to %s, want RS03", name, model.Name)
		}
	}
	if _, err := LookupModel("RS09"); err == nil {
		t.Error("an unknown model should be rejected")
	}
}

func TestEveryModelIsRegistered(t *testing.T) {
	names := ModelNames()
	if len(names) != 7 {
		t.Fatalf("got %d models, want 7: %v", len(names), names)
	}
	for _, want := range []string{"RS00", "RS01", "RS02", "RS03", "RS04", "RS05", "RS06"} {
		if _, err := LookupModel(want); err != nil {
			t.Errorf("%s is missing from the table", want)
		}
	}
}

func TestGainRangesSplitByModelFamily(t *testing.T) {
	// RS00/01/02/05 use Kp 0..500 and Kd 0..5; RS03/04/06 use 0..5000 and
	// 0..100.  A gain that is right for one family is off by a factor of ten
	// or twenty on the other.
	small := []Model{RS00, RS01, RS02, RS05}
	large := []Model{RS03, RS04, RS06}

	for _, model := range small {
		if model.KpMax != 500 || model.KdMax != 5 {
			t.Errorf("%s: got Kp max %v, Kd max %v; want 500 and 5",
				model.Name, model.KpMax, model.KdMax)
		}
	}
	for _, model := range large {
		if model.KpMax != 5000 || model.KdMax != 100 {
			t.Errorf("%s: got Kp max %v, Kd max %v; want 5000 and 100",
				model.Name, model.KpMax, model.KdMax)
		}
	}
}

func TestDefaultGainsStayInsideTheirRange(t *testing.T) {
	for _, model := range []Model{RS00, RS01, RS02, RS03, RS04, RS05, RS06} {
		if model.DefaultKp <= 0 || model.DefaultKp > model.KpMax {
			t.Errorf("%s: default Kp %v is outside 0..%v", model.Name, model.DefaultKp, model.KpMax)
		}
		if model.DefaultKd <= 0 || model.DefaultKd > model.KdMax {
			t.Errorf("%s: default Kd %v is outside 0..%v", model.Name, model.DefaultKd, model.KdMax)
		}
	}
}

func TestWrapAngleFoldsIntoTheProtocolWindow(t *testing.T) {
	// The position field can only express ±4π, so a target beyond it has to be
	// folded rather than clamped — clamping would stop a multi-turn axis dead
	// at four revolutions.
	cases := []struct{ in, want float64 }{
		{0, 0},
		{PositionRange / 2, PositionRange / 2},
		{PositionRange + 1, -PositionRange + 1},
		{-PositionRange - 1, PositionRange - 1},
	}
	for _, tc := range cases {
		if got := wrapAngle(tc.in); math.Abs(got-tc.want) > 1e-9 {
			t.Errorf("wrapAngle(%v): got %v, want %v", tc.in, got, tc.want)
		}
	}
	for _, angle := range []float64{-100, -13, 0, 13, 100, 1000} {
		got := wrapAngle(angle)
		if got < -PositionRange-1e-9 || got > PositionRange+1e-9 {
			t.Errorf("wrapAngle(%v) = %v, outside ±4π", angle, got)
		}
	}
}
