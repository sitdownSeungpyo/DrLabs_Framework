package moteus

import (
	"encoding/hex"
	"math"
	"strings"
	"testing"
)

func TestVaruintRoundTrip(t *testing.T) {
	// The boundaries are where a varuint gains a byte; the register numbers
	// that straddle them (0x7f/0x80, 0x100) are all real moteus registers.
	values := []uint32{0, 1, 0x7f, 0x80, 0xff, 0x100, 0x131, 0x3fff, 0x4000, 0xffffffff}

	for _, want := range values {
		w := &writer{}
		w.writeVaruint(want)

		r := &reader{data: w.bytes()}
		got, err := r.readVaruint()
		if err != nil {
			t.Fatalf("varuint %d: %v", want, err)
		}
		if got != want {
			t.Errorf("varuint round trip: got %d, want %d", got, want)
		}
		if r.remaining() != 0 {
			t.Errorf("varuint %d: %d bytes left unconsumed", want, r.remaining())
		}
	}
}

func TestVaruintEncoding(t *testing.T) {
	// Pinned against the protocol definition: seven bits per byte, little end
	// first, continuation bit on every byte but the last.
	cases := map[uint32]string{
		0x00:  "00",
		0x7f:  "7F",
		0x80:  "8001",
		0x131: "B102",
	}
	for value, want := range cases {
		w := &writer{}
		w.writeVaruint(value)
		got := strings.ToUpper(hex.EncodeToString(w.bytes()))
		if got != want {
			t.Errorf("varuint 0x%x: got %s, want %s", value, got, want)
		}
	}
}

func TestSaturateEncodesNonFiniteAsTheNaNSentinel(t *testing.T) {
	// The negative-most value of each integer width is the protocol's NaN.
	cases := []struct {
		res  Resolution
		want float64
	}{
		{Int8, -128},
		{Int16, -32768},
		{Int32, -2147483648},
	}
	for _, tc := range cases {
		for _, input := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
			if got := saturate(input, tc.res, 0.001); got != tc.want {
				t.Errorf("saturate(%v, %v): got %v, want %v", input, tc.res, got, tc.want)
			}
		}
	}
}

func TestSaturateClampsRatherThanWrapping(t *testing.T) {
	// A value beyond the range has to pin to the maximum.  Letting it wrap
	// would turn "as fast as possible" into "full speed backwards".
	if got := saturate(1e9, Int16, 0.001); got != 32767 {
		t.Errorf("positive overflow: got %v, want 32767", got)
	}
	if got := saturate(-1e9, Int16, 0.001); got != -32767 {
		t.Errorf("negative overflow: got %v, want -32767", got)
	}
}

func TestSaturateTruncatesTowardsZero(t *testing.T) {
	// 0.25 / 0.00001 is 24999.999… in binary floating point.  The reference
	// implementation truncates, so rounding here would put every position
	// command one count away from what the reference produces.
	if got := saturate(0.25, Int32, 0.00001); got != 24999 {
		t.Errorf("got %v, want 24999", got)
	}
}

func TestNanifyRecognisesTheSentinel(t *testing.T) {
	if !math.IsNaN(nanify(-128, Int8)) {
		t.Error("int8 -128 should decode as NaN")
	}
	if !math.IsNaN(nanify(-32768, Int16)) {
		t.Error("int16 -32768 should decode as NaN")
	}
	if !math.IsNaN(nanify(-2147483648, Int32)) {
		t.Error("int32 -2147483648 should decode as NaN")
	}
	// One count away is an ordinary measurement, not a sentinel.
	if got := nanify(-127, Int8); got != -127 {
		t.Errorf("int8 -127 should stay -127, got %v", got)
	}
}

func TestCombinerGroupsConsecutiveRegisters(t *testing.T) {
	// Four registers of the same width should cost one framing byte plus a
	// count and a register number — not four separate subframes.  This is the
	// whole reason a command and its query fit in one CAN-FD frame.
	w := &writer{}
	c := newCombiner(w, writeBase, 0x20, []Resolution{Int32, Int32, Int32, Int32})
	for i := 0; i < c.size(); i++ {
		if !c.maybeWrite() {
			t.Fatalf("register %d should have been written", i)
		}
		w.writeInt32(0)
	}

	// 0x08 (write int32, long form) + count 4 + register 0x20 + 4x4 bytes.
	const want = "0804" + "20" + "00000000000000000000000000000000"
	if got := strings.ToUpper(hex.EncodeToString(w.bytes())); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
	if c.replySize != 3+16 {
		t.Errorf("reply size: got %d, want %d", c.replySize, 3+16)
	}
}

func TestCombinerUsesShortFormForRunsOfThreeOrFewer(t *testing.T) {
	w := &writer{}
	c := newCombiner(w, writeBase, 0x22, []Resolution{Int16, Int16, Int16})
	for i := 0; i < c.size(); i++ {
		c.maybeWrite()
		w.writeInt16(0)
	}
	// 0x04|3 = 0x07, register 0x22, then three 16-bit zeros.
	const want = "0722" + "000000000000"
	if got := strings.ToUpper(hex.EncodeToString(w.bytes())); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
}

func TestCombinerSkipsIgnoredRegisters(t *testing.T) {
	w := &writer{}
	c := newCombiner(w, writeBase, 0x20, []Resolution{Int32, Ignore, Int32})

	if !c.maybeWrite() {
		t.Fatal("register 0 should be written")
	}
	w.writeInt32(1)
	if c.maybeWrite() {
		t.Fatal("an ignored register must not be written")
	}
	if !c.maybeWrite() {
		t.Fatal("register 2 should be written")
	}
	w.writeInt32(2)

	// Two separate single-register subframes, at 0x20 and 0x22.
	const want = "0920" + "01000000" + "0922" + "02000000"
	if got := strings.ToUpper(hex.EncodeToString(w.bytes())); got != want {
		t.Errorf("got %s, want %s", got, want)
	}
}

func TestParseReplyDecodesRegisterValues(t *testing.T) {
	// A reply the way a controller sends one: mode as int8 at 0x000, then
	// position and velocity as int32 at 0x001.
	frame, err := hex.DecodeString("2100" + "0A" + "2A01" + "A7610000" + "00000000")
	if err != nil {
		t.Fatal(err)
	}

	values, err := parseReply(frame)
	if err != nil {
		t.Fatalf("parseReply: %v", err)
	}

	if got := values[RegMode].raw; got != 10 {
		t.Errorf("mode: got %v, want 10", got)
	}
	if got := scaleRegister(RegPosition, values[RegPosition]); math.Abs(got-0.24999) > 1e-9 {
		t.Errorf("position: got %v, want 0.24999", got)
	}
	if got := scaleRegister(RegVelocity, values[RegVelocity]); got != 0 {
		t.Errorf("velocity: got %v, want 0", got)
	}
}

func TestParseReplyIgnoresPadding(t *testing.T) {
	// CAN-FD has no byte-granular length above 8, so a short payload is
	// padded to the next valid size with NOP bytes.  Those must be skipped,
	// not decoded as registers.
	frame, err := hex.DecodeString("2100" + "0A" + "50505050")
	if err != nil {
		t.Fatal(err)
	}
	values, err := parseReply(frame)
	if err != nil {
		t.Fatalf("parseReply: %v", err)
	}
	if len(values) != 1 {
		t.Errorf("got %d registers, want 1", len(values))
	}
}

func TestParseReplyReportsControllerErrors(t *testing.T) {
	// 0x31 is a read error: the controller refused a register.  Reporting it
	// matters — silently returning no value looks identical to a timeout, and
	// the two call for completely different responses.
	frame, err := hex.DecodeString("31" + "20" + "05")
	if err != nil {
		t.Fatal(err)
	}
	if _, err := parseReply(frame); err == nil {
		t.Error("a read-error subframe should produce an error")
	}
}

func TestParseReplyReturnsPartialDataOnTruncation(t *testing.T) {
	// A frame cut short still carries whatever was decoded before the break.
	// Losing the tail of a telemetry frame should not blind the safety
	// monitor to the mode and fault registers at its head.
	frame, err := hex.DecodeString("2100" + "0A" + "2A01" + "A761")
	if err != nil {
		t.Fatal(err)
	}
	values, err := parseReply(frame)
	if err == nil {
		t.Error("a truncated frame should report an error")
	}
	if _, ok := values[RegMode]; !ok {
		t.Error("the registers decoded before the truncation should be returned")
	}
}

func TestDecodeProducesFrameworkUnits(t *testing.T) {
	values := map[Register]regValue{
		RegMode:        {res: Int8, raw: 10},
		RegFault:       {res: Int8, raw: 0},
		RegPosition:    {res: Int32, raw: 25000},
		RegVelocity:    {res: Int32, raw: -50000},
		RegTorque:      {res: Int16, raw: 25},
		RegQCurrent:    {res: Int16, raw: 15},
		RegVoltage:     {res: Int16, raw: 240},
		RegTemperature: {res: Int16, raw: 355},
	}

	telemetry := decode(values)
	if telemetry == nil {
		t.Fatal("decode returned nil")
	}

	checks := []struct {
		name string
		got  float64
		want float64
	}{
		{"position (rev)", telemetry.Position, 0.25},
		{"velocity (rev/s)", telemetry.Velocity, -0.5},
		{"torque (Nm)", telemetry.Torque, 0.25},
		{"current (A)", telemetry.QCurrent, 1.5},
		{"voltage (V)", telemetry.Voltage, 24.0},
		{"temperature (°C)", telemetry.Temperature, 35.5},
	}
	for _, check := range checks {
		if math.Abs(check.got-check.want) > 1e-9 {
			t.Errorf("%s: got %v, want %v", check.name, check.got, check.want)
		}
	}
	if telemetry.ModeName != "Position" {
		t.Errorf("mode name: got %q, want %q", telemetry.ModeName, "Position")
	}
}

func TestDecodeMarksUnreportedFieldsUnmeasured(t *testing.T) {
	// A fast-profile reply carries no motor temperature.  It has to come back
	// as "not measured" rather than as zero, or a thermal trip would read an
	// absent sensor as a cold motor.
	telemetry := decode(map[Register]regValue{RegMode: {res: Int8, raw: 0}})
	if !math.IsNaN(telemetry.MotorTemperature) {
		t.Errorf("unreported motor temperature should be NaN, got %v", telemetry.MotorTemperature)
	}
	if !math.IsNaN(telemetry.Position) {
		t.Errorf("unreported position should be NaN, got %v", telemetry.Position)
	}
}

func TestHardFaultDistinguishesLimitingStates(t *testing.T) {
	// Codes at and above 96 say "the output is being limited", which is
	// information, not a reason to stop the machine.
	if !IsHardFault(39) {
		t.Error("fault 39 (outside limit) is a hard fault")
	}
	if IsHardFault(102) {
		t.Error("code 102 (limiting: max torque) is not a hard fault")
	}
	if IsHardFault(0) {
		t.Error("code 0 is no fault at all")
	}
	if FaultText(102).Name == "" {
		t.Error("limiting codes should have a name")
	}
}
