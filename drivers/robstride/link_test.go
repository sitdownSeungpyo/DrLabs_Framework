package robstride

import (
	"context"
	"encoding/binary"
	"math"
	"testing"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
)

func testConfig(options map[string]string) motor.Config {
	if options == nil {
		options = map[string]string{}
	}
	if _, ok := options["model"]; !ok {
		options["model"] = "RS01"
	}
	return motor.Config{Driver: "robstride", ID: 1, Options: options}
}

func TestModelOptionIsRequired(t *testing.T) {
	// The scaling differs by up to a factor of twenty across the family, so a
	// default would mean commanding an unknown multiple of the intended
	// torque.  Refusing is the only safe behaviour.
	_, err := newLink(motor.Config{Driver: "robstride", ID: 1})
	if err == nil {
		t.Fatal("a missing model should be rejected")
	}
	if _, err := newLink(testConfig(map[string]string{"model": "RS99"})); err == nil {
		t.Error("an unknown model should be rejected")
	}
}

func TestConfigRejectsUnknownOptionsAndBadIDs(t *testing.T) {
	if _, err := newLink(testConfig(map[string]string{"kP": "20"})); err == nil {
		t.Error("a mis-spelled option should be rejected, not ignored")
	}
	if _, err := newLink(motor.Config{Driver: "robstride", ID: 0,
		Options: map[string]string{"model": "RS01"}}); err == nil {
		t.Error("CAN id 0 should be rejected")
	}
	if _, err := newLink(motor.Config{Driver: "robstride", ID: 200,
		Options: map[string]string{"model": "RS01"}}); err == nil {
		t.Error("a CAN id above 127 should be rejected")
	}
}

func TestGainOptionsAreRangeChecked(t *testing.T) {
	// A Kp beyond the model's field cannot be transmitted, so accepting it
	// would silently saturate at a stiffness the caller did not ask for.
	if _, err := newLink(testConfig(map[string]string{"model": "RS01", "kp": "9000"})); err == nil {
		t.Error("a Kp above the model's maximum should be rejected")
	}
	// The same value is legal on the large-gain family.
	if _, err := newLink(testConfig(map[string]string{"model": "RS03", "kp": "900"})); err != nil {
		t.Errorf("RS03 should accept Kp 900: %v", err)
	}
}

func TestCapabilitiesReportWhatIsMissing(t *testing.T) {
	link, err := newLink(testConfig(nil))
	if err != nil {
		t.Fatal(err)
	}
	caps := link.Capabilities()

	// There is no phase-short brake mode and no bounded-region mode.  Claiming
	// them and approximating would give a caller a machine that coasts further
	// than it was told to.
	if caps.Supports(motor.CtrlBrake) {
		t.Error("RobStride has no brake mode and should not claim one")
	}
	if caps.Supports(motor.CtrlStayWithin) {
		t.Error("RobStride has no stay-within mode and should not claim one")
	}
	for _, mode := range []motor.ControlMode{
		motor.CtrlStop, motor.CtrlQuery, motor.CtrlPosition,
		motor.CtrlVelocity, motor.CtrlTorque, motor.CtrlHold,
	} {
		if !caps.Supports(mode) {
			t.Errorf("%s should be supported", mode)
		}
	}
}

func TestEnvelopeUsesTheModelsRatings(t *testing.T) {
	link, err := newLink(testConfig(map[string]string{"model": "RS04"}))
	if err != nil {
		t.Fatal(err)
	}
	envelope := link.(motor.Enveloped).Envelope()

	if envelope.PeakCurrent != RS04.CurrentRefMax {
		t.Errorf("peak current: got %v, want %v", envelope.PeakCurrent, RS04.CurrentRefMax)
	}
	// The framework speaks rev/s; the manual speaks rad/s.
	want := RS04.VelocityRange / (2 * math.Pi)
	if math.Abs(envelope.VelocityMax-want) > 1e-9 {
		t.Errorf("velocity: got %v rev/s, want %v", envelope.VelocityMax, want)
	}
	// The supply range comes from the manual, and the rated point with it —
	// the rated value is what lets a host-side voltage trip band tell whether
	// it is plausible for this hardware at all.
	if envelope.VoltageMin != 24 || envelope.VoltageMax != 60 {
		t.Errorf("voltage range: got [%v, %v], want [24, 60]", envelope.VoltageMin, envelope.VoltageMax)
	}
	if envelope.VoltageRated != 48 {
		t.Errorf("rated voltage: got %v, want 48", envelope.VoltageRated)
	}
}

// --- Link behaviour against a fake bus -----------------------------------

// linkOnFakeBus wires a Link to a scripted bus, bypassing Open.
func linkOnFakeBus(t *testing.T, model Model) (*Link, *fakeBus) {
	t.Helper()
	bus := &fakeBus{}
	link := NewLinkWithBus(bus, 0x7F, model)
	link.controller.HostID = 0xFD
	link.controller.Timeout = 50 * time.Millisecond
	return link, bus
}

// replyN queues n identical feedback frames.
func replyN(bus *fakeBus, n int, model Model) {
	for i := 0; i < n; i++ {
		bus.replies = append(bus.replies, feedbackFrame(0x7F, 0xFD, ModeRun, 0, 0, 0, 0, 25, model))
	}
}

func TestQueryDoesNotDriveTheMotor(t *testing.T) {
	// A disarmed session polls every cycle.  Whatever it sends must leave the
	// output released, or "watching the axis" would quietly energise it.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 1, RS01)

	if _, err := link.Cycle(context.Background(),
		motor.NewCommand(motor.CtrlQuery), motor.ProfileFast); err != nil {
		t.Fatalf("Cycle: %v", err)
	}
	if len(bus.sent) != 1 {
		t.Fatalf("sent %d frames, want 1", len(bus.sent))
	}
	commType, _, _ := parseArbitrationID(bus.sent[0].ID)
	if commType != TypeStop {
		t.Errorf("a query sent communication type %d; want %d (stop), which releases the output",
			commType, TypeStop)
	}
}

func TestPositionCycleIsOneFrame(t *testing.T) {
	// The steady-state cost of position control has to be a single
	// transaction, or the loop cannot keep its period.
	for _, tc := range []struct {
		name     string
		mode     RunMode
		wantType CommunicationType
	}{
		{"csp", RunPositionCSP, TypeWriteParam},
		{"operation", RunOperation, TypeControl},
	} {
		t.Run(tc.name, func(t *testing.T) {
			link, bus := linkOnFakeBus(t, RS01)
			link.positionMode = tc.mode
			replyN(bus, 12, RS01)

			cmd := motor.NewCommand(motor.CtrlPosition)
			cmd.Position = 0.1
			cmd.MaxTorque = 2.0
			cmd.VelocityLimit = 1.0

			// The first cycle sets the run mode, enables, and writes limits.
			if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
				t.Fatalf("first cycle: %v", err)
			}
			before := len(bus.sent)

			// The second, with nothing changed, should cost one frame.
			if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
				t.Fatalf("second cycle: %v", err)
			}
			if sent := len(bus.sent) - before; sent != 1 {
				t.Errorf("steady-state cycle sent %d frames, want 1", sent)
			}

			commType, _, _ := parseArbitrationID(bus.sent[len(bus.sent)-1].ID)
			if commType != tc.wantType {
				t.Errorf("steady-state frame was type %d, want %d", commType, tc.wantType)
			}
		})
	}
}

func TestCSPPositionHonoursTheVelocityLimit(t *testing.T) {
	// The framework's CtrlPosition promises "drive here, respecting the
	// velocity limit".  Only the controller's own profile generator can
	// deliver that, so the limit has to reach it as limit_spd.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 12, RS01)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0.25
	cmd.VelocityLimit = 1.0 // rev/s

	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}

	var sawRunMode, sawLimit, sawTarget bool
	for _, frame := range bus.sent {
		if commType, _, _ := parseArbitrationID(frame.ID); commType != TypeWriteParam {
			continue
		}
		index, value, err := decodeParamReply(frame.Data)
		if err != nil {
			continue
		}
		switch index {
		case ParamRunMode:
			sawRunMode = true
			if RunMode(value) != RunPositionCSP {
				t.Errorf("run mode: got %v, want position-csp", RunMode(value))
			}
		case ParamLimitSpd:
			sawLimit = true
			if math.Abs(value-2*math.Pi) > 1e-4 {
				t.Errorf("limit_spd: got %v rad/s, want %v", value, 2*math.Pi)
			}
		case ParamLocRef:
			sawTarget = true
			if math.Abs(value-math.Pi/2) > 1e-4 {
				t.Errorf("loc_ref: got %v rad, want %v", value, math.Pi/2)
			}
		}
	}
	if !sawRunMode {
		t.Error("the CSP run mode was never selected")
	}
	if !sawLimit {
		t.Error("the velocity limit never reached the controller")
	}
	if !sawTarget {
		t.Error("no position target was written")
	}
}

func TestOperationModeAppliesTheGains(t *testing.T) {
	// The impedance path has to send the gains, or a caller asking for a soft
	// axis gets whatever stiffness was last in force.
	link, bus := linkOnFakeBus(t, RS01)
	link.positionMode = RunOperation
	replyN(bus, 8, RS01)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0
	cmd.KpScale = 0.5
	cmd.KdScale = 1.0

	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}

	last := bus.sent[len(bus.sent)-1]
	if commType, _, _ := parseArbitrationID(last.ID); commType != TypeControl {
		t.Fatalf("expected an operation control frame, got type %d", commType)
	}

	kpRaw := binary.BigEndian.Uint16(last.Data[4:6])
	kp := float64(kpRaw) / 65535 * RS01.KpMax
	if math.Abs(kp-RS01.DefaultKp*0.5) > 0.1 {
		t.Errorf("Kp: got %v, want %v (half the configured default)", kp, RS01.DefaultKp*0.5)
	}
}

func TestPositionModeOptionIsValidated(t *testing.T) {
	if _, err := newLink(testConfig(map[string]string{"position_mode": "csp"})); err != nil {
		t.Errorf("csp should be accepted: %v", err)
	}
	if _, err := newLink(testConfig(map[string]string{"position_mode": "operation"})); err != nil {
		t.Errorf("operation should be accepted: %v", err)
	}
	if _, err := newLink(testConfig(map[string]string{"position_mode": "profile"})); err == nil {
		t.Error("an unknown position mode should be rejected")
	}
}

func TestTorqueLimitIsWrittenOnChangeOnly(t *testing.T) {
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 12, RS01)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0
	cmd.MaxTorque = 2.0

	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}
	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}
	before := len(bus.sent)

	cmd.MaxTorque = 3.0
	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}
	if sent := len(bus.sent) - before; sent != 2 {
		t.Errorf("a changed torque limit sent %d frames, want 2 (the write and the control frame)", sent)
	}

	// The write must carry the new limit.
	write := bus.sent[len(bus.sent)-2]
	commType, _, _ := parseArbitrationID(write.ID)
	if commType != TypeWriteParam {
		t.Fatalf("expected a parameter write, got type %d", commType)
	}
	index, value, err := decodeParamReply(write.Data)
	if err != nil {
		t.Fatal(err)
	}
	if index != ParamLimitTorque {
		t.Errorf("wrote %s, want limit_torque", index)
	}
	if math.Abs(value-3.0) > 1e-5 {
		t.Errorf("limit: got %v, want 3", value)
	}
}

func TestTorqueLimitIsClampedToTheModel(t *testing.T) {
	// Asking an RS05 for 100 Nm cannot be honoured; it must be reduced rather
	// than sent as a value the parameter cannot hold.
	link, bus := linkOnFakeBus(t, RS05)
	replyN(bus, 6, RS05)

	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0
	cmd.MaxTorque = 100

	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}

	var wrote bool
	for _, frame := range bus.sent {
		if commType, _, _ := parseArbitrationID(frame.ID); commType != TypeWriteParam {
			continue
		}
		index, value, err := decodeParamReply(frame.Data)
		if err != nil || index != ParamLimitTorque {
			continue
		}
		wrote = true
		if math.Abs(value-RS05.TorqueLimitMax) > 1e-5 {
			t.Errorf("limit: got %v, want the model maximum %v", value, RS05.TorqueLimitMax)
		}
	}
	if !wrote {
		t.Error("no torque limit was written")
	}
}

func TestVelocityUsesTheDedicatedRunMode(t *testing.T) {
	// The operation mode's proportional damping term sags under load; the
	// velocity run mode closes a PI loop and holds its setpoint.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 10, RS01)

	cmd := motor.NewCommand(motor.CtrlVelocity)
	cmd.Velocity = 0.5 // rev/s

	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}

	var sawRunMode, sawSetpoint bool
	for _, frame := range bus.sent {
		commType, _, _ := parseArbitrationID(frame.ID)
		if commType != TypeWriteParam {
			continue
		}
		index, value, err := decodeParamReply(frame.Data)
		if err != nil {
			continue
		}
		switch index {
		case ParamRunMode:
			sawRunMode = true
			if RunMode(value) != RunVelocity {
				t.Errorf("run mode: got %v, want velocity", RunMode(value))
			}
		case ParamSpdRef:
			sawSetpoint = true
			// 0.5 rev/s is π rad/s.
			if math.Abs(value-math.Pi) > 1e-4 {
				t.Errorf("spd_ref: got %v rad/s, want %v", value, math.Pi)
			}
		}
	}
	if !sawRunMode {
		t.Error("the velocity run mode was never selected")
	}
	if !sawSetpoint {
		t.Error("no velocity setpoint was written")
	}
}

func TestSwitchingModeStopsTheMotorFirst(t *testing.T) {
	// Handing the output to a different loop while it is driving gives control
	// to a controller whose state was never initialised for this position.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 20, RS01)

	position := motor.NewCommand(motor.CtrlPosition)
	position.Position = 0
	if _, err := link.Cycle(context.Background(), position, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}
	before := len(bus.sent)

	velocity := motor.NewCommand(motor.CtrlVelocity)
	velocity.Velocity = 0.1
	if _, err := link.Cycle(context.Background(), velocity, motor.ProfileFast); err != nil {
		t.Fatal(err)
	}

	var sawStop bool
	for _, frame := range bus.sent[before:] {
		if commType, _, _ := parseArbitrationID(frame.ID); commType == TypeStop {
			sawStop = true
			break
		}
	}
	if !sawStop {
		t.Error("the run mode changed without stopping the motor first")
	}
}

func TestUnsupportedModesAreRefused(t *testing.T) {
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 4, RS01)

	for _, mode := range []motor.ControlMode{motor.CtrlBrake, motor.CtrlStayWithin} {
		if _, err := link.Cycle(context.Background(), motor.NewCommand(mode), motor.ProfileFast); err != motor.ErrUnsupportedMode {
			t.Errorf("%s: got %v, want ErrUnsupportedMode", mode, err)
		}
	}
}

func TestSetOutputOnlyReferencesZero(t *testing.T) {
	// The controller can declare the current position to be zero and nothing
	// else.  Silently ignoring the requested value would leave the axis
	// believing something the caller never asked for.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 4, RS01)

	cmd := motor.NewCommand(motor.CtrlSetOutput)
	cmd.Position = 0.3
	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err == nil {
		t.Error("re-referencing to a non-zero position should be refused")
	}

	cmd.Position = 0
	if _, err := link.Cycle(context.Background(), cmd, motor.ProfileFast); err != nil {
		t.Errorf("re-referencing to zero should be allowed: %v", err)
	}
	commType, _, _ := parseArbitrationID(bus.sent[len(bus.sent)-1].ID)
	if commType != TypeSetZero {
		t.Errorf("got communication type %d, want %d (set mechanical zero)", commType, TypeSetZero)
	}
}

func TestTelemetryIsInFrameworkUnits(t *testing.T) {
	link, bus := linkOnFakeBus(t, RS01)
	// One revolution per second is 2π rad/s; a quarter turn is π/2 rad.
	bus.replies = append(bus.replies,
		feedbackFrame(0x7F, 0xFD, ModeRun, 0, math.Pi/2, 2*math.Pi, 1.5, 31.5, RS01))

	telemetry, err := link.Cycle(context.Background(),
		motor.NewCommand(motor.CtrlQuery), motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}
	if telemetry == nil {
		t.Fatal("no telemetry returned")
	}

	if math.Abs(telemetry.Position-0.25) > 1e-3 {
		t.Errorf("position: got %v rev, want 0.25", telemetry.Position)
	}
	if math.Abs(telemetry.Velocity-1.0) > 1e-3 {
		t.Errorf("velocity: got %v rev/s, want 1.0", telemetry.Velocity)
	}
	if math.Abs(telemetry.Torque-1.5) > 1e-2 {
		t.Errorf("torque: got %v Nm, want 1.5", telemetry.Torque)
	}
	if math.Abs(telemetry.Temperature-31.5) > 1e-6 {
		t.Errorf("temperature: got %v, want 31.5", telemetry.Temperature)
	}
}

func TestFastProfileDoesNotSpendExtraTransactions(t *testing.T) {
	// Voltage and current live in the parameter table, so reading them costs a
	// round trip each.  They belong on the wider profiles, not on every cycle.
	link, bus := linkOnFakeBus(t, RS01)
	replyN(bus, 1, RS01)

	telemetry, err := link.Cycle(context.Background(),
		motor.NewCommand(motor.CtrlQuery), motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}
	if len(bus.sent) != 1 {
		t.Errorf("a fast cycle sent %d frames, want 1", len(bus.sent))
	}
	if motor.Valid(telemetry.Voltage) {
		t.Error("the fast profile should not claim a bus voltage it did not read")
	}
	if motor.Valid(telemetry.QCurrent) {
		t.Error("the fast profile should not claim a current it did not read")
	}
}

func TestPositionUnwrapsBeyondTheProtocolWindow(t *testing.T) {
	// The reported angle wraps at ±4π.  Without turn counting a multi-turn
	// axis appears to jump eight π backwards every four revolutions, and a
	// position envelope checked against that trips on a healthy machine.
	link, bus := linkOnFakeBus(t, RS01)

	bus.replies = append(bus.replies,
		feedbackFrame(0x7F, 0xFD, ModeRun, 0, PositionRange-0.1, 0, 0, 25, RS01),
		// Past the top of the window: the motor reports the wrapped value.
		feedbackFrame(0x7F, 0xFD, ModeRun, 0, -PositionRange+0.1, 0, 0, 25, RS01))

	query := motor.NewCommand(motor.CtrlQuery)
	first, err := link.Cycle(context.Background(), query, motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}
	second, err := link.Cycle(context.Background(), query, motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}

	// The axis moved forward by 0.2 rad, not backwards by 8π.
	delta := motor.RevToRad(second.Position - first.Position)
	if math.Abs(delta-0.2) > 1e-2 {
		t.Errorf("position moved by %v rad across the wrap, want about 0.2", delta)
	}
}

func TestFaultFlagsBecomeAHardFault(t *testing.T) {
	link, bus := linkOnFakeBus(t, RS01)
	bus.replies = append(bus.replies,
		feedbackFrame(0x7F, 0xFD, ModeReset, flagOvertemperature, 0, 0, 0, 120, RS01))

	telemetry, err := link.Cycle(context.Background(),
		motor.NewCommand(motor.CtrlQuery), motor.ProfileFast)
	if err != nil {
		t.Fatal(err)
	}
	if !telemetry.HardFault {
		t.Error("an over-temperature flag should be reported as a hard fault")
	}
	if telemetry.FaultName != "Over temperature" {
		t.Errorf("fault name: got %q", telemetry.FaultName)
	}
	if telemetry.FaultDetail == "" {
		t.Error("a fault should carry an explanation, not just a name")
	}
}

func TestDescribeNamesTheModel(t *testing.T) {
	link, _ := linkOnFakeBus(t, RS03)
	if got := link.Describe()["model"]; got != "RS03" {
		t.Errorf("model: got %q, want RS03", got)
	}
}

var _ can.Bus = (*fakeBus)(nil)
