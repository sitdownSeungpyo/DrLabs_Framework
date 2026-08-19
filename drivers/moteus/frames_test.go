package moteus

import (
	"encoding/hex"
	"math"
	"strings"
	"testing"

	"drlabs/motor"
	"drlabs/transport/can"
)

// The expected byte strings in this file were produced by the reference
// implementation — the official `moteus` Python package — and are pinned here
// verbatim.  They are the only check that this encoder is compatible with real
// firmware rather than merely self-consistent, so they should be regenerated
// against the reference rather than adjusted to match a change here.
//
// Reproduce with:
//
//	c = moteus.Controller(id=1, query_resolution=…, position_resolution=…)
//	print(c.make_position(...).data.hex().upper())
//
// using the resolutions in testQuery and testPositionResolution below.

// testQuery mirrors FastQuery, which is what a control loop sends every cycle.
func testQuery() QueryResolution { return FastQuery() }

func testPositionResolution() PositionResolution { return DefaultPositionResolution() }

func TestFramesMatchReferenceEncoding(t *testing.T) {
	fast := testQuery()
	pr := testPositionResolution()

	// The query suffix that every "…_fastquery" case carries.
	const fastQuerySuffix = "11001A011603160D110F"

	cases := []struct {
		name string
		want string
		got  []byte
	}{
		{
			name: "stop without query",
			want: "010000",
			got:  stopFrame(nil),
		},
		{
			name: "stop with fast query",
			want: "010000" + fastQuerySuffix,
			got:  stopFrame(&fast),
		},
		{
			name: "query only",
			want: fastQuerySuffix,
			got:  queryFrame(&fast),
		},
		{
			name: "brake with fast query",
			want: "01000F" + fastQuerySuffix,
			got:  brakeFrame(&fast),
		},
		{
			// Only the registers a steady-state position cycle needs.
			// Note the position value: 0.25 / 0.00001 is 24999.999… in binary
			// floating point, and the protocol truncates rather than rounds,
			// so the wire carries 24999 (0x61A7).  Rounding here would put us
			// one count away from the reference on almost every command.
			name: "position, minimal parameter set",
			want: "01000A" +
				"0A20" + "A7610000" + "00000000" + // position, velocity
				"0925" + "F4010000" + // maximum torque 0.5 Nm
				"0527" + "2800" + // watchdog 0.04 s
				fastQuerySuffix,
			got: positionFrame(positionCommand{
				Position:        val(0.25),
				Velocity:        val(0.0),
				MaximumTorque:   val(0.5),
				WatchdogTimeout: val(0.04),
			}, pr, &fast),
		},
		{
			name: "position, full parameter set",
			want: "01000A" +
				"0A20" + "2DCFFFFF" + "00000000" +
				"0722" + "0000" + "FF7F" + "FF7F" + // feedforward, kp, kd
				"0925" + "2C010000" + // maximum torque 0.3 Nm
				"0527" + "2800" +
				"0A28" + "F0490200" + "1FA10700" + // velocity limit, accel limit
				"052B" + "FF7F" + // ilimit scale
				"012D" + "00" + // ignore position bounds
				fastQuerySuffix,
			got: positionFrame(positionCommand{
				Position:             val(-0.125),
				Velocity:             val(0.0),
				FeedforwardTorque:    val(0.0),
				KpScale:              val(1.0),
				KdScale:              val(1.0),
				MaximumTorque:        val(0.3),
				WatchdogTimeout:      val(0.04),
				VelocityLimit:        val(1.5),
				AccelLimit:           val(5.0),
				ILimitScale:          val(1.0),
				IgnorePositionBounds: val(0),
			}, pr, &fast),
		},
		{
			// NaN in the maximum torque register hands the limit back to the
			// controller's own configuration.  It must be transmitted as the
			// negative-most int32, not omitted.
			name: "position deferring the torque limit",
			want: "01000A" +
				"0A20" + "10270000" + "00000000" +
				"0925" + "00000080" +
				"0527" + "2800" +
				fastQuerySuffix,
			got: positionFrame(positionCommand{
				Position:        val(0.1),
				Velocity:        val(0.0),
				MaximumTorque:   val(math.NaN()),
				WatchdogTimeout: val(0.04),
			}, pr, &fast),
		},
		{
			// Velocity mode: a NaN position is what says "no position
			// target".  If it were omitted instead, the controller would keep
			// servoing to whatever position was last commanded.
			name: "velocity, position sent as NaN",
			want: "01000A" +
				"0A20" + "00000080" + "F8240100" +
				"0925" + "F4010000" +
				"0527" + "2800" +
				fastQuerySuffix,
			got: positionFrame(positionCommand{
				Position:        val(math.NaN()),
				Velocity:        val(0.75),
				MaximumTorque:   val(0.5),
				WatchdogTimeout: val(0.04),
			}, pr, &fast),
		},
		{
			name: "set output exact",
			want: "0DB102" + "00000000" + fastQuerySuffix,
			got:  setOutputFrame(RegSetOutputExact, 0.0, &fast),
		},
		{
			name: "stay within",
			want: "01000D" +
				"0A40" + "D18AFFFF" + "2F750000" + // lower -0.3, upper 0.3
				"0742" + "0000" + "FF7F" + "FF7F" +
				"0945" + "F4010000" +
				"0646" + "2800" + "FF7F" + // watchdog, ilimit
				"0148" + "00" +
				fastQuerySuffix,
			got: stayWithinFrame(stayWithinCommand{
				LowerBound:           val(-0.3),
				UpperBound:           val(0.3),
				FeedforwardTorque:    val(0.0),
				KpScale:              val(1.0),
				KdScale:              val(1.0),
				MaximumTorque:        val(0.5),
				WatchdogTimeout:      val(0.04),
				ILimitScale:          val(1.0),
				IgnorePositionBounds: val(0),
			}, pr, &fast),
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := strings.ToUpper(hex.EncodeToString(tc.got))
			if got != tc.want {
				t.Errorf("frame does not match the reference encoding\n got: %s\nwant: %s", got, tc.want)
			}
		})
	}
}

func TestVerifyQueryMatchesReferenceEncoding(t *testing.T) {
	// The verify profile appends reads of the controller's active command
	// registers: max torque and timeout as int16, velocity and accel limits
	// as int32.  Registers 0x25..0x29, with 0x26 skipped.
	const want = "11001A011603160D110F" + "152515271A28"

	verify := VerifyQuery()
	got := strings.ToUpper(hex.EncodeToString(queryFrame(&verify)))
	if got != want {
		t.Errorf("verify query does not match the reference encoding\n got: %s\nwant: %s", got, want)
	}
}

func TestOmittedFieldsAreNotTransmitted(t *testing.T) {
	pr := testPositionResolution()

	// A command that mentions only the position should carry only the mode
	// prologue and that one register.  This is what makes a per-cycle frame
	// small enough to share with a telemetry query.
	frame := positionFrame(positionCommand{Position: val(0.0)}, pr, nil)

	const want = "01000A" + "0920" + "00000000"
	got := strings.ToUpper(hex.EncodeToString(frame))
	if got != want {
		t.Errorf("unset fields leaked into the frame\n got: %s\nwant: %s", got, want)
	}
}

func TestPresentTreatsNaNAsAbsent(t *testing.T) {
	// present() is for the fields where NaN really does mean "no opinion".
	if present(math.NaN()).send {
		t.Error("present(NaN) should omit the field")
	}
	if !present(0).send {
		t.Error("present(0) should transmit the field")
	}
	// val() transmits whatever it is given, NaN included.
	if !val(math.NaN()).send {
		t.Error("val(NaN) should transmit the NaN sentinel")
	}
}

// newTestLink builds a Link the way newLink does, so the tests exercise the
// same assembly the driver uses rather than a hand-made stand-in.
func newTestLink() *Link {
	return &Link{
		fast:   FastQuery(),
		full:   FullQuery(),
		verify: VerifyQuery(),
		controller: &Controller{
			ID:                 1,
			PositionResolution: DefaultPositionResolution(),
		},
	}
}

// fullCycleCommand is what the session sends every period: every parameter
// populated, exactly as buildCommandLocked fills it in.
func fullCycleCommand() motor.Command {
	cmd := motor.NewCommand(motor.CtrlPosition)
	cmd.Position = 0.25
	cmd.Velocity = 0.1
	cmd.MaxTorque = 0.4
	cmd.WatchdogTimeout = 0.04
	cmd.VelocityLimit = 1.5
	cmd.AccelLimit = 5.0
	cmd.KpScale = 1.0
	cmd.KdScale = 1.0
	cmd.ILimitScale = 1.0
	cmd.FeedforwardTorque = 0.0
	return cmd
}

func TestCycleFramesFitInOneCANFDFrame(t *testing.T) {
	// The failure this guards against is not subtle once it happens — the
	// adapter refuses the frame and the link dies — but it only happens on the
	// cycles where a wide query lands on a full parameter set, which is one in
	// ten.  A frame that is 66 bytes on real hardware and never checked here
	// is exactly how that reached a bench.
	link := newTestLink()

	profiles := []struct {
		name  string
		query *QueryResolution
	}{
		{"fast", &link.fast},
		{"full", &link.full},
		{"verify", &link.verify},
	}

	for _, refresh := range []bool{true, false} {
		label := "steady"
		if refresh {
			label = "full refresh"
		}
		for _, profile := range profiles {
			t.Run(label+"/"+profile.name, func(t *testing.T) {
				// A fresh Link so the sticky state starts empty, then one
				// priming cycle so the steady case really is steady.
				subject := newTestLink()
				priming := fullCycleCommand()
				priming.Full = true
				_ = positionFrame(subject.position(priming), DefaultPositionResolution(), &subject.fast)

				cmd := fullCycleCommand()
				cmd.Full = refresh

				// Assembled the way Cycle does it, so the budget logic is part
				// of what is under test rather than bypassed by it.
				payload, err := subject.commandPayload(cmd)
				if err != nil {
					t.Fatal(err)
				}
				query := subject.fitQuery(len(payload), profile.query)
				if query != nil {
					data, _ := queryData(*query)
					payload = append(payload, data...)
				}

				if len(payload) > can.MaxFDPayload {
					t.Errorf("frame is %d bytes, over the %d byte CAN-FD limit",
						len(payload), can.MaxFDPayload)
				}
				if query == nil {
					t.Errorf("no query fitted beside a %d byte command; the cycle would return no telemetry",
						len(payload))
				}
			})
		}
	}
}

func TestSteadyCycleOmitsUnchangedRegisters(t *testing.T) {
	// Command registers keep their value until they are written again, so a
	// parameter that has not moved should cost nothing.  This is what keeps a
	// per-cycle frame small enough to share with a telemetry query.
	link := newTestLink()

	first := fullCycleCommand()
	first.Full = true
	full := positionFrame(link.position(first), DefaultPositionResolution(), &link.fast)

	steady := positionFrame(link.position(fullCycleCommand()), DefaultPositionResolution(), &link.fast)

	if len(steady) >= len(full) {
		t.Errorf("steady cycle is %d bytes against a full refresh of %d; nothing was made sticky",
			len(steady), len(full))
	}
	// Position, velocity, torque limit and watchdog still have to go out every
	// cycle — they are the command and the limits that protect it.
	if len(steady) < 20 {
		t.Errorf("steady cycle is only %d bytes; the per-cycle command looks incomplete", len(steady))
	}
	t.Logf("full refresh %d bytes, steady cycle %d bytes", len(full), len(steady))
}

func TestChangedParameterIsResent(t *testing.T) {
	link := newTestLink()

	first := fullCycleCommand()
	first.Full = true
	_ = positionFrame(link.position(first), DefaultPositionResolution(), &link.fast)

	steady := positionFrame(link.position(fullCycleCommand()), DefaultPositionResolution(), &link.fast)

	changed := fullCycleCommand()
	changed.VelocityLimit = 0.5 // the operator moved it
	resent := positionFrame(link.position(changed), DefaultPositionResolution(), &link.fast)

	if len(resent) <= len(steady) {
		t.Error("a changed velocity limit was not transmitted")
	}
}
