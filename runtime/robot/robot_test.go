package robot

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"

	"drlabs/runtime/safety"
)

// The mapping is where a wrong number produces a machine that moves — just not
// the way anyone asked — so it is worth more test than its size suggests.

func TestMappingRoundTrips(t *testing.T) {
	for _, mapping := range []Mapping{
		{GearRatio: 1, Direction: 1},
		{GearRatio: 6, Direction: 1},
		{GearRatio: 6, Direction: -1},
		{GearRatio: 36, Direction: -1, OffsetRad: 0.4},
		{GearRatio: 0.5, Direction: 1, OffsetRad: -1.2},
	} {
		for _, radians := range []float64{-2.0, -0.35, 0, 0.35, 1.57} {
			motorRev := mapping.MotorRevolutions(radians)
			back := mapping.JointRadians(motorRev)
			if math.Abs(back-radians) > 1e-9 {
				t.Errorf("gear %v dir %v offset %v: %v rad -> %v rev -> %v rad",
					mapping.GearRatio, mapping.Direction, mapping.OffsetRad,
					radians, motorRev, back)
			}
		}
	}
}

func TestGearRatioScalesPosition(t *testing.T) {
	// Six motor turns to one joint turn: a full joint revolution is six motor
	// revolutions.
	mapping := Mapping{GearRatio: 6, Direction: 1}
	if got := mapping.MotorRevolutions(2 * math.Pi); math.Abs(got-6) > 1e-9 {
		t.Errorf("one joint turn: got %v motor rev, want 6", got)
	}
}

func TestDirectionReversesTheJoint(t *testing.T) {
	forward := Mapping{GearRatio: 1, Direction: 1}
	reversed := Mapping{GearRatio: 1, Direction: -1}

	if forward.MotorRevolutions(1.0) != -reversed.MotorRevolutions(1.0) {
		t.Error("direction -1 should mirror the motor position")
	}
	// And the reported angle comes back the right way round.
	if got := reversed.JointRadians(reversed.MotorRevolutions(0.7)); math.Abs(got-0.7) > 1e-9 {
		t.Errorf("reversed joint reported %v, want 0.7", got)
	}
}

func TestOffsetMovesTheJointZero(t *testing.T) {
	// With an offset of 0.4 rad, commanding the joint to zero drives the motor
	// to where 0.4 rad would otherwise have been.
	plain := Mapping{GearRatio: 1, Direction: 1}
	offset := Mapping{GearRatio: 1, Direction: 1, OffsetRad: 0.4}

	if math.Abs(offset.MotorRevolutions(0)-plain.MotorRevolutions(0.4)) > 1e-9 {
		t.Error("the offset did not shift the joint zero")
	}
}

func TestTorqueGoesThroughTheGearboxTheRightWay(t *testing.T) {
	// A gearbox multiplies torque as it divides speed.  Six newton-metres at a
	// joint behind a 6:1 reduction is one newton-metre at the rotor — and
	// getting this backwards gives a limit thirty-six times too generous.
	mapping := Mapping{GearRatio: 6, Direction: 1}

	if got := mapping.MotorTorque(6.0); math.Abs(got-1.0) > 1e-9 {
		t.Errorf("6 Nm at the joint should be 1 Nm at the motor, got %v", got)
	}
	if got := mapping.JointTorque(1.0); math.Abs(got-6.0) > 1e-9 {
		t.Errorf("1 Nm at the motor should be 6 Nm at the joint, got %v", got)
	}
}

func TestVelocityIgnoresTheOffset(t *testing.T) {
	// An offset shifts a position; it has no meaning for a rate.  Applying it
	// to velocity would add a constant drift to every command.
	plain := Mapping{GearRatio: 6, Direction: 1}
	offset := Mapping{GearRatio: 6, Direction: 1, OffsetRad: 1.3}

	if plain.MotorRevPerSecond(0.5) != offset.MotorRevPerSecond(0.5) {
		t.Error("the offset leaked into the velocity conversion")
	}
}

func TestSafetyEnvelopeOrdersReversedLimits(t *testing.T) {
	// With direction -1 the joint minimum maps to the larger motor position.
	// Leaving them unswapped would produce an envelope that excludes
	// everything, and the axis would refuse to arm for no visible reason.
	mapping := Mapping{GearRatio: 6, Direction: -1}
	limits := JointLimits{
		MinRad: -1.0, MaxRad: 1.0,
		MaxTorqueNm: 6, MaxVelocityRadS: 2,
	}

	config := mapping.safetyConfig(limits, safety.DefaultConfig())
	if config.PositionMin >= config.PositionMax {
		t.Fatalf("envelope is inverted: [%v, %v]", config.PositionMin, config.PositionMax)
	}
	// And it covers the joint's actual travel.
	for _, radians := range []float64{-1.0, 0, 1.0} {
		position := mapping.MotorRevolutions(radians)
		if position < config.PositionMin-1e-9 || position > config.PositionMax+1e-9 {
			t.Errorf("%v rad maps to %v rev, outside the envelope [%v, %v]",
				radians, position, config.PositionMin, config.PositionMax)
		}
	}
}

func TestSafetyEnvelopeConvertsTorqueToMotorSpace(t *testing.T) {
	mapping := Mapping{GearRatio: 6, Direction: 1}
	limits := JointLimits{MinRad: -1, MaxRad: 1, MaxTorqueNm: 6, MaxVelocityRadS: 2}

	config := mapping.safetyConfig(limits, safety.DefaultConfig())
	if math.Abs(config.MaxTorque-1.0) > 1e-9 {
		t.Errorf("motor torque ceiling: got %v, want 1.0", config.MaxTorque)
	}
	if config.MaxVelocity <= 0 {
		t.Error("the velocity ceiling should be positive whatever the direction")
	}
}

// --- configuration ---------------------------------------------------------

func write(t *testing.T, config map[string]any) string {
	t.Helper()
	data, err := json.Marshal(config)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "robot.json")
	if err := os.WriteFile(path, data, 0o600); err != nil {
		t.Fatal(err)
	}
	return path
}

func validJoint() map[string]any {
	return map[string]any{
		"name": "r_sh_p", "bus": "can0", "driver": "moteus", "id": 1,
		"gear_ratio": 6.0, "direction": 1.0, "offset_rad": 0.0,
		"limits": map[string]any{
			"min_rad": -1.0, "max_rad": 1.0,
			"max_torque_nm": 6.0, "max_velocity_rad_s": 2.0,
		},
	}
}

func validConfig() map[string]any {
	return map[string]any{
		"robot": "test", "rate_hz": 100,
		"buses":  []any{map[string]any{"name": "can0", "transport": "fdcanusb", "port": "COM5"}},
		"joints": []any{validJoint()},
	}
}

func TestLoadAcceptsAValidDescription(t *testing.T) {
	config, err := Load(write(t, validConfig()))
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if len(config.Joints) != 1 || config.Joints[0].Name != "r_sh_p" {
		t.Errorf("unexpected joints: %+v", config.Joints)
	}
}

func TestLoadRejectsMistakesThatWouldMoveTheMachine(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(map[string]any)
	}{
		{"no gear ratio", func(c map[string]any) {
			delete(c["joints"].([]any)[0].(map[string]any), "gear_ratio")
		}},
		{"negative gear ratio", func(c map[string]any) {
			c["joints"].([]any)[0].(map[string]any)["gear_ratio"] = -6.0
		}},
		{"direction not ±1", func(c map[string]any) {
			c["joints"].([]any)[0].(map[string]any)["direction"] = 0.0
		}},
		{"no torque ceiling", func(c map[string]any) {
			joint := c["joints"].([]any)[0].(map[string]any)
			delete(joint["limits"].(map[string]any), "max_torque_nm")
		}},
		{"empty travel range", func(c map[string]any) {
			limits := c["joints"].([]any)[0].(map[string]any)["limits"].(map[string]any)
			limits["min_rad"], limits["max_rad"] = 1.0, 1.0
		}},
		{"undefined bus", func(c map[string]any) {
			c["joints"].([]any)[0].(map[string]any)["bus"] = "nowhere"
		}},
		{"unknown transport", func(c map[string]any) {
			c["buses"].([]any)[0].(map[string]any)["transport"] = "smoke-signal"
		}},
		{"misspelled key", func(c map[string]any) {
			c["joints"].([]any)[0].(map[string]any)["gear_ration"] = 6.0
		}},
		{"no joints", func(c map[string]any) {
			c["joints"] = []any{}
		}},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			config := validConfig()
			tc.mutate(config)
			if _, err := Load(write(t, config)); err == nil {
				t.Error("this description should have been rejected")
			}
		})
	}
}

func TestLoadRejectsTwoJointsAtOneAddress(t *testing.T) {
	// Both would be commanding the other's motor, and the machine fights
	// itself.
	config := validConfig()
	second := validJoint()
	second["name"] = "r_elb_p"
	config["joints"] = []any{validJoint(), second}

	if _, err := Load(write(t, config)); err == nil {
		t.Error("two joints on one address should be rejected")
	}
}

func TestLoadRejectsDuplicateJointNames(t *testing.T) {
	config := validConfig()
	second := validJoint()
	second["id"] = 2
	config["joints"] = []any{validJoint(), second}

	if _, err := Load(write(t, config)); err == nil {
		t.Error("two joints with the same name should be rejected")
	}
}

func TestBusLoadCountsEveryJoint(t *testing.T) {
	// The number that decides whether a rate is achievable.
	config := validConfig()
	second := validJoint()
	second["name"], second["id"] = "r_elb_p", 2
	config["joints"] = []any{validJoint(), second}

	loaded, err := Load(write(t, config))
	if err != nil {
		t.Fatal(err)
	}
	if got := loaded.BusLoad()["can0"]; got != 200 {
		t.Errorf("two joints at 100 Hz should be 200 transactions/s, got %d", got)
	}
}

func TestExampleDescriptionsAreValid(t *testing.T) {
	// The examples are the first thing anyone copies; a broken one teaches a
	// broken schema.
	matches, err := filepath.Glob(filepath.Join("..", "..", "examples", "robots", "*.json"))
	if err != nil {
		t.Fatal(err)
	}
	if len(matches) == 0 {
		t.Skip("no example descriptions found")
	}
	for _, path := range matches {
		if _, err := Load(path); err != nil {
			t.Errorf("%s: %v", filepath.Base(path), err)
		}
	}
}
