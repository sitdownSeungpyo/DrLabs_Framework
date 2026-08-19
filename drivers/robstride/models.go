package robstride

import (
	"fmt"
	"sort"
	"strings"
)

// Model is everything about a RobStride variant that the protocol depends on.
//
// The critical fields are TorqueRange, VelocityRange, KpMax and KdMax: they are
// the full-scale values of the 16-bit control and feedback fields, and they are
// *not* the same across the family.  Driving an RS04 with the RS01 table would
// scale every torque command and every torque reading by a factor of seven, in
// the dangerous direction — so the model is a required setting, never a guess.
//
// Every number is quoted from the model's official user manual
// ("<model>User Manual260713.pdf" in RobStride/Product_Information).
type Model struct {
	Name string

	// --- control and feedback frame full-scale values ---------------------

	// TorqueRange is the ± range of the torque field, which appears in the
	// identifier of a control command and in the payload of a feedback frame.
	TorqueRange float64 // Nm

	// VelocityRange is the ± range of the velocity field.
	VelocityRange float64 // rad/s

	// KpMax and KdMax are the top of the one-sided gain fields.
	KpMax float64 // Nm/rad
	KdMax float64 // Nm/(rad/s)

	// --- parameter table limits (the 0x70xx registers) ---------------------

	// CurrentRefMax bounds iq_ref (0x7006).
	CurrentRefMax float64 // A
	// SpeedRefMax bounds spd_ref (0x700A).
	SpeedRefMax float64 // rad/s
	// TorqueLimitMax bounds limit_torque (0x700B).
	TorqueLimitMax float64 // Nm
	// SpeedLimitMax bounds limit_spd (0x7017).
	SpeedLimitMax float64 // rad/s
	// CurrentLimitMax bounds limit_cur (0x7018).
	CurrentLimitMax float64 // A

	// --- supply -------------------------------------------------------------

	// VoltageMin, VoltageMax and VoltageRated are the model's documented
	// supply range and its rated operating point.
	VoltageMin   float64 // V
	VoltageMax   float64 // V
	VoltageRated float64 // V

	// --- host-side defaults -------------------------------------------------

	// DefaultKp and DefaultKd are what a gain scale of 1.0 maps to.
	//
	// These are deliberately conservative starting points, not tuned values.
	// The right stiffness depends on the load, the gearing and the mechanism,
	// and no table can know those — set them explicitly for a real machine
	// (the `kp` and `kd` driver options) once the axis is characterised.
	DefaultKp float64
	DefaultKd float64
}

// The family, smallest to largest.
//
// Note the split in gain ranges: RS00, RS01, RS02 and RS05 use Kp 0..500 and
// Kd 0..5, while RS03, RS04 and RS06 use Kp 0..5000 and Kd 0..100.  A gain
// value that is safe on one is ten to twenty times too weak or too stiff on the
// other, which is the second reason the model has to be declared.
var (
	// RS00 — 14 Nm class.
	RS00 = Model{
		Name:          "RS00",
		TorqueRange:   14,
		VelocityRange: 33,
		KpMax:         500,
		KdMax:         5,

		CurrentRefMax:   16,
		SpeedRefMax:     33,
		TorqueLimitMax:  14,
		SpeedLimitMax:   33,
		CurrentLimitMax: 16,

		VoltageMin:   24,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 20,
		DefaultKd: 1,
	}

	// RS01 — 17 Nm class.
	RS01 = Model{
		Name:          "RS01",
		TorqueRange:   17,
		VelocityRange: 44,
		KpMax:         500,
		KdMax:         5,

		CurrentRefMax:   23,
		SpeedRefMax:     44,
		TorqueLimitMax:  17,
		SpeedLimitMax:   44,
		CurrentLimitMax: 23,

		VoltageMin:   24,
		VoltageMax:   50,
		VoltageRated: 36,

		DefaultKp: 20,
		DefaultKd: 1,
	}

	// RS02 — 17 Nm class.
	//
	// Its manual is internally inconsistent: the control-frame table gives the
	// codec a ±17 Nm and ±44 rad/s full scale (confirmed in both the English
	// and Chinese editions, and matching the 0x2007 limit_torque default of
	// 17), while the CAN parameter table caps limit_torque at 14 Nm and
	// spd_ref at 33 rad/s.  The codec values are used for the wire format and
	// the parameter table values for validating parameter writes, which is the
	// reading that cannot silently mis-scale a command either way.
	RS02 = Model{
		Name:          "RS02",
		TorqueRange:   17,
		VelocityRange: 44,
		KpMax:         500,
		KdMax:         5,

		CurrentRefMax:   16,
		SpeedRefMax:     33,
		TorqueLimitMax:  14,
		SpeedLimitMax:   33,
		CurrentLimitMax: 16,

		VoltageMin:   24,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 20,
		DefaultKd: 1,
	}

	// RS03 — 60 Nm class.
	RS03 = Model{
		Name:          "RS03",
		TorqueRange:   60,
		VelocityRange: 20,
		KpMax:         5000,
		KdMax:         100,

		CurrentRefMax:   43,
		SpeedRefMax:     20,
		TorqueLimitMax:  60,
		SpeedLimitMax:   20,
		CurrentLimitMax: 43,

		VoltageMin:   24,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 200,
		DefaultKd: 10,
	}

	// RS04 — 120 Nm class.
	//
	// Its control frame maps velocity over ±15 rad/s while the parameter table
	// allows spd_ref and limit_spd up to 20 rad/s.  The codec range is what the
	// control and feedback frames use; a velocity beyond it is clamped rather
	// than wrapped.
	RS04 = Model{
		Name:          "RS04",
		TorqueRange:   120,
		VelocityRange: 15,
		KpMax:         5000,
		KdMax:         100,

		CurrentRefMax:   90,
		SpeedRefMax:     20,
		TorqueLimitMax:  120,
		SpeedLimitMax:   20,
		CurrentLimitMax: 90,

		VoltageMin:   24,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 200,
		DefaultKd: 10,
	}

	// RS05 — 5.5 Nm class, the smallest of the family.
	RS05 = Model{
		Name:          "RS05",
		TorqueRange:   5.5,
		VelocityRange: 50,
		KpMax:         500,
		KdMax:         5,

		CurrentRefMax:   11,
		SpeedRefMax:     50,
		TorqueLimitMax:  5.5,
		SpeedLimitMax:   50,
		CurrentLimitMax: 11,

		VoltageMin:   15,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 10,
		DefaultKd: 0.5,
	}

	// RS06 — 36 Nm class.
	RS06 = Model{
		Name:          "RS06",
		TorqueRange:   36,
		VelocityRange: 50,
		KpMax:         5000,
		KdMax:         100,

		CurrentRefMax:   57,
		SpeedRefMax:     50,
		TorqueLimitMax:  36,
		SpeedLimitMax:   50,
		CurrentLimitMax: 57,

		VoltageMin:   15,
		VoltageMax:   60,
		VoltageRated: 48,

		DefaultKp: 200,
		DefaultKd: 10,
	}
)

// Models is the lookup used by configuration.  There is deliberately no
// default: an unset model is an error, because every wrong guess mis-scales
// torque.
var Models = map[string]Model{
	"rs00": RS00,
	"rs01": RS01,
	"rs02": RS02,
	"rs03": RS03,
	"rs04": RS04,
	"rs05": RS05,
	"rs06": RS06,
}

// LookupModel resolves a model name, case-insensitively and tolerating the
// bare number ("03") and the hyphenated spelling ("RS-03") that the product
// literature uses interchangeably.
func LookupModel(name string) (Model, error) {
	key := strings.ToLower(strings.TrimSpace(name))
	key = strings.ReplaceAll(key, "-", "")
	key = strings.ReplaceAll(key, "_", "")
	key = strings.ReplaceAll(key, " ", "")

	if !strings.HasPrefix(key, "rs") {
		key = "rs" + key
	}
	// "rs3" is a plausible thing to type for RS03.
	if len(key) == 3 {
		key = "rs0" + key[2:]
	}

	model, ok := Models[key]
	if !ok {
		return Model{}, fmt.Errorf("robstride: unknown model %q (known: %s)",
			name, strings.Join(ModelNames(), ", "))
	}
	return model, nil
}

// ModelNames lists the known models, sorted.
func ModelNames() []string {
	names := make([]string, 0, len(Models))
	for _, model := range Models {
		names = append(names, model.Name)
	}
	sort.Strings(names)
	return names
}
