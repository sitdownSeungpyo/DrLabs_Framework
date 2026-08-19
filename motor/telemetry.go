package motor

// Telemetry is one controller's reported state, in framework units.
//
// Fields a driver does not report are NaN (floats) or -1 (integers), so a
// caller can always distinguish "zero" from "not measured".  Use Valid on the
// float fields before doing arithmetic.
type Telemetry struct {
	// Mode is the vendor's own mode register, kept raw so that driver
	// specific names and fault semantics stay meaningful.  ModeName carries
	// the decoded text.
	Mode     int
	ModeName string

	// Fault is the vendor fault code, 0 when healthy.  FaultName and
	// FaultDetail carry the decoded explanation.
	Fault       int
	FaultName   string
	FaultDetail string
	// HardFault distinguishes a latching fault from an informational
	// "output is being limited" state.
	HardFault bool

	Position    float64 // rev
	Velocity    float64 // rev/s
	Torque      float64 // Nm
	QCurrent    float64 // A, torque-producing
	DCurrent    float64 // A, flux
	AbsPosition float64 // rev, absolute encoder if fitted
	Power       float64 // W, electrical

	Voltage          float64 // V, bus
	Temperature      float64 // °C, board
	MotorTemperature float64 // °C, motor

	TrajectoryComplete int // -1 unknown, 0 moving, 1 arrived
	HomeState          int // -1 unknown; driver specific otherwise

	// The controller's *active* command registers, read back so the host can
	// verify that the limits it believes it set are the ones in force.  Only
	// populated by ProfileVerify cycles.
	ActiveMaxTorque     float64
	ActiveTimeout       float64
	ActiveVelocityLimit float64
	ActiveAccelLimit    float64
}

// NewTelemetry returns a Telemetry with every field marked "not reported".
func NewTelemetry() Telemetry {
	nan := NaN()
	return Telemetry{
		Mode:                -1,
		Fault:               -1,
		Position:            nan,
		Velocity:            nan,
		Torque:              nan,
		QCurrent:            nan,
		DCurrent:            nan,
		AbsPosition:         nan,
		Power:               nan,
		Voltage:             nan,
		Temperature:         nan,
		MotorTemperature:    nan,
		TrajectoryComplete:  -1,
		HomeState:           -1,
		ActiveMaxTorque:     nan,
		ActiveTimeout:       nan,
		ActiveVelocityLimit: nan,
		ActiveAccelLimit:    nan,
	}
}

// Merge overlays the fields that other actually reported onto t, leaving the
// rest untouched.  Rotating query profiles means a single cycle only refreshes
// part of the picture; merging keeps the last known value for the rest instead
// of blanking it.
func (t *Telemetry) Merge(other Telemetry) {
	if other.Mode >= 0 {
		t.Mode = other.Mode
		t.ModeName = other.ModeName
	}
	if other.Fault >= 0 {
		t.Fault = other.Fault
		t.FaultName = other.FaultName
		t.FaultDetail = other.FaultDetail
		t.HardFault = other.HardFault
	}
	if other.TrajectoryComplete >= 0 {
		t.TrajectoryComplete = other.TrajectoryComplete
	}
	if other.HomeState >= 0 {
		t.HomeState = other.HomeState
	}
	for _, pair := range []struct{ dst, src *float64 }{
		{&t.Position, &other.Position},
		{&t.Velocity, &other.Velocity},
		{&t.Torque, &other.Torque},
		{&t.QCurrent, &other.QCurrent},
		{&t.DCurrent, &other.DCurrent},
		{&t.AbsPosition, &other.AbsPosition},
		{&t.Power, &other.Power},
		{&t.Voltage, &other.Voltage},
		{&t.Temperature, &other.Temperature},
		{&t.MotorTemperature, &other.MotorTemperature},
		{&t.ActiveMaxTorque, &other.ActiveMaxTorque},
		{&t.ActiveTimeout, &other.ActiveTimeout},
		{&t.ActiveVelocityLimit, &other.ActiveVelocityLimit},
		{&t.ActiveAccelLimit, &other.ActiveAccelLimit},
	} {
		if Valid(*pair.src) {
			*pair.dst = *pair.src
		}
	}
}
