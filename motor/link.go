package motor

import "context"

// QueryProfile selects how much telemetry a cycle asks for.
//
// Bus bandwidth is the binding constraint at high control rates: a CAN-FD
// frame carries at most 64 bytes and a command shares that frame with its
// query.  Drivers therefore rotate profiles instead of asking for everything
// every period — the fast set on every cycle, the wider ones occasionally.
type QueryProfile uint8

const (
	// ProfileFast is the minimum needed to control and to trip safely:
	// mode, position, velocity, torque, current, voltage, temperature, fault.
	ProfileFast QueryProfile = iota
	// ProfileFull adds the diagnostic registers — d current, absolute
	// position, power, motor temperature, trajectory and homing state.
	ProfileFull
	// ProfileVerify adds the controller's live command registers, so the host
	// can confirm the limits in force are the ones it intended to set.
	ProfileVerify
)

func (p QueryProfile) String() string {
	switch p {
	case ProfileFull:
		return "full"
	case ProfileVerify:
		return "verify"
	default:
		return "fast"
	}
}

// BoardStatus is the verdict on whether the framework may drive the attached
// hardware.
//
// The four states answer two different questions — what did the device say
// about itself, and what does this build know about that answer — and they are
// kept apart because the right response differs:
//
//	OK        read from the device and recognised
//	Declared  the device cannot report its identity at all; the operator
//	          stated it in configuration and nothing on the bus can confirm it
//	Unknown   the device answered with something this build does not recognise
//	Rejected  the device answered with something known to be out of scope
//
// Declared is not a weaker Unknown.  A controller that has no identity
// register is behaving normally, so demanding an override on every connect
// would train operators to pass the override always — which would hollow out
// the check for the devices that do report a revision.  It is surfaced as an
// advisory instead: the declaration is unverified, and it will stay unverified.
type BoardStatus string

const (
	BoardOK       BoardStatus = "supported"
	BoardDeclared BoardStatus = "declared"
	BoardUnknown  BoardStatus = "unknown_revision"
	BoardRejected BoardStatus = "unsupported"
)

// Identity is what a driver can learn about the device at connect time.
type Identity struct {
	Vendor       string // "moteus", "dynamixel", …
	Board        string // human readable board or model name
	Firmware     string
	SerialNumber string
	Family       int // -1 when not reported
	HWRev        int // -1 when not reported
	ModelNumber  uint32

	// PositionMin/Max are the travel limits stored *in the controller*.
	// Commands outside them are silently clamped or rejected by the firmware,
	// which from the outside looks exactly like "the motor does not move", so
	// the host reads them and explains the constraint up front.  NaN when the
	// device reports no bounds.
	PositionMin float64
	PositionMax float64

	Status    BoardStatus
	Supported bool
	Known     bool
	Simulated bool

	// Accepted and Overridden are filled in by the session once the operator
	// has (or has not) consented to an unrecognised revision.
	Accepted   bool
	Overridden bool
}

// NewIdentity returns an Identity with the "not reported" markers in place.
func NewIdentity(vendor string) *Identity {
	return &Identity{
		Vendor:      vendor,
		Family:      -1,
		HWRev:       -1,
		PositionMin: NaN(),
		PositionMax: NaN(),
		Status:      BoardUnknown,
	}
}

// Bounds returns the controller's own travel envelope, if it reports a usable
// one.
func (id *Identity) Bounds() (lower, upper float64, ok bool) {
	if id == nil || !Valid(id.PositionMin) || !Valid(id.PositionMax) {
		return 0, 0, false
	}
	if id.PositionMax <= id.PositionMin {
		return 0, 0, false
	}
	return id.PositionMin, id.PositionMax, true
}

// Link is a live connection to exactly one controller.
//
// It is the single seam the runtime is written against: the control loop, the
// safety monitor and the CLI never name a vendor.  Implementations are found
// in drivers/*, plus the simulators used to exercise every safety path with no
// hardware attached.
//
// Cycle is expected to be called on a fixed period.  It must not be cancelled
// mid-transaction just because the period elapsed — on several adapters that
// leaves the device's pending-request state inconsistent and every subsequent
// transaction returns empty.  Implementations carry their own transaction
// timeout, and a Link that exceeds it should be torn down rather than reused.
type Link interface {
	// Kind is the driver name this Link was built from.
	Kind() string

	// Describe returns connection details for display and logging.
	Describe() map[string]string

	// Open acquires the transport and leaves the controller stopped.
	Open(ctx context.Context) error

	// Close stops the motor and releases the transport.  It is safe to call
	// on a Link that never opened.
	Close() error

	// Identify reads board identification.  A nil Identity with a nil error
	// means the device answered but reports nothing identifying.
	Identify(ctx context.Context) (*Identity, error)

	// Cycle transmits one command and returns the telemetry that came back.
	// A nil Telemetry with a nil error is a timeout: the device did not
	// answer this period, which is not by itself fatal.
	Cycle(ctx context.Context, cmd Command, profile QueryProfile) (*Telemetry, error)

	// Capabilities reports which control modes this Link can express.
	Capabilities() Capabilities
}

// Capabilities describes what a driver supports, so that a UI can grey out
// what is unavailable instead of failing at command time.
type Capabilities struct {
	Modes []ControlMode

	// MaxRateHz is the highest control rate the transport can sustain.
	MaxRateHz int

	// HasWatchdog reports whether the controller stops on its own when host
	// commands stop arriving.  Without it, the host loop is the only thing
	// standing between a wedged process and a runaway axis.
	HasWatchdog bool

	// ReportsTemperature, ReportsCurrent and ReportsVoltage tell the safety
	// monitor which trips can actually fire.
	ReportsTemperature bool
	ReportsCurrent     bool
	ReportsVoltage     bool
}

// Envelope is the hardware's own documented limits.  The host safety layer
// checks the operator's declared envelope against it, so that a host setting
// can never be looser than what the controller is rated for.
//
// A zero field means "not characterised by this driver", and is skipped rather
// than treated as a limit of zero.
type Envelope struct {
	VoltageMin float64 // V
	VoltageMax float64 // V
	// VoltageRated is the supply the hardware is designed to run on.  It is
	// what tells a host-side voltage trip band whether it is plausible for
	// this device at all: a band that excludes the rated supply is not a
	// safety limit, it is a guaranteed nuisance trip.
	VoltageRated float64 // V
	PeakCurrent  float64 // A
	VelocityMax  float64 // rev/s
}

// Enveloped is implemented by Links that know their hardware's ratings.
// Drivers that do not simply omit it, and the safety layer falls back to the
// operator's declared limits alone.
type Enveloped interface {
	Envelope() Envelope
}

// Supports reports whether mode is in the capability set.
func (c Capabilities) Supports(mode ControlMode) bool {
	for _, m := range c.Modes {
		if m == mode {
			return true
		}
	}
	return false
}
