package dynamixel

// Control table addresses and unit scales, per motor family.
//
// A Dynamixel reports everything in raw encoder and driver counts, and the
// counts mean different things on different models.  The framework speaks
// physical units, so the conversion has to live somewhere — it lives here, in
// one struct per family, rather than being scattered through the driver.
//
// Addresses are from the ROBOTIS e-Manual control tables for Protocol 2.0.

// Profile describes one motor family's control table and units.
type Profile struct {
	Name string

	// EEPROM area.
	AddrModelNumber     uint16
	AddrFirmwareVersion uint16
	AddrOperatingMode   uint16

	// RAM area.
	AddrTorqueEnable    uint16
	AddrLED             uint16
	AddrHardwareError   uint16
	AddrBusWatchdog     uint16
	AddrGoalPWM         uint16
	AddrGoalCurrent     uint16
	AddrGoalVelocity    uint16
	AddrProfileAccel    uint16
	AddrProfileVelocity uint16
	AddrGoalPosition    uint16
	AddrPresentCurrent  uint16
	AddrPresentVelocity uint16
	AddrPresentPosition uint16
	AddrPresentVoltage  uint16
	AddrPresentTemp     uint16

	// CountsPerRev converts the position register to revolutions.
	CountsPerRev float64

	// VelocityRPMPerCount converts the velocity register to RPM.
	VelocityRPMPerCount float64

	// CurrentAmpsPerCount converts the current register to amperes.  It is
	// the value that varies most between models — 2.69 mA on an XM430,
	// 1.0 mA on an XL330 — so a wrong profile shows up as a current reading
	// that is out by a factor of three, which is exactly the sort of error
	// that makes a current trip useless.
	CurrentAmpsPerCount float64

	// VoltageVoltsPerCount converts the input voltage register to volts.
	VoltageVoltsPerCount float64

	// AccelRevPerMin2PerCount converts the profile acceleration register.
	AccelRevPerMin2PerCount float64

	// WatchdogSecondsPerCount converts the bus watchdog register.  The
	// register is one byte, so the longest timeout it can express is
	// 127 counts — a little over two and a half seconds.
	WatchdogSecondsPerCount float64
	WatchdogMaxCount        int

	// TorqueConstant converts current to torque, in Nm/A.  Zero means the
	// motor's constant is unknown, and torque is reported as unmeasured
	// rather than as a number derived from a guess.
	TorqueConstant float64

	// MaxVelocityRevPerSec is the family's no-load speed, used as the
	// hardware envelope the host safety limits are checked against.
	MaxVelocityRevPerSec float64

	// VoltageMin and VoltageMax are the family's supply range.
	VoltageMin float64
	VoltageMax float64
}

// XSeries covers the X-series (XM430, XC430, XH430, XW430 and relatives) with
// a 4096-count encoder and the standard 2.69 mA current unit.
var XSeries = Profile{
	Name:                "X-series",
	AddrModelNumber:     0,
	AddrFirmwareVersion: 6,
	AddrOperatingMode:   11,
	AddrTorqueEnable:    64,
	AddrLED:             65,
	AddrHardwareError:   70,
	AddrBusWatchdog:     98,
	AddrGoalPWM:         100,
	AddrGoalCurrent:     102,
	AddrGoalVelocity:    104,
	AddrProfileAccel:    108,
	AddrProfileVelocity: 112,
	AddrGoalPosition:    116,
	AddrPresentCurrent:  126,
	AddrPresentVelocity: 128,
	AddrPresentPosition: 132,
	AddrPresentVoltage:  144,
	AddrPresentTemp:     146,

	CountsPerRev:            4096.0,
	VelocityRPMPerCount:     0.229,
	CurrentAmpsPerCount:     0.00269,
	VoltageVoltsPerCount:    0.1,
	AccelRevPerMin2PerCount: 214.577,
	WatchdogSecondsPerCount: 0.020,
	WatchdogMaxCount:        127,

	TorqueConstant:       0, // varies per model; set it explicitly to get torque
	MaxVelocityRevPerSec: 4.0,
	VoltageMin:           10.0,
	VoltageMax:           14.8,
}

// XL330 is the 5 V X-series family.  It shares the control table but uses a
// 1 mA current unit and a lower supply range.
var XL330 = func() Profile {
	profile := XSeries
	profile.Name = "XL330"
	profile.CurrentAmpsPerCount = 0.001
	profile.MaxVelocityRevPerSec = 6.0
	profile.VoltageMin = 3.7
	profile.VoltageMax = 6.0
	return profile
}()

// Profiles are the families that can be named in a Config.
var Profiles = map[string]Profile{
	"x":       XSeries,
	"xseries": XSeries,
	"xl330":   XL330,
}

// The operating mode constants live in models.go, alongside the control table
// presets they belong with.

// hardwareErrorBits decodes the hardware error status register, so a fault is
// reported as a cause rather than as a bitmask.
var hardwareErrorBits = []struct {
	Bit    uint8
	Name   string
	Detail string
}{
	{0x01, "Input voltage error", "The supply is outside the motor's rated range."},
	{0x04, "Overheating error", "The internal temperature exceeded its limit."},
	{0x08, "Motor encoder error", "The encoder is not reporting usable positions."},
	{0x10, "Electrical shock error", "A short circuit or an over-current on the power line."},
	{0x20, "Overload error", "A load beyond the motor's continuous rating was sustained."},
}

// decodeHardwareError turns the status register into a name and explanation.
func decodeHardwareError(status uint8) (name, detail string) {
	if status == 0 {
		return "No fault", ""
	}
	for _, entry := range hardwareErrorBits {
		if status&entry.Bit != 0 {
			return entry.Name, entry.Detail
		}
	}
	return "Hardware error", "The motor reports an error this build does not recognise."
}
