package dynamixel

// MotorModel defines the Control Table addresses for a specific Dynamixel motor type.
type MotorModel struct {
	AddrTorqueEnable    uint16
	AddrGoalPosition    uint16
	AddrGoalVelocity    uint16
	AddrGoalPWM         uint16
	AddrGoalCurrent     uint16
	AddrPresentPosition uint16
	AddrOperatingMode   uint16
}

// Common motor models
var (
	// ModelXSeries covers X-Series (XM430, XC430, etc.) & MX-Series (Protocol 2.0).
	ModelXSeries = MotorModel{
		AddrTorqueEnable:    64,
		AddrGoalPosition:    116,
		AddrGoalVelocity:    104,
		AddrGoalPWM:         100,
		AddrGoalCurrent:     102,
		AddrPresentPosition: 132,
		AddrOperatingMode:   11,
	}

	// ModelProSeries covers Pro-Series (H54, H42, etc.).
	ModelProSeries = MotorModel{
		AddrTorqueEnable:    562,
		AddrGoalPosition:    596,
		AddrGoalVelocity:    600,
		AddrGoalPWM:         584,
		AddrGoalCurrent:     590,
		AddrPresentPosition: 611,
		AddrOperatingMode:   11,
	}
)

// Operating mode constants
const (
	OpModeCurrent          = 0
	OpModeVelocity         = 1
	OpModePosition         = 3
	OpModeExtendedPosition = 4
	OpModeCurrentBasedPos  = 5
	OpModePWM              = 16
)

// MaxValidMotorID is the maximum valid Dynamixel motor ID (253-255 are reserved).
const MaxValidMotorID = 252
