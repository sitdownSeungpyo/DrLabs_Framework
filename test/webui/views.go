package main

import (
	"drlabs/motor"
	"drlabs/runtime/session"
)

// The wire format for the page.
//
// These types exist for one unglamorous reason: JSON cannot represent NaN, and
// the framework uses NaN throughout to mean "not measured".  Marshalling a
// Telemetry directly fails outright, and rounding NaN to zero on the way out
// would tell the page that an unread sensor is reading zero — exactly the
// confusion the NaN convention exists to prevent.  Every optional number
// therefore becomes a pointer, and the page renders null as an em dash.

// number converts a framework float into something JSON can carry: a value
// when it was measured, null when it was not.
func number(v float64) *float64 {
	if !motor.Valid(v) {
		return nil
	}
	out := v
	return &out
}

type snapshot struct {
	Status  statusView   `json:"status"`
	Events  []eventView  `json:"events"`
	Samples []sampleView `json:"samples"`
}

type statusView struct {
	Connection   string `json:"connection"`
	ConnectError string `json:"connect_error"`

	Link     map[string]string `json:"link"`
	Identity *identityView     `json:"identity"`

	Armed         bool     `json:"armed"`
	Mode          string   `json:"mode"`
	ArmBlockers   []string `json:"arm_blockers"`
	ArmAdvisories []string `json:"arm_advisories"`

	EStop estopView `json:"estop"`

	Setpoint   setpointView   `json:"setpoint"`
	Params     session.Params `json:"params"`
	Shuttle    shuttleView    `json:"shuttle"`
	Trajectory trajectoryView `json:"trajectory"`
	Safety     safetyView     `json:"safety"`
	Preset     string         `json:"preset"`

	ControllerBounds []float64 `json:"controller_bounds"`
	EffectiveLimits  []float64 `json:"effective_limits"`

	Telemetry telemetryView `json:"telemetry"`
	Warnings  []string      `json:"warnings"`

	Stats statsView `json:"stats"`
}

type identityView struct {
	Vendor       string   `json:"vendor"`
	Board        string   `json:"board"`
	Firmware     string   `json:"firmware"`
	SerialNumber string   `json:"serial_number"`
	Status       string   `json:"status"`
	Simulated    bool     `json:"simulated"`
	PositionMin  *float64 `json:"position_min"`
	PositionMax  *float64 `json:"position_max"`
}

type estopView struct {
	Latched bool   `json:"latched"`
	Code    string `json:"code"`
	Reason  string `json:"reason"`
	Detail  string `json:"detail"`
}

type setpointView struct {
	Position    float64 `json:"position"`
	Velocity    float64 `json:"velocity"`
	WithinLower float64 `json:"within_lower"`
	WithinUpper float64 `json:"within_upper"`
}

type shuttleView struct {
	Running   bool    `json:"running"`
	A         float64 `json:"a"`
	B         float64 `json:"b"`
	Dwell     float64 `json:"dwell"`
	Cycles    int     `json:"cycles"`
	Completed int     `json:"completed"`
	Phase     string  `json:"phase"`
	Status    string  `json:"status"`
}

type trajectoryView struct {
	Running     bool    `json:"running"`
	Start       float64 `json:"start"`
	Target      float64 `json:"target"`
	MaxVelocity float64 `json:"max_velocity"`
	Accel       float64 `json:"accel"`
	Elapsed     float64 `json:"elapsed"`
	Total       float64 `json:"total"`
	Progress    float64 `json:"progress"`
	Setpoint    float64 `json:"setpoint"`
	Feedforward float64 `json:"feedforward"`
	Status      string  `json:"status"`
}

type safetyView struct {
	MaxTorque   float64 `json:"max_torque"`
	MaxVelocity float64 `json:"max_velocity"`
	AccelLimit  float64 `json:"accel_limit"`

	PositionLimitEnabled bool    `json:"position_limit_enabled"`
	PositionMin          float64 `json:"position_min"`
	PositionMax          float64 `json:"position_max"`
	PositionTripMargin   float64 `json:"position_trip_margin"`

	BusVoltageMin   float64 `json:"bus_voltage_min"`
	BusVoltageMax   float64 `json:"bus_voltage_max"`
	CurrentTrip     float64 `json:"current_trip"`
	CurrentTripTime float64 `json:"current_trip_time"`

	BoardTempTrip float64 `json:"board_temp_trip"`
	MotorTempTrip float64 `json:"motor_temp_trip"`

	StallDetectEnabled bool    `json:"stall_detect_enabled"`
	DeadmanTimeout     float64 `json:"deadman_timeout"`
}

type telemetryView struct {
	Mode        int    `json:"mode"`
	ModeName    string `json:"mode_name"`
	Fault       int    `json:"fault"`
	FaultName   string `json:"fault_name"`
	FaultDetail string `json:"fault_detail"`
	HardFault   bool   `json:"hard_fault"`

	Position         *float64 `json:"position"`
	Velocity         *float64 `json:"velocity"`
	Torque           *float64 `json:"torque"`
	QCurrent         *float64 `json:"q_current"`
	Power            *float64 `json:"power"`
	Voltage          *float64 `json:"voltage"`
	Temperature      *float64 `json:"temperature"`
	MotorTemperature *float64 `json:"motor_temperature"`
}

type statsView struct {
	RequestedHz int      `json:"requested_hz"`
	AchievedHz  float64  `json:"achieved_hz"`
	JitterMS    float64  `json:"jitter_ms"`
	MeanMS      float64  `json:"mean_ms"`
	P95MS       float64  `json:"p95_ms"`
	MaxMS       float64  `json:"max_ms"`
	Cycles      uint64   `json:"cycles"`
	Replies     uint64   `json:"replies"`
	Timeouts    uint64   `json:"timeouts"`
	ReplyRate   float64  `json:"reply_rate"`
	WatchdogMS  float64  `json:"watchdog_ms"`
	Consecutive int      `json:"consecutive_timeouts"`
	_           struct{} `json:"-"`
}

type eventView struct {
	Seq     uint64 `json:"seq"`
	At      int64  `json:"at"`
	Level   string `json:"level"`
	Message string `json:"message"`
	Detail  string `json:"detail"`
}

type sampleView struct {
	Seq      uint64   `json:"seq"`
	T        float64  `json:"t"`
	Position *float64 `json:"position"`
	Velocity *float64 `json:"velocity"`
	Current  *float64 `json:"current"`
	Torque   *float64 `json:"torque"`
}

func newStatusView(s session.Status) statusView {
	out := statusView{
		Connection:    string(s.Connection),
		ConnectError:  s.ConnectError,
		Link:          s.Link,
		Armed:         s.Armed,
		Mode:          s.Mode.String(),
		ArmBlockers:   orEmpty(s.ArmBlockers),
		ArmAdvisories: orEmpty(s.ArmAdvisories),
		EStop: estopView{
			Latched: s.EStop.Latched,
			Code:    s.EStop.Code,
			Reason:  s.EStop.Reason,
			Detail:  s.EStop.Detail,
		},
		Setpoint: setpointView{
			Position:    s.Setpoint.Position,
			Velocity:    s.Setpoint.Velocity,
			WithinLower: s.Setpoint.WithinLower,
			WithinUpper: s.Setpoint.WithinUpper,
		},
		Params: s.Params,
		Shuttle: shuttleView{
			Running:   s.Shuttle.Running,
			A:         s.Shuttle.A,
			B:         s.Shuttle.B,
			Dwell:     s.Shuttle.Dwell,
			Cycles:    s.Shuttle.Cycles,
			Completed: s.Shuttle.Completed,
			Phase:     s.Shuttle.Phase,
			Status:    s.Shuttle.Status,
		},
		Trajectory: trajectoryView{
			Running:     s.Trajectory.Running,
			Start:       s.Trajectory.Start,
			Target:      s.Trajectory.Target,
			MaxVelocity: s.Trajectory.MaxVelocity,
			Accel:       s.Trajectory.Accel,
			Elapsed:     s.Trajectory.Elapsed,
			Total:       s.Trajectory.Total,
			Progress:    s.Trajectory.Progress,
			Setpoint:    s.Trajectory.Setpoint,
			Feedforward: s.Trajectory.Feedforward,
			Status:      s.Trajectory.Status,
		},
		Safety: safetyView{
			MaxTorque:            s.Safety.MaxTorque,
			MaxVelocity:          s.Safety.MaxVelocity,
			AccelLimit:           s.Safety.AccelLimit,
			PositionLimitEnabled: s.Safety.PositionLimitEnabled,
			PositionMin:          s.Safety.PositionMin,
			PositionMax:          s.Safety.PositionMax,
			PositionTripMargin:   s.Safety.PositionTripMargin,
			BusVoltageMin:        s.Safety.BusVoltageMin,
			BusVoltageMax:        s.Safety.BusVoltageMax,
			CurrentTrip:          s.Safety.CurrentTrip,
			CurrentTripTime:      s.Safety.CurrentTripTime,
			BoardTempTrip:        s.Safety.BoardTempTrip,
			MotorTempTrip:        s.Safety.MotorTempTrip,
			StallDetectEnabled:   s.Safety.StallDetectEnabled,
			DeadmanTimeout:       s.Safety.DeadmanTimeout,
		},
		Preset:          s.Preset,
		EffectiveLimits: []float64{s.EffectiveLimits[0], s.EffectiveLimits[1]},
		Telemetry: telemetryView{
			Mode:             s.Telemetry.Mode,
			ModeName:         s.Telemetry.ModeName,
			Fault:            s.Telemetry.Fault,
			FaultName:        s.Telemetry.FaultName,
			FaultDetail:      s.Telemetry.FaultDetail,
			HardFault:        s.Telemetry.HardFault,
			Position:         number(s.Telemetry.Position),
			Velocity:         number(s.Telemetry.Velocity),
			Torque:           number(s.Telemetry.Torque),
			QCurrent:         number(s.Telemetry.QCurrent),
			Power:            number(s.Telemetry.Power),
			Voltage:          number(s.Telemetry.Voltage),
			Temperature:      number(s.Telemetry.Temperature),
			MotorTemperature: number(s.Telemetry.MotorTemperature),
		},
		Warnings: orEmpty(s.Warnings),
		Stats: statsView{
			RequestedHz: s.Stats.RequestedHz,
			AchievedHz:  s.Stats.AchievedHz,
			JitterMS:    s.Stats.JitterMS,
			MeanMS:      float64(s.Stats.Transaction.Mean.Microseconds()) / 1000,
			P95MS:       float64(s.Stats.Transaction.P95.Microseconds()) / 1000,
			MaxMS:       float64(s.Stats.Transaction.Max.Microseconds()) / 1000,
			Cycles:      s.Stats.Cycles,
			Replies:     s.Stats.Replies,
			Timeouts:    s.Stats.Timeouts,
			ReplyRate:   s.Stats.ReplyRate,
			WatchdogMS:  s.Stats.WatchdogMS,
			Consecutive: s.Stats.ConsecutiveTimeouts,
		},
	}

	if s.Identity != nil {
		out.Identity = &identityView{
			Vendor:       s.Identity.Vendor,
			Board:        s.Identity.Board,
			Firmware:     s.Identity.Firmware,
			SerialNumber: s.Identity.SerialNumber,
			Status:       string(s.Identity.Status),
			Simulated:    s.Identity.Simulated,
			PositionMin:  number(s.Identity.PositionMin),
			PositionMax:  number(s.Identity.PositionMax),
		}
	}
	if s.ControllerBounds != nil {
		out.ControllerBounds = []float64{s.ControllerBounds[0], s.ControllerBounds[1]}
	}
	return out
}

func newEventViews(events []session.Event) []eventView {
	out := make([]eventView, 0, len(events))
	for _, event := range events {
		out = append(out, eventView{
			Seq:     event.Seq,
			At:      event.At.UnixMilli(),
			Level:   string(event.Level),
			Message: event.Message,
			Detail:  event.Detail,
		})
	}
	return out
}

func newSampleViews(samples []session.Sample) []sampleView {
	out := make([]sampleView, 0, len(samples))
	for _, sample := range samples {
		out = append(out, sampleView{
			Seq:      sample.Seq,
			T:        sample.Elapsed.Seconds(),
			Position: number(sample.Position),
			Velocity: number(sample.Velocity),
			Current:  number(sample.Current),
			Torque:   number(sample.Torque),
		})
	}
	return out
}

// orEmpty keeps a nil slice out of the JSON, so the page can iterate without
// a null check on every list.
func orEmpty(items []string) []string {
	if items == nil {
		return []string{}
	}
	return items
}
