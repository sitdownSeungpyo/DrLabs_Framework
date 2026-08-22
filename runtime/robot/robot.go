package robot

import (
	"context"
	"fmt"
	"sort"
	"strings"
	"sync"
	"time"

	"drlabs/drivers/dynamixel"
	"drlabs/drivers/moteus"
	"drlabs/drivers/robstride"
	"drlabs/motor"
	"drlabs/runtime/safety"
	"drlabs/runtime/session"
	"drlabs/transport/can"
	"drlabs/transport/fdcanusb"
	"drlabs/transport/serial"
)

// Robot is a whole machine: named joints, the motors behind them, and the
// buses those are spread across.
//
// Each joint runs its own control loop against a shared bus.  The alternative —
// one loop per bus, cycling every joint in turn — would give tighter coupling
// between joints on the same wire, and is the right shape if coordinated
// motion ever needs to be simultaneous to better than a cycle.  What it costs
// is the whole of runtime/session: arming, the trips, the trajectory
// sequencer, the emergency stop, all of which are written per axis.  Sharing
// the bus with atomic transactions keeps all of that and gives up only exact
// phase alignment between joints, which at 200 Hz is a few hundred
// microseconds of skew.
type Robot struct {
	Config *Config

	mu     sync.RWMutex
	joints map[string]*Joint
	order  []string

	// buses are closed in reverse order of opening, after every joint has let
	// go of them.
	buses  map[string]io
	opened []string
}

// io is the closable part of a bus, whatever kind it is.
type io interface{ Close() error }

// Joint is one named degree of freedom.
type Joint struct {
	Name    string
	Config  JointConfig
	Mapping Mapping

	// Session owns this joint's control loop.  It is exposed because
	// everything the framework can do to one axis is already on it, and
	// wrapping all of it again would only hide capability.
	Session *session.Session

	link motor.Link
}

// New builds a robot from a description, without touching any hardware.
func New(config *Config) (*Robot, error) {
	if err := config.Validate(); err != nil {
		return nil, err
	}
	return &Robot{
		Config: config,
		joints: map[string]*Joint{},
		buses:  map[string]io{},
	}, nil
}

// BringUp opens every bus, connects every joint, and energises the machine.
//
// It is all or nothing.  A humanoid with one dead shoulder is more dangerous
// than one that never came up: the rest of the machine will happily hold poses
// that assume the missing joint is holding too.  So the first joint that
// cannot be brought up takes the whole robot down with it, torque off, and
// says which one and why.
func (r *Robot) BringUp(ctx context.Context) error {
	r.mu.Lock()
	defer r.mu.Unlock()

	if len(r.joints) > 0 {
		return fmt.Errorf("robot: already brought up")
	}

	if err := r.bringUpLocked(ctx); err != nil {
		// Unwind whatever came up before the failure.
		r.shutdownLocked()
		return err
	}
	return nil
}

func (r *Robot) bringUpLocked(ctx context.Context) error {
	base := safety.DefaultConfig()

	for _, config := range r.Config.Joints {
		link, err := r.linkFor(config)
		if err != nil {
			return fmt.Errorf("joint %q: %w", config.Name, err)
		}

		mapping := mappingOf(config)
		envelope := mapping.safetyConfig(config.Limits, base)

		joint := &Joint{
			Name:    config.Name,
			Config:  config,
			Mapping: mapping,
			link:    link,
			Session: session.New(session.Options{
				// The bus decides the rate, because the bus is what runs out.
				RateHz:      r.Config.RateForBus(config.Bus),
				Safety:      &envelope,
				AutoArm:     true,
				InitialMode: motor.CtrlPosition,
				// The joint's own units, so that a limit written in radians
				// is not reported back in revolutions.
				Units: jointUnits{mapping: mapping},
			}),
		}

		if err := joint.Session.ConnectLink(ctx, link, true); err != nil {
			return fmt.Errorf("joint %q (%s id %d on %s): %w",
				config.Name, config.Driver, config.ID, config.Bus, err)
		}

		// The working parameters come from the joint's own declaration, not
		// from the session defaults.  A joint that says it needs six newton
		// metres and is then driven at the framework's cautious 0.3 would sag
		// under its own arm and never say why — the ceiling would be right and
		// the command still wrong.
		params := session.DefaultParams()
		params.MaxTorque = envelope.MaxTorque
		params.VelocityLimit = envelope.MaxVelocity
		params.AccelLimit = envelope.AccelLimit
		joint.Session.SetParams(params)

		r.joints[config.Name] = joint
		r.order = append(r.order, config.Name)
	}

	return r.waitUntilArmed(ctx)
}

// waitUntilArmed blocks until every joint has energised, or reports the ones
// that would not.
//
// Arming is not instant: the session waits for telemetry and for preflight to
// be clear, and a supply that is still settling can hold a joint back for a
// moment.  Reporting "brought up" before that is true would hand the upper
// layer a machine that is only partly live.
func (r *Robot) waitUntilArmed(ctx context.Context) error {
	deadline := time.Now().Add(10 * time.Second)

	for {
		var waiting []string
		for _, name := range r.order {
			status := r.joints[name].Session.Status()
			if status.EStop.Latched {
				return fmt.Errorf("joint %q stopped while coming up: %s",
					name, status.EStop.Reason)
			}
			if !status.Armed {
				reason := "still arming"
				if blockers := status.ArmBlockers; len(blockers) > 0 {
					reason = blockers[0]
				}
				waiting = append(waiting, fmt.Sprintf("%s (%s)", name, reason))
			}
		}
		if len(waiting) == 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("joints did not energise: %s", strings.Join(waiting, "; "))
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(50 * time.Millisecond):
		}
	}
}

// linkFor builds the driver link for one joint, opening its bus if this is the
// first joint to need it.
func (r *Robot) linkFor(config JointConfig) (motor.Link, error) {
	bus, declared := r.Config.Bus(config.Bus)
	if !declared && config.Bus != "" {
		return nil, fmt.Errorf("bus %q is not defined", config.Bus)
	}

	switch config.Driver {
	case "moteus":
		shared, err := r.canBus(bus)
		if err != nil {
			return nil, err
		}
		return moteus.NewLinkWithBus(shared, uint8(config.ID)), nil

	case "robstride":
		model, err := robstride.LookupModel(config.Options["model"])
		if err != nil {
			return nil, err
		}
		shared, err := r.canBus(bus)
		if err != nil {
			return nil, err
		}
		return robstride.NewLinkWithBus(shared, uint8(config.ID), model), nil

	case "dynamixel":
		port, err := r.serialPort(bus)
		if err != nil {
			return nil, err
		}
		// The zero Profile leaves the choice to the servo's model number.
		// Naming one in the description is still worth doing — it is what
		// catches a joint being rebuilt with a different servo — but a
		// description that says nothing must not be treated as if it had.
		var profile dynamixel.Profile
		if name := config.Options["profile"]; name != "" {
			found, known := dynamixel.Profiles[name]
			if !known {
				return nil, fmt.Errorf("unknown dynamixel profile %q", name)
			}
			profile = found
		}
		return dynamixel.NewLinkWithPort(port, uint8(config.ID), profile), nil
	}

	// Anything else goes through the registry, which means its own transport
	// and no sharing — correct for a driver this package does not know how to
	// put on a shared bus.
	return motor.New(motor.Config{
		Driver:  config.Driver,
		Port:    bus.Port,
		Baud:    bus.Baud,
		ID:      config.ID,
		Options: config.Options,
	})
}

// canBus opens a CAN bus once and hands the same shared wrapper to every joint
// that names it.
func (r *Robot) canBus(config BusConfig) (can.Bus, error) {
	if existing, open := r.buses[config.Name]; open {
		shared, ok := existing.(can.Bus)
		if !ok {
			return nil, fmt.Errorf("bus %q is already open as a different kind of transport", config.Name)
		}
		return shared, nil
	}
	if config.Transport != "fdcanusb" {
		return nil, fmt.Errorf("bus %q has transport %q; a CAN driver needs fdcanusb", config.Name, config.Transport)
	}

	port := config.Port
	if port == "" {
		detected, err := fdcanusb.Detect()
		if err != nil || len(detected) == 0 || !detected[0].IsCANAdapter {
			return nil, fmt.Errorf("bus %q has no port and no CAN adapter was recognised", config.Name)
		}
		port = detected[0].Path
	}

	device, err := fdcanusb.Open(port)
	if err != nil {
		return nil, err
	}
	shared := can.NewShared(device)
	r.buses[config.Name] = shared
	r.opened = append(r.opened, config.Name)
	return shared, nil
}

// serialPort opens a serial chain once and shares it.
func (r *Robot) serialPort(config BusConfig) (serial.Port, error) {
	if existing, open := r.buses[config.Name]; open {
		port, ok := existing.(serial.Port)
		if !ok {
			return nil, fmt.Errorf("bus %q is already open as a different kind of transport", config.Name)
		}
		return port, nil
	}
	if config.Transport != "serial" {
		return nil, fmt.Errorf("bus %q has transport %q; a Dynamixel chain needs serial", config.Name, config.Transport)
	}
	if config.Port == "" {
		return nil, fmt.Errorf("bus %q has no port", config.Name)
	}

	baud := config.Baud
	if baud == 0 {
		baud = 1000000
	}
	opened, err := serial.Open(config.Port, baud)
	if err != nil {
		return nil, err
	}
	shared := serial.NewShared(opened)
	r.buses[config.Name] = shared
	r.opened = append(r.opened, config.Name)
	return shared, nil
}

// Shutdown releases every joint and closes every bus.
func (r *Robot) Shutdown() {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.shutdownLocked()
}

func (r *Robot) shutdownLocked() {
	// Joints first: each disarms and stops its motor while the bus it needs is
	// still open.
	for _, name := range r.order {
		joint := r.joints[name]
		joint.Session.Disarm("robot shutting down")
		_ = joint.Session.Disconnect()
	}
	for i := len(r.opened) - 1; i >= 0; i-- {
		if bus, open := r.buses[r.opened[i]]; open {
			_ = bus.Close()
		}
	}
	r.joints = map[string]*Joint{}
	r.order = nil
	r.buses = map[string]io{}
	r.opened = nil
}

// Joint returns one joint by name.
func (r *Robot) Joint(name string) (*Joint, error) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	joint, known := r.joints[name]
	if !known {
		return nil, fmt.Errorf("robot: no joint named %q (have %s)",
			name, strings.Join(r.jointNamesLocked(), ", "))
	}
	return joint, nil
}

// JointNames lists the joints in declaration order.
func (r *Robot) JointNames() []string {
	r.mu.RLock()
	defer r.mu.RUnlock()
	return r.jointNamesLocked()
}

func (r *Robot) jointNamesLocked() []string {
	names := make([]string, len(r.order))
	copy(names, r.order)
	return names
}

// --- commands, in joint space ---------------------------------------------

// SetPosition commands one joint to an angle in radians.
func (r *Robot) SetPosition(name string, radians float64) error {
	joint, err := r.Joint(name)
	if err != nil {
		return err
	}
	if err := joint.Session.SetMode(motor.CtrlPosition); err != nil {
		return fmt.Errorf("joint %q: %w", name, err)
	}
	joint.Session.SetPosition(joint.Mapping.MotorRevolutions(radians))
	return nil
}

// SetPositions commands several joints at once.
//
// The joints are commanded as fast as the loop can pick them up rather than
// simultaneously — each runs its own cycle — so this is coordinated to within
// a control period, not to the microsecond.  For a pose that is right; for
// anything that needs true synchrony, the joints would have to share a loop.
//
// An unknown joint name fails the whole call before anything is commanded,
// because a pose applied to half a machine is not a pose.
func (r *Robot) SetPositions(targets map[string]float64) error {
	// Everything is checked before anything is commanded, and the joints are
	// visited in the order they were declared.
	//
	// Both halves of that matter.  Ranging a map applies a pose in whatever
	// order Go feels like today, so a joint that refuses leaves a different
	// subset of the machine moved every time — the same command, the same
	// state, a different posture.  And a posture assembled out of the joints
	// that happened to be reached first is not the pose anybody asked for.
	r.mu.RLock()
	ordered := make([]string, 0, len(targets))
	for _, name := range r.order {
		if _, wanted := targets[name]; wanted {
			ordered = append(ordered, name)
		}
	}
	if len(ordered) != len(targets) {
		for name := range targets {
			if _, known := r.joints[name]; !known {
				r.mu.RUnlock()
				return fmt.Errorf("robot: no joint named %q", name)
			}
		}
	}
	for _, name := range ordered {
		if err := r.joints[name].Session.CanCommand(motor.CtrlPosition); err != nil {
			r.mu.RUnlock()
			return fmt.Errorf("joint %q: %w (the pose was not applied)", name, err)
		}
	}
	r.mu.RUnlock()

	for _, name := range ordered {
		if err := r.SetPosition(name, targets[name]); err != nil {
			return err
		}
	}
	return nil
}

// SetVelocity commands one joint to a velocity in radians per second.
func (r *Robot) SetVelocity(name string, radiansPerSecond float64) error {
	joint, err := r.Joint(name)
	if err != nil {
		return err
	}
	if err := joint.Session.SetMode(motor.CtrlVelocity); err != nil {
		return fmt.Errorf("joint %q: %w", name, err)
	}
	joint.Session.SetVelocity(joint.Mapping.MotorRevPerSecond(radiansPerSecond))
	return nil
}

// MoveTo runs a profiled move on one joint, in joint units.
func (r *Robot) MoveTo(name string, radians, maxVelocity, accel float64) error {
	joint, err := r.Joint(name)
	if err != nil {
		return err
	}
	return joint.Session.StartTrajectory(
		joint.Mapping.MotorRevolutions(radians),
		abs(joint.Mapping.MotorRevPerSecond(maxVelocity)),
		abs(joint.Mapping.MotorAccel(accel)))
}

// Hold puts a joint into a zero-velocity hold.
func (r *Robot) Hold(name string) error {
	joint, err := r.Joint(name)
	if err != nil {
		return err
	}
	return joint.Session.SetMode(motor.CtrlHold)
}

// Stop releases one joint's bridge.
func (r *Robot) Stop(name string) error {
	joint, err := r.Joint(name)
	if err != nil {
		return err
	}
	return joint.Session.SetMode(motor.CtrlStop)
}

// EStop latches an emergency stop on every joint.
//
// It is the one command that never fails and never asks: whatever state the
// machine is in, every axis is told to stop.
func (r *Robot) EStop(reason string) {
	r.mu.RLock()
	defer r.mu.RUnlock()
	for _, name := range r.order {
		r.joints[name].Session.EStop("robot", "Emergency stop", reason)
	}
}

// ResetEStop clears the latch on every joint.  The joints re-energise on their
// own afterwards, because each session is set to arm when it can.
func (r *Robot) ResetEStop() error {
	r.mu.RLock()
	defer r.mu.RUnlock()
	for _, name := range r.order {
		if err := r.joints[name].Session.ResetEStop(); err != nil {
			return fmt.Errorf("joint %q: %w", name, err)
		}
	}
	return nil
}

// Heartbeat feeds every joint's deadman, for configurations that use one.
func (r *Robot) Heartbeat() {
	r.mu.RLock()
	defer r.mu.RUnlock()
	for _, name := range r.order {
		r.joints[name].Session.Heartbeat()
	}
}

// --- status ----------------------------------------------------------------

// JointState is one joint's state, in joint units.
type JointState struct {
	Name string `json:"name"`

	Armed bool   `json:"armed"`
	Mode  string `json:"mode"`
	EStop bool   `json:"estop"`
	// EStopReason is empty unless the stop is latched.
	EStopReason string `json:"estop_reason,omitempty"`

	PositionRad *float64 `json:"position_rad"`
	VelocityRad *float64 `json:"velocity_rad_s"`
	TorqueNm    *float64 `json:"torque_nm"`
	CurrentA    *float64 `json:"current_a"`
	VoltageV    *float64 `json:"voltage_v"`
	TempC       *float64 `json:"temperature_c"`

	TargetRad float64 `json:"target_rad"`

	Fault     int    `json:"fault"`
	FaultName string `json:"fault_name,omitempty"`

	MovingTo *float64 `json:"moving_to_rad,omitempty"`
	Progress float64  `json:"progress,omitempty"`

	RateHz float64 `json:"rate_hz"`

	// Blocked carries the preflight reasons while a joint is not armed.
	Blocked []string `json:"blocked,omitempty"`
}

// State returns every joint's state, in declaration order.
func (r *Robot) State() []JointState {
	r.mu.RLock()
	defer r.mu.RUnlock()

	states := make([]JointState, 0, len(r.order))
	for _, name := range r.order {
		states = append(states, r.joints[name].state())
	}
	return states
}

func (j *Joint) state() JointState {
	status := j.Session.Status()
	telemetry := status.Telemetry

	state := JointState{
		Name:        j.Name,
		Armed:       status.Armed,
		Mode:        status.Mode.String(),
		EStop:       status.EStop.Latched,
		EStopReason: status.EStop.Reason,
		PositionRad: optional(j.Mapping.JointRadians(telemetry.Position)),
		VelocityRad: optional(j.Mapping.JointRadPerSecond(telemetry.Velocity)),
		TorqueNm:    optional(j.Mapping.JointTorque(telemetry.Torque)),
		CurrentA:    optional(telemetry.QCurrent),
		VoltageV:    optional(telemetry.Voltage),
		TempC:       optional(telemetry.Temperature),
		TargetRad:   j.Mapping.JointRadians(status.Setpoint.Position),
		Fault:       telemetry.Fault,
		FaultName:   telemetry.FaultName,
		RateHz:      status.Stats.AchievedHz,
	}
	if !status.Armed {
		state.Blocked = status.ArmBlockers
	}
	if status.Trajectory.Running {
		target := j.Mapping.JointRadians(status.Trajectory.Target)
		state.MovingTo = &target
		state.Progress = status.Trajectory.Progress
	}
	return state
}

// Events collects the log from every joint, newest last, tagged with the joint
// it came from.
func (r *Robot) Events(since map[string]uint64) ([]JointEvent, map[string]uint64) {
	r.mu.RLock()
	defer r.mu.RUnlock()

	seen := map[string]uint64{}
	var out []JointEvent

	for _, name := range r.order {
		from := since[name]
		for _, event := range r.joints[name].Session.Events(from) {
			out = append(out, JointEvent{
				Joint:   name,
				Seq:     event.Seq,
				At:      event.At.UnixMilli(),
				Level:   string(event.Level),
				Message: event.Message,
				Detail:  event.Detail,
			})
			seen[name] = event.Seq
		}
		if _, advanced := seen[name]; !advanced {
			seen[name] = from
		}
	}

	sort.SliceStable(out, func(i, j int) bool { return out[i].At < out[j].At })
	return out, seen
}

// JointEvent is a log line with the joint that produced it.
type JointEvent struct {
	Joint   string `json:"joint"`
	Seq     uint64 `json:"seq"`
	At      int64  `json:"at"`
	Level   string `json:"level"`
	Message string `json:"message"`
	Detail  string `json:"detail,omitempty"`
}

func optional(v float64) *float64 {
	if !motor.Valid(v) {
		return nil
	}
	out := v
	return &out
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}
