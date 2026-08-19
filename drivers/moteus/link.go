// Package moteus implements the mjbots moteus brushless controller family in
// pure Go: the mjlib multiplex register protocol, the CAN-FD framing it rides
// on, and the tunnelled diagnostic console.
//
// The wire details follow the moteus reference documentation
// (https://github.com/mjbots/moteus/blob/main/docs/reference.md).  Everything
// this package exposes upward is in framework units — revolutions, rev/s, Nm,
// A, V, °C — so callers work against motor.Link and never see a register.
//
//	moteus.Link  ->  moteus.Controller  ->  can.Bus  ->  fdcanusb / socketcan
package moteus

import (
	"context"
	"fmt"
	"math"
	"strconv"
	"time"

	"drlabs/motor"
	"drlabs/transport/can"
	"drlabs/transport/fdcanusb"
)

func nan() float64 { return math.NaN() }

func init() {
	motor.Register("moteus", "mjbots moteus brushless controller over CAN-FD", newLink)
	motor.Register("moteus-sim", "simulated moteus, for exercising control and safety paths", newSimLink)
}

// Link is a motor.Link backed by one moteus controller.
type Link struct {
	controller *Controller
	bus        can.Bus
	ownsBus    bool
	config     motor.Config

	fast    QueryResolution
	full    QueryResolution
	verify  QueryResolution
	timeout time.Duration

	// sent remembers the sticky command registers as the controller last had
	// them, so an unchanged parameter costs no frame budget.
	sent sentParams
}

// sentParams tracks the command registers that keep their value until they are
// written again.
//
// Re-sending them every cycle is not merely wasteful, it is the difference
// between a frame that fits and one that does not: the full parameter set is
// about 48 bytes, a wide telemetry query is another 18, and a CAN-FD frame
// holds 64.  The controller answers an over-long frame by not receiving it.
type sentParams struct {
	valid bool

	feedforwardTorque    float64
	kpScale              float64
	kdScale              float64
	iLimitScale          float64
	velocityLimit        float64
	accelLimit           float64
	stopPosition         float64
	ignorePositionBounds float64
}

// reset forgets what the controller was told, so the next command re-sends
// everything.  Used on connect, when the controller's registers are whatever a
// previous session or a power cycle left in them.
func (p *sentParams) reset() { *p = sentParams{} }

// sticky returns a field only when it has to go out: on a full refresh, or
// when the value differs from what the controller was last told.
func sticky(last *float64, value float64, full bool) opt {
	if math.IsNaN(value) {
		return unset()
	}
	if p := *last; !full && p == value {
		return unset()
	}
	*last = value
	return val(value)
}

// newLink builds a moteus Link from a Config.
//
// Recognised options:
//
//	source_id   CAN identifier this host answers to (default 0)
//	can_prefix  13-bit bus prefix, decimal (default 0)
//	timeout_ms  transaction timeout (default 2000)
//	no_brs      "1" to disable the CAN-FD bit rate switch
func newLink(cfg motor.Config) (motor.Link, error) {
	if cfg.ID < 1 || cfg.ID > 126 {
		return nil, fmt.Errorf("moteus: CAN id %d is out of range (1..126)", cfg.ID)
	}
	if err := checkOptions(cfg, "source_id", "can_prefix", "timeout_ms", "no_brs"); err != nil {
		return nil, err
	}

	sourceID, err := intOption(cfg, "source_id", 0, 0, 126)
	if err != nil {
		return nil, err
	}
	canPrefix, err := intOption(cfg, "can_prefix", 0, 0, 0x1fff)
	if err != nil {
		return nil, err
	}
	timeoutMS, err := intOption(cfg, "timeout_ms", int(DefaultTimeout/time.Millisecond), 1, 60000)
	if err != nil {
		return nil, err
	}

	return &Link{
		config:  cfg,
		fast:    FastQuery(),
		full:    FullQuery(),
		verify:  VerifyQuery(),
		timeout: time.Duration(timeoutMS) * time.Millisecond,
		controller: &Controller{
			ID:                 uint8(cfg.ID),
			SourceID:           uint8(sourceID),
			CANPrefix:          uint16(canPrefix),
			PositionResolution: DefaultPositionResolution(),
			Timeout:            time.Duration(timeoutMS) * time.Millisecond,
		},
	}, nil
}

// NewLinkWithBus builds a Link on a bus the caller already owns.  Close will
// not close that bus — whoever opened it keeps the responsibility, which is
// what makes it possible to drive several controllers over one adapter.
func NewLinkWithBus(bus can.Bus, id uint8) *Link {
	link := &Link{
		bus:        bus,
		ownsBus:    false,
		fast:       FastQuery(),
		full:       FullQuery(),
		verify:     VerifyQuery(),
		timeout:    DefaultTimeout,
		controller: NewController(bus, id),
	}
	link.config = motor.Config{Driver: "moteus", ID: int(id)}
	return link
}

// Kind identifies the driver.
func (l *Link) Kind() string { return "moteus" }

// Controller exposes the low-level interface, for callers that need registers
// or the diagnostic console.
func (l *Link) Controller() *Controller { return l.controller }

// Describe reports the connection for display and logging.
func (l *Link) Describe() map[string]string {
	out := map[string]string{
		"driver": "moteus",
		"can_id": strconv.Itoa(int(l.controller.ID)),
	}
	if describer, ok := l.bus.(can.Describer); ok {
		for key, value := range describer.Describe() {
			out[key] = value
		}
	} else if l.config.Port != "" {
		out["port"] = l.config.Port
	}
	return out
}

// Open acquires the transport and leaves the controller stopped.
//
// The opening stop is not a formality.  A controller that was left driving by
// a previous session keeps driving until it is told otherwise, and its command
// registers still hold that session's limits — so the connect sequence stops
// the axis and drains whatever was already queued before anything else.
func (l *Link) Open(ctx context.Context) error {
	if l.bus == nil {
		port := l.config.Port
		if port == "" {
			detected, err := autoDetectPort()
			if err != nil {
				return err
			}
			port = detected
		}
		device, err := fdcanusb.Open(port)
		if err != nil {
			return err
		}
		device.DisableBRS = l.config.Option("no_brs", "") == "1"
		l.bus = device
		l.ownsBus = true
		l.controller.Bus = device
	}

	if err := l.bus.Drain(20 * time.Millisecond); err != nil {
		return err
	}
	// The controller's command registers hold whatever a previous session or a
	// power cycle left in them, so nothing may be assumed to be already set.
	l.sent.reset()

	values, err := l.controller.Stop(ctx, &l.fast)
	if err != nil {
		l.closeBus()
		return err
	}
	if values == nil {
		l.closeBus()
		return &motor.LinkError{
			Op: "open",
			Err: fmt.Errorf("no reply from CAN id %d — check the id, the bus wiring and "+
				"termination, and that the controller is powered", l.controller.ID),
		}
	}
	return nil
}

// Close stops the motor and, if this Link opened the bus, releases it.
func (l *Link) Close() error {
	if l.bus == nil {
		return nil
	}
	// Leave the controller in the safest state it has, but do not let a
	// failure here prevent the transport from being released.
	stopCtx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	_, _ = l.controller.Stop(stopCtx, nil)
	cancel()

	return l.closeBus()
}

func (l *Link) closeBus() error {
	bus := l.bus
	l.bus = nil
	l.controller.Bus = nil
	if bus == nil || !l.ownsBus {
		return nil
	}
	return bus.Close()
}

// Identify reads what the controller can tell us about itself.
//
// Model number, firmware version and serial number come from registers.  The
// stored travel bounds come from the diagnostic console, because they are
// configuration rather than state.  Failures here are not fatal: an
// unidentified controller can still be driven, it just cannot be checked
// against a known envelope, and the session says so.
func (l *Link) Identify(ctx context.Context) (*motor.Identity, error) {
	identity := motor.NewIdentity("moteus")

	values, err := l.controller.ReadRegisters(ctx, map[Register]Resolution{
		RegModelNumber:     Int32,
		RegFirmwareVersion: Int32,
	})
	if err == nil {
		if model, ok := values[RegModelNumber]; ok && !math.IsNaN(model) {
			identity.ModelNumber = uint32(model)
		}
		if version, ok := values[RegFirmwareVersion]; ok && !math.IsNaN(version) {
			identity.Firmware = formatFirmware(uint32(version))
		}
	}

	if serial, err := l.controller.ReadRegisters(ctx, map[Register]Resolution{
		RegSerialNumber1: Int32,
		RegSerialNumber2: Int32,
		RegSerialNumber3: Int32,
	}); err == nil {
		identity.SerialNumber = formatSerial(serial)
	}

	// The console is slow, so it is read once here and never inside the
	// control loop.
	//
	// The order matters.  A board that was emitting telemetry when we
	// connected is filling the same channel that carries command responses,
	// so the reply to "tel stop" arrives interleaved with binary records and
	// frequently cannot be parsed at all.  The command is issued for its
	// effect and its result is discarded; what makes the channel usable is
	// draining on both sides of it.
	stream := NewStream(l.controller)
	stream.Drain(ctx)
	_, _ = stream.Command(ctx, "tel stop")

	// If the console will not go quiet, stop here rather than reading it
	// anyway.  Every console read is a read of "whatever comes next", so a
	// controller that is still emitting answers each question with somebody
	// else's output — and a travel bound invented that way is worse than an
	// absent one, because the preflight check would then trust it.
	if !stream.Drain(ctx) {
		identity.Board = BoardName(identity.Family, identity.HWRev)
		identity.Status = BoardStatus(identity.Family, identity.HWRev)
		identity.Known = identity.ModelNumber != 0 || identity.Firmware != ""
		return identity, nil
	}

	// The board family and revision have no register; they live only in the
	// firmware telemetry record.  Without them the board cannot be checked
	// against a known envelope, and every connection has to be waved through
	// by hand — which trains people to wave through the ones that matter.
	if firmware, err := stream.ReadFirmware(ctx); err == nil {
		identity.Family = firmware.Family
		identity.HWRev = firmware.HWRev
		if firmware.Model >= 0 {
			identity.ModelNumber = uint32(firmware.Model)
		}
		if identity.Firmware == "" && firmware.Version >= 0 {
			identity.Firmware = formatFirmware(uint32(firmware.Version))
		}
	}
	if lower, err := stream.ConfGetFloat(ctx, "servopos.position_min"); err == nil {
		identity.PositionMin = lower
	}
	if upper, err := stream.ConfGetFloat(ctx, "servopos.position_max"); err == nil {
		identity.PositionMax = upper
	}

	identity.Board = BoardName(identity.Family, identity.HWRev)
	identity.Status = BoardStatus(identity.Family, identity.HWRev)
	identity.Supported = identity.Status == motor.BoardOK
	identity.Known = identity.ModelNumber != 0 || identity.Firmware != ""
	return identity, nil
}

// Capabilities reports what a moteus can be asked to do.
func (l *Link) Capabilities() motor.Capabilities {
	return motor.Capabilities{
		Modes: []motor.ControlMode{
			motor.CtrlStop, motor.CtrlQuery, motor.CtrlPosition, motor.CtrlVelocity,
			motor.CtrlTorque, motor.CtrlHold, motor.CtrlBrake, motor.CtrlStayWithin,
			motor.CtrlSetOutput,
		},
		MaxRateHz:          CANDocumentedMaxCycleHz,
		HasWatchdog:        true,
		ReportsTemperature: true,
		ReportsCurrent:     true,
		ReportsVoltage:     true,
	}
}

// Envelope reports the controller's documented electrical and motion ratings,
// so the host safety layer can refuse a setting looser than the hardware.
func (l *Link) Envelope() motor.Envelope {
	return motor.Envelope{
		VoltageMin:   R4Envelope.InputVoltageMin,
		VoltageMax:   R4Envelope.InputVoltageMax,
		VoltageRated: R4Envelope.InputVoltageNominal,
		PeakCurrent:  R4Envelope.PeakPhaseCurrent,
		VelocityMax:  R4Envelope.VelocityMax,
	}
}

// Cycle transmits one command and returns the telemetry that came back.
func (l *Link) Cycle(ctx context.Context, cmd motor.Command, profile motor.QueryProfile) (*motor.Telemetry, error) {
	query := l.queryFor(profile)

	// Build the command first, then choose a query that fits beside it.  The
	// two share one 64-byte frame, and the command is not negotiable.
	payload, err := l.commandPayload(cmd)
	if err != nil {
		return nil, err
	}

	query = l.fitQuery(len(payload), query)
	if query != nil {
		data, _ := queryData(*query)
		payload = append(payload, data...)
	}

	values, err := l.controller.transact(ctx, payload, query != nil)
	if err != nil {
		return nil, err
	}
	return decode(values), nil
}

// commandPayload renders one command with no telemetry query attached.
func (l *Link) commandPayload(cmd motor.Command) ([]byte, error) {
	pr := l.controller.PositionResolution

	switch cmd.Mode {
	case motor.CtrlStop:
		return stopFrame(nil), nil
	case motor.CtrlQuery:
		return nil, nil
	case motor.CtrlBrake:
		return brakeFrame(nil), nil
	case motor.CtrlHold:
		return zeroVelocityFrame(cmd.KdScale, pr, nil), nil
	case motor.CtrlSetOutput:
		position := cmd.Position
		if math.IsNaN(position) {
			position = 0
		}
		return setOutputFrame(RegSetOutputExact, position, nil), nil
	case motor.CtrlStayWithin:
		return stayWithinFrame(l.stayWithin(cmd), pr, nil), nil
	case motor.CtrlPosition, motor.CtrlVelocity, motor.CtrlTorque:
		return positionFrame(l.position(cmd), pr, nil), nil
	}
	return nil, motor.ErrUnsupportedMode
}

// fitQuery returns the widest query that still fits alongside a command of the
// given size, narrowing to the fast set and then to nothing.
//
// The frame budget belongs to this layer, because this is the layer that knows
// both halves of it.  Leaving it to the caller means every caller has to know
// the size of every parameter set — and the one that gets it wrong produces a
// frame the adapter simply refuses, which surfaces as a dead link rather than
// as missing telemetry.
//
// Narrowing costs a cycle of the wider registers, which the next rotation
// picks up.  Overflowing costs the connection.
func (l *Link) fitQuery(commandBytes int, requested *QueryResolution) *QueryResolution {
	if requested == nil {
		return nil
	}
	if commandBytes+l.querySize(requested) <= can.MaxFDPayload {
		return requested
	}
	if commandBytes+l.querySize(&l.fast) <= can.MaxFDPayload {
		return &l.fast
	}
	// A command this large leaves no room to ask anything.  It still has to go
	// out; the cycle simply reports no telemetry.
	return nil
}

// querySize is the on-wire length of a query's read subframes.
func (l *Link) querySize(query *QueryResolution) int {
	data, _ := queryData(*query)
	return len(data)
}

func (l *Link) queryFor(profile motor.QueryProfile) *QueryResolution {
	switch profile {
	case motor.ProfileFull:
		return &l.full
	case motor.ProfileVerify:
		return &l.verify
	default:
		return &l.fast
	}
}

// position translates a framework command into moteus position-mode fields.
//
// Velocity and torque modes both ride on position mode: a velocity command is
// a position command with a NaN target, and a torque command is one with no
// position or velocity opinion and a feedforward torque.  That is how the
// controller itself is built, so expressing it any other way would mean
// fighting the firmware.
func (l *Link) position(cmd motor.Command) positionCommand {
	// Position, velocity, the torque ceiling and the watchdog go out every
	// cycle: the first two are the command, and the last two are the limits
	// that must never be left to chance.  Everything else is sticky, and is
	// sent when it changes or when a full refresh is asked for.
	full := cmd.Full || !l.sent.valid
	l.sent.valid = true

	out := positionCommand{
		Position:             present(cmd.Position),
		Velocity:             present(cmd.Velocity),
		MaximumTorque:        torqueLimit(cmd),
		WatchdogTimeout:      present(cmd.WatchdogTimeout),
		FeedforwardTorque:    sticky(&l.sent.feedforwardTorque, cmd.FeedforwardTorque, full),
		KpScale:              sticky(&l.sent.kpScale, cmd.KpScale, full),
		KdScale:              sticky(&l.sent.kdScale, cmd.KdScale, full),
		StopPosition:         sticky(&l.sent.stopPosition, cmd.StopPosition, full),
		VelocityLimit:        sticky(&l.sent.velocityLimit, cmd.VelocityLimit, full),
		AccelLimit:           sticky(&l.sent.accelLimit, cmd.AccelLimit, full),
		ILimitScale:          sticky(&l.sent.iLimitScale, cmd.ILimitScale, full),
		IgnorePositionBounds: sticky(&l.sent.ignorePositionBounds, boolToFloat(cmd.IgnorePositionBounds), full),
	}

	switch cmd.Mode {
	case motor.CtrlVelocity:
		// A NaN position must be *sent*, not omitted.  It is what tells the
		// controller there is no position target, so the axis holds the
		// commanded velocity; omitting the register would leave the previous
		// position command in force and the axis would servo back to it.
		out.Position = val(nan())
		if !motor.Valid(cmd.Velocity) {
			out.Velocity = val(0)
		}
	case motor.CtrlTorque:
		// Neither a position nor a velocity opinion: the feedforward torque
		// is the whole command.
		out.Position = val(nan())
		out.Velocity = val(nan())
	}

	return out
}

func (l *Link) stayWithin(cmd motor.Command) stayWithinCommand {
	return stayWithinCommand{
		LowerBound:           present(cmd.LowerBound),
		UpperBound:           present(cmd.UpperBound),
		FeedforwardTorque:    present(cmd.FeedforwardTorque),
		KpScale:              present(cmd.KpScale),
		KdScale:              present(cmd.KdScale),
		MaximumTorque:        torqueLimit(cmd),
		WatchdogTimeout:      present(cmd.WatchdogTimeout),
		ILimitScale:          present(cmd.ILimitScale),
		IgnorePositionBounds: val(boolToFloat(cmd.IgnorePositionBounds)),
	}
}

// torqueLimit resolves the three ways a command can talk about maximum torque:
// a number, a deliberate hand-back to the controller's own limit (the NaN
// sentinel), or no opinion at all.
func torqueLimit(cmd motor.Command) opt {
	if cmd.DeferTorqueToController {
		return val(nan())
	}
	return present(cmd.MaxTorque)
}

func boolToFloat(v bool) float64 {
	if v {
		return 1
	}
	return 0
}

// formatFirmware renders the packed firmware version register.
func formatFirmware(version uint32) string {
	return fmt.Sprintf("%d.%d.%d", (version>>16)&0xff, (version>>8)&0xff, version&0xff)
}

// formatSerial joins the three serial number registers.
func formatSerial(values map[Register]float64) string {
	var parts [3]uint32
	for i, register := range []Register{RegSerialNumber1, RegSerialNumber2, RegSerialNumber3} {
		if value, ok := values[register]; ok && !math.IsNaN(value) {
			parts[i] = uint32(int32(value))
		}
	}
	if parts[0] == 0 && parts[1] == 0 && parts[2] == 0 {
		return ""
	}
	return fmt.Sprintf("%08x%08x%08x", parts[0], parts[1], parts[2])
}

// autoDetectPort picks the most likely CAN adapter when none was configured.
func autoDetectPort() (string, error) {
	ports, err := fdcanusb.Detect()
	if err != nil {
		return "", fmt.Errorf("moteus: could not enumerate serial ports: %w", err)
	}
	if len(ports) == 0 {
		return "", fmt.Errorf("moteus: no serial ports found; plug in a CAN-FD adapter or name the port explicitly")
	}
	if ports[0].IsCANAdapter {
		return ports[0].Path, nil
	}
	// Nothing identified itself as an adapter.  Refusing here is better than
	// opening whatever happens to be first — that could be a modem, a GPS, or
	// another machine's console.
	names := make([]string, 0, len(ports))
	for _, port := range ports {
		names = append(names, port.Path)
	}
	return "", fmt.Errorf("moteus: no CAN adapter recognised among %v; name the port explicitly", names)
}

// checkOptions rejects unrecognised option keys, so that a typo in a config
// file is reported rather than silently ignored.
func checkOptions(cfg motor.Config, allowed ...string) error {
	for key := range cfg.Options {
		known := false
		for _, name := range allowed {
			if key == name {
				known = true
				break
			}
		}
		if !known {
			return fmt.Errorf("moteus: unknown option %q (accepted: %v)", key, allowed)
		}
	}
	return nil
}

func intOption(cfg motor.Config, key string, def, low, high int) (int, error) {
	text := cfg.Option(key, "")
	if text == "" {
		return def, nil
	}
	value, err := strconv.Atoi(text)
	if err != nil {
		return 0, fmt.Errorf("moteus: option %s=%q is not a number", key, text)
	}
	if value < low || value > high {
		return 0, fmt.Errorf("moteus: option %s=%d is out of range (%d..%d)", key, value, low, high)
	}
	return value, nil
}
