package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"net"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"drlabs/motor"
	"drlabs/runtime/safety"
	"drlabs/runtime/session"
)

// The servo role.
//
// This is the framework doing the job it exists for: connect, energise, and
// then execute whatever the layer above sends, for as long as it keeps
// sending.  There is no operator here and no checklist to press through — the
// axis is live from start-up and waiting.
//
// Commands arrive as newline-delimited JSON over TCP.  That is not a rich
// protocol, and deliberately so: it is trivially callable from Python, from
// ROS 2, from a shell, and it needs no dependency on either side.  One object
// per line in, one reply per line back, plus an unsolicited state line at a
// fixed rate so the caller never has to poll.
//
//	{"cmd":"position","value":0.25}   ->  {"ok":true}
//	{"cmd":"velocity","value":-1.0}   ->  {"ok":true}
//	{"cmd":"stop"}                    ->  {"ok":true}
//
// What is *not* given up: the safety envelope still clamps every command, the
// trips still stop the machine, and the controller's own watchdog still runs.
// The framework being subordinate does not make it credulous.

func runServe(args []string) error {
	fs := flag.NewFlagSet("serve", flag.ExitOnError)
	var common commonFlags
	common.register(fs)

	addr := fs.String("addr", "127.0.0.1:7777", "address to listen on for commands")
	rate := fs.Int("rate", 200, "control rate in Hz")
	stateRate := fs.Int("state-rate", 50, "unsolicited state lines per second (0 disables)")

	maxTorque := fs.Float64("max-torque", 0.3, "torque ceiling in Nm")
	velocityLimit := fs.Float64("velocity-limit", 1.0, "velocity limit in rev/s")
	accelLimit := fs.Float64("accel-limit", 5.0, "acceleration limit in rev/s²")
	posMin := fs.Float64("position-min", -0.5, "soft position envelope, minimum in rev")
	posMax := fs.Float64("position-max", 0.5, "soft position envelope, maximum in rev")

	commandTimeout := fs.Float64("command-timeout", 1.0,
		"seconds of silence after which a velocity or torque command reverts to holding position (0 disables)")
	deadman := fs.Float64("deadman", 0,
		"seconds without a client heartbeat before stopping (0 disables; the controller's own watchdog still applies)")

	noArm := fs.Bool("no-auto-arm", false, "connect but do not energise until told to")
	allowUnknown := fs.Bool("allow-unknown-board", false,
		"proceed past a controller this build cannot positively identify")

	if err := fs.Parse(args); err != nil {
		return err
	}

	config := session.DefaultParams()
	config.MaxTorque = *maxTorque
	config.VelocityLimit = *velocityLimit
	config.AccelLimit = *accelLimit

	envelope := defaultSafety(*maxTorque, *velocityLimit, *accelLimit, *posMin, *posMax, *deadman)

	s := session.New(session.Options{
		RateHz:      *rate,
		Safety:      &envelope,
		AutoArm:     !*noArm,
		InitialMode: motor.CtrlPosition,
	})

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	if err := s.Connect(ctx, common.config(), *allowUnknown); err != nil {
		printEvents(s, 0)
		return err
	}
	// Whatever happens from here, the axis is released and the transport
	// closed before this process goes away.
	defer func() {
		s.Disarm("servo shutting down")
		_ = s.Disconnect()
	}()

	s.SetParams(config)

	listener, err := net.Listen("tcp", *addr)
	if err != nil {
		return fmt.Errorf("cannot listen on %s: %w", *addr, err)
	}
	defer listener.Close()

	srv := &servo{
		session:        s,
		stateRate:      *stateRate,
		commandTimeout: *commandTimeout,
	}

	fmt.Printf("drlabs servo on %s\n", listener.Addr())
	fmt.Printf("  driver %s · id %d%s · %d Hz\n", common.driver, common.id, portSuffix(common.port), *rate)
	if *noArm {
		fmt.Println("  auto-arm disabled — send {\"cmd\":\"arm\"} to energise")
	} else {
		fmt.Println("  energising and waiting for commands")
	}
	fmt.Println("  Ctrl+C to stop")

	go srv.watchCommands(ctx)
	go srv.reportEvents(ctx)

	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go srv.handle(ctx, conn)
		}
	}()

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)
	<-signals
	fmt.Println("\nStopping…")
	return nil
}

func portSuffix(port string) string {
	if port == "" {
		return ""
	}
	return " · " + port
}

// defaultSafety builds the envelope a servo starts with.
//
// The ceilings come from the command line because in this role there is no
// operator to set them later: whoever launches the process is declaring the
// envelope, and the layer above cannot exceed it however it is commanded.
func defaultSafety(maxTorque, maxVelocity, accel, posMin, posMax, deadman float64) safety.Config {
	envelope := safety.DefaultConfig()
	envelope.MaxTorque = maxTorque
	envelope.MaxVelocity = maxVelocity
	envelope.AccelLimit = accel
	envelope.PositionMin = posMin
	envelope.PositionMax = posMax
	// A deadman is off by default here.  Waiting with no client attached is
	// the normal state of a servo, not a fault, and a guard that stopped the
	// axis for being idle would make the role impossible.  What replaces it is
	// the command timeout below, which addresses the case that actually
	// matters: a caller that dies mid-motion.
	envelope.DeadmanTimeout = deadman
	return envelope
}

// servo is the command surface.
type servo struct {
	session        *session.Session
	stateRate      int
	commandTimeout float64

	mu          sync.Mutex
	lastCommand time.Time
	// motionCommanded records whether the last thing asked for was open-ended
	// motion — a velocity or a torque, which continues until countermanded —
	// as opposed to a position, which is self-limiting.
	motionCommanded bool

	clients sync.Map // net.Conn -> struct{}
}

// watchCommands reverts open-ended motion to a position hold when the caller
// goes quiet.
//
// A position command is safe to leave standing: the axis arrives and holds.  A
// velocity or torque command is not — it runs until something stops it, so a
// caller that crashes mid-move would leave the machine turning.  This is the
// servo equivalent of the controller's own watchdog, one level up.
func (s *servo) watchCommands(ctx context.Context) {
	if s.commandTimeout <= 0 {
		return
	}
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.mu.Lock()
			idle := s.motionCommanded && !s.lastCommand.IsZero() &&
				time.Since(s.lastCommand).Seconds() > s.commandTimeout
			if idle {
				s.motionCommanded = false
			}
			s.mu.Unlock()

			if !idle {
				continue
			}
			status := s.session.Status()
			if !status.Armed {
				continue
			}
			if motor.Valid(status.Telemetry.Position) {
				s.session.SetPosition(status.Telemetry.Position)
			}
			_ = s.session.SetMode(motor.CtrlPosition)
		}
	}
}

// reportEvents mirrors the session log to stdout, so a servo running headless
// still says what happened.
func (s *servo) reportEvents(ctx context.Context) {
	ticker := time.NewTicker(200 * time.Millisecond)
	defer ticker.Stop()

	var seq uint64
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			seq = printEvents(s.session, seq)
		}
	}
}

// handle serves one client for as long as it stays connected.
func (s *servo) handle(ctx context.Context, conn net.Conn) {
	defer conn.Close()
	s.clients.Store(conn, struct{}{})
	defer s.clients.Delete(conn)

	encoder := json.NewEncoder(conn)
	var writeMu sync.Mutex
	send := func(value any) error {
		writeMu.Lock()
		defer writeMu.Unlock()
		return encoder.Encode(value)
	}

	clientCtx, stop := context.WithCancel(ctx)
	defer stop()

	if s.stateRate > 0 {
		go s.streamState(clientCtx, send)
	}

	scanner := bufio.NewScanner(conn)
	// A command is small; anything larger is a client that has lost its place
	// in the stream, and reading it would only delay finding that out.
	scanner.Buffer(make([]byte, 0, 4096), 64*1024)

	for scanner.Scan() {
		line := scanner.Bytes()
		if len(line) == 0 {
			continue
		}
		_ = send(s.execute(line))
	}
}

// streamState pushes unsolicited state lines.
func (s *servo) streamState(ctx context.Context, send func(any) error) {
	interval := time.Second / time.Duration(s.stateRate)
	ticker := time.NewTicker(interval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			// A connected client is a supervising client, so its presence
			// feeds the deadman when one is configured.
			s.session.Heartbeat()
			if err := send(newStateLine(s.session.Status())); err != nil {
				return
			}
		}
	}
}

// --- command dispatch ------------------------------------------------------

type command struct {
	Cmd   string   `json:"cmd"`
	Value *float64 `json:"value"`

	Target      *float64 `json:"target"`
	MaxVelocity *float64 `json:"max_velocity"`
	Accel       *float64 `json:"accel"`

	A      *float64 `json:"a"`
	B      *float64 `json:"b"`
	Dwell  *float64 `json:"dwell"`
	Cycles *int     `json:"cycles"`

	MaxTorque     *float64 `json:"max_torque"`
	VelocityLimit *float64 `json:"velocity_limit"`
	AccelLimit    *float64 `json:"accel_limit"`
	KpScale       *float64 `json:"kp_scale"`
	KdScale       *float64 `json:"kd_scale"`
	Feedforward   *float64 `json:"feedforward"`

	Mode string `json:"mode"`
}

type reply struct {
	Type  string `json:"type"`
	OK    bool   `json:"ok"`
	Error string `json:"error,omitempty"`
	Value any    `json:"value,omitempty"`
}

func (s *servo) execute(line []byte) reply {
	var cmd command
	if err := json.Unmarshal(line, &cmd); err != nil {
		return reply{Type: "reply", OK: false, Error: "malformed command: " + err.Error()}
	}

	s.mu.Lock()
	s.lastCommand = time.Now()
	s.mu.Unlock()
	s.session.Heartbeat()

	switch cmd.Cmd {
	case "position":
		if cmd.Value == nil {
			return refuse("position needs a value")
		}
		s.setMotion(false)
		if err := s.session.SetMode(motor.CtrlPosition); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true, Value: s.session.SetPosition(*cmd.Value)}

	case "velocity":
		if cmd.Value == nil {
			return refuse("velocity needs a value")
		}
		s.setMotion(true)
		if err := s.session.SetMode(motor.CtrlVelocity); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true, Value: s.session.SetVelocity(*cmd.Value)}

	case "torque":
		if cmd.Value == nil {
			return refuse("torque needs a value")
		}
		s.setMotion(true)
		params := s.session.Params()
		params.Feedforward = *cmd.Value
		s.session.SetParams(params)
		if err := s.session.SetMode(motor.CtrlTorque); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true, Value: *cmd.Value}

	case "mode":
		mode, valid := motor.ParseControlMode(cmd.Mode)
		if !valid {
			return refuse("unknown mode " + cmd.Mode)
		}
		s.setMotion(mode == motor.CtrlVelocity || mode == motor.CtrlTorque)
		if err := s.session.SetMode(mode); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "stop":
		s.setMotion(false)
		if err := s.session.SetMode(motor.CtrlStop); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "hold":
		s.setMotion(false)
		if err := s.session.SetMode(motor.CtrlHold); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "trajectory":
		if cmd.Target == nil {
			return refuse("trajectory needs a target")
		}
		params := s.session.Params()
		maxVelocity := params.VelocityLimit
		if cmd.MaxVelocity != nil {
			maxVelocity = *cmd.MaxVelocity
		}
		accel := params.AccelLimit
		if cmd.Accel != nil {
			accel = *cmd.Accel
		}
		s.setMotion(false)
		if err := s.session.StartTrajectory(*cmd.Target, maxVelocity, accel); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "trajectory_stop":
		s.session.StopTrajectory()
		return reply{Type: "reply", OK: true}

	case "shuttle":
		if cmd.A == nil || cmd.B == nil {
			return refuse("shuttle needs a and b")
		}
		dwell, cycles := 1.0, 0
		if cmd.Dwell != nil {
			dwell = *cmd.Dwell
		}
		if cmd.Cycles != nil {
			cycles = *cmd.Cycles
		}
		s.setMotion(false)
		s.session.ConfigureShuttle(*cmd.A, *cmd.B, dwell, cycles)
		if err := s.session.StartShuttle(); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "shuttle_stop":
		s.session.StopShuttle()
		return reply{Type: "reply", OK: true}

	case "params":
		params := s.session.Params()
		assign(&params.MaxTorque, cmd.MaxTorque)
		assign(&params.VelocityLimit, cmd.VelocityLimit)
		assign(&params.AccelLimit, cmd.AccelLimit)
		assign(&params.KpScale, cmd.KpScale)
		assign(&params.KdScale, cmd.KdScale)
		assign(&params.Feedforward, cmd.Feedforward)
		return reply{Type: "reply", OK: true, Value: s.session.SetParams(params)}

	case "arm":
		if err := s.session.Arm(true); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "disarm":
		s.setMotion(false)
		s.session.Disarm("commanded")
		return reply{Type: "reply", OK: true}

	case "estop":
		s.setMotion(false)
		s.session.EStop("commanded", "Emergency stop", "Raised by the controlling client.")
		return reply{Type: "reply", OK: true}

	case "reset":
		if err := s.session.ResetEStop(); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "zero":
		if err := s.session.ZeroOutput(0); err != nil {
			return refuse(err.Error())
		}
		return reply{Type: "reply", OK: true}

	case "state":
		return reply{Type: "reply", OK: true, Value: newStateLine(s.session.Status())}

	case "ping":
		return reply{Type: "reply", OK: true}
	}

	return refuse("unknown command " + cmd.Cmd)
}

func refuse(message string) reply {
	return reply{Type: "reply", OK: false, Error: message}
}

func assign(dst *float64, src *float64) {
	if src != nil {
		*dst = *src
	}
}

func (s *servo) setMotion(open bool) {
	s.mu.Lock()
	s.motionCommanded = open
	s.mu.Unlock()
}

// --- state line ------------------------------------------------------------

// stateLine is what a client sees on every tick.
//
// Optional measurements are pointers so that an unread sensor crosses as null
// rather than as zero — JSON cannot carry NaN, and a caller that treats a
// missing temperature as 0 °C will draw the wrong conclusion from it.
type stateLine struct {
	Type string `json:"type"`

	Connection string `json:"connection"`
	Armed      bool   `json:"armed"`
	Mode       string `json:"mode"`

	EStop       bool   `json:"estop"`
	EStopReason string `json:"estop_reason,omitempty"`

	Position *float64 `json:"position"`
	Velocity *float64 `json:"velocity"`
	Torque   *float64 `json:"torque"`
	Current  *float64 `json:"current"`
	Voltage  *float64 `json:"voltage"`
	Temp     *float64 `json:"temperature"`

	Fault     int    `json:"fault"`
	FaultName string `json:"fault_name,omitempty"`

	SetpointPosition float64 `json:"setpoint_position"`
	SetpointVelocity float64 `json:"setpoint_velocity"`

	TrajectoryRunning  bool    `json:"trajectory_running"`
	TrajectoryProgress float64 `json:"trajectory_progress"`

	RateHz  float64  `json:"rate_hz"`
	Blocked []string `json:"blocked,omitempty"`
}

func newStateLine(status session.Status) stateLine {
	line := stateLine{
		Type:               "state",
		Connection:         string(status.Connection),
		Armed:              status.Armed,
		Mode:               status.Mode.String(),
		EStop:              status.EStop.Latched,
		EStopReason:        status.EStop.Reason,
		Position:           optional(status.Telemetry.Position),
		Velocity:           optional(status.Telemetry.Velocity),
		Torque:             optional(status.Telemetry.Torque),
		Current:            optional(status.Telemetry.QCurrent),
		Voltage:            optional(status.Telemetry.Voltage),
		Temp:               optional(status.Telemetry.Temperature),
		Fault:              status.Telemetry.Fault,
		FaultName:          status.Telemetry.FaultName,
		SetpointPosition:   status.Setpoint.Position,
		SetpointVelocity:   status.Setpoint.Velocity,
		TrajectoryRunning:  status.Trajectory.Running,
		TrajectoryProgress: status.Trajectory.Progress,
		RateHz:             status.Stats.AchievedHz,
	}
	if !status.Armed {
		line.Blocked = status.ArmBlockers
	}
	return line
}

func optional(v float64) *float64 {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return nil
	}
	out := v
	return &out
}
