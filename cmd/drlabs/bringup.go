package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"net"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"text/tabwriter"
	"time"

	"drlabs/runtime/robot"
)

// Bringing a whole machine up from one file.
//
// Everything else in this binary drives one axis.  This drives a robot: it
// reads a description, opens each bus once, connects every joint on it, and
// energises the machine — then takes joint-addressed commands from whatever is
// above it.
//
//	drlabs bringup robot.json
//	  {"cmd":"position","joint":"r_sh_p","rad":0.5}
//
// Bring-up is all or nothing.  A machine with one dead joint is more dangerous
// than one that never came up, because everything else will hold poses that
// assume the missing joint is holding too.

func runBringUp(args []string) error {
	fs := flag.NewFlagSet("bringup", flag.ExitOnError)
	addr := fs.String("addr", "127.0.0.1:7777", "address to listen on for commands")
	stateRate := fs.Int("state-rate", 50, "unsolicited state lines per second (0 disables)")
	check := fs.Bool("check", false, "validate the description and print the plan, without touching hardware")
	if err := fs.Parse(args); err != nil {
		return err
	}

	if fs.NArg() != 1 {
		return fmt.Errorf("usage: drlabs bringup [flags] <robot.json>")
	}
	path := fs.Arg(0)

	config, err := robot.Load(path)
	if err != nil {
		return err
	}

	printPlan(config)
	if *check {
		fmt.Println("\nDescription is valid. Nothing was opened.")
		return nil
	}

	machine, err := robot.New(config)
	if err != nil {
		return err
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	fmt.Printf("\nBringing up %d joints…\n", len(config.Joints))
	bringUpCtx, bringUpCancel := context.WithTimeout(ctx, 60*time.Second)
	err = machine.BringUp(bringUpCtx)
	bringUpCancel()

	// Whatever happened, print what the joints said about it — and remember
	// where each joint's log had got to, so the stream below carries on rather
	// than replaying the bring-up.
	events, seen := machine.Events(nil)
	for _, event := range events {
		printJointEvent(event)
	}
	if err != nil {
		return err
	}

	// Let the loops run long enough to have a rate worth reporting; the
	// measurement is over a window, and there is no window yet.
	time.Sleep(500 * time.Millisecond)
	printState(machine)

	listener, err := net.Listen("tcp", *addr)
	if err != nil {
		machine.Shutdown()
		return fmt.Errorf("cannot listen on %s: %w", *addr, err)
	}
	defer listener.Close()

	// The machine is live from here; it must be released whatever happens
	// next, including a panic on the way out.
	defer machine.Shutdown()

	srv := &robotServer{robot: machine, stateRate: *stateRate}
	go srv.reportEvents(ctx, seen)
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go srv.handle(ctx, conn)
		}
	}()

	fmt.Printf("\nrobot %q live on %s · %d joints · %d Hz\n",
		config.Robot, listener.Addr(), len(config.Joints), config.RateHz)
	fmt.Println("  Ctrl+C to stop")

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)
	<-signals
	fmt.Println("\nStopping…")
	return nil
}

// printPlan says what the description asks for before anything is opened.
//
// The bus load line is the one worth reading: every joint on a bus costs a
// transaction per cycle, so the rate that matters is joints × rate, and a
// number the adapter cannot carry is better seen here than diagnosed later
// through a machine that will not keep time.
func printPlan(config *robot.Config) {
	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintf(w, "robot\t%s\n", config.Robot)
	fmt.Fprintf(w, "rate\t%d Hz\n", config.RateHz)
	_ = w.Flush()

	fmt.Println()
	w = tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintln(w, "JOINT\tDRIVER\tBUS\tID\tGEAR\tDIR\tTRAVEL (rad)\tTORQUE")
	for _, joint := range config.Joints {
		fmt.Fprintf(w, "%s\t%s\t%s\t%d\t%.3g:1\t%+.0f\t%.3f … %.3f\t%.2f Nm\n",
			joint.Name, joint.Driver, joint.Bus, joint.ID,
			joint.GearRatio, joint.Direction,
			joint.Limits.MinRad, joint.Limits.MaxRad, joint.Limits.MaxTorqueNm)
	}
	_ = w.Flush()

	fmt.Println()
	w = tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintln(w, "BUS\tTRANSPORT\tPORT\tRATE\tLOAD")
	load := config.BusLoad()
	for _, name := range config.SortedBusNames() {
		bus, _ := config.Bus(name)
		port := bus.Port
		if port == "" {
			port = "auto"
		}
		rate := fmt.Sprintf("%d Hz", config.RateForBus(name))
		if bus.RateHz == 0 {
			rate += " (default)"
		}
		fmt.Fprintf(w, "%s\t%s\t%s\t%s\t%d transactions/s\n",
			name, bus.Transport, port, rate, load[name])
	}
	_ = w.Flush()
}

func printState(machine *robot.Robot) {
	fmt.Println()
	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintln(w, "JOINT\tARMED\tMODE\tPOSITION\tTORQUE\tRATE")
	for _, state := range machine.State() {
		fmt.Fprintf(w, "%s\t%v\t%s\t%s rad\t%s Nm\t%.0f Hz\n",
			state.Name, state.Armed, state.Mode,
			number(state.PositionRad, 4), number(state.TorqueNm, 3), state.RateHz)
	}
	_ = w.Flush()
}

func number(value *float64, digits int) string {
	if value == nil {
		return "—"
	}
	return fmt.Sprintf("%+.*f", digits, *value)
}

func printJointEvent(event robot.JointEvent) {
	fmt.Printf("%-5s %-10s %s\n", event.Level, event.Joint, event.Message)
	if event.Detail != "" {
		fmt.Printf("                 %s\n", event.Detail)
	}
}

// --- the command surface ---------------------------------------------------

type robotServer struct {
	robot     *robot.Robot
	stateRate int
}

func (s *robotServer) reportEvents(ctx context.Context, seen map[string]uint64) {
	ticker := time.NewTicker(200 * time.Millisecond)
	defer ticker.Stop()

	if seen == nil {
		seen = map[string]uint64{}
	}
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			events, advanced := s.robot.Events(seen)
			seen = advanced
			for _, event := range events {
				printJointEvent(event)
			}
		}
	}
}

func (s *robotServer) handle(ctx context.Context, conn net.Conn) {
	defer conn.Close()

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
	// A joint that trips latches on its own, and the reason is the only thing
	// that makes that state actionable.  Printing it on this process's console
	// is no use to whatever is holding the socket, so it goes out here too.
	go s.streamEvents(clientCtx, send)

	scanner := bufio.NewScanner(conn)
	scanner.Buffer(make([]byte, 0, 4096), 256*1024)
	for scanner.Scan() {
		if len(scanner.Bytes()) == 0 {
			continue
		}
		_ = send(s.execute(scanner.Bytes()))
	}
}

// streamEvents pushes each joint's log to one client as it happens.
//
// The client gets its own cursor, so a panel that connects late is not handed
// the whole bring-up over again, and two clients cannot steal each other's
// events.
func (s *robotServer) streamEvents(ctx context.Context, send func(any) error) {
	ticker := time.NewTicker(200 * time.Millisecond)
	defer ticker.Stop()

	// Start from the beginning of the retained log rather than from now.
	//
	// The question a client most often has is "why is this joint stopped",
	// and the answer was logged when it tripped — before anyone thought to
	// connect.  Withholding it because the client is late leaves them looking
	// at a latched axis with no reason attached, which is the state this
	// stream exists to prevent.
	var seen map[string]uint64

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			events, advanced := s.robot.Events(seen)
			seen = advanced
			for _, event := range events {
				if err := send(map[string]any{
					"type":    "event",
					"joint":   event.Joint,
					"level":   event.Level,
					"message": event.Message,
					"detail":  event.Detail,
					"at":      event.At,
					"seq":     event.Seq,
				}); err != nil {
					return
				}
			}
		}
	}
}

func (s *robotServer) streamState(ctx context.Context, send func(any) error) {
	ticker := time.NewTicker(time.Second / time.Duration(s.stateRate))
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.robot.Heartbeat()
			if err := send(map[string]any{
				"type":   "state",
				"joints": s.robot.State(),
			}); err != nil {
				return
			}
		}
	}
}

type robotCommand struct {
	Cmd   string `json:"cmd"`
	Joint string `json:"joint"`

	Rad  *float64 `json:"rad"`
	RadS *float64 `json:"rad_s"`

	// Joints carries a whole pose in one command.
	Joints map[string]float64 `json:"joints"`

	MaxVelocity *float64 `json:"max_velocity"`
	Accel       *float64 `json:"accel"`

	Reason string `json:"reason"`
}

func (s *robotServer) execute(line []byte) map[string]any {
	var cmd robotCommand
	if err := json.Unmarshal(line, &cmd); err != nil {
		return refusal("malformed command: " + err.Error())
	}
	s.robot.Heartbeat()

	switch cmd.Cmd {
	case "position":
		if cmd.Rad == nil {
			return refusal("position needs rad")
		}
		if err := s.robot.SetPosition(cmd.Joint, *cmd.Rad); err != nil {
			return refusal(err.Error())
		}

	case "pose":
		if len(cmd.Joints) == 0 {
			return refusal("pose needs joints")
		}
		if err := s.robot.SetPositions(cmd.Joints); err != nil {
			return refusal(err.Error())
		}

	case "velocity":
		if cmd.RadS == nil {
			return refusal("velocity needs rad_s")
		}
		if err := s.robot.SetVelocity(cmd.Joint, *cmd.RadS); err != nil {
			return refusal(err.Error())
		}

	case "move":
		if cmd.Rad == nil {
			return refusal("move needs rad")
		}
		maxVelocity, accel := 1.0, 2.0
		if cmd.MaxVelocity != nil {
			maxVelocity = *cmd.MaxVelocity
		}
		if cmd.Accel != nil {
			accel = *cmd.Accel
		}
		if err := s.robot.MoveTo(cmd.Joint, *cmd.Rad, maxVelocity, accel); err != nil {
			return refusal(err.Error())
		}

	case "hold":
		if err := s.robot.Hold(cmd.Joint); err != nil {
			return refusal(err.Error())
		}

	case "stop":
		if err := s.robot.Stop(cmd.Joint); err != nil {
			return refusal(err.Error())
		}

	case "estop":
		reason := cmd.Reason
		if reason == "" {
			reason = "Raised by the controlling client."
		}
		s.robot.EStop(reason)

	case "reset":
		if err := s.robot.ResetEStop(); err != nil {
			return refusal(err.Error())
		}

	case "joints":
		return map[string]any{"type": "reply", "ok": true, "value": s.robot.JointNames()}

	case "state":
		return map[string]any{"type": "reply", "ok": true, "value": s.robot.State()}

	case "ping":

	default:
		return refusal("unknown command " + cmd.Cmd)
	}

	return map[string]any{"type": "reply", "ok": true}
}

func refusal(message string) map[string]any {
	return map[string]any{"type": "reply", "ok": false, "error": message}
}
