package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"text/tabwriter"
	"time"

	"drlabs/motor"
	"drlabs/runtime/session"
	"drlabs/transport/serial"
)

// runPorts lists the serial devices on this machine.
func runPorts(args []string) error {
	fs := flag.NewFlagSet("ports", flag.ExitOnError)
	if err := fs.Parse(args); err != nil {
		return err
	}

	ports, err := serial.List()
	if err != nil {
		return err
	}
	if len(ports) == 0 {
		fmt.Println("No serial ports found.")
		return nil
	}

	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintln(w, "PORT\tCAN ADAPTER\tDESCRIPTION")
	for _, port := range ports {
		adapter := ""
		if port.IsCANAdapter {
			adapter = "yes"
		}
		fmt.Fprintf(w, "%s\t%s\t%s\n", port.Path, adapter, port.Description)
	}
	return w.Flush()
}

// runDrivers lists what this binary can drive.
func runDrivers(args []string) error {
	fs := flag.NewFlagSet("drivers", flag.ExitOnError)
	if err := fs.Parse(args); err != nil {
		return err
	}

	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintln(w, "DRIVER\tDESCRIPTION")
	for _, name := range motor.Drivers() {
		info, _ := motor.DriverInfo(name)
		fmt.Fprintf(w, "%s\t%s\n", info.Name, info.Description)
	}
	return w.Flush()
}

// runProbe connects, reports what is attached and disconnects.
//
// It never arms, so it is safe to run against a live machine: the only thing
// it commands is a stop.
func runProbe(args []string) error {
	fs := flag.NewFlagSet("probe", flag.ExitOnError)
	var common commonFlags
	common.register(fs)
	allowUnknown := fs.Bool("allow-unknown-board", false,
		"proceed past a controller this build cannot positively identify")
	if err := fs.Parse(args); err != nil {
		return err
	}

	s := session.New(session.Options{})
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()

	if err := s.Connect(ctx, common.config(), *allowUnknown); err != nil {
		printEvents(s, 0)
		return err
	}
	defer s.Disconnect()

	// Let a few cycles run so the telemetry is real rather than the initial
	// blank state.
	time.Sleep(300 * time.Millisecond)
	printStatus(s.Status())
	printEvents(s, 0)
	return nil
}

// runSession runs a control session until interrupted.
func runSession(args []string) error {
	fs := flag.NewFlagSet("run", flag.ExitOnError)
	var common commonFlags
	common.register(fs)

	mode := fs.String("mode", "position", "control mode: position, velocity, hold, brake, within, stop")
	rate := fs.Int("rate", session.DefaultRateHz, "control rate in Hz")
	position := fs.Float64("position", 0, "position target in revolutions")
	velocity := fs.Float64("velocity", 0, "velocity target in rev/s")
	maxTorque := fs.Float64("max-torque", 0.3, "torque ceiling in Nm; sets both the safety limit and the working parameter")
	velocityLimit := fs.Float64("velocity-limit", 1.0, "trajectory velocity limit in rev/s")
	accelLimit := fs.Float64("accel-limit", 5.0, "trajectory acceleration limit in rev/s²")
	duration := fs.Duration("duration", 0, "stop after this long (0 runs until interrupted)")
	allowUnknown := fs.Bool("allow-unknown-board", false,
		"proceed past a controller this build cannot positively identify")

	shuttleA := fs.Float64("shuttle-a", 0, "shuttle endpoint A in revolutions")
	shuttleB := fs.Float64("shuttle-b", 0, "shuttle endpoint B in revolutions")
	shuttleDwell := fs.Float64("shuttle-dwell", 1.0, "shuttle dwell at each end, seconds")
	shuttleCycles := fs.Int("shuttle-cycles", 0, "shuttle cycles (0 runs until stopped)")

	if err := fs.Parse(args); err != nil {
		return err
	}

	controlMode, ok := motor.ParseControlMode(*mode)
	if !ok {
		return fmt.Errorf("unknown mode %q", *mode)
	}

	s := session.New(session.Options{RateHz: *rate})

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	if err := s.Connect(ctx, common.config(), *allowUnknown); err != nil {
		printEvents(s, 0)
		return err
	}

	// Whatever happens from here — a signal, a fault, a panic in the printing
	// path — the axis is stopped and the transport released before we exit.
	defer func() {
		s.Disarm("session ending")
		_ = s.Disconnect()
	}()

	// On the command line these flags are the operator declaring their
	// envelope, so they set the safety ceilings as well as the working
	// parameters.  Setting only the parameters would leave them silently
	// clamped by the defaults, and the axis would move with a torque limit
	// nobody chose.
	safetyConfig := s.Safety()
	safetyConfig.MaxTorque = *maxTorque
	if *velocityLimit > safetyConfig.MaxVelocity {
		safetyConfig.MaxVelocity = *velocityLimit
	}
	if *accelLimit > safetyConfig.AccelLimit {
		safetyConfig.AccelLimit = *accelLimit
	}
	if _, notes := s.SetSafety(safetyConfig); len(notes) > 0 {
		for _, note := range notes {
			fmt.Println("  !", note)
		}
	}

	params := session.DefaultParams()
	params.MaxTorque = *maxTorque
	params.VelocityLimit = *velocityLimit
	params.AccelLimit = *accelLimit
	s.SetParams(params)

	time.Sleep(200 * time.Millisecond)

	if blocking, advisory := s.Preflight(); len(blocking) > 0 {
		fmt.Println("Cannot arm:")
		for _, item := range blocking {
			fmt.Println("  ✗", item)
		}
		for _, item := range advisory {
			fmt.Println("  !", item)
		}
		return fmt.Errorf("preflight failed")
	} else {
		for _, item := range advisory {
			fmt.Println("  !", item)
		}
	}

	if err := s.Arm(true); err != nil {
		return err
	}
	if err := s.SetMode(controlMode); err != nil {
		return err
	}

	switch controlMode {
	case motor.CtrlPosition:
		s.SetPosition(*position)
	case motor.CtrlVelocity:
		s.SetVelocity(*velocity)
	}

	if *shuttleA != *shuttleB {
		s.ConfigureShuttle(*shuttleA, *shuttleB, *shuttleDwell, *shuttleCycles)
		if err := s.StartShuttle(); err != nil {
			return err
		}
	}

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)

	ticker := time.NewTicker(250 * time.Millisecond)
	defer ticker.Stop()

	var deadline <-chan time.Time
	if *duration > 0 {
		timer := time.NewTimer(*duration)
		defer timer.Stop()
		deadline = timer.C
	}

	var lastEvent uint64
	fmt.Println()

	for {
		select {
		case <-signals:
			fmt.Println("\nInterrupted.")
			return nil

		case <-deadline:
			fmt.Println("\nDuration elapsed.")
			return nil

		case <-ticker.C:
			// The session's deadman stops the axis if this loop dies, so the
			// heartbeat has to come from the same place that is watching.
			s.Heartbeat()

			status := s.Status()
			lastEvent = printEvents(s, lastEvent)
			printLine(status)

			if status.EStop.Latched {
				fmt.Printf("\n\nEMERGENCY STOP · %s\n", status.EStop.Reason)
				if status.EStop.Detail != "" {
					fmt.Printf("  %s\n", status.EStop.Detail)
				}
				return fmt.Errorf("stopped: %s", status.EStop.Code)
			}
			if !status.Shuttle.Running && *shuttleA != *shuttleB &&
				status.Shuttle.Completed > 0 && *shuttleCycles > 0 {
				fmt.Printf("\n\nShuttle finished: %d cycles.\n", status.Shuttle.Completed)
				return nil
			}
		}
	}
}

func (c *commonFlags) config() motor.Config {
	return motor.Config{
		Driver:  c.driver,
		Port:    c.port,
		Baud:    c.baud,
		ID:      c.id,
		Options: c.options,
	}
}

// printLine renders one status line, rewritten in place.
func printLine(status session.Status) {
	armed := "disarmed"
	if status.Armed {
		armed = "ARMED"
	}
	line := fmt.Sprintf(
		"\r%-8s %-9s pos %+8.4f rev  vel %+7.3f rev/s  %5.2f A  %4.1f V  %3.0f°C  %5.1f Hz  jitter %4.2f ms  ",
		armed, status.Mode,
		safeFloat(status.Telemetry.Position),
		safeFloat(status.Telemetry.Velocity),
		safeFloat(status.Telemetry.QCurrent),
		safeFloat(status.Telemetry.Voltage),
		safeFloat(status.Telemetry.Temperature),
		status.Stats.AchievedHz,
		status.Stats.JitterMS,
	)
	if status.Shuttle.Running {
		line += " " + status.Shuttle.Status
	}
	fmt.Print(line)
}

// printEvents prints the log entries newer than seq and returns the newest.
func printEvents(s *session.Session, seq uint64) uint64 {
	for _, event := range s.Events(seq) {
		fmt.Printf("\r%-5s %s\n", strings.ToUpper(string(event.Level)), event.Message)
		if event.Detail != "" {
			fmt.Printf("      %s\n", event.Detail)
		}
		seq = event.Seq
	}
	return seq
}

func printStatus(status session.Status) {
	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	fmt.Fprintf(w, "Connection\t%s\n", status.Connection)
	for key, value := range status.Link {
		fmt.Fprintf(w, "  %s\t%s\n", key, value)
	}
	if status.Identity != nil {
		fmt.Fprintf(w, "Board\t%s\n", status.Identity.Board)
		fmt.Fprintf(w, "  firmware\t%s\n", status.Identity.Firmware)
		fmt.Fprintf(w, "  serial\t%s\n", status.Identity.SerialNumber)
		fmt.Fprintf(w, "  status\t%s\n", status.Identity.Status)
		if lower, upper, ok := status.Identity.Bounds(); ok {
			fmt.Fprintf(w, "  travel bounds\t[%.3f, %.3f] rev\n", lower, upper)
		} else {
			fmt.Fprintf(w, "  travel bounds\tnone reported\n")
		}
	}
	fmt.Fprintf(w, "Mode\t%s (%s)\n", status.Telemetry.ModeName, status.Mode)
	fmt.Fprintf(w, "Fault\t%d %s\n", status.Telemetry.Fault, status.Telemetry.FaultName)
	fmt.Fprintf(w, "Position\t%.4f rev\n", safeFloat(status.Telemetry.Position))
	fmt.Fprintf(w, "Velocity\t%.4f rev/s\n", safeFloat(status.Telemetry.Velocity))
	fmt.Fprintf(w, "Current\t%.2f A\n", safeFloat(status.Telemetry.QCurrent))
	fmt.Fprintf(w, "Voltage\t%.2f V\n", safeFloat(status.Telemetry.Voltage))
	fmt.Fprintf(w, "Temperature\t%.1f °C\n", safeFloat(status.Telemetry.Temperature))
	fmt.Fprintf(w, "Effective limits\t[%.3f, %.3f] rev\n",
		status.EffectiveLimits[0], status.EffectiveLimits[1])
	fmt.Fprintf(w, "Loop\t%.1f Hz requested %d Hz · jitter %.2f ms · replies %.0f%%\n",
		status.Stats.AchievedHz, status.Stats.RequestedHz, status.Stats.JitterMS,
		status.Stats.ReplyRate*100)
	_ = w.Flush()

	if len(status.ArmBlockers) > 0 {
		fmt.Println("\nCannot arm:")
		for _, item := range status.ArmBlockers {
			fmt.Println("  ✗", item)
		}
	}
	if len(status.ArmAdvisories) > 0 {
		fmt.Println("\nAdvisories:")
		for _, item := range status.ArmAdvisories {
			fmt.Println("  !", item)
		}
	}
}

// safeFloat renders an unmeasured value as zero for display width, since a
// printed "NaN" throws off every column it appears in.
func safeFloat(v float64) float64 {
	if !motor.Valid(v) {
		return 0
	}
	return v
}
