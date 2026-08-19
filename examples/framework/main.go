// Command framework shows the vendor-neutral API end to end: connect, run the
// arming checklist, drive an axis, and shut down safely.
//
// It runs against the simulator by default, so it can be tried with nothing
// plugged in:
//
//	go run ./examples/framework
//	go run ./examples/framework -driver moteus -port COM7 -id 1
//	go run ./examples/framework -driver dynamixel -port COM3 -id 1
//
// Nothing below names a vendor except the -driver flag.  That is the point of
// the framework: the same code drives a moteus over CAN-FD and a Dynamixel
// over serial, because both present the same motor.Link.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"math"
	"time"

	"drlabs/motor"
	"drlabs/runtime/session"

	// Importing a driver for effect is what makes it available by name.
	_ "drlabs/drivers/dynamixel"
	_ "drlabs/drivers/moteus"
	_ "drlabs/drivers/robstride"
)

func main() {
	driver := flag.String("driver", "moteus-sim", "driver name")
	port := flag.String("port", "", "serial port (empty auto-detects)")
	baud := flag.Int("baud", 1000000, "serial baud rate")
	id := flag.Int("id", 1, "bus address")
	target := flag.Float64("target", 0.25, "position target in revolutions")
	flag.Parse()

	// A Session owns the control loop.  Everything below only reads snapshots
	// and pushes requests; nothing here ever touches the bus directly, which
	// is what keeps this program's own pauses out of motor control.
	s := session.New(session.Options{RateHz: 200})

	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()

	config := motor.Config{Driver: *driver, Port: *port, Baud: *baud, ID: *id}
	if err := s.Connect(ctx, config, false); err != nil {
		log.Fatalf("connect: %v", err)
	}
	// Stop the axis and release the transport whatever happens next.
	defer func() {
		s.Disarm("example finished")
		_ = s.Disconnect()
	}()

	// Give the loop a few cycles so the first status carries real telemetry
	// rather than the blank state a fresh session starts in.
	time.Sleep(200 * time.Millisecond)

	// Tighten the envelope before arming.  These are host-side limits: they
	// are enforced on every command, and nothing is written to the
	// controller's stored configuration.
	safety := s.Safety()
	safety.MaxTorque = 0.5
	safety.MaxVelocity = 2.0
	safety.PositionMin, safety.PositionMax = -0.6, 0.6
	if _, notes := s.SetSafety(safety); len(notes) > 0 {
		for _, note := range notes {
			fmt.Println("safety adjusted:", note)
		}
	}

	params := session.DefaultParams()
	params.MaxTorque = 0.4
	params.VelocityLimit = 1.5
	params.AccelLimit = 6.0
	s.SetParams(params)

	// The arming checklist.  Blockers must be cleared; advisories are for the
	// operator to read and accept.
	blocking, advisory := s.Preflight()
	for _, item := range advisory {
		fmt.Println("advisory:", item)
	}
	if len(blocking) > 0 {
		for _, item := range blocking {
			fmt.Println("blocked:", item)
		}
		log.Fatal("cannot arm")
	}

	// The acknowledgement is the operator saying they have read the above.
	if err := s.Arm(true); err != nil {
		log.Fatalf("arm: %v", err)
	}
	if err := s.SetMode(motor.CtrlPosition); err != nil {
		log.Fatalf("set mode: %v", err)
	}

	fmt.Printf("moving to %.3f rev\n", *target)
	s.SetPosition(*target)

	deadline := time.Now().Add(10 * time.Second)
	for time.Now().Before(deadline) {
		// The session stops the axis if this loop dies, so the heartbeat has
		// to come from whatever is actually supervising.
		s.Heartbeat()

		status := s.Status()
		if status.EStop.Latched {
			log.Fatalf("emergency stop: %s\n%s", status.EStop.Reason, status.EStop.Detail)
		}
		if motor.Valid(status.Telemetry.Position) &&
			math.Abs(status.Telemetry.Position-*target) < 0.005 {
			fmt.Printf("arrived at %.4f rev after %.1f s\n",
				status.Telemetry.Position, 10-time.Until(deadline).Seconds())
			break
		}
		time.Sleep(20 * time.Millisecond)
	}

	final := s.Status()
	fmt.Printf("position %.4f rev · velocity %.3f rev/s · %.2f A · %.1f Hz · jitter %.2f ms\n",
		final.Telemetry.Position, final.Telemetry.Velocity, final.Telemetry.QCurrent,
		final.Stats.AchievedHz, final.Stats.JitterMS)
}
