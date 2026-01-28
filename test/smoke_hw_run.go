package main

import (
	"flag"
	"fmt"
	"go_dxl/dxl"
	"os"
	"os/signal"
	"time"
)

func main() {
	portVal := flag.String("port", "COM4", "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")
	idVal := flag.Int("id", 1, "Motor ID")
	flag.Parse()

	motorID := uint8(*idVal)
	passed := 0
	failed := 0

	pass := func(name string) {
		passed++
		fmt.Printf("  [PASS] %s\n", name)
	}
	fail := func(name string, err interface{}) {
		failed++
		fmt.Printf("  [FAIL] %s: %v\n", name, err)
	}

	fmt.Println("=== Hardware Smoke Test ===")
	fmt.Printf("Port: %s, Baud: %d, Motor ID: %d\n\n", *portVal, *baudVal, motorID)

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// --- Test 1: Controller Start (pings all configured motors) ---
	fmt.Println("[Test 1] Controller Start - ping configured motor IDs")
	ctrl := dxl.NewController(*portVal, *baudVal, dxl.ModelXSeries)
	ctrl.SetMotorIDs([]uint8{motorID})

	if err := ctrl.Start(); err != nil {
		fail("Controller Start", err)
		fmt.Println("\nCannot continue without controller. Exiting.")
		os.Exit(1)
	}
	pass("Controller Start")
	defer ctrl.Stop()

	// --- Test 2: SetOperatingMode with read-back verification ---
	fmt.Println("[Test 2] SetOperatingMode - position mode with verification")
	if err := ctrl.SetOperatingMode(motorID, dxl.OpModePosition); err != nil {
		fail("SetOperatingMode(Position)", err)
	} else {
		pass("SetOperatingMode(Position)")
	}

	// --- Test 3: Basic command/feedback loop ---
	fmt.Println("[Test 3] Command/Feedback round-trip")
	testPos := uint32(2048)
	ctrl.CommandChan <- []dxl.Command{{ID: motorID, Value: testPos}}

	feedbackReceived := false
	timeout := time.After(2 * time.Second)
feedbackLoop:
	for {
		select {
		case fbs := <-ctrl.FeedbackChan:
			for _, fb := range fbs {
				if fb.ID == motorID && fb.Error == nil {
					fmt.Printf("    Position feedback: %d\n", fb.Value)
					feedbackReceived = true
					break feedbackLoop
				}
			}
		case <-timeout:
			break feedbackLoop
		}
	}
	if feedbackReceived {
		pass("Command/Feedback round-trip")
	} else {
		fail("Command/Feedback round-trip", "no feedback received within 2s")
	}

	// --- Test 4: Trajectory execution with context cancel ---
	fmt.Println("[Test 4] Trajectory execution (short move)")
	profile, err := dxl.NewTrapezoidalProfile(2048, 2500, 300, 1000)
	if err != nil {
		fail("Trajectory profile creation", err)
	} else {
		executor := dxl.NewTrajectoryExecutor(ctrl, motorID)
		fmt.Printf("    Profile duration: %.3f s\n", profile.TotalTime())

		execErr := executor.Execute(profile, 50)
		if execErr != nil {
			fail("Trajectory Execute", execErr)
		} else {
			pass("Trajectory Execute")
		}
	}

	// Wait for motor to settle
	time.Sleep(500 * time.Millisecond)

	// --- Test 5: Trajectory return (reverse) ---
	fmt.Println("[Test 5] Trajectory return (reverse move)")
	profileBack, err := dxl.NewTrapezoidalProfile(2500, 2048, 300, 1000)
	if err != nil {
		fail("Reverse trajectory creation", err)
	} else {
		executor := dxl.NewTrajectoryExecutor(ctrl, motorID)
		execErr := executor.Execute(profileBack, 50)
		if execErr != nil {
			fail("Reverse trajectory Execute", execErr)
		} else {
			pass("Reverse trajectory Execute")
		}
	}

	// --- Test 6: Velocity mode switch ---
	fmt.Println("[Test 6] SetOperatingMode - velocity mode switch")
	if err := ctrl.SetOperatingMode(motorID, dxl.OpModeVelocity); err != nil {
		fail("SetOperatingMode(Velocity)", err)
	} else {
		pass("SetOperatingMode(Velocity)")
	}

	// Switch back to position for safety
	time.Sleep(100 * time.Millisecond)
	ctrl.SetOperatingMode(motorID, dxl.OpModePosition)

	// --- Summary ---
	fmt.Printf("\n=== Results: %d passed, %d failed ===\n", passed, failed)
	if failed > 0 {
		os.Exit(1)
	}
}
