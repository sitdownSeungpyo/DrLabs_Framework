package main

import (
	"flag"
	"fmt"
	"go_dxl/dxl"
	"os"
	"os/signal"
	"sync"
	"time"
)

func main() {
	// Command line flags
	portVal := flag.String("port", "COM4", "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")
	maxVel := flag.Float64("vel", 500, "Max velocity (units/sec)")
	accel := flag.Float64("accel", 2000, "Acceleration (units/sec^2)")
	updateRate := flag.Float64("rate", 100, "Update rate in Hz")
	flag.Parse()

	motorIDs := []uint8{1, 2}

	fmt.Println("=== Multi-Motor Trajectory Test ===")
	fmt.Printf("Port: %s, Baud: %d\n", *portVal, *baudVal)
	fmt.Printf("Motor IDs: %v\n", motorIDs)
	fmt.Printf("Max Velocity: %.0f units/sec\n", *maxVel)
	fmt.Printf("Acceleration: %.0f units/sec^2\n", *accel)
	fmt.Printf("Update Rate: %.0f Hz\n", *updateRate)
	fmt.Println()

	// Create Controller
	ctrl := dxl.NewController(*portVal, *baudVal, dxl.ModelXSeries)
	if err := ctrl.SetMotorIDs(motorIDs); err != nil {
		fmt.Printf("Invalid motor IDs: %v\n", err)
		os.Exit(1)
	}

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error starting controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	// Set Position Mode for all motors
	fmt.Println("Setting motors to Position Control Mode...")
	for _, id := range motorIDs {
		if err := ctrl.SetOperatingMode(id, dxl.OpModePosition); err != nil {
			fmt.Printf("Failed to set mode for motor %d: %v\n", id, err)
			return
		}
	}
	fmt.Println("Mode set successfully!")

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// Read initial positions
	fmt.Println("\nReading initial positions...")
	time.Sleep(100 * time.Millisecond)

	var initialPos1, initialPos2 uint32 = 2048, 2048 // Default center position
	select {
	case fb := <-ctrl.FeedbackChan:
		for _, f := range fb {
			if f.Error == nil {
				if f.ID == 1 {
					initialPos1 = f.Value
				} else if f.ID == 2 {
					initialPos2 = f.Value
				}
				fmt.Printf("Motor %d initial position: %d\n", f.ID, f.Value)
			}
		}
	case <-time.After(500 * time.Millisecond):
		fmt.Println("Warning: Could not read initial positions, using defaults")
	}

	// Define trajectories - motors move in opposite directions for visual effect
	// Motor 1: current -> 1024
	// Motor 2: current -> 3072
	target1 := float64(1024)
	target2 := float64(3072)

	profile1, err := dxl.NewTrapezoidalProfile(float64(initialPos1), target1, *maxVel, *accel)
	if err != nil {
		fmt.Printf("Failed to create profile for motor 1: %v\n", err)
		return
	}

	profile2, err := dxl.NewTrapezoidalProfile(float64(initialPos2), target2, *maxVel, *accel)
	if err != nil {
		fmt.Printf("Failed to create profile for motor 2: %v\n", err)
		return
	}

	fmt.Printf("\nTrajectory 1: %.0f -> %.0f (duration: %.2fs)\n", float64(initialPos1), target1, profile1.TotalTime())
	fmt.Printf("Trajectory 2: %.0f -> %.0f (duration: %.2fs)\n", float64(initialPos2), target2, profile2.TotalTime())
	fmt.Println("\nStarting synchronized trajectory execution...")
	fmt.Println("Press Ctrl+C to stop\n")

	// Execute trajectories in parallel using goroutines
	var wg sync.WaitGroup
	errChan := make(chan error, 2)

	// Executor for motor 1
	executor1 := dxl.NewTrajectoryExecutor(ctrl, 1)
	wg.Add(1)
	go func() {
		defer wg.Done()
		if err := executor1.Execute(profile1, *updateRate); err != nil {
			errChan <- fmt.Errorf("motor 1: %v", err)
		}
	}()

	// Executor for motor 2
	executor2 := dxl.NewTrajectoryExecutor(ctrl, 2)
	wg.Add(1)
	go func() {
		defer wg.Done()
		if err := executor2.Execute(profile2, *updateRate); err != nil {
			errChan <- fmt.Errorf("motor 2: %v", err)
		}
	}()

	// Monitor feedback while trajectories execute
	done := make(chan struct{})
	go func() {
		wg.Wait()
		close(done)
	}()

	// Print feedback during execution
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nInterrupted! Stopping...")
			ctrl.Stop()
			break Loop
		case <-done:
			fmt.Println("\nTrajectories completed!")
			break Loop
		case err := <-errChan:
			fmt.Printf("Trajectory error: %v\n", err)
		case <-ticker.C:
			// Drain and print latest feedback
			select {
			case fb := <-ctrl.FeedbackChan:
				fmt.Print("Position: ")
				for _, f := range fb {
					if f.Error == nil {
						fmt.Printf("[M%d: %d] ", f.ID, f.Value)
					} else {
						fmt.Printf("[M%d: ERR] ", f.ID)
					}
				}
				fmt.Println()
			default:
			}
		}
	}

	// Wait a moment then read final positions
	time.Sleep(200 * time.Millisecond)
	fmt.Println("\nFinal positions:")
	select {
	case fb := <-ctrl.FeedbackChan:
		for _, f := range fb {
			if f.Error == nil {
				fmt.Printf("  Motor %d: %d\n", f.ID, f.Value)
			}
		}
	case <-time.After(500 * time.Millisecond):
	}

	fmt.Println("\nDone!")
}
