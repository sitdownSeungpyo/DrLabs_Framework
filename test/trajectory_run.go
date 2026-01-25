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
	// Command line flags
	portVal := flag.String("port", "COM4", "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")
	idVal := flag.Int("id", 1, "Motor ID")
	startPos := flag.Float64("start", 0, "Start position (0-4095)")
	targetPos := flag.Float64("target", 2048, "Target position (0-4095)")
	maxVel := flag.Float64("vel", 500, "Max velocity (units/sec)")
	accel := flag.Float64("accel", 2000, "Acceleration (units/sec^2)")
	updateRate := flag.Float64("rate", 100, "Update rate in Hz")
	loop := flag.Bool("loop", false, "Loop back and forth continuously")
	flag.Parse()

	fmt.Println("=== Trapezoidal Trajectory Test ===")
	fmt.Printf("Port: %s, Baud: %d, Motor ID: %d\n", *portVal, *baudVal, *idVal)
	fmt.Printf("Start: %.0f -> Target: %.0f\n", *startPos, *targetPos)
	fmt.Printf("Max Velocity: %.0f, Acceleration: %.0f\n", *maxVel, *accel)
	fmt.Printf("Update Rate: %.0f Hz\n", *updateRate)
	fmt.Println()

	// Create trajectory profile
	profile, err := dxl.NewTrapezoidalProfile(*startPos, *targetPos, *maxVel, *accel)
	if err != nil {
		fmt.Printf("Failed to create profile: %v\n", err)
		os.Exit(1)
	}

	fmt.Printf("Profile calculated:\n")
	fmt.Printf("  Total duration: %.3f seconds\n", profile.TotalTime())
	fmt.Printf("  Total points: %d\n", int(profile.TotalTime()**updateRate)+1)
	fmt.Println()

	// Preview trajectory (first 10 points)
	fmt.Println("Trajectory preview (first 10 points):")
	points := profile.Generate(*updateRate)
	previewCount := 10
	if len(points) < previewCount {
		previewCount = len(points)
	}
	for i := 0; i < previewCount; i++ {
		p := points[i]
		fmt.Printf("  t=%.3fs: pos=%.1f, vel=%.1f, accel=%.1f\n",
			p.Time, p.Position, p.Velocity, p.Accel)
	}
	if len(points) > previewCount {
		fmt.Printf("  ... (%d more points)\n", len(points)-previewCount)
	}
	fmt.Println()

	// Ask for confirmation
	fmt.Print("Start motor control? (y/n): ")
	var confirm string
	fmt.Scanln(&confirm)
	if confirm != "y" && confirm != "Y" {
		fmt.Println("Cancelled.")
		return
	}

	// Create Controller
	ctrl := dxl.NewController(*portVal, *baudVal, dxl.ModelXSeries)
	ctrl.SetMotorIDs([]uint8{uint8(*idVal)})

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error starting controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// Set to Position Mode
	if err := ctrl.SetOperatingMode(uint8(*idVal), dxl.OpModePosition); err != nil {
		fmt.Printf("Failed to set Position Mode: %v\n", err)
		return
	}
	fmt.Println("Mode set to Position Control.")

	// Move to start position first
	fmt.Printf("Moving to start position: %.0f\n", *startPos)
	ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: uint32(*startPos)}}
	time.Sleep(2 * time.Second) // Wait for initial positioning

	// Create executor
	executor := dxl.NewTrajectoryExecutor(ctrl, uint8(*idVal))

	running := true
	forward := true
	iteration := 0

	for running {
		iteration++
		var currentProfile *dxl.TrapezoidalProfile

		if forward {
			currentProfile, _ = dxl.NewTrapezoidalProfile(*startPos, *targetPos, *maxVel, *accel)
			fmt.Printf("\n[Iteration %d] Forward: %.0f -> %.0f\n", iteration, *startPos, *targetPos)
		} else {
			currentProfile, _ = dxl.NewTrapezoidalProfile(*targetPos, *startPos, *maxVel, *accel)
			fmt.Printf("\n[Iteration %d] Backward: %.0f -> %.0f\n", iteration, *targetPos, *startPos)
		}

		fmt.Printf("Executing trajectory (%.3f seconds)...\n", currentProfile.TotalTime())

		// Execute trajectory with progress monitoring
		done := make(chan struct{})
		go func() {
			executor.Execute(currentProfile, *updateRate)
			close(done)
		}()

		// Monitor feedback while executing
		startTime := time.Now()
	executeLoop:
		for {
			select {
			case <-sigChan:
				fmt.Println("\nInterrupted! Stopping...")
				running = false
				break executeLoop

			case <-done:
				elapsed := time.Since(startTime)
				fmt.Printf("Trajectory complete! Elapsed: %.3f seconds\n", elapsed.Seconds())
				break executeLoop

			case fbs := <-ctrl.FeedbackChan:
				for _, fb := range fbs {
					if fb.ID == uint8(*idVal) && fb.Error == nil {
						elapsed := time.Since(startTime)
						fmt.Printf("\r  t=%.2fs: position=%d", elapsed.Seconds(), fb.Value)
					}
				}
			}
		}

		if !running {
			break
		}

		if *loop {
			forward = !forward
			time.Sleep(500 * time.Millisecond) // Brief pause between iterations
		} else {
			running = false
		}
	}

	fmt.Println("\nTest complete.")
}
