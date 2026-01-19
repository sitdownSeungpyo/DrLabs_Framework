package main

import (
	"fmt"
	"os"
	"os/signal"
	"runtime"
	"syscall"
	"time"

	"go_dxl/dxl"
)

func main() {
	var devicePort string
	if runtime.GOOS == "windows" {
		devicePort = "COM3" // Change to your port
	} else {
		devicePort = "/dev/ttyUSB0" // Change to your port
	}

	// Create controller for X-Series motors
	ctrl := dxl.NewController(devicePort, 57600, dxl.ModelXSeries)

	// Configure multiple motors - adjust IDs to match your setup
	motorIDs := []uint8{1, 2, 3} // Control 3 motors simultaneously
	ctrl.SetMotorIDs(motorIDs)

	fmt.Printf("Multi-Motor Control Test\n")
	fmt.Printf("Motor IDs: %v\n", motorIDs)
	fmt.Printf("Using Sync Read/Write for efficient communication\n\n")

	// Start controller
	if err := ctrl.Start(); err != nil {
		fmt.Printf("Failed to start controller: %v\n", err)
		os.Exit(1)
	}

	// Set to Position Control Mode
	fmt.Println("Setting all motors to Position Control Mode...")
	for _, id := range motorIDs {
		if err := ctrl.SetOperatingMode(id, dxl.OpModePosition); err != nil {
			fmt.Printf("Failed to set mode for motor %d: %v\n", id, err)
			os.Exit(1)
		}
	}

	// Graceful shutdown
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)

	// Control loop - move motors in sync
	go func() {
		ticker := time.NewTicker(2 * time.Second)
		defer ticker.Stop()

		positions := []uint32{2048, 3072, 1024} // Different positions for variety
		posIdx := 0

		for {
			select {
			case <-ticker.C:
				// Send commands to all motors at once using Sync Write
				cmds := make([]dxl.Command, len(motorIDs))
				for i, id := range motorIDs {
					// Rotate through different positions
					cmds[i] = dxl.Command{
						ID:    id,
						Value: positions[(posIdx+i)%len(positions)],
					}
					fmt.Printf("Motor %d -> Position %d\n", id, cmds[i].Value)
				}

				// Send all commands together - much faster than individual writes
				ctrl.CommandChan <- cmds
				posIdx = (posIdx + 1) % len(positions)
				fmt.Println()
			}
		}
	}()

	// Feedback monitoring loop - read all motors efficiently
	go func() {
		for {
			select {
			case feedbacks := <-ctrl.FeedbackChan:
				// Print feedback from all motors
				fmt.Print("Feedback: ")
				for _, fb := range feedbacks {
					if fb.Error != nil {
						fmt.Printf("[ID %d: ERROR %v] ", fb.ID, fb.Error)
					} else {
						fmt.Printf("[ID %d: Pos=%d] ", fb.ID, fb.Value)
					}
				}
				fmt.Println()
			}
		}
	}()

	// Wait for interrupt
	fmt.Println("Running... Press Ctrl+C to stop")
	<-sigChan
	fmt.Println("\nStopping controller...")
	ctrl.Stop()
	time.Sleep(100 * time.Millisecond)
	fmt.Println("Done!")
}
