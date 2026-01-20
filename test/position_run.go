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
	portVal := flag.String("port", "COM3", "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")
	idVal := flag.Int("id", 1, "Motor ID")
	flag.Parse()

	fmt.Printf("Starting Position Test on %s at %d baud, ID %d...\n", *portVal, *baudVal, *idVal)

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

	// Set Mode
	if err := ctrl.SetOperatingMode(uint8(*idVal), dxl.OpModePosition); err != nil {
		fmt.Printf("Failed to set Position Mode: %v\n", err)
		return
	}
	fmt.Println("Mode set to Position Control.")

	// targets
	targetPositions := []uint32{0, 1024, 2048, 3072, 4095}
	idx := 0
	forward := true

	// Initial Move
	currentTarget := targetPositions[0]
	fmt.Printf("Moving to: %d\n", currentTarget)
	ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: currentTarget}}

	// Loop
Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping...")
			break Loop
		case fbs := <-ctrl.FeedbackChan:
			for _, fb := range fbs {
				if fb.ID != uint8(*idVal) {
					continue
				}
				if fb.Error != nil {
					fmt.Printf("Error: %v\n", fb.Error)
					continue
				}

				// Check arrival
				diff := int(fb.Value) - int(currentTarget)
				if diff < 0 {
					diff = -diff
				}

				if diff < 20 {
					fmt.Printf("Reached %d. Move Next.\n", fb.Value)

					if forward {
						idx++
						if idx >= len(targetPositions) {
							idx = len(targetPositions) - 2
							forward = false
						}
					} else {
						idx--
						if idx < 0 {
							idx = 1
							forward = true
						}
					}

					currentTarget = targetPositions[idx]
					time.Sleep(500 * time.Millisecond)
					fmt.Printf("Moving to: %d\n", currentTarget)
					ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: currentTarget}}
				}
			}
		}
	}
}
