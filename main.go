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
	flag.Parse()

	fmt.Printf("Starting Rotation Test on %s at %d baud...\n", *portVal, *baudVal)
	fmt.Println("This test will rotate Motor ID 1 from position 0 to 4096 and back.")

	// Create Controller with X-Series Model
	ctrl := dxl.NewController(*portVal, *baudVal, 100, dxl.ModelXSeries)

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error starting controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// Combined Motion & Feedback Loop
	targetPositions := []uint32{0, 1024, 2048, 3072, 4095}
	idx := 0
	forward := true

	// Initial Move
	currentTarget := targetPositions[0]
	fmt.Printf("Moving to: %d\n", currentTarget)
	ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: currentTarget}}

	// Wait loop
Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping...")
			break Loop
		case fbs := <-ctrl.FeedbackChan:
			if len(fbs) > 0 {
				fb := fbs[0]
				if fb.Error != nil {
					fmt.Printf("Error: %v\n", fb.Error)
					continue
				}

				// Check arrival
				diff := int(fb.Value) - int(currentTarget)
				if diff < 0 {
					diff = -diff
				}

				// Threshold: 20 ticks
				if diff < 20 {
					fmt.Printf("Reached %d (Req: %d). Move Next.\n", fb.Value, currentTarget)

					// Update Target
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
					// Small delay to see it stop
					time.Sleep(500 * time.Millisecond)
					fmt.Printf("Moving to: %d\n", currentTarget)
					ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: currentTarget}}
				} else {
					// Still moving
					// fmt.Printf("Pos: %d (Target: %d)\n", fb.Value, currentTarget)
				}
			}
		}
	}
}
