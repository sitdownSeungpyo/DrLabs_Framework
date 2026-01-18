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

	fmt.Printf("Starting Velocity Control Test on %s at %d baud...\n", *portVal, *baudVal)
	fmt.Println("This test will rotate Motor ID 1 at different speeds.")

	// Create Controller with X-Series Model
	ctrl := dxl.NewController(*portVal, *baudVal, 100, dxl.ModelXSeries)

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error starting controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	// Set to Velocity Mode (1)
	if err := ctrl.SetOperatingMode(1, dxl.OpModeVelocity); err != nil {
		fmt.Printf("Error setting operating mode: %v\n", err)
		os.Exit(1)
	}

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// Velocities to test
	velocities := []uint32{50, 100, 200, 0, 0xFFFFFF9C} // 50, 100, 200, Stop, -100(approx)
	// Note: Negative velocity in Protocol 2.0 is 2's complement implementation dependent?
	// Actually X-Series Goal Velocity is 4 bytes.
	// 0 ~ 2,147,483,647 for forward?
	// -2,147,483,648 ~ 0 for reverse? (Standard Int32 cast to Uint32)
	// Let's stick to positive for now or test simple values.
	// 100 = 100 units (0.229 rpm unit usually)

	idx := 0

	ticker := time.NewTicker(3 * time.Second)
	defer ticker.Stop()

	// Initial
	fmt.Printf("Setting Velocity to %d\n", velocities[0])
	ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: velocities[0]}}

Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping... Setting Velocity 0")
			ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: 0}}
			time.Sleep(500 * time.Millisecond)
			break Loop
		case <-ticker.C:
			idx++
			if idx >= len(velocities) {
				idx = 0
			}
			val := velocities[idx]
			fmt.Printf("Setting Velocity to %d\n", val)
			ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: val}}
		case fbs := <-ctrl.FeedbackChan:
			if len(fbs) > 0 {
				fb := fbs[0]
				// Just print current velocity/position if needed?
				// Since we are in Velocity mode, "Present Position" might still update.
				// But "FeedbackChan" is hardcoded to read "Present Position" in controller.go currently.
				// We might want to read "Present Velocity" (Addr 128) instead?
				// For now, seeing position change is proof of velocity.
				if fb.Error != nil {
					// fmt.Printf("Error: %v\n", fb.Error)
				} else {
					// fmt.Printf("Pos: %d\n", fb.Value)
				}
			}
		}
	}
}
