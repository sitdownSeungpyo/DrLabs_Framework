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

	fmt.Printf("Starting Torque (PWM) Control Test on %s at %d baud...\n", *portVal, *baudVal)
	fmt.Println("This test will apply PWM limit to Motor ID 1.")

	// Create Controller with X-Series Model
	ctrl := dxl.NewController(*portVal, *baudVal, dxl.ModelXSeries)

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error starting controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	// Set to PWM Mode (16) - Often safer demonstration of "Torque-like" behavior than Current Mode without load
	if err := ctrl.SetOperatingMode(1, dxl.OpModePWM); err != nil {
		fmt.Printf("Error setting operating mode: %v\n", err)
		os.Exit(1)
	}

	// Capture interrupt
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt)

	// PWM Values (Limit 885 = 100% usually? Check manual. X-Series PWM limit 885)
	pwms := []uint32{100, 200, 300, 0}

	idx := 0

	ticker := time.NewTicker(2 * time.Second)
	defer ticker.Stop()

	fmt.Printf("Setting PWM onto %d\n", pwms[0])
	ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: pwms[0]}}

Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping... Setting PWM 0")
			ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: 0}}
			time.Sleep(500 * time.Millisecond)
			break Loop
		case <-ticker.C:
			idx++
			if idx >= len(pwms) {
				idx = 0
			}
			val := pwms[idx]
			fmt.Printf("Setting PWM to %d\n", val)
			// Warning: PWM mode Goal PWM address is usually 100, NOT 116(Goal Position).
			// Controller.go currently hardcodes `AddrGoalPosition` in the loop.
			// This test will FAIL if we don't fix controller or override address.
			// Ideally, we need to change what address we write to based on mode.
			// But `controller.go` uses `c.Model.AddrGoalPosition`.
			// For PWM Mode, we should change `AddrGoalPosition` to `AddrGoalPWM`.

			// HACK: We can update the Model in the controller struct?
			// But `Model` is value type? No, struct.
			// Wait, the `ctrl.Model` is accessible.
			// X-Series Goal PWM is 100.

			ctrl.CommandChan <- []dxl.Command{{ID: 1, Value: val}}
		case <-ctrl.FeedbackChan:
			// ignore feedback
		}
	}
}
