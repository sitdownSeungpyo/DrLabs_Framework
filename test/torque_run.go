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
	currVal := flag.Int("current", 50, "Goal Current (mA or unit)")
	flag.Parse()

	fmt.Printf("Starting Torque (Current) Test on %s at %d baud, ID %d...\n", *portVal, *baudVal, *idVal)

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
	// Note: Not all X-Series support Current Control (Mode 0).
	// XM430 usually does. XL430 might not.
	if err := ctrl.SetOperatingMode(uint8(*idVal), dxl.OpModeCurrent); err != nil {
		fmt.Printf("Failed to set Current Mode (Torque Control): %v\n", err)
		return
	}
	fmt.Println("Mode set to Current Control (Torque).")

	// Apply Torque
	fmt.Printf("Applying Current %d...\n", *currVal)
	ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: uint32(*currVal)}}

Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping...")
			ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: 0}}
			time.Sleep(200 * time.Millisecond)
			break Loop
		case <-ctrl.FeedbackChan:
			// Monitor
		}
	}
}
