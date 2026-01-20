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
	velVal := flag.Int("vel", 200, "Goal Velocity")
	flag.Parse()

	fmt.Printf("Starting Velocity Test on %s at %d baud, ID %d...\n", *portVal, *baudVal, *idVal)

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
	if err := ctrl.SetOperatingMode(uint8(*idVal), dxl.OpModeVelocity); err != nil {
		fmt.Printf("Failed to set Velocity Mode: %v\n", err)
		return
	}
	fmt.Println("Mode set to Velocity Control.")

	// Move
	fmt.Printf("Setting Velocity to %d...\n", *velVal)
	ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: uint32(*velVal)}}

	// Loop
Loop:
	for {
		select {
		case <-sigChan:
			fmt.Println("\nStopping...")
			ctrl.CommandChan <- []dxl.Command{{ID: uint8(*idVal), Value: 0}}
			time.Sleep(200 * time.Millisecond)
			break Loop
		case fbs := <-ctrl.FeedbackChan:
			for _, fb := range fbs {
				if fb.ID == uint8(*idVal) {
					// fmt.Printf("Present Velocity/Pos: %d\n", fb.Value)
				}
			}
		}
	}
}
