package main

import (
	"flag"
	"fmt"
	"go_dxl/dxl"
	"os"
)

func main() {
	portVal := flag.String("port", "COM3", "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")
	flag.Parse()

	fmt.Printf("Dynamixel CLI Tool (v1.0)\n")
	fmt.Printf("Connecting to %s at %d baud...\n", *portVal, *baudVal)

	// Create Controller with X-Series Model
	ctrl := dxl.NewController(*portVal, *baudVal, 100, dxl.ModelXSeries)

	if err := ctrl.Start(); err != nil {
		fmt.Printf("Error connecting to controller: %v\n", err)
		os.Exit(1)
	}
	defer ctrl.Stop()

	fmt.Println("Successfully connected to Motor ID 1!")
	fmt.Println("Use the test scripts in 'test/' directory to run specific modes:")
	fmt.Println("  go run test/position_run.go")
	fmt.Println("  go run test/velocity_run.go")
	fmt.Println("  go run test/torque_run.go")
}
