package main

import (
	"flag"
	"fmt"
	"go_dxl/dxl"
	"runtime"
)

func main() {
	fmt.Println("Pure Go Dynamixel Controller CLI")
	fmt.Println("--------------------------------")

	defaultPort := defaultSerialPort()
	testType := flag.String("test", "", "Test type: position, velocity, torque")
	portVal := flag.String("port", defaultPort, "Serial port name")
	baudVal := flag.Int("baud", 1000000, "Baudrate")

	flag.Parse()

	if *testType == "" {
		fmt.Println("Usage: go run main.go -test [position|velocity|torque] -port [PORT] -baud [1000000]")
		fmt.Println("  Port examples:")
		fmt.Println("    Windows: COM3, COM4")
		fmt.Println("    macOS:   /dev/tty.usbserial-XXXX or /dev/tty.usbmodemXXXX")
		fmt.Println("    Linux:   /dev/ttyUSB0 or /dev/ttyACM0")
		fmt.Println("Or run individual tests in test/ directory.")

		// Simple Ping Test
		fmt.Printf("\nRunning basic connection test on %s...\n", *portVal)
		sp, err := dxl.OpenSerial(*portVal, *baudVal)
		if err != nil {
			fmt.Printf("Failed to open port: %v\n", err)
			return
		}
		defer sp.Close()

		driver := dxl.NewDriver(sp)
		model, err := driver.Ping(1)
		if err != nil {
			fmt.Printf("Ping ID 1 failed: %v\n", err)
		} else {
			fmt.Printf("Success! Motor ID 1 Model: %d\n", model)
		}
		return
	}

	fmt.Printf("Please run the specific test script directly:\n")
	fmt.Printf("go run test/%s_run.go -port %s -baud %d\n", *testType, *portVal, *baudVal)
}

func defaultSerialPort() string {
	switch runtime.GOOS {
	case "darwin":
		return "/dev/tty.usbmodem0000"
	case "linux":
		return "/dev/ttyUSB0"
	default:
		return "COM3"
	}
}
