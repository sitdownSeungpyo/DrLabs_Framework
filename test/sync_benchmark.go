package main

import (
	"fmt"
	"os"
	"runtime"
	"time"

	"go_dxl/dxl"
)

// Demonstrates the performance difference between individual writes and sync write

func main() {
	var devicePort string
	if runtime.GOOS == "windows" {
		devicePort = "COM4"
	} else {
		devicePort = "/dev/ttyUSB0"
	}

	// Open serial port
	sp, err := dxl.OpenSerial(devicePort, 57600)
	if err != nil {
		fmt.Printf("Failed to open port: %v\n", err)
		os.Exit(1)
	}
	defer sp.Close()

	driver := dxl.NewDriver(sp)

	motorIDs := []uint8{1, 2, 3}
	goalPosition := uint16(116) // X-Series Goal Position address
	testPositions := []uint32{2048, 3072, 1024}

	fmt.Printf("=== Sync Write vs Individual Write Benchmark ===\n")
	fmt.Printf("Motor IDs: %v\n", motorIDs)
	fmt.Printf("Iterations: 100\n\n")

	// Benchmark 1: Individual Writes
	fmt.Println("Testing Individual Writes...")
	start := time.Now()
	iterations := 100

	for i := 0; i < iterations; i++ {
		for j, id := range motorIDs {
			_ = driver.Write4Byte(id, goalPosition, testPositions[j])
		}
	}

	individualTime := time.Since(start)
	individualAvg := individualTime / time.Duration(iterations)
	fmt.Printf("Total Time: %v\n", individualTime)
	fmt.Printf("Avg per cycle: %v\n", individualAvg)
	fmt.Printf("Commands/sec: %.2f\n\n", float64(iterations*len(motorIDs))/individualTime.Seconds())

	// Small delay between tests
	time.Sleep(500 * time.Millisecond)

	// Benchmark 2: Sync Write
	fmt.Println("Testing Sync Write...")
	start = time.Now()

	for i := 0; i < iterations; i++ {
		values := make(map[uint8]uint32)
		for j, id := range motorIDs {
			values[id] = testPositions[j]
		}
		_ = driver.SyncWrite4Byte(goalPosition, values)
	}

	syncTime := time.Since(start)
	syncAvg := syncTime / time.Duration(iterations)
	fmt.Printf("Total Time: %v\n", syncTime)
	fmt.Printf("Avg per cycle: %v\n", syncAvg)
	fmt.Printf("Commands/sec: %.2f\n\n", float64(iterations*len(motorIDs))/syncTime.Seconds())

	// Results
	fmt.Println("=== Results ===")
	speedup := float64(individualTime) / float64(syncTime)
	fmt.Printf("Sync Write is %.2fx faster\n", speedup)
	fmt.Printf("Time saved per cycle: %v\n", individualAvg-syncAvg)

	if speedup > 1 {
		fmt.Printf("\nConclusion: Sync Write is significantly faster for %d motors!\n", len(motorIDs))
		fmt.Printf("For real-time control loops, this improvement is critical.\n")
	}
}
