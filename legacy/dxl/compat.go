// Backward compatibility shims.
//
// The dxl package is maintained for backward compatibility.
// New code should import the specific packages directly:
//   - drlabs/drivers/dynamixel   (Dynamixel protocol and driver)
//   - drlabs/motor       (universal motor interfaces)
//   - drlabs/trajectory  (motion planning)
//   - drlabs/transport/serial (serial port abstraction)

package dxl

import (
	"drlabs/drivers/dynamixel"
)

// Re-export dynamixel types for backward compatibility.
// New code should use the dynamixel package directly.

// DynDriver is the dynamixel driver type (new code: dynamixel.Driver).
type DynDriver = dynamixel.Driver

// DynController is the dynamixel controller type (new code: dynamixel.Controller).
type DynController = dynamixel.Controller

// DynMotorModel is the motor model definition (new code: dynamixel.MotorModel).
type DynMotorModel = dynamixel.MotorModel

// DynSyncWriteData is sync write data (new code: dynamixel.SyncWriteData).
type DynSyncWriteData = dynamixel.SyncWriteData

// DynSyncReadData is sync read data (new code: dynamixel.SyncReadData).
type DynSyncReadData = dynamixel.SyncReadData

// Re-export motor model presets.
var (
	DynModelXSeries   = dynamixel.ModelXSeries
	DynModelProSeries = dynamixel.ModelProSeries
)
