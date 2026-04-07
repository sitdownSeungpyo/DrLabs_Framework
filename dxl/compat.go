// Backward compatibility shims.
//
// The dxl package is maintained for backward compatibility.
// New code should import the specific packages directly:
//   - go_dxl/dynamixel   (Dynamixel protocol and driver)
//   - go_dxl/motor       (universal motor interfaces)
//   - go_dxl/trajectory  (motion planning)
//   - go_dxl/transport/serial (serial port abstraction)

package dxl

import (
	"go_dxl/dynamixel"
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
