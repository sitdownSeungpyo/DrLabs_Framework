// Package odrive implements the ODrive motor controller protocol.
// Supports both CAN and UART (ASCII) communication.
//
// CAN: (node_id << 5) | command_id
// UART: ASCII commands (e.g., "w axis0.requested_state 8\n")
//
// This package implements motor.Motor and motor.Controller from go_dxl/motor.
package odrive
