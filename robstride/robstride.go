// Package robstride implements the Robstride CAN motor protocol.
// Models: 01, 02, 03, 04
//
// Protocol: CAN 2.0B extended frame (29-bit ID encodes motor ID + command type).
// Data: 8 bytes per frame (CAN handles CRC).
//
// This package implements motor.Motor and motor.Controller from go_dxl/motor.
package robstride
