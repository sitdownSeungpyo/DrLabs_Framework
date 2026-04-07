package dxl

import "encoding/binary"

// ReadTemperature reads the present temperature from a motor.
// addr is the control table address for present temperature (e.g., 146 for X-Series).
// Returns temperature in degrees Celsius.
func (d *Driver) ReadTemperature(id uint8, addr uint16) (uint8, error) {
	data, err := d.Read(id, addr, 1)
	if err != nil {
		return 0, err
	}
	if len(data) < 1 {
		return 0, nil
	}
	return data[0], nil
}

// ReadVoltage reads the present input voltage from a motor.
// addr is the control table address for present voltage (e.g., 144 for X-Series).
// Returns voltage in Volts (raw value / 10.0).
func (d *Driver) ReadVoltage(id uint8, addr uint16) (float64, error) {
	data, err := d.Read(id, addr, 2)
	if err != nil {
		return 0, err
	}
	if len(data) < 2 {
		return 0, nil
	}
	raw := binary.LittleEndian.Uint16(data)
	return float64(raw) / 10.0, nil
}

// ReadCurrent reads the present current from a motor.
// addr is the control table address for present current (e.g., 126 for X-Series).
// Returns current in raw units (1 unit = ~2.69mA for X-Series).
func (d *Driver) ReadCurrent(id uint8, addr uint16) (int16, error) {
	data, err := d.Read(id, addr, 2)
	if err != nil {
		return 0, err
	}
	if len(data) < 2 {
		return 0, nil
	}
	return int16(binary.LittleEndian.Uint16(data)), nil
}

// ReadHardwareError reads the hardware error status from a motor.
// addr is the control table address for hardware error status (e.g., 70 for X-Series).
// Returns a bitmask of hardware errors.
func (d *Driver) ReadHardwareError(id uint8, addr uint16) (uint8, error) {
	data, err := d.Read(id, addr, 1)
	if err != nil {
		return 0, err
	}
	if len(data) < 1 {
		return 0, nil
	}
	return data[0], nil
}

// WriteLED sets the LED on/off for a motor.
// addr is the control table address for LED (e.g., 65 for X-Series).
func (d *Driver) WriteLED(id uint8, addr uint16, on bool) error {
	val := byte(0)
	if on {
		val = 1
	}
	return d.Write(id, addr, []byte{val})
}
