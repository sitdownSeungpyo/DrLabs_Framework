package dynamixel

import (
	"encoding/binary"
	"fmt"
	"time"

	"go_dxl/motor"
	"go_dxl/transport/serial"
)

// Default timing constants
const (
	DefaultTimeout      = 100 * time.Millisecond
	DefaultPollInterval = 500 * time.Microsecond
)

// Driver implements low-level Dynamixel Protocol 2.0 communication.
// It also implements motor.Motor when bound to a specific motor ID.
type Driver struct {
	port         serial.Port
	Timeout      time.Duration
	PollInterval time.Duration
}

// NewDriver creates a Dynamixel driver using the given serial port.
func NewDriver(port serial.Port) *Driver {
	return &Driver{port: port, Timeout: DefaultTimeout, PollInterval: DefaultPollInterval}
}

// Flush clears the serial port buffers if supported.
func (d *Driver) Flush() {
	if flusher, ok := d.port.(serial.Flusher); ok {
		flusher.Flush()
	}
}

// readPacketWithTimeout reads a complete Dynamixel packet with CRC validation.
func (d *Driver) readPacketWithTimeout(timeout time.Duration) ([]byte, error) {
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 0, ReadBufferSize)
	tmp := make([]byte, ReadBufferSize)

	for time.Now().Before(deadline) {
		n, err := d.port.Read(tmp)
		if err != nil {
			return nil, err
		}
		if n == 0 {
			time.Sleep(d.PollInterval)
			continue
		}
		buf = append(buf, tmp[:n]...)

		for len(buf) >= MinHeaderSize {
			startIdx := FindPacketStart(buf)
			if startIdx == -1 {
				if len(buf) > 2 {
					buf = buf[len(buf)-2:]
				}
				break
			}
			if startIdx > 0 {
				buf = buf[startIdx:]
			}
			if len(buf) < MinHeaderSize {
				break
			}

			bodyLen := uint16(buf[5]) | (uint16(buf[6]) << 8)
			if int(bodyLen) > MaxPacketSize-MinHeaderSize {
				buf = buf[3:]
				continue
			}

			totalLen := MinHeaderSize + int(bodyLen)
			if totalLen > MaxPacketSize {
				buf = buf[3:]
				continue
			}
			if len(buf) < totalLen {
				break
			}

			pkt := buf[:totalLen]
			receivedCRC := uint16(pkt[totalLen-2]) | (uint16(pkt[totalLen-1]) << 8)
			calcCRC := UpdateCRC(0, pkt[:totalLen-2])
			if receivedCRC == calcCRC {
				return pkt, nil
			}
			buf = buf[3:]
		}
	}

	return nil, fmt.Errorf("read timeout, buffered: %x", buf)
}

// Transfer sends a packet and waits for a response.
func (d *Driver) Transfer(txPacket []byte) ([]byte, error) {
	d.Flush()
	_, err := d.port.Write(txPacket)
	if err != nil {
		return nil, fmt.Errorf("write failed: %v", err)
	}
	return d.readPacketWithTimeout(d.Timeout)
}

// Write writes data to a motor register.
func (d *Driver) Write(id uint8, addr uint16, data []byte) error {
	params := make([]byte, 2+len(data))
	binary.LittleEndian.PutUint16(params[0:], addr)
	copy(params[2:], data)

	tx := BuildPacket(id, InstWrite, params)
	rx, err := d.Transfer(tx)
	if err != nil {
		return err
	}

	_, errCode, _, err := ParsePacket(rx)
	if err != nil {
		return err
	}
	if errCode != 0 {
		return fmt.Errorf("dxl error code: %02X", errCode)
	}
	return nil
}

// Read reads data from a motor register.
func (d *Driver) Read(id uint8, addr uint16, length uint16) ([]byte, error) {
	params := make([]byte, 4)
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], length)

	tx := BuildPacket(id, InstRead, params)
	rx, err := d.Transfer(tx)
	if err != nil {
		return nil, err
	}

	_, errCode, readParams, err := ParsePacket(rx)
	if err != nil {
		return nil, err
	}
	if errCode != 0 {
		return nil, fmt.Errorf("dxl error code: %02X", errCode)
	}
	return readParams, nil
}

// Ping verifies communication with a motor.
func (d *Driver) Ping(id uint8) (modelNum uint16, err error) {
	tx := BuildPacket(id, InstPing, nil)
	rx, err := d.Transfer(tx)
	if err != nil {
		return 0, err
	}

	_, errCode, params, err := ParsePacket(rx)
	if err != nil {
		return 0, err
	}
	if errCode != 0 {
		return 0, fmt.Errorf("dxl error code: %02X", errCode)
	}
	if len(params) >= 3 {
		modelNum = binary.LittleEndian.Uint16(params[0:])
	}
	return modelNum, nil
}

// PingMotor implements motor.Motor.Ping returning ModelInfo.
func (d *Driver) PingMotor(id uint8) (motor.ModelInfo, error) {
	tx := BuildPacket(id, InstPing, nil)
	rx, err := d.Transfer(tx)
	if err != nil {
		return motor.ModelInfo{}, err
	}

	_, errCode, params, err := ParsePacket(rx)
	if err != nil {
		return motor.ModelInfo{}, err
	}
	if errCode != 0 {
		return motor.ModelInfo{}, fmt.Errorf("dxl error code: %02X", errCode)
	}

	info := motor.ModelInfo{}
	if len(params) >= 2 {
		info.ModelNumber = binary.LittleEndian.Uint16(params[0:])
	}
	if len(params) >= 3 {
		info.Firmware = params[2]
	}
	return info, nil
}

// Reboot sends a reboot instruction to the motor.
func (d *Driver) Reboot(id uint8) error {
	tx := BuildPacket(id, InstReboot, nil)
	rx, err := d.Transfer(tx)
	if err != nil {
		return err
	}
	_, errCode, _, err := ParsePacket(rx)
	if err != nil {
		return err
	}
	if errCode != 0 {
		return fmt.Errorf("dxl error code: %02X", errCode)
	}
	return nil
}

// FactoryReset resets the motor to factory defaults.
// Level: 0xFF=all, 0x01=except ID, 0x02=except ID and baud rate.
func (d *Driver) FactoryReset(id uint8, level uint8) error {
	tx := BuildPacket(id, InstFactoryReset, []byte{level})
	rx, err := d.Transfer(tx)
	if err != nil {
		return err
	}
	_, errCode, _, err := ParsePacket(rx)
	if err != nil {
		return err
	}
	if errCode != 0 {
		return fmt.Errorf("dxl error code: %02X", errCode)
	}
	return nil
}

// Write4Byte writes a 4-byte value to a motor register.
func (d *Driver) Write4Byte(id uint8, addr uint16, val uint32) error {
	buf := make([]byte, 4)
	binary.LittleEndian.PutUint32(buf, val)
	return d.Write(id, addr, buf)
}

// Read4Byte reads a 4-byte value from a motor register.
func (d *Driver) Read4Byte(id uint8, addr uint16) (uint32, error) {
	data, err := d.Read(id, addr, 4)
	if err != nil {
		return 0, err
	}
	if len(data) != 4 {
		return 0, fmt.Errorf("invalid length: %d", len(data))
	}
	return binary.LittleEndian.Uint32(data), nil
}

// SyncWriteData represents data for a single motor in a sync write operation.
type SyncWriteData struct {
	ID   uint8
	Data []byte
}

// SyncWrite writes the same address range to multiple motors in a single packet.
func (d *Driver) SyncWrite(addr uint16, dataLength uint16, motors []SyncWriteData) error {
	if len(motors) == 0 {
		return fmt.Errorf("no motors provided")
	}
	for _, m := range motors {
		if len(m.Data) != int(dataLength) {
			return fmt.Errorf("motor ID %d: data length mismatch (expected %d, got %d)", m.ID, dataLength, len(m.Data))
		}
	}

	totalSize := 4 + len(motors)*(1+int(dataLength))
	params := make([]byte, 4, totalSize)
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], dataLength)

	for _, m := range motors {
		params = append(params, m.ID)
		params = append(params, m.Data...)
	}

	tx := BuildPacket(0xFE, InstSyncWrite, params)
	n, err := d.port.Write(tx)
	if err != nil {
		return fmt.Errorf("sync write failed: %v", err)
	}
	if n != len(tx) {
		return fmt.Errorf("sync write incomplete: sent %d/%d bytes", n, len(tx))
	}

	time.Sleep(time.Millisecond)
	return nil
}

// SyncWrite4Byte writes 4-byte values to multiple motors.
func (d *Driver) SyncWrite4Byte(addr uint16, values map[uint8]uint32) error {
	motors := make([]SyncWriteData, 0, len(values))
	for id, val := range values {
		data := make([]byte, 4)
		binary.LittleEndian.PutUint32(data, val)
		motors = append(motors, SyncWriteData{ID: id, Data: data})
	}
	return d.SyncWrite(addr, 4, motors)
}

// SyncReadData represents the result for a single motor in a sync read.
type SyncReadData struct {
	ID   uint8
	Data []byte
	Err  error
}

// extractPacketFromBuffer extracts a single packet from buffer.
func extractPacketFromBuffer(data []byte) (packet []byte, remaining []byte, err error) {
	if len(data) < MinHeaderSize {
		return nil, data, nil
	}
	startIdx := FindPacketStart(data)
	if startIdx == -1 {
		return nil, data, nil
	}
	if len(data) < startIdx+MinHeaderSize {
		return nil, data, nil
	}

	bodyLen := uint16(data[startIdx+5]) | (uint16(data[startIdx+6]) << 8)
	if int(bodyLen) > MaxPacketSize-MinHeaderSize {
		return nil, nil, fmt.Errorf("body length exceeds maximum: %d", bodyLen)
	}

	totalLen := startIdx + MinHeaderSize + int(bodyLen)
	if totalLen > MaxPacketSize {
		return nil, nil, fmt.Errorf("packet length exceeds maximum: %d", totalLen)
	}
	if len(data) < totalLen {
		return nil, data, nil
	}

	return data[startIdx:totalLen], data[totalLen:], nil
}

// SyncRead reads the same address range from multiple motors.
func (d *Driver) SyncRead(addr uint16, dataLength uint16, ids []uint8) ([]SyncReadData, error) {
	if len(ids) == 0 {
		return nil, fmt.Errorf("no motor IDs provided")
	}

	params := make([]byte, 4+len(ids))
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], dataLength)
	copy(params[4:], ids)

	tx := BuildPacket(0xFE, InstSyncRead, params)
	_, err := d.port.Write(tx)
	if err != nil {
		return nil, fmt.Errorf("sync read tx failed: %v", err)
	}

	results := make([]SyncReadData, len(ids))
	for i := range ids {
		results[i].ID = ids[i]
	}

	deadline := time.Now().Add(d.Timeout * time.Duration(len(ids)))
	buf := make([]byte, 0, ReadBufferSize)
	tmp := make([]byte, ReadBufferSize)
	packetsFound := 0

	for time.Now().Before(deadline) && packetsFound < len(ids) {
		for packetsFound < len(ids) {
			packet, remaining, err := extractPacketFromBuffer(buf)
			if err != nil {
				if len(buf) > 3 {
					buf = buf[1:]
					continue
				}
				break
			}
			if packet == nil {
				break
			}

			respID, errCode, readParams, parseErr := ParsePacket(packet)
			buf = remaining

			for i, id := range ids {
				if id == respID && results[i].Data == nil && results[i].Err == nil {
					if parseErr != nil {
						results[i].Err = parseErr
					} else if errCode != 0 {
						results[i].Err = fmt.Errorf("motor error code: %02X", errCode)
					} else {
						results[i].Data = readParams
					}
					packetsFound++
					break
				}
			}
		}

		if packetsFound >= len(ids) {
			break
		}

		n, err := d.port.Read(tmp)
		if err != nil {
			break
		}
		if n == 0 {
			time.Sleep(500 * time.Microsecond)
			continue
		}
		buf = append(buf, tmp[:n]...)
	}

	for i := range results {
		if results[i].Data == nil && results[i].Err == nil {
			results[i].Err = fmt.Errorf("timeout waiting for motor %d", ids[i])
		}
	}

	return results, nil
}

// SyncRead4Byte reads 4-byte values from multiple motors (partial results supported).
func (d *Driver) SyncRead4Byte(addr uint16, ids []uint8) (map[uint8]uint32, error) {
	results, err := d.SyncRead(addr, 4, ids)
	if err != nil {
		return nil, err
	}

	values := make(map[uint8]uint32)
	var lastErr error
	for _, r := range results {
		if r.Err != nil {
			lastErr = fmt.Errorf("motor %d error: %v", r.ID, r.Err)
			continue
		}
		if len(r.Data) != 4 {
			lastErr = fmt.Errorf("motor %d: invalid data length %d", r.ID, len(r.Data))
			continue
		}
		values[r.ID] = binary.LittleEndian.Uint32(r.Data)
	}

	if len(values) == 0 && lastErr != nil {
		return nil, lastErr
	}
	return values, nil
}
