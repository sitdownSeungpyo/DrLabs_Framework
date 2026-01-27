package dxl

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"time"
)

// Protocol constants
const (
	// ReadBufferSize is the size of the temporary buffer for reading serial data
	ReadBufferSize = 1024
	// MinHeaderSize is the minimum bytes needed to parse packet header and length
	MinHeaderSize = 7 // Header(4) + ID(1) + Length(2)
	// DefaultTimeout is the default timeout for packet read operations
	DefaultTimeout = 100 * time.Millisecond
)

// SerialPortInterface defines the contract for serial port operations.
// Implementations must handle platform-specific serial I/O (Windows/Linux).
// This interface enables dependency injection and mocking for unit tests.
type SerialPortInterface interface {
	// Read reads up to len(b) bytes into b from the serial port.
	// Returns the number of bytes read (0 <= n <= len(b)) and any error encountered.
	Read(b []byte) (int, error)

	// Write writes len(b) bytes from b to the serial port.
	// Returns the number of bytes written and any error encountered.
	Write(b []byte) (int, error)

	// Close closes the serial port and releases associated resources.
	Close() error
}

type Driver struct {
	port    SerialPortInterface
	Timeout time.Duration // Configurable timeout for read operations
}

func NewDriver(port SerialPortInterface) *Driver {
	return &Driver{port: port, Timeout: DefaultTimeout}
}

// findPacketStart finds the start index of a valid packet header (FF FF FD)
// Returns -1 if no valid header is found
func findPacketStart(data []byte) int {
	for i := 0; i < len(data)-2; i++ {
		if data[i] == 0xFF && data[i+1] == 0xFF && data[i+2] == 0xFD {
			return i
		}
	}
	return -1
}

// readPacketWithTimeout reads a complete Dynamixel packet from the serial port.
// It accumulates bytes until a complete packet is received or timeout occurs.
// Returns the complete packet bytes or an error if timeout/read failure occurs.
func (d *Driver) readPacketWithTimeout(timeout time.Duration) ([]byte, error) {
	deadline := time.Now().Add(timeout)
	buf := bytes.NewBuffer(nil)
	tmp := make([]byte, ReadBufferSize)

	for time.Now().Before(deadline) {
		n, err := d.port.Read(tmp)
		if err != nil {
			return nil, err
		}
		if n > 0 {
			buf.Write(tmp[:n])

			// Check if we have enough bytes for header + length fields
			if buf.Len() >= MinHeaderSize {
				b := buf.Bytes()
				startIdx := findPacketStart(b)

				if startIdx != -1 && buf.Len() >= startIdx+MinHeaderSize {
					pkt := buf.Bytes()
					bodyLen := uint16(pkt[startIdx+5]) | (uint16(pkt[startIdx+6]) << 8)
					totalLen := startIdx + MinHeaderSize + int(bodyLen)

					if buf.Len() >= totalLen {
						return pkt[startIdx:totalLen], nil
					}
				}
			}
		}
	}

	return nil, fmt.Errorf("read timeout, buffered: %x", buf.Bytes())
}

// Transfer sends a packet and waits for a response.
// This is the fundamental request-response pattern for Dynamixel communication.
func (d *Driver) Transfer(txPacket []byte) ([]byte, error) {
	_, err := d.port.Write(txPacket)
	if err != nil {
		return nil, fmt.Errorf("write failed: %v", err)
	}

	return d.readPacketWithTimeout(d.Timeout)
}

func (d *Driver) Write(id uint8, addr uint16, data []byte) error {
	// Build Packet
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

func (d *Driver) Read(id uint8, addr uint16, length uint16) ([]byte, error) {
	// Build Packet
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

// Write4Byte Helper
func (d *Driver) Write4Byte(id uint8, addr uint16, val uint32) error {
	buf := make([]byte, 4)
	binary.LittleEndian.PutUint32(buf, val)
	return d.Write(id, addr, buf)
}

// Read4Byte Helper
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

// SyncWriteData represents data for a single motor in sync write
type SyncWriteData struct {
	ID   uint8
	Data []byte
}

// SyncWrite writes the same address to multiple motors in a single packet.
// This is significantly more efficient than individual writes when controlling
// multiple motors simultaneously (e.g., synchronized motion).
func (d *Driver) SyncWrite(addr uint16, dataLength uint16, motors []SyncWriteData) error {
	if len(motors) == 0 {
		return fmt.Errorf("no motors provided")
	}

	// Validate data length for all motors first
	for _, m := range motors {
		if len(m.Data) != int(dataLength) {
			return fmt.Errorf("motor ID %d: data length mismatch (expected %d, got %d)", m.ID, dataLength, len(m.Data))
		}
	}

	// Pre-allocate buffer with exact size to avoid reallocations
	// Format: [Addr_L, Addr_H, Len_L, Len_H, [ID1, Data1...], [ID2, Data2...], ...]
	totalSize := 4 + len(motors)*(1+int(dataLength))
	params := make([]byte, 4, totalSize)
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], dataLength)

	// Append motor data efficiently
	for _, m := range motors {
		params = append(params, m.ID)
		params = append(params, m.Data...)
	}

	// Use broadcast ID (0xFE) - no status response expected
	tx := BuildPacket(0xFE, InstSyncWrite, params)

	_, err := d.port.Write(tx)
	if err != nil {
		return fmt.Errorf("sync write failed: %v", err)
	}

	// Small delay to ensure packet transmission completes
	time.Sleep(time.Millisecond)

	return nil
}

// SyncWrite4Byte writes 4-byte values to multiple motors
func (d *Driver) SyncWrite4Byte(addr uint16, values map[uint8]uint32) error {
	motors := make([]SyncWriteData, 0, len(values))
	for id, val := range values {
		data := make([]byte, 4)
		binary.LittleEndian.PutUint32(data, val)
		motors = append(motors, SyncWriteData{ID: id, Data: data})
	}
	return d.SyncWrite(addr, 4, motors)
}

// SyncReadData represents expected data for a motor in sync read
type SyncReadData struct {
	ID   uint8
	Data []byte
	Err  error
}

// SyncRead reads same address from multiple motors
func (d *Driver) SyncRead(addr uint16, dataLength uint16, ids []uint8) ([]SyncReadData, error) {
	if len(ids) == 0 {
		return nil, fmt.Errorf("no motor IDs provided")
	}

	// Build parameters: [Addr_L, Addr_H, Len_L, Len_H, ID1, ID2, ...]
	params := make([]byte, 4+len(ids))
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], dataLength)
	copy(params[4:], ids)

	// Use broadcast ID for sync read request
	tx := BuildPacket(0xFE, InstSyncRead, params)

	// Send request
	_, err := d.port.Write(tx)
	if err != nil {
		return nil, fmt.Errorf("sync read tx failed: %v", err)
	}

	// Read responses from each motor using the shared helper
	results := make([]SyncReadData, len(ids))
	for i, id := range ids {
		results[i].ID = id

		rx, err := d.readPacketWithTimeout(d.Timeout)
		if err != nil {
			results[i].Err = fmt.Errorf("timeout waiting for motor %d: %v", id, err)
			continue
		}

		_, errCode, readParams, err := ParsePacket(rx)
		if err != nil {
			results[i].Err = err
		} else if errCode != 0 {
			results[i].Err = fmt.Errorf("motor error code: %02X", errCode)
		} else {
			results[i].Data = readParams
		}
	}

	return results, nil
}

// SyncRead4Byte reads 4-byte values from multiple motors.
// Returns partial results: motors that responded successfully are included in the map.
// Returns an error only if no motor responded at all.
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
