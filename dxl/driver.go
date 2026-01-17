package dxl

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"time"
)

type Driver struct {
	port *SerialPort
}

func NewDriver(port *SerialPort) *Driver {
	return &Driver{port: port}
}

// Transfer sends a packet and waits for a response
func (d *Driver) Transfer(txPacket []byte) ([]byte, error) {
	// 1. Clear Input Buffer (Optional but recommended)
	// d.port.PurgeRX() // TODO if implemented

	// 2. Write
	_, err := d.port.Write(txPacket)
	if err != nil {
		return nil, fmt.Errorf("write failed: %v", err)
	}

	// 3. Read Response
	// Protocol 2.0 Header: FF FF FD 00 ID LEN_L LEN_H ...
	// We read state by state or just read a chunk and parse.
	// For simplicity, we try to read Header first.

	// Read with timeout loop
	deadline := time.Now().Add(20 * time.Millisecond) // 20ms Timeout

	// Poor man's read loop
	buf := bytes.NewBuffer(nil)
	tmp := make([]byte, 1024)

	for time.Now().Before(deadline) {
		n, err := d.port.Read(tmp)
		if err != nil {
			return nil, err
		}
		if n > 0 {
			buf.Write(tmp[:n])
			// Check if we have header length
			if buf.Len() >= 7 {
				// Parse Length
				b := buf.Bytes()
				// Find valid header start
				startIdx := -1
				for i := 0; i < len(b)-3; i++ {
					if b[i] == 0xFF && b[i+1] == 0xFF && b[i+2] == 0xFD {
						startIdx = i
						break
					}
				}

				if startIdx != -1 {
					// Discard garbage before header
					if startIdx > 0 {
						// This is expensive slice op, optim: just track index
						// buf = bytes.NewBuffer(b[startIdx:])
					}

					// If we have enough for Length parsing
					if buf.Len() >= startIdx+7 {
						pkt := buf.Bytes()
						bodyLen := uint16(pkt[startIdx+5]) | (uint16(pkt[startIdx+6]) << 8)
						totalLen := startIdx + 7 + int(bodyLen)

						if buf.Len() >= totalLen {
							// Return complete packet
							return pkt[startIdx:totalLen], nil
						}
					}
				}
			}
		} else {
			// Small sleep to yield
			time.Sleep(1 * time.Millisecond)
		}
	}

	return nil, fmt.Errorf("read timeout, buffered: %x", buf.Bytes())
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
