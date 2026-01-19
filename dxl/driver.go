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

	// SDK uses (TxTime + Latency*2 + 2ms). With default 16ms latency, this can be >34ms.
	// We set a safe fixed timeout of 100ms to ensure stability on all platforms.
	deadline := time.Now().Add(100 * time.Millisecond)

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
			// Yield CPU to allow OS to process IO
			// SDK uses Sleep(0), Go's runtime.Gosched() is similar.
			// runtime.Gosched() // Need 'runtime' import if used
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

// SyncWriteData represents data for a single motor in sync write
type SyncWriteData struct {
	ID   uint8
	Data []byte
}

// SyncWrite writes same address to multiple motors in one packet
// More efficient than individual writes
func (d *Driver) SyncWrite(addr uint16, dataLength uint16, motors []SyncWriteData) error {
	if len(motors) == 0 {
		return fmt.Errorf("no motors provided")
	}

	// Build parameters: [Addr_L, Addr_H, Len_L, Len_H, [ID1, Data1...], [ID2, Data2...], ...]
	params := make([]byte, 4)
	binary.LittleEndian.PutUint16(params[0:], addr)
	binary.LittleEndian.PutUint16(params[2:], dataLength)

	// Append each motor's ID and data
	for _, m := range motors {
		if len(m.Data) != int(dataLength) {
			return fmt.Errorf("motor ID %d: data length mismatch (expected %d, got %d)", m.ID, dataLength, len(m.Data))
		}
		params = append(params, m.ID)
		params = append(params, m.Data...)
	}

	// Use broadcast ID (0xFE) for sync write - no response expected
	tx := BuildPacket(0xFE, InstSyncWrite, params)

	// For sync write, we don't expect response, just send
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

	// Read responses from each motor
	results := make([]SyncReadData, len(ids))
	for i, id := range ids {
		results[i].ID = id

		// Read response packet
		deadline := time.Now().Add(100 * time.Millisecond)
		buf := bytes.NewBuffer(nil)
		tmp := make([]byte, 1024)

		for time.Now().Before(deadline) {
			n, err := d.port.Read(tmp)
			if err != nil {
				results[i].Err = err
				break
			}
			if n > 0 {
				buf.Write(tmp[:n])
				if buf.Len() >= 7 {
					b := buf.Bytes()
					startIdx := -1
					for j := 0; j < len(b)-3; j++ {
						if b[j] == 0xFF && b[j+1] == 0xFF && b[j+2] == 0xFD {
							startIdx = j
							break
						}
					}

					if startIdx != -1 && buf.Len() >= startIdx+7 {
						pkt := buf.Bytes()
						bodyLen := uint16(pkt[startIdx+5]) | (uint16(pkt[startIdx+6]) << 8)
						totalLen := startIdx + 7 + int(bodyLen)

						if buf.Len() >= totalLen {
							rx := pkt[startIdx:totalLen]
							_, errCode, readParams, err := ParsePacket(rx)
							if err != nil {
								results[i].Err = err
							} else if errCode != 0 {
								results[i].Err = fmt.Errorf("motor error code: %02X", errCode)
							} else {
								results[i].Data = readParams
							}
							break
						}
					}
				}
			}
		}

		if results[i].Data == nil && results[i].Err == nil {
			results[i].Err = fmt.Errorf("timeout waiting for motor %d", id)
		}
	}

	return results, nil
}

// SyncRead4Byte reads 4-byte values from multiple motors
func (d *Driver) SyncRead4Byte(addr uint16, ids []uint8) (map[uint8]uint32, error) {
	results, err := d.SyncRead(addr, 4, ids)
	if err != nil {
		return nil, err
	}

	values := make(map[uint8]uint32)
	for _, r := range results {
		if r.Err != nil {
			return nil, fmt.Errorf("motor %d error: %v", r.ID, r.Err)
		}
		if len(r.Data) != 4 {
			return nil, fmt.Errorf("motor %d: invalid data length %d", r.ID, len(r.Data))
		}
		values[r.ID] = binary.LittleEndian.Uint32(r.Data)
	}

	return values, nil
}
