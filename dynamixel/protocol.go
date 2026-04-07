// Package dynamixel implements Dynamixel Protocol 2.0 motor control.
// It provides Driver (motor.Motor) and Controller (motor.Controller) implementations.
package dynamixel

import (
	"errors"
	"fmt"
)

// Instruction constants for Dynamixel Protocol 2.0
const (
	Header1  = 0xFF
	Header2  = 0xFF
	Header3  = 0xFD
	Reserved = 0x00

	InstPing         = 0x01
	InstRead         = 0x02
	InstWrite        = 0x03
	InstRegWrite     = 0x04
	InstAction       = 0x05
	InstFactoryReset = 0x06
	InstReboot       = 0x08
	InstStatus       = 0x55
	InstSyncRead     = 0x82
	InstSyncWrite    = 0x83
	InstBulkRead     = 0x92
	InstBulkWrite    = 0x93
)

// Packet size limits
const (
	ReadBufferSize = 1024
	MinHeaderSize  = 7 // Header(4) + ID(1) + Length(2)
	MaxPacketSize  = 1024
)

// CRC table (CRC-16-IBM used by DXL 2.0)
var crcTable [256]uint16

func init() {
	poly := uint16(0x8005)
	for i := 0; i < 256; i++ {
		crc := uint16(i) << 8
		for j := 0; j < 8; j++ {
			if (crc & 0x8000) != 0 {
				crc = (crc << 1) ^ poly
			} else {
				crc = crc << 1
			}
		}
		crcTable[i] = crc
	}
}

// UpdateCRC computes the CRC-16 checksum for Dynamixel Protocol 2.0.
func UpdateCRC(crcStart uint16, data []byte) uint16 {
	crc := crcStart
	for _, b := range data {
		i := ((crc >> 8) ^ uint16(b)) & 0xFF
		crc = (crc << 8) ^ crcTable[i]
	}
	return crc
}

// StuffParams applies byte stuffing (FF FF FD → FF FF FD FD).
func StuffParams(params []byte) []byte {
	stuffed := make([]byte, 0, len(params)+2)
	ffCount := 0
	for _, b := range params {
		stuffed = append(stuffed, b)
		if b == 0xFF {
			ffCount++
		} else {
			if ffCount >= 2 && b == 0xFD {
				stuffed = append(stuffed, 0xFD)
			}
			ffCount = 0
		}
	}
	return stuffed
}

// DestuffParams removes byte stuffing (FF FF FD FD → FF FF FD).
func DestuffParams(data []byte) []byte {
	if len(data) < 4 {
		return data
	}
	result := make([]byte, 0, len(data))
	for i := 0; i < len(data); {
		if i+3 < len(data) &&
			data[i] == 0xFF && data[i+1] == 0xFF &&
			data[i+2] == 0xFD && data[i+3] == 0xFD {
			result = append(result, 0xFF, 0xFF, 0xFD)
			i += 4
		} else {
			result = append(result, data[i])
			i++
		}
	}
	return result
}

// BuildPacket constructs a Protocol 2.0 packet.
func BuildPacket(id uint8, inst uint8, params []byte) []byte {
	pkt := []byte{Header1, Header2, Header3, Reserved, id}
	stuffedParams := StuffParams(params)
	length := 1 + len(stuffedParams) + 2
	pkt = append(pkt, byte(length&0xFF), byte((length>>8)&0xFF))
	pkt = append(pkt, inst)
	pkt = append(pkt, stuffedParams...)
	crc := UpdateCRC(0, pkt)
	pkt = append(pkt, byte(crc&0xFF), byte((crc>>8)&0xFF))
	return pkt
}

// ParsePacket validates a Protocol 2.0 status response.
func ParsePacket(packet []byte) (id uint8, errCode uint8, params []byte, err error) {
	if len(packet) < 11 {
		return 0, 0, nil, errors.New("packet too short")
	}
	if packet[0] != Header1 || packet[1] != Header2 || packet[2] != Header3 {
		return 0, 0, nil, errors.New("invalid header")
	}

	id = packet[4]
	length := uint16(packet[5]) | (uint16(packet[6]) << 8)

	if len(packet) != int(length+7) {
		return 0, 0, nil, fmt.Errorf("length mismatch: expected %d, got %d", length+7, len(packet))
	}

	receivedCRC := uint16(packet[len(packet)-2]) | (uint16(packet[len(packet)-1]) << 8)
	calcCRC := UpdateCRC(0, packet[:len(packet)-2])
	if receivedCRC != calcCRC {
		return 0, 0, nil, fmt.Errorf("CRC error: expected %04X, got %04X", calcCRC, receivedCRC)
	}

	errCode = packet[8]

	if len(packet) > 11 {
		rawParams := packet[9 : len(packet)-2]
		params = DestuffParams(rawParams)
	}

	return id, errCode, params, nil
}

// FindPacketStart finds the start index of a valid packet header (FF FF FD).
func FindPacketStart(data []byte) int {
	for i := 0; i < len(data)-2; i++ {
		if data[i] == 0xFF && data[i+1] == 0xFF && data[i+2] == 0xFD {
			return i
		}
	}
	return -1
}
