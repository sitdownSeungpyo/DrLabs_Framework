package dxl

import (
	"errors"
	"fmt"
)

// Constants
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

// CRC16 Lookup Table (CRC-16-IBM / XMODEM variant used by DXL 2.0)
// CRC16 Lookup Table
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

func UpdateCRC(crcStart uint16, data []byte) uint16 {
	crc := crcStart
	for _, b := range data {
		i := ((crc >> 8) ^ uint16(b)) & 0xFF
		crc = (crc << 8) ^ crcTable[i]
	}
	return crc
}

// StuffParams applies Dynamixel Protocol 2.0 byte stuffing.
// When the pattern 0xFF 0xFF 0xFD appears in parameter data, an additional
// 0xFD is inserted to prevent confusion with the packet header (0xFF 0xFF 0xFD 0x00).
// Example: [0xFF 0xFF 0xFD] -> [0xFF 0xFF 0xFD 0xFD]
func StuffParams(params []byte) []byte {
	stuffed := make([]byte, 0, len(params)+2)
	ffCount := 0

	for _, b := range params {
		stuffed = append(stuffed, b)
		if b == 0xFF {
			ffCount++
		} else {
			if ffCount >= 2 && b == 0xFD {
				stuffed = append(stuffed, 0xFD) // Insert stuffing byte
			}
			ffCount = 0
		}
	}
	return stuffed
}

// DestuffParams removes Dynamixel Protocol 2.0 byte stuffing from received data.
// Reverses the stuffing process: [0xFF 0xFF 0xFD 0xFD] -> [0xFF 0xFF 0xFD]
func DestuffParams(data []byte) []byte {
	if len(data) < 4 {
		return data
	}

	result := make([]byte, 0, len(data))

	for i := 0; i < len(data); {
		// Look for the stuffed pattern: FF FF FD FD
		if i+3 < len(data) &&
			data[i] == 0xFF &&
			data[i+1] == 0xFF &&
			data[i+2] == 0xFD &&
			data[i+3] == 0xFD {
			// Found stuffed pattern, output only FF FF FD
			result = append(result, 0xFF, 0xFF, 0xFD)
			i += 4 // Skip all 4 bytes
		} else {
			result = append(result, data[i])
			i++
		}
	}

	return result
}

// BuildPacket constructs a Protocol 2.0 Packet
func BuildPacket(id uint8, inst uint8, params []byte) []byte {
	// 1. Header
	pkt := []byte{Header1, Header2, Header3, Reserved, id}

	// 2. Length (Low, High)
	// Length = Instruction(1) + Params(N) + CRC(2)
	// Byte stuffing MUST be applied to params first!
	stuffedParams := StuffParams(params)
	length := 1 + len(stuffedParams) + 2

	pkt = append(pkt, byte(length&0xFF), byte((length>>8)&0xFF))

	// 3. Instruction
	pkt = append(pkt, inst)

	// 4. Params
	pkt = append(pkt, stuffedParams...)

	// 5. CRC
	crc := UpdateCRC(0, pkt)
	pkt = append(pkt, byte(crc&0xFF), byte((crc>>8)&0xFF))

	return pkt
}

// ParsePacket validates a response from stream
// Returns: ID, ErrorCode, Params, valid/error
func ParsePacket(packet []byte) (id uint8, errCode uint8, params []byte, err error) {
	// Min packet size: H(4)+ID(1)+Len(2)+Inst(1)+Err(1)+CRC(2) = 11 bytes
	if len(packet) < 11 {
		return 0, 0, nil, errors.New("packet too short")
	}

	// Check Header
	if packet[0] != Header1 || packet[1] != Header2 || packet[2] != Header3 {
		return 0, 0, nil, errors.New("invalid header")
	}

	id = packet[4]
	length := uint16(packet[5]) | (uint16(packet[6]) << 8)

	// Verify Packet Length
	if len(packet) != int(length+7) { // 7 = H(4)+ID(1)+Len(2)
		return 0, 0, nil, fmt.Errorf("length mismatch: expected %d, got %d", length+7, len(packet))
	}

	// Verify CRC
	receivedCRC := uint16(packet[len(packet)-2]) | (uint16(packet[len(packet)-1]) << 8)
	calcCRC := UpdateCRC(0, packet[:len(packet)-2])
	if receivedCRC != calcCRC {
		return 0, 0, nil, fmt.Errorf("CRC error: expected %04X, got %04X", calcCRC, receivedCRC)
	}

	// Instruction (Should be 0x55 for Status)
	inst := packet[7]
	_ = inst // Instruction byte available for future use if needed

	errCode = packet[8]

	// Params: start at index 9, end before CRC (len-2)
	// For minimum packet (11 bytes), 9 to 9 = empty params
	if len(packet) > 11 {
		rawParams := packet[9 : len(packet)-2]
		params = DestuffParams(rawParams)
	} else {
		params = nil
	}

	return id, errCode, params, nil
}
