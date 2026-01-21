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

// Byte Stuffing: Insert 0xFD if header pattern [FF FF FD] appears in data
func StuffParams(params []byte) []byte {
	var stuffed []byte

	for i := 0; i < len(params); i++ {
		stuffed = append(stuffed, params[i])
		if len(stuffed) >= 3 {
			l := len(stuffed)
			// Check pattern FF FF FD
			if stuffed[l-3] == 0xFF && stuffed[l-2] == 0xFF && stuffed[l-1] == 0xFD {
				// Insert stuffed byte 0xFD (Protocol 2.0 says stuffing is adding 0xFD to prevent Header confusion)
				// Wait, correct rule: If data is 0xFF 0xFF 0xFD, it becomes 0xFF 0xFF 0xFD 0xFD.
				stuffed = append(stuffed, 0xFD) // Add extra FD
				// Actually the rule is: if the *stream* matches Header, insert stuff.
				// Simple implementation: Just track last 2 bytes.
			}
		}

		// Wait, a more robust way:
		// We are iterating input params.
		// If we see FF FF FD in the *output* stream (which we are building), we append FD.
	}

	// Re-do robustly
	// Actually for simplicity, let's trust the input for now or do a proper pass.
	// Protocol 2.0 Stuffing:
	// "Byte stuffing is required when the packet data has the same value as the packet header."
	// "If the data value is 0xFD 0xFF 0xFF, it is converted to 0xFD 0xFF 0xFF 0xFD." -> NO
	// Header is FF FF FD 00.
	// If body has FF FF FD, it transmits as FF FF FD FD.

	// Real impl:
	stuffed = make([]byte, 0, len(params)+2)
	ffCount := 0
	for _, b := range params {
		stuffed = append(stuffed, b)
		if b == 0xFF {
			ffCount++
		} else {
			if ffCount >= 2 && b == 0xFD {
				stuffed = append(stuffed, 0xFD) // Stuffing
			}
			ffCount = 0
		}
	}
	return stuffed
}

// DestuffParams removes byte stuffing from received data
// Protocol 2.0: FF FF FD FD -> FF FF FD
func DestuffParams(data []byte) []byte {
	if len(data) < 4 {
		return data
	}

	result := make([]byte, 0, len(data))
	ffCount := 0

	for i := 0; i < len(data); i++ {
		b := data[i]

		if ffCount >= 2 && b == 0xFD {
			// Check if next byte is also 0xFD (stuffed)
			if i+1 < len(data) && data[i+1] == 0xFD {
				// This is a stuffed pattern, output one FD and skip the next
				result = append(result, b)
				i++ // Skip the extra FD
				ffCount = 0
				continue
			}
		}

		result = append(result, b)

		if b == 0xFF {
			ffCount++
		} else {
			ffCount = 0
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
