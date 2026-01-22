package dxl

import (
	"bytes"
	"testing"
)

func TestUpdateCRC(t *testing.T) {
	tests := []struct {
		name     string
		data     []byte
		expected uint16
	}{
		{
			name:     "empty data",
			data:     []byte{},
			expected: 0,
		},
		{
			name:     "ping packet without CRC",
			data:     []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01},
			expected: 0x4E19, // Expected CRC for ping ID=1
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := UpdateCRC(0, tt.data)
			if result != tt.expected {
				t.Errorf("UpdateCRC() = %04X, want %04X", result, tt.expected)
			}
		})
	}
}

func TestStuffParams(t *testing.T) {
	tests := []struct {
		name     string
		input    []byte
		expected []byte
	}{
		{
			name:     "no stuffing needed",
			input:    []byte{0x01, 0x02, 0x03},
			expected: []byte{0x01, 0x02, 0x03},
		},
		{
			name:     "single FF",
			input:    []byte{0xFF, 0x01, 0x02},
			expected: []byte{0xFF, 0x01, 0x02},
		},
		{
			name:     "double FF without FD",
			input:    []byte{0xFF, 0xFF, 0x01},
			expected: []byte{0xFF, 0xFF, 0x01},
		},
		{
			name:     "header pattern needs stuffing",
			input:    []byte{0xFF, 0xFF, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD, 0xFD},
		},
		{
			name:     "header pattern in middle",
			input:    []byte{0x01, 0xFF, 0xFF, 0xFD, 0x02},
			expected: []byte{0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02},
		},
		{
			name:     "multiple header patterns",
			input:    []byte{0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD},
		},
		{
			name:     "empty input",
			input:    []byte{},
			expected: []byte{},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := StuffParams(tt.input)
			if !bytes.Equal(result, tt.expected) {
				t.Errorf("StuffParams(%X) = %X, want %X", tt.input, result, tt.expected)
			}
		})
	}
}

func TestDestuffParams(t *testing.T) {
	tests := []struct {
		name     string
		input    []byte
		expected []byte
	}{
		{
			name:     "no destuffing needed",
			input:    []byte{0x01, 0x02, 0x03},
			expected: []byte{0x01, 0x02, 0x03},
		},
		{
			name:     "stuffed pattern",
			input:    []byte{0xFF, 0xFF, 0xFD, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD},
		},
		{
			name:     "stuffed pattern in middle",
			input:    []byte{0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02},
			expected: []byte{0x01, 0xFF, 0xFF, 0xFD, 0x02},
		},
		{
			name:     "multiple stuffed patterns",
			input:    []byte{0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD},
		},
		{
			name:     "empty input",
			input:    []byte{},
			expected: []byte{},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := DestuffParams(tt.input)
			if !bytes.Equal(result, tt.expected) {
				t.Errorf("DestuffParams(%X) = %X, want %X", tt.input, result, tt.expected)
			}
		})
	}
}

func TestStuffDestuffRoundTrip(t *testing.T) {
	tests := []struct {
		name  string
		input []byte
	}{
		{"simple data", []byte{0x01, 0x02, 0x03}},
		{"with header pattern", []byte{0xFF, 0xFF, 0xFD}},
		{"complex pattern", []byte{0xFF, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFD}},
		{"all FFs", []byte{0xFF, 0xFF, 0xFF, 0xFF}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			stuffed := StuffParams(tt.input)
			result := DestuffParams(stuffed)
			if !bytes.Equal(result, tt.input) {
				t.Errorf("Round trip failed: input=%X, stuffed=%X, result=%X", tt.input, stuffed, result)
			}
		})
	}
}

func TestBuildPacket(t *testing.T) {
	tests := []struct {
		name           string
		id             uint8
		inst           uint8
		params         []byte
		expectedHeader []byte
		expectedLen    int // Expected packet length
	}{
		{
			name:           "ping packet",
			id:             1,
			inst:           InstPing,
			params:         nil,
			expectedHeader: []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01},
			expectedLen:    10, // H(4)+ID(1)+Len(2)+Inst(1)+CRC(2) = 10 (no params)
		},
		{
			name:           "read packet",
			id:             2,
			inst:           InstRead,
			params:         []byte{0x84, 0x00, 0x04, 0x00}, // addr=132, len=4
			expectedHeader: []byte{0xFF, 0xFF, 0xFD, 0x00, 0x02},
			expectedLen:    14, // H(4)+ID(1)+Len(2)+Inst(1)+Params(4)+CRC(2) = 14
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := BuildPacket(tt.id, tt.inst, tt.params)

			// Check header
			if !bytes.Equal(result[:5], tt.expectedHeader) {
				t.Errorf("Header mismatch: got %X, want %X", result[:5], tt.expectedHeader)
			}

			// Check expected length
			if len(result) != tt.expectedLen {
				t.Errorf("Packet length: got %d, want %d", len(result), tt.expectedLen)
			}

			// Verify CRC calculation
			// CRC is calculated over everything except the last 2 bytes
			crc := UpdateCRC(0, result[:len(result)-2])
			receivedCRC := uint16(result[len(result)-2]) | (uint16(result[len(result)-1]) << 8)
			if crc != receivedCRC {
				t.Errorf("CRC mismatch: calculated %04X, got %04X", crc, receivedCRC)
			}
		})
	}
}

func TestParsePacket(t *testing.T) {
	tests := []struct {
		name        string
		packet      []byte
		expectedID  uint8
		expectedErr bool
		errContains string
	}{
		{
			name:        "packet too short",
			packet:      []byte{0xFF, 0xFF, 0xFD},
			expectedErr: true,
			errContains: "too short",
		},
		{
			name:        "invalid header",
			packet:      []byte{0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0x00, 0x00},
			expectedErr: true,
			errContains: "invalid header",
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			id, _, _, err := ParsePacket(tt.packet)

			if tt.expectedErr {
				if err == nil {
					t.Errorf("Expected error containing '%s', got nil", tt.errContains)
				}
			} else {
				if err != nil {
					t.Errorf("Unexpected error: %v", err)
				}
				if id != tt.expectedID {
					t.Errorf("ID mismatch: got %d, want %d", id, tt.expectedID)
				}
			}
		})
	}
}

func TestParsePacketValidStatus(t *testing.T) {
	// Build a valid status packet manually
	// Header: FF FF FD 00, ID: 01, Length: 04 00, Inst: 55, Err: 00, CRC: XX XX
	packet := []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00}
	crc := UpdateCRC(0, packet)
	packet = append(packet, byte(crc&0xFF), byte((crc>>8)&0xFF))

	id, errCode, params, err := ParsePacket(packet)
	if err != nil {
		t.Errorf("Failed to parse valid packet: %v", err)
	}
	if id != 1 {
		t.Errorf("ID mismatch: got %d, want 1", id)
	}
	if errCode != 0 {
		t.Errorf("ErrCode mismatch: got %d, want 0", errCode)
	}
	if len(params) != 0 {
		t.Errorf("Params should be empty, got %X", params)
	}
}

func TestParsePacketCRCError(t *testing.T) {
	// Valid packet with wrong CRC
	packet := []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0x00, 0x00}

	_, _, _, err := ParsePacket(packet)
	if err == nil {
		t.Error("Expected CRC error, got nil")
	}
}

func TestBuildAndParseRoundTrip(t *testing.T) {
	// Build a packet
	tx := BuildPacket(5, InstPing, nil)

	// Simulate a status response
	// Status packet: Header + ID + Length + InstStatus + Error + Params + CRC
	response := []byte{0xFF, 0xFF, 0xFD, 0x00, 0x05, 0x07, 0x00, 0x55, 0x00, 0x50, 0x01, 0x00}
	crc := UpdateCRC(0, response)
	response = append(response, byte(crc&0xFF), byte((crc>>8)&0xFF))

	id, errCode, params, err := ParsePacket(response)
	if err != nil {
		t.Errorf("Failed to parse response: %v", err)
	}
	if id != 5 {
		t.Errorf("ID mismatch: got %d, want 5", id)
	}
	if errCode != 0 {
		t.Errorf("ErrCode mismatch: got %d, want 0", errCode)
	}
	if len(params) != 3 {
		t.Errorf("Params length mismatch: got %d, want 3", len(params))
	}

	// Verify tx packet is valid
	if len(tx) < 10 {
		t.Errorf("TX packet too short: %d", len(tx))
	}
}

func TestStuffParamsWithMultiplePatterns(t *testing.T) {
	// Test data with multiple header patterns
	input := []byte{0x01, 0xFF, 0xFF, 0xFD, 0x02, 0xFF, 0xFF, 0xFD, 0x03}
	result := StuffParams(input)

	// Should have 2 extra FD bytes inserted
	expected := []byte{0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02, 0xFF, 0xFF, 0xFD, 0xFD, 0x03}
	if !bytes.Equal(result, expected) {
		t.Errorf("StuffParams multi-pattern: got %X, want %X", result, expected)
	}
}

func TestDestuffParamsEdgeCases(t *testing.T) {
	tests := []struct {
		name     string
		input    []byte
		expected []byte
	}{
		{
			name:     "short data",
			input:    []byte{0xFF, 0xFF},
			expected: []byte{0xFF, 0xFF},
		},
		{
			name:     "partial stuffed pattern",
			input:    []byte{0xFF, 0xFF, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD},
		},
		{
			name:     "consecutive stuffed patterns",
			input:    []byte{0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0xFD},
			expected: []byte{0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := DestuffParams(tt.input)
			if !bytes.Equal(result, tt.expected) {
				t.Errorf("DestuffParams(%s) = %X, want %X", tt.name, result, tt.expected)
			}
		})
	}
}

func TestBuildPacketWithStuffing(t *testing.T) {
	// Params that contain header pattern
	params := []byte{0xFF, 0xFF, 0xFD, 0x42}
	packet := BuildPacket(1, InstWrite, params)

	// Packet should contain stuffed params
	// Parse it back to verify
	// This is an instruction packet, so we can't parse it with ParsePacket (expects status)
	// Just verify the packet is longer due to stuffing
	if len(packet) <= 10+len(params) {
		t.Error("Expected packet to be longer due to byte stuffing")
	}
}

func TestCRCConsistency(t *testing.T) {
	data := []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01}
	crc1 := UpdateCRC(0, data)
	crc2 := UpdateCRC(0, data)

	if crc1 != crc2 {
		t.Errorf("CRC not consistent: %04X vs %04X", crc1, crc2)
	}
}
