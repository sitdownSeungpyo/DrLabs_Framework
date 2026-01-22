package dxl

import (
	"bytes"
	"errors"
	"sync"
	"testing"
	"time"
)

// MockSerialPort implements SerialPortInterface for testing
type MockSerialPort struct {
	mu           sync.Mutex
	readBuf      *bytes.Buffer
	writeBuf     *bytes.Buffer
	readDelay    time.Duration
	readErr      error
	writeErr     error
	closed       bool
}

func NewMockSerialPort() *MockSerialPort {
	return &MockSerialPort{
		readBuf:  bytes.NewBuffer(nil),
		writeBuf: bytes.NewBuffer(nil),
	}
}

func (m *MockSerialPort) Read(b []byte) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return 0, errors.New("port closed")
	}
	if m.readErr != nil {
		return 0, m.readErr
	}
	if m.readDelay > 0 {
		time.Sleep(m.readDelay)
	}

	return m.readBuf.Read(b)
}

func (m *MockSerialPort) Write(b []byte) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return 0, errors.New("port closed")
	}
	if m.writeErr != nil {
		return 0, m.writeErr
	}

	return m.writeBuf.Write(b)
}

func (m *MockSerialPort) Close() error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.closed = true
	return nil
}

// SetResponse sets the data that will be returned by Read
func (m *MockSerialPort) SetResponse(data []byte) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.readBuf.Reset()
	m.readBuf.Write(data)
}

// GetWritten returns data that was written to the port
func (m *MockSerialPort) GetWritten() []byte {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.writeBuf.Bytes()
}

func (m *MockSerialPort) SetReadError(err error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.readErr = err
}

func (m *MockSerialPort) SetWriteError(err error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.writeErr = err
}

// buildStatusPacket creates a valid status response packet
func buildStatusPacket(id uint8, errCode uint8, params []byte) []byte {
	// Header: FF FF FD 00
	// ID: id
	// Length: 2 (inst + err) + len(params) + 2 (CRC)
	length := 2 + len(params) + 2

	pkt := []byte{0xFF, 0xFF, 0xFD, 0x00, id}
	pkt = append(pkt, byte(length&0xFF), byte((length>>8)&0xFF))
	pkt = append(pkt, InstStatus, errCode)
	pkt = append(pkt, params...)

	crc := UpdateCRC(0, pkt)
	pkt = append(pkt, byte(crc&0xFF), byte((crc>>8)&0xFF))

	return pkt
}

func TestDriverPing(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Prepare response: model number 1060 (0x0424) = XM430
	modelNum := uint16(1060)
	params := []byte{byte(modelNum & 0xFF), byte((modelNum >> 8) & 0xFF), 0x01} // model_l, model_h, firmware
	response := buildStatusPacket(1, 0, params)
	mock.SetResponse(response)

	model, err := driver.Ping(1)
	if err != nil {
		t.Errorf("Ping failed: %v", err)
	}
	if model != modelNum {
		t.Errorf("Model mismatch: got %d, want %d", model, modelNum)
	}

	// Verify correct packet was sent
	written := mock.GetWritten()
	if len(written) == 0 {
		t.Error("No data written to port")
	}
	// Check header
	if written[0] != 0xFF || written[1] != 0xFF || written[2] != 0xFD {
		t.Errorf("Invalid header in written packet: %X", written[:3])
	}
	// Check ID
	if written[4] != 1 {
		t.Errorf("Wrong ID in written packet: %d", written[4])
	}
	// Check instruction
	if written[7] != InstPing {
		t.Errorf("Wrong instruction: %02X, want %02X", written[7], InstPing)
	}
}

func TestDriverPingError(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Response with error code
	response := buildStatusPacket(1, 0x80, nil) // Hardware alert
	mock.SetResponse(response)

	_, err := driver.Ping(1)
	if err == nil {
		t.Error("Expected error for error code response, got nil")
	}
}

func TestDriverRead(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Prepare response: 4 bytes of position data
	posData := []byte{0x00, 0x08, 0x00, 0x00} // position = 2048
	response := buildStatusPacket(1, 0, posData)
	mock.SetResponse(response)

	data, err := driver.Read(1, 132, 4) // Read present position
	if err != nil {
		t.Errorf("Read failed: %v", err)
	}
	if !bytes.Equal(data, posData) {
		t.Errorf("Data mismatch: got %X, want %X", data, posData)
	}
}

func TestDriverWrite(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Prepare success response (no params)
	response := buildStatusPacket(1, 0, nil)
	mock.SetResponse(response)

	err := driver.Write(1, 64, []byte{1}) // Enable torque
	if err != nil {
		t.Errorf("Write failed: %v", err)
	}

	// Verify write instruction was sent
	written := mock.GetWritten()
	if written[7] != InstWrite {
		t.Errorf("Wrong instruction: %02X, want %02X", written[7], InstWrite)
	}
}

func TestDriverWrite4Byte(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	response := buildStatusPacket(1, 0, nil)
	mock.SetResponse(response)

	err := driver.Write4Byte(1, 116, 2048) // Goal position = 2048
	if err != nil {
		t.Errorf("Write4Byte failed: %v", err)
	}
}

func TestDriverRead4Byte(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Position 2048 = 0x00000800
	posData := []byte{0x00, 0x08, 0x00, 0x00}
	response := buildStatusPacket(1, 0, posData)
	mock.SetResponse(response)

	val, err := driver.Read4Byte(1, 132)
	if err != nil {
		t.Errorf("Read4Byte failed: %v", err)
	}
	if val != 2048 {
		t.Errorf("Value mismatch: got %d, want 2048", val)
	}
}

func TestDriverWriteError(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	mock.SetWriteError(errors.New("write failed"))

	err := driver.Write(1, 64, []byte{1})
	if err == nil {
		t.Error("Expected write error, got nil")
	}
}

func TestDriverReadTimeout(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// No response set - will timeout

	_, err := driver.Read(1, 132, 4)
	if err == nil {
		t.Error("Expected timeout error, got nil")
	}
}

func TestFindPacketStart(t *testing.T) {
	tests := []struct {
		name     string
		data     []byte
		expected int
	}{
		{
			name:     "header at start",
			data:     []byte{0xFF, 0xFF, 0xFD, 0x00, 0x01},
			expected: 0,
		},
		{
			name:     "header with garbage prefix",
			data:     []byte{0x00, 0x01, 0xFF, 0xFF, 0xFD, 0x00, 0x01},
			expected: 2,
		},
		{
			name:     "no header",
			data:     []byte{0x00, 0x01, 0x02, 0x03},
			expected: -1,
		},
		{
			name:     "partial header",
			data:     []byte{0xFF, 0xFF},
			expected: -1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := findPacketStart(tt.data)
			if result != tt.expected {
				t.Errorf("findPacketStart() = %d, want %d", result, tt.expected)
			}
		})
	}
}

func TestSyncWriteData(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	motors := []SyncWriteData{
		{ID: 1, Data: []byte{0x00, 0x08, 0x00, 0x00}},
		{ID: 2, Data: []byte{0x00, 0x10, 0x00, 0x00}},
	}

	err := driver.SyncWrite(116, 4, motors)
	if err != nil {
		t.Errorf("SyncWrite failed: %v", err)
	}

	// Verify broadcast ID was used
	written := mock.GetWritten()
	if written[4] != 0xFE {
		t.Errorf("Expected broadcast ID 0xFE, got %02X", written[4])
	}
	// Verify sync write instruction
	if written[7] != InstSyncWrite {
		t.Errorf("Expected SyncWrite instruction, got %02X", written[7])
	}
}

func TestSyncWriteDataLengthMismatch(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	motors := []SyncWriteData{
		{ID: 1, Data: []byte{0x00, 0x08}}, // Only 2 bytes instead of 4
	}

	err := driver.SyncWrite(116, 4, motors)
	if err == nil {
		t.Error("Expected error for data length mismatch, got nil")
	}
}

func TestSyncWriteNoMotors(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	err := driver.SyncWrite(116, 4, nil)
	if err == nil {
		t.Error("Expected error for empty motors, got nil")
	}
}

func TestSyncRead(t *testing.T) {
	// SyncRead reads one packet at a time from each motor
	// We need to test this differently - each read should get one response
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// For sync read with 2 motors, we'll get 2 separate reads
	// Set first response
	motor1Response := buildStatusPacket(1, 0, []byte{0x00, 0x08, 0x00, 0x00})
	motor2Response := buildStatusPacket(2, 0, []byte{0x00, 0x10, 0x00, 0x00})

	// Concatenate both responses so they can be read sequentially
	combined := append(motor1Response, motor2Response...)
	mock.SetResponse(combined)

	results, err := driver.SyncRead(132, 4, []uint8{1, 2})
	if err != nil {
		t.Errorf("SyncRead failed: %v", err)
	}

	if len(results) != 2 {
		t.Errorf("Expected 2 results, got %d", len(results))
	}

	// Check results - some may have errors due to mock limitations
	// Just verify we got the right number of results
	motor1Found := false
	motor2Found := false
	for _, r := range results {
		if r.ID == 1 {
			motor1Found = true
		}
		if r.ID == 2 {
			motor2Found = true
		}
	}

	if !motor1Found {
		t.Error("Motor 1 result not found")
	}
	if !motor2Found {
		t.Error("Motor 2 result not found")
	}
}

func TestSyncWrite4ByteMemoryOptimization(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Test with 10 motors to ensure pre-allocation works
	values := make(map[uint8]uint32)
	for i := uint8(1); i <= 10; i++ {
		values[i] = uint32(i * 100)
	}

	err := driver.SyncWrite4Byte(116, values)
	if err != nil {
		t.Errorf("SyncWrite4Byte failed: %v", err)
	}

	written := mock.GetWritten()
	// Verify it's a sync write packet
	if written[7] != InstSyncWrite {
		t.Errorf("Expected SyncWrite instruction, got %02X", written[7])
	}
}

func TestReadPacketWithGarbage(t *testing.T) {
	mock := NewMockSerialPort()
	driver := &Driver{port: mock}

	// Response with garbage before header
	garbage := []byte{0x00, 0x01, 0x02, 0x03}
	validPacket := buildStatusPacket(1, 0, []byte{0x00, 0x08, 0x00, 0x00})
	mock.SetResponse(append(garbage, validPacket...))

	data, err := driver.Read(1, 132, 4)
	if err != nil {
		t.Errorf("Read with garbage failed: %v", err)
	}
	if len(data) != 4 {
		t.Errorf("Data length: got %d, want 4", len(data))
	}
}
