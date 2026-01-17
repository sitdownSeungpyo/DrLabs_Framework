package dxl

import (
	"fmt"
	"syscall"
	"unsafe"
)

// Windows Constants
const (
	GENERIC_READ  = 0x80000000
	GENERIC_WRITE = 0x40000000
	OPEN_EXISTING = 3

	FILE_ATTRIBUTE_NORMAL = 0x80
	FILE_FLAG_OVERLAPPED  = 0x40000000

	NOPARITY           = 0
	ONESTOPBIT         = 0
	TWOSTOPBITS        = 2
	RTS_CONTROL_ENABLE = 0x01
	DTR_CONTROL_ENABLE = 0x01

	PURGE_TXABORT = 0x0001
	PURGE_RXABORT = 0x0002
	PURGE_TXCLEAR = 0x0004
	PURGE_RXCLEAR = 0x0008
)

// SerialPort represents a Windows COM port
type SerialPort struct {
	handle syscall.Handle
}

// DCB struct for SetCommState
type dcb struct {
	DCBlength  uint32
	BaudRate   uint32
	Flags      uint32
	wReserved  uint16
	XonLim     uint16
	XoffLim    uint16
	ByteSize   byte
	Parity     byte
	StopBits   byte
	XonChar    byte
	XoffChar   byte
	ErrorChar  byte
	EofChar    byte
	EvtChar    byte
	wReserved1 uint16
}

// COMMTIMEOUTS struct
type commTimeouts struct {
	ReadIntervalTimeout         uint32
	ReadTotalTimeoutMultiplier  uint32
	ReadTotalTimeoutConstant    uint32
	WriteTotalTimeoutMultiplier uint32
	WriteTotalTimeoutConstant   uint32
}

func OpenSerial(portName string, baudRate int) (*SerialPort, error) {
	// 1. CreateFile
	path, err := syscall.UTF16PtrFromString("\\\\.\\" + portName)
	if err != nil {
		return nil, err
	}

	handle, err := syscall.CreateFile(
		path,
		GENERIC_READ|GENERIC_WRITE,
		0,   // Exclusive access
		nil, // Security
		OPEN_EXISTING,
		0, // No Overlapped for simplicity (Blocking)
		0,
	)

	if err != nil {
		return nil, fmt.Errorf("CreateFile failed: %v", err)
	}

	sp := &SerialPort{handle: handle}

	// 2. Setup DCB
	var dcbState dcb
	dcbState.DCBlength = uint32(unsafe.Sizeof(dcbState))

	// Get current state
	// We need to implement GetCommState/SetCommState wrapper or use syscall.Syscall
	// Go's syscall package has these but they might be tricky.
	// Actually `syscall.GetCommState` exists in `golang.org/x/sys/windows` but not standard `syscall`.
	// We must load them manually from kernel32.dll for pure dependency-free Go.

	if err := sp.setParams(baudRate); err != nil {
		sp.Close()
		return nil, err
	}

	// 3. Setup Timeouts
	if err := sp.setTimeouts(); err != nil {
		sp.Close()
		return nil, err
	}

	return sp, nil
}

func (sp *SerialPort) Close() error {
	return syscall.CloseHandle(sp.handle)
}

func (sp *SerialPort) Read(b []byte) (int, error) {
	var n uint32
	// syscall.ReadFile(handle, buf, &n, overlapped)
	err := syscall.ReadFile(sp.handle, b, &n, nil)
	return int(n), err
}

func (sp *SerialPort) Write(b []byte) (int, error) {
	var n uint32
	err := syscall.WriteFile(sp.handle, b, &n, nil)
	return int(n), err
}

// Internal DLL loading
var (
	modkernel32         = syscall.NewLazyDLL("kernel32.dll")
	procGetCommState    = modkernel32.NewProc("GetCommState")
	procSetCommState    = modkernel32.NewProc("SetCommState")
	procSetCommTimeouts = modkernel32.NewProc("SetCommTimeouts")
	procSetupComm       = modkernel32.NewProc("SetupComm")
	procPurgeComm       = modkernel32.NewProc("PurgeComm")
)

func (sp *SerialPort) setParams(baud int) error {
	var dcbState dcb
	dcbState.DCBlength = uint32(unsafe.Sizeof(dcbState))

	// GetCommState
	r1, _, e1 := procGetCommState.Call(
		uintptr(sp.handle),
		uintptr(unsafe.Pointer(&dcbState)),
	)
	if r1 == 0 {
		return fmt.Errorf("GetCommState failed: %v", e1)
	}

	// Update params
	dcbState.BaudRate = uint32(baud)
	dcbState.ByteSize = 8
	dcbState.Parity = NOPARITY
	dcbState.StopBits = ONESTOPBIT
	// Flags: Binary (1), DTR/RTS typically needed? DXL usually just 8N1.
	dcbState.Flags = 1 // fBinary

	// SetCommState
	r1, _, e1 = procSetCommState.Call(
		uintptr(sp.handle),
		uintptr(unsafe.Pointer(&dcbState)),
	)
	if r1 == 0 {
		return fmt.Errorf("SetCommState failed: %v", e1)
	}

	// SetupComm (Buffers)
	procSetupComm.Call(uintptr(sp.handle), 4096, 4096)

	// Purge
	procPurgeComm.Call(uintptr(sp.handle), uintptr(PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR))

	return nil
}

func (sp *SerialPort) setTimeouts() error {
	var timeouts commTimeouts

	// Non-blocking read (return immediately with what's available)
	// OR short timeout. DXL packets are fast.
	// Constant=1ms, Multiplier=0 -> wait max 1ms per Read call if buffer empty?
	// To replicate 'PacketHandler' logic which often has a timeout logic:

	// Behavior: ReadFile returns immediately if data exists. If not, wait up to Constant.
	// We want fast reads.

	timeouts.ReadIntervalTimeout = 0 // MAXDWORD for return immediately? No, 0 is ignored?
	// MAXDWORD interval, 0 others = return immediately even if 0 bytes
	timeouts.ReadIntervalTimeout = 0xFFFFFFFF
	timeouts.ReadTotalTimeoutMultiplier = 0
	timeouts.ReadTotalTimeoutConstant = 0

	// If we want a blocking read with timeout (e.g. wait 5ms for packet):
	// Let's set a small timeout: 5ms
	timeouts.ReadIntervalTimeout = 0
	timeouts.ReadTotalTimeoutMultiplier = 0
	timeouts.ReadTotalTimeoutConstant = 5

	timeouts.WriteTotalTimeoutMultiplier = 0
	timeouts.WriteTotalTimeoutConstant = 5

	r1, _, e1 := procSetCommTimeouts.Call(
		uintptr(sp.handle),
		uintptr(unsafe.Pointer(&timeouts)),
	)
	if r1 == 0 {
		return fmt.Errorf("SetCommTimeouts failed: %v", e1)
	}
	return nil
}
