package serial

import (
	"fmt"
	"syscall"
	"unsafe"
)

const (
	GENERIC_READ  = 0x80000000
	GENERIC_WRITE = 0x40000000
	OPEN_EXISTING = 3

	FILE_ATTRIBUTE_NORMAL = 0x80

	NOPARITY   = 0
	ONESTOPBIT = 0

	PURGE_TXABORT = 0x0001
	PURGE_RXABORT = 0x0002
	PURGE_TXCLEAR = 0x0004
	PURGE_RXCLEAR = 0x0008
)

// SerialPort represents a Windows COM port.
type SerialPort struct {
	handle syscall.Handle
}

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

type commTimeouts struct {
	ReadIntervalTimeout         uint32
	ReadTotalTimeoutMultiplier  uint32
	ReadTotalTimeoutConstant    uint32
	WriteTotalTimeoutMultiplier uint32
	WriteTotalTimeoutConstant   uint32
}

// Open opens a serial port on Windows with the specified baud rate.
func Open(portName string, baudRate int) (*SerialPort, error) {
	path, err := syscall.UTF16PtrFromString("\\\\.\\" + portName)
	if err != nil {
		return nil, err
	}

	handle, err := syscall.CreateFile(
		path,
		GENERIC_READ|GENERIC_WRITE,
		0,
		nil,
		OPEN_EXISTING,
		0,
		0,
	)
	if err != nil {
		return nil, fmt.Errorf("CreateFile failed: %v", err)
	}

	sp := &SerialPort{handle: handle}

	if err := sp.setParams(baudRate); err != nil {
		sp.Close()
		return nil, err
	}

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
	err := syscall.ReadFile(sp.handle, b, &n, nil)
	return int(n), err
}

func (sp *SerialPort) Write(b []byte) (int, error) {
	var n uint32
	err := syscall.WriteFile(sp.handle, b, &n, nil)
	return int(n), err
}

// Flush clears both input and output buffers.
func (sp *SerialPort) Flush() error {
	r1, _, e1 := procPurgeComm.Call(
		uintptr(sp.handle),
		uintptr(PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR),
	)
	if r1 == 0 {
		return fmt.Errorf("PurgeComm failed: %v", e1)
	}
	return nil
}

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

	r1, _, e1 := procGetCommState.Call(uintptr(sp.handle), uintptr(unsafe.Pointer(&dcbState)))
	if r1 == 0 {
		return fmt.Errorf("GetCommState failed: %v", e1)
	}

	dcbState.BaudRate = uint32(baud)
	dcbState.ByteSize = 8
	dcbState.Parity = NOPARITY
	dcbState.StopBits = ONESTOPBIT
	dcbState.Flags = 1

	r1, _, e1 = procSetCommState.Call(uintptr(sp.handle), uintptr(unsafe.Pointer(&dcbState)))
	if r1 == 0 {
		return fmt.Errorf("SetCommState failed: %v", e1)
	}

	r1, _, e1 = procSetupComm.Call(uintptr(sp.handle), 4096, 4096)
	if r1 == 0 {
		return fmt.Errorf("SetupComm failed: %v", e1)
	}

	r1, _, e1 = procPurgeComm.Call(
		uintptr(sp.handle),
		uintptr(PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR),
	)
	if r1 == 0 {
		return fmt.Errorf("PurgeComm failed: %v", e1)
	}

	return nil
}

func (sp *SerialPort) setTimeouts() error {
	var timeouts commTimeouts
	timeouts.ReadIntervalTimeout = 0
	timeouts.ReadTotalTimeoutMultiplier = 0
	timeouts.ReadTotalTimeoutConstant = 10
	timeouts.WriteTotalTimeoutMultiplier = 0
	timeouts.WriteTotalTimeoutConstant = 10

	r1, _, e1 := procSetCommTimeouts.Call(uintptr(sp.handle), uintptr(unsafe.Pointer(&timeouts)))
	if r1 == 0 {
		return fmt.Errorf("SetCommTimeouts failed: %v", e1)
	}
	return nil
}
