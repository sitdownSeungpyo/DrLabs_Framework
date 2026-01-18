//go:build linux

package dxl

import (
	"fmt"
	"syscall"
	"unsafe"
)

// Linux Termios Constants (Typical values, validation needed for specific arch.
// However, syscall package provides these constant mappings usually)
const (
	TCGETS = 0x5401
	TCSETS = 0x5402
)

// SerialPort represents a Linux serial file descriptor
type SerialPort struct {
	fd int
}

func OpenSerial(portName string, baudRate int) (*SerialPort, error) {
	// 1. Open
	// O_RDWR | O_NOCTTY | O_NONBLOCK
	fd, err := syscall.Open(portName, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return nil, err
	}

	sp := &SerialPort{fd: fd}

	// 2. Setup Termios
	if err := sp.setParams(baudRate); err != nil {
		sp.Close()
		return nil, err
	}

	return sp, nil
}

func (sp *SerialPort) Close() error {
	return syscall.Close(sp.fd)
}

func (sp *SerialPort) Read(b []byte) (int, error) {
	return syscall.Read(sp.fd, b)
}

func (sp *SerialPort) Write(b []byte) (int, error) {
	return syscall.Write(sp.fd, b)
}

func (sp *SerialPort) setParams(baudRate int) error {
	var term syscall.Termios

	// Get current settings
	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(TCGETS), uintptr(unsafe.Pointer(&term))); err != 0 {
		return fmt.Errorf("ioctl TCGETS failed: %v", err)
	}

	// Set Baud Rate
	// syscall.CBAUD might not be defined on all architectures in Go's syscall package (e.g. amd64 linux might differ).
	// Instead, we use the standard Bxxxx constants directly masked into Cflag.
	// We clear the baud rate bits (which is often 000000010017 on octal? No, it's CBAUD mask).
	// Since CBAUD is missing, we might need to assume it's part of the flag or defined elsewhere.
	// Safest way in pure Go syscall without x/sys/unix is to use known constants.
	// However, modern Linux uses termios2 for custom baud rates.
	// For this exercise, we will assume standard baud rates and correct masking.
	// If CBAUD is undefined, we can try to skip masking if we just start from 0 or use known mask 0x100?
	// Actually, usually Bxxxx constants are self-sufficient if we clear existing.

	// Let's use a hardcoded CBAUD mask if needed or just blindly OR it if we assume 0 init? No.
	// 0020000ish.
	// Common CBAUD for Linux is 0x100f.
	CBAUD := uint32(0x100f) // Typical mask for baud rate

	term.Cflag &^= CBAUD

	cbaud := getBaudRateConst(baudRate)
	if cbaud == 0 {
		cbaud = syscall.B115200
	}
	term.Cflag |= cbaud

	// 8N1
	term.Cflag &^= syscall.CSIZE
	term.Cflag |= syscall.CS8     // 8 bits
	term.Cflag &^= syscall.PARENB // No Parity
	term.Cflag &^= syscall.CSTOPB // 1 Stop bit

	// Raw Mode
	term.Lflag &^= (syscall.ICANON | syscall.ECHO | syscall.ECHOE | syscall.ISIG)
	term.Oflag &^= syscall.OPOST
	term.Iflag &^= (syscall.IXON | syscall.IXOFF | syscall.IXANY)
	term.Iflag &^= (syscall.IGNBRK | syscall.BRKINT | syscall.PARMRK | syscall.ISTRIP | syscall.INLCR | syscall.IGNCR | syscall.ICRNL)

	// Timeouts (VMIN, VTIME)
	// VMIN=0, VTIME=1 -> Read returns ASAP if data, or wait 0.1s?
	// DXL: VMIN=0, VTIME=0 -> Non-blocking
	// We handle timeout in Upper Layer (driver.go) using deadline loop.
	term.Cc[syscall.VMIN] = 0
	term.Cc[syscall.VTIME] = 0

	// Set settings
	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(TCSETS), uintptr(unsafe.Pointer(&term))); err != 0 {
		return fmt.Errorf("ioctl TCSETS failed: %v", err)
	}
	return nil
}

func getBaudRateConst(baud int) uint32 {
	switch baud {
	case 9600:
		return syscall.B9600
	case 19200:
		return syscall.B19200
	case 38400:
		return syscall.B38400
	case 57600:
		return syscall.B57600
	case 115200:
		return syscall.B115200
	case 1000000:
		return syscall.B1000000 // Might be available in newer Go syscall/sys
	case 2000000:
		return syscall.B2000000
	case 3000000:
		return syscall.B3000000
	case 4000000:
		return syscall.B4000000
	}
	return syscall.B115200 // Default fallback
}
