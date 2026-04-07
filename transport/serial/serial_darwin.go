//go:build darwin
// +build darwin

package serial

import (
	"fmt"
	"syscall"
	"unsafe"
)

// SerialPort represents a macOS serial file descriptor.
type SerialPort struct {
	fd int
}

// Open opens a serial port on macOS with the specified baud rate.
func Open(portName string, baudRate int) (*SerialPort, error) {
	// Open with O_NONBLOCK to prevent hanging on open (DCD not asserted).
	// Note: prefer /dev/cu.* over /dev/tty.* for CDC ACM devices on macOS.
	fd, err := syscall.Open(portName, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return nil, fmt.Errorf("open %s failed: %v (tip: try /dev/cu.* instead of /dev/tty.*)", portName, err)
	}

	// Clear O_NONBLOCK after opening — use blocking mode with VMIN/VTIME control.
	flags, _, errno := syscall.Syscall(syscall.SYS_FCNTL, uintptr(fd), syscall.F_GETFL, 0)
	if errno != 0 {
		syscall.Close(fd)
		return nil, fmt.Errorf("fcntl F_GETFL failed: %v", errno)
	}
	_, _, errno = syscall.Syscall(syscall.SYS_FCNTL, uintptr(fd), syscall.F_SETFL, flags&^uintptr(syscall.O_NONBLOCK))
	if errno != 0 {
		syscall.Close(fd)
		return nil, fmt.Errorf("fcntl F_SETFL failed: %v", errno)
	}

	sp := &SerialPort{fd: fd}

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
	n, err := syscall.Read(sp.fd, b)
	if n < 0 {
		n = 0
	}
	return n, err
}

func (sp *SerialPort) Write(b []byte) (int, error) {
	return syscall.Write(sp.fd, b)
}

// TIOCFLUSH ioctl constant for macOS
const TIOCFLUSH = 0x80047410

// Flush clears the input buffer using TIOCFLUSH ioctl (input only).
func (sp *SerialPort) Flush() error {
	what := int32(1) // FREAD only — flush input buffer
	_, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd),
		uintptr(TIOCFLUSH), uintptr(unsafe.Pointer(&what)))
	if err != 0 {
		return fmt.Errorf("TIOCFLUSH failed: %v", err)
	}
	return nil
}

// IOSSIOSPEED is the Darwin ioctl for custom baud rates
const IOSSIOSPEED = 0x80045402

func (sp *SerialPort) setParams(baudRate int) error {
	var term syscall.Termios

	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(syscall.TIOCGETA), uintptr(unsafe.Pointer(&term))); err != 0 {
		return fmt.Errorf("ioctl TIOCGETA failed: %v", err)
	}

	// 8N1
	term.Cflag &^= syscall.CSIZE
	term.Cflag |= syscall.CS8
	term.Cflag &^= syscall.PARENB
	term.Cflag &^= syscall.CSTOPB

	// Enable receiver, local line
	term.Cflag |= syscall.CREAD | syscall.CLOCAL

	// Raw mode
	term.Lflag = 0
	term.Oflag = 0
	term.Iflag = syscall.IGNPAR

	// VMIN=0, VTIME=0: polling read
	term.Cc[syscall.VMIN] = 0
	term.Cc[syscall.VTIME] = 0

	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(syscall.TIOCSETA), uintptr(unsafe.Pointer(&term))); err != 0 {
		return fmt.Errorf("ioctl TIOCSETA failed: %v", err)
	}

	// Set custom baud rate using IOSSIOSPEED
	speed := uint32(baudRate)
	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(IOSSIOSPEED), uintptr(unsafe.Pointer(&speed))); err != 0 {
		return fmt.Errorf("ioctl IOSSIOSPEED failed (baudrate %d): %v", baudRate, err)
	}

	sp.Flush()
	return nil
}
