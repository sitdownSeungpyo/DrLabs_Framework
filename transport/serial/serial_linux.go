//go:build linux

package serial

import (
	"fmt"
	"syscall"
	"unsafe"
)

const (
	TCGETS    = 0x5401
	TCSETS    = 0x5402
	TCFLSH    = 0x540B
	TCIOFLUSH = 2
)

// SerialPort represents a Linux serial file descriptor.
type SerialPort struct {
	fd int
}

// Open opens a serial port on Linux with the specified baud rate.
func Open(portName string, baudRate int) (*SerialPort, error) {
	fd, err := syscall.Open(portName, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return nil, err
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
	if err == syscall.EAGAIN || err == syscall.EWOULDBLOCK {
		return 0, nil
	}
	if n < 0 {
		n = 0
	}
	return n, err
}

func (sp *SerialPort) Write(b []byte) (int, error) {
	return syscall.Write(sp.fd, b)
}

// Flush clears both input and output buffers.
func (sp *SerialPort) Flush() error {
	_, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd),
		uintptr(TCFLSH), uintptr(TCIOFLUSH))
	if err != 0 {
		return fmt.Errorf("TCFLSH failed: %v", err)
	}
	return nil
}

func (sp *SerialPort) setParams(baudRate int) error {
	var term syscall.Termios

	if _, _, err := syscall.Syscall(syscall.SYS_IOCTL, uintptr(sp.fd), uintptr(TCGETS), uintptr(unsafe.Pointer(&term))); err != 0 {
		return fmt.Errorf("ioctl TCGETS failed: %v", err)
	}

	CBAUD := uint32(0x100f)
	term.Cflag &^= CBAUD
	cbaud := getBaudRateConst(baudRate)
	if cbaud == 0 {
		cbaud = syscall.B115200
	}
	term.Cflag |= cbaud

	// 8N1
	term.Cflag &^= syscall.CSIZE
	term.Cflag |= syscall.CS8
	term.Cflag &^= syscall.PARENB
	term.Cflag &^= syscall.CSTOPB

	// Raw mode
	term.Lflag &^= (syscall.ICANON | syscall.ECHO | syscall.ECHOE | syscall.ISIG)
	term.Oflag &^= syscall.OPOST
	term.Iflag &^= (syscall.IXON | syscall.IXOFF | syscall.IXANY)
	term.Iflag &^= (syscall.IGNBRK | syscall.BRKINT | syscall.PARMRK | syscall.ISTRIP | syscall.INLCR | syscall.IGNCR | syscall.ICRNL)

	term.Cc[syscall.VMIN] = 0
	term.Cc[syscall.VTIME] = 0

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
		return syscall.B1000000
	case 2000000:
		return syscall.B2000000
	case 3000000:
		return syscall.B3000000
	case 4000000:
		return syscall.B4000000
	}
	return syscall.B115200
}
