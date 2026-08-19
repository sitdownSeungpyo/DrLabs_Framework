//go:build windows

package serial

import (
	"strings"
	"syscall"
	"unsafe"
)

var procQueryDosDeviceW = modkernel32.NewProc("QueryDosDeviceW")

// listPorts enumerates COM ports through the DOS device namespace.
//
// QueryDosDeviceW with a nil device name returns every symbolic link in the
// namespace as a block of NUL-terminated UTF-16 strings.  The COM* entries in
// it are exactly the ports that can be opened, which is what we want — the
// registry's SERIALCOMM key can hold stale entries for devices that have been
// unplugged.
func listPorts() ([]PortInfo, error) {
	// The namespace is large and its size is not knowable in advance; grow
	// until the call stops reporting ERROR_INSUFFICIENT_BUFFER.
	size := 8192
	for attempts := 0; attempts < 6; attempts++ {
		buf := make([]uint16, size)
		n, _, errno := procQueryDosDeviceW.Call(
			0,
			uintptr(unsafe.Pointer(&buf[0])),
			uintptr(size),
		)
		if n == 0 {
			const errorInsufficientBuffer = syscall.Errno(122)
			if errno == errorInsufficientBuffer {
				size *= 4
				continue
			}
			// Nothing enumerable — not an error the caller can act on.
			return nil, nil
		}
		return parseDosDevices(buf[:n]), nil
	}
	return nil, nil
}

func parseDosDevices(buf []uint16) []PortInfo {
	var ports []PortInfo
	start := 0
	for i := 0; i < len(buf); i++ {
		if buf[i] != 0 {
			continue
		}
		if i > start {
			name := syscall.UTF16ToString(buf[start:i])
			if isCOMPort(name) {
				ports = append(ports, PortInfo{
					Path:         name,
					IsCANAdapter: looksLikeCANAdapter(name),
				})
			}
		}
		start = i + 1
	}
	return ports
}

// isCOMPort matches COM followed by digits only, so that entries such as
// "COMPOSITE" or "COM_something" do not get mistaken for ports.
func isCOMPort(name string) bool {
	if !strings.HasPrefix(name, "COM") || len(name) == 3 {
		return false
	}
	for _, r := range name[3:] {
		if r < '0' || r > '9' {
			return false
		}
	}
	return true
}
