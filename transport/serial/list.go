package serial

import (
	"sort"
	"strings"
)

// PortInfo describes a serial device found on this machine.
type PortInfo struct {
	// Path is what Open expects: "COM7" on Windows, "/dev/ttyACM0" on Linux,
	// "/dev/cu.usbmodem…" on macOS.
	Path string

	// Description is whatever the OS reveals about the device — a driver
	// name, a by-id symlink, or empty when nothing is available.
	Description string

	// IsCANAdapter is a heuristic: true when the name looks like one of the
	// USB-CAN adapters this framework knows how to drive.  It only ranks the
	// list for the operator; it is never used to decide what to open.
	IsCANAdapter bool
}

// List enumerates the serial ports on this machine, CAN adapters first.
// A platform that cannot enumerate returns an empty list and no error, so
// callers can always fall back to an explicitly configured path.
func List() ([]PortInfo, error) {
	ports, err := listPorts()
	if err != nil {
		return nil, err
	}
	sort.SliceStable(ports, func(i, j int) bool {
		if ports[i].IsCANAdapter != ports[j].IsCANAdapter {
			return ports[i].IsCANAdapter
		}
		return ports[i].Path < ports[j].Path
	})
	return ports, nil
}

// looksLikeCANAdapter recognises the USB-CAN adapters this framework can
// drive, so the operator sees them at the top of the list.  It matches on
// whatever text the platform exposes; on Windows that is only "COM7", so the
// hint never fires there and the list stays in port order.
func looksLikeCANAdapter(text string) bool {
	lower := strings.ToLower(text)
	for _, hint := range []string{"fdcanusb", "mjbots", "canable", "slcan", "gs_usb"} {
		if strings.Contains(lower, hint) {
			return true
		}
	}
	return false
}
