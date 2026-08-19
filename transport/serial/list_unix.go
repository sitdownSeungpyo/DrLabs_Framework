//go:build linux || darwin

package serial

import (
	"os"
	"path/filepath"
)

// listPorts enumerates character devices that can plausibly be a serial port.
//
// /dev/serial/by-id (Linux, udev) is preferred when present because its names
// carry the manufacturer and serial number, which is the only way to tell two
// identical adapters apart across reboots.  The plain device nodes are always
// included as well, since the by-id tree does not exist on every system.
func listPorts() ([]PortInfo, error) {
	seen := map[string]bool{}
	var ports []PortInfo

	add := func(path, description string) {
		resolved := path
		if target, err := filepath.EvalSymlinks(path); err == nil {
			resolved = target
		}
		if seen[resolved] {
			return
		}
		seen[resolved] = true
		ports = append(ports, PortInfo{
			Path:         path,
			Description:  description,
			IsCANAdapter: looksLikeCANAdapter(path + " " + description),
		})
	}

	if entries, err := os.ReadDir("/dev/serial/by-id"); err == nil {
		for _, entry := range entries {
			add(filepath.Join("/dev/serial/by-id", entry.Name()), entry.Name())
		}
	}

	for _, pattern := range devicePatterns {
		matches, err := filepath.Glob(pattern)
		if err != nil {
			continue
		}
		for _, match := range matches {
			// macOS exposes each device twice: /dev/tty.* blocks until DCD is
			// asserted, /dev/cu.* does not.  devicePatterns only lists the
			// callout side for that reason.
			add(match, "")
		}
	}
	return ports, nil
}
