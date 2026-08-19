package motor

import (
	"fmt"
	"sort"
	"sync"
)

// Config describes one controller to connect to.  It is deliberately flat and
// stringly-typed so that it can come straight from CLI flags, a JSON file or a
// ROS parameter server without a driver-specific parser in between.
type Config struct {
	// Driver selects the registered factory, e.g. "moteus", "moteus-sim",
	// "dynamixel".
	Driver string

	// Port is the transport endpoint: a serial device (COM7, /dev/ttyACM0),
	// a SocketCAN interface (can0), or empty to auto-detect.
	Port string

	// Baud applies to serial transports only.
	Baud int

	// ID is the bus address — CAN id for moteus, servo id for Dynamixel.
	ID int

	// Options carries driver-specific settings that do not deserve a field of
	// their own.  Unknown keys are an error, so that a typo in a config file
	// is reported instead of silently ignored.
	Options map[string]string
}

// Option returns the value for key, or def when it is absent.
func (c Config) Option(key, def string) string {
	if c.Options == nil {
		return def
	}
	if v, ok := c.Options[key]; ok {
		return v
	}
	return def
}

// Factory builds a Link from a Config.  It must not perform I/O; the returned
// Link is not connected until Open is called.
type Factory func(Config) (Link, error)

// Driver is a registered driver and its human-facing description.
type Driver struct {
	Name        string
	Description string
	Factory     Factory
}

var (
	registryMu sync.RWMutex
	registry   = map[string]Driver{}
)

// Register makes a driver available to New.  It is meant to be called from a
// driver package's init, so that applications select drivers by blank import
// and the core never depends on any of them.  Registering the same name twice
// panics — that is a build-time mistake, not a runtime condition.
func Register(name, description string, factory Factory) {
	registryMu.Lock()
	defer registryMu.Unlock()
	if _, exists := registry[name]; exists {
		panic("motor: driver already registered: " + name)
	}
	registry[name] = Driver{Name: name, Description: description, Factory: factory}
}

// New builds a Link for cfg.Driver.
func New(cfg Config) (Link, error) {
	registryMu.RLock()
	driver, ok := registry[cfg.Driver]
	registryMu.RUnlock()
	if !ok {
		return nil, fmt.Errorf("motor: unknown driver %q (available: %v)", cfg.Driver, Drivers())
	}
	return driver.Factory(cfg)
}

// Drivers lists the registered driver names, sorted.
func Drivers() []string {
	registryMu.RLock()
	defer registryMu.RUnlock()
	names := make([]string, 0, len(registry))
	for name := range registry {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// DriverInfo returns the full registration for name.
func DriverInfo(name string) (Driver, bool) {
	registryMu.RLock()
	defer registryMu.RUnlock()
	driver, ok := registry[name]
	return driver, ok
}
