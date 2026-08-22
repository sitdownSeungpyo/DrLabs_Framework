// Package robot brings a whole machine up from one description of it.
//
// A robot is a set of named joints, each backed by a motor, and the motors are
// spread across a handful of buses.  Nothing above this layer should have to
// know which — an upper layer asks for "r_sh_p at 0.5 radians" and this is
// where that becomes a moteus at CAN id 3 on COM5, geared six to one, turning
// the other way.
//
// The description is a JSON file, so the same binary brings up any machine.
package robot

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
)

// Config is a whole robot, as read from disk.
type Config struct {
	// Robot is a name for logs and status.  Not otherwise used.
	Robot string `json:"robot"`

	// RateHz is the default control rate for every joint.
	//
	// It is not per joint, because the binding constraint is not a joint's:
	// every joint on a bus takes a transaction, so the bus carries joints ×
	// rate transactions a second whatever the joints individually want.  A
	// bus that cannot keep up is the one place this shows, so a bus may
	// override it — see BusConfig.RateHz.  Bringing a robot up reports the
	// load each bus is being asked for.
	RateHz int `json:"rate_hz"`

	Buses  []BusConfig   `json:"buses"`
	Joints []JointConfig `json:"joints"`
}

// BusConfig is one physical link, shared by every joint that names it.
type BusConfig struct {
	// Name is how joints refer to this bus.
	Name string `json:"name"`

	// Transport is "fdcanusb" for CAN, or "serial" for a Dynamixel chain.
	Transport string `json:"transport"`

	// Port is the OS device: COM5, /dev/ttyACM0.  Empty means auto-detect,
	// which only works for a CAN adapter that identifies itself.
	Port string `json:"port"`

	// Baud applies to serial transports only.
	Baud int `json:"baud"`

	// RateHz overrides the robot's rate for the joints on this bus.  Zero
	// means use the robot's.
	//
	// The bus is where the constraint actually lives.  A CAN-FD adapter will
	// carry several hundred transactions a second without complaint and a
	// Dynamixel chain will not, so a machine with both is either throttled to
	// what the slowest bus can take or pushed past what it can — and a bus
	// pushed past its budget does not degrade gently.  It falls behind, the
	// servo's own watchdog latches, and the joint stops.
	RateHz int `json:"rate_hz"`
}

// JointConfig is one joint and the motor that drives it.
type JointConfig struct {
	// Name is what the upper layer uses: r_sh_p, r_elb_p.
	Name string `json:"name"`

	// Bus names an entry in Buses.
	Bus string `json:"bus"`

	// Driver is a registered motor driver: moteus, dynamixel, robstride.
	Driver string `json:"driver"`

	// ID is the address on the bus — CAN id, or Dynamixel servo id.
	ID int `json:"id"`

	// Options are driver-specific settings, such as a RobStride model.
	Options map[string]string `json:"options"`

	// --- joint to motor mapping -------------------------------------------

	// GearRatio is motor turns per joint turn.  6.0 means the motor turns six
	// times for one turn of the joint.
	GearRatio float64 `json:"gear_ratio"`

	// Direction is +1 or -1, and flips the sense of the joint relative to the
	// motor.  Recorded during bring-up, when somebody watches which way the
	// axis actually goes.
	Direction float64 `json:"direction"`

	// OffsetRad is where the motor sits, in joint radians, when the joint
	// reads zero.  It moves the joint's zero without touching the
	// controller's own reference.
	OffsetRad float64 `json:"offset_rad"`

	Limits JointLimits `json:"limits"`
}

// JointLimits is the envelope for one joint, in joint units.
//
// These are the numbers bring-up produces, and they are enforced on the host
// before anything reaches a bus.  Torque is joint torque: what the mechanism
// sees, not what the motor produces, because the gearbox is between them.
type JointLimits struct {
	MinRad float64 `json:"min_rad"`
	MaxRad float64 `json:"max_rad"`

	MaxTorqueNm     float64 `json:"max_torque_nm"`
	MaxVelocityRadS float64 `json:"max_velocity_rad_s"`
	MaxAccelRadS2   float64 `json:"max_accel_rad_s2"`
}

// Load reads and validates a robot description.
func Load(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("robot: cannot read %s: %w", path, err)
	}

	var config Config
	decoder := json.NewDecoder(strings.NewReader(string(data)))
	// A key nobody reads is a key that was meant to do something.  Silently
	// ignoring "gear_ration" would leave a joint geared 1:1 and nothing said.
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&config); err != nil {
		return nil, fmt.Errorf("robot: %s: %w", path, err)
	}

	if err := config.Validate(); err != nil {
		return nil, fmt.Errorf("robot: %s: %w", path, err)
	}
	return &config, nil
}

// Validate checks the description for the mistakes that would otherwise show
// up as a machine moving the wrong way.
//
// It is deliberately strict.  A robot description is written once and then
// trusted by everything above it, so the moment to catch a missing gear ratio
// is while reading the file, not while the arm is travelling.
func (c *Config) Validate() error {
	if len(c.Joints) == 0 {
		return fmt.Errorf("no joints defined")
	}
	if c.RateHz == 0 {
		c.RateHz = 200
	}
	if c.RateHz < 10 || c.RateHz > 500 {
		return fmt.Errorf("rate_hz %d is outside 10..500", c.RateHz)
	}

	buses := map[string]bool{}
	for i, bus := range c.Buses {
		if bus.Name == "" {
			return fmt.Errorf("bus %d has no name", i)
		}
		if buses[bus.Name] {
			return fmt.Errorf("two buses are both named %q", bus.Name)
		}
		buses[bus.Name] = true

		switch bus.Transport {
		case "fdcanusb", "serial":
		default:
			return fmt.Errorf("bus %q: unknown transport %q (fdcanusb or serial)", bus.Name, bus.Transport)
		}
		if bus.RateHz != 0 && (bus.RateHz < 10 || bus.RateHz > 500) {
			return fmt.Errorf("bus %q: rate_hz %d is outside 10..500", bus.Name, bus.RateHz)
		}
	}

	seen := map[string]bool{}
	addresses := map[string]string{}

	for i, joint := range c.Joints {
		where := fmt.Sprintf("joint %d", i)
		if joint.Name != "" {
			where = fmt.Sprintf("joint %q", joint.Name)
		}

		if joint.Name == "" {
			return fmt.Errorf("%s has no name", where)
		}
		if seen[joint.Name] {
			return fmt.Errorf("two joints are both named %q", joint.Name)
		}
		seen[joint.Name] = true

		if len(c.Buses) > 0 && !buses[joint.Bus] {
			return fmt.Errorf("%s names bus %q, which is not defined", where, joint.Bus)
		}
		if joint.Driver == "" {
			return fmt.Errorf("%s has no driver", where)
		}

		// Two joints on one address would each be commanding the other's
		// motor, and the symptom is a machine that fights itself.
		key := joint.Bus + "/" + fmt.Sprint(joint.ID)
		if other, clash := addresses[key]; clash {
			return fmt.Errorf("%s and %q are both at address %d on bus %q",
				where, other, joint.ID, joint.Bus)
		}
		addresses[key] = joint.Name

		if joint.GearRatio == 0 {
			return fmt.Errorf("%s has no gear_ratio (use 1.0 for a direct drive)", where)
		}
		if joint.GearRatio < 0 {
			return fmt.Errorf("%s has a negative gear_ratio; use direction: -1 to reverse it", where)
		}
		if joint.Direction != 1 && joint.Direction != -1 {
			return fmt.Errorf("%s has direction %v; it must be 1 or -1", where, joint.Direction)
		}

		limits := joint.Limits
		if limits.MaxRad <= limits.MinRad {
			return fmt.Errorf("%s has an empty travel range [%v, %v]", where, limits.MinRad, limits.MaxRad)
		}
		if limits.MaxTorqueNm <= 0 {
			return fmt.Errorf("%s has no max_torque_nm; a joint with no torque ceiling has no envelope", where)
		}
		if limits.MaxVelocityRadS <= 0 {
			return fmt.Errorf("%s has no max_velocity_rad_s", where)
		}
		if math.IsNaN(limits.MinRad) || math.IsNaN(limits.MaxRad) {
			return fmt.Errorf("%s has a non-numeric travel limit", where)
		}
	}

	return nil
}

// Bus returns the named bus.
func (c *Config) Bus(name string) (BusConfig, bool) {
	for _, bus := range c.Buses {
		if bus.Name == name {
			return bus, true
		}
	}
	return BusConfig{}, false
}

// JointNames lists the joints in the order they were declared, which is the
// order bring-up follows.
func (c *Config) JointNames() []string {
	names := make([]string, 0, len(c.Joints))
	for _, joint := range c.Joints {
		names = append(names, joint.Name)
	}
	return names
}

// RateForBus is the control rate the joints on this bus run at.
func (c *Config) RateForBus(name string) int {
	if bus, ok := c.Bus(name); ok && bus.RateHz > 0 {
		return bus.RateHz
	}
	return c.RateHz
}

// BusLoad reports how many transactions a second each bus will carry.
//
// This is the number that decides whether a rate is achievable: every joint on
// a bus costs one transaction per cycle, so eight joints at 200 Hz ask the
// adapter for 1600 round trips a second whether or not it can manage them.
func (c *Config) BusLoad() map[string]int {
	load := map[string]int{}
	for _, joint := range c.Joints {
		load[joint.Bus] += c.RateForBus(joint.Bus)
	}
	return load
}

// SortedBusNames lists the buses actually used by joints.
func (c *Config) SortedBusNames() []string {
	seen := map[string]bool{}
	var names []string
	for _, joint := range c.Joints {
		if !seen[joint.Bus] {
			seen[joint.Bus] = true
			names = append(names, joint.Bus)
		}
	}
	sort.Strings(names)
	return names
}
