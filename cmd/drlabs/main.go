// Command drlabs is the framework's command line front end.
//
//	drlabs ports                       list serial ports, CAN adapters first
//	drlabs drivers                     list registered drivers
//	drlabs probe    -driver … -id …    connect, identify, report, disconnect
//	drlabs run      -driver … -id …    run a control session
//
// Every subcommand goes through the same motor.Link and runtime/session that
// an application would, so what the CLI can do is a lower bound on what the
// framework can do.
package main

import (
	"flag"
	"fmt"
	"os"

	// Drivers register themselves; importing them for effect is what decides
	// which hardware this binary can talk to.
	_ "drlabs/drivers/dynamixel"
	_ "drlabs/drivers/moteus"
	_ "drlabs/drivers/robstride"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}

	var err error
	switch os.Args[1] {
	case "ports":
		err = runPorts(os.Args[2:])
	case "drivers":
		err = runDrivers(os.Args[2:])
	case "probe":
		err = runProbe(os.Args[2:])
	case "run":
		err = runSession(os.Args[2:])
	case "serve":
		err = runServe(os.Args[2:])
	case "bringup":
		err = runBringUp(os.Args[2:])
	case "-h", "--help", "help":
		usage()
		return
	default:
		fmt.Fprintf(os.Stderr, "drlabs: unknown command %q\n\n", os.Args[1])
		usage()
		os.Exit(2)
	}

	if err != nil {
		fmt.Fprintf(os.Stderr, "drlabs: %v\n", err)
		os.Exit(1)
	}
}

func usage() {
	fmt.Fprint(os.Stderr, `drlabs — multi-vendor motor control framework

Usage:
  drlabs ports                       List serial ports, CAN adapters first
  drlabs drivers                     List registered drivers
  drlabs probe   [flags]             Connect, identify, print state, disconnect
  drlabs run     [flags]             Run a control session
  drlabs serve   [flags]             Energise and execute commands from an
                                     upper layer over TCP (the servo role)
  drlabs bringup [flags] <robot.json>
                                     Bring a whole robot up from a description
                                     and take joint-addressed commands

Common flags:
  -driver   moteus | moteus-sim | dynamixel   (default moteus-sim)
  -port     serial port; empty auto-detects a CAN adapter
  -baud     serial baud rate (dynamixel only, default 1000000)
  -id       bus address: CAN id or servo id (default 1)
  -opt      driver option, repeatable: -opt key=value

Run "drlabs run -h" or "drlabs serve -h" for the remaining flags.
`)
}

// commonFlags are the flags every connecting subcommand shares.
type commonFlags struct {
	driver  string
	port    string
	baud    int
	id      int
	options optionList
}

func (c *commonFlags) register(fs *flag.FlagSet) {
	fs.StringVar(&c.driver, "driver", "moteus-sim", "driver name")
	fs.StringVar(&c.port, "port", "", "serial port (empty auto-detects)")
	fs.IntVar(&c.baud, "baud", 1000000, "serial baud rate")
	fs.IntVar(&c.id, "id", 1, "bus address")
	fs.Var(&c.options, "opt", "driver option key=value (repeatable)")
}

// optionList collects repeated -opt key=value flags.
type optionList map[string]string

func (o *optionList) String() string { return "" }

func (o *optionList) Set(value string) error {
	if *o == nil {
		*o = optionList{}
	}
	for i := 0; i < len(value); i++ {
		if value[i] == '=' {
			(*o)[value[:i]] = value[i+1:]
			return nil
		}
	}
	return fmt.Errorf("option %q must be key=value", value)
}
