//go:build !windows

package timing

import "runtime"

// Linux and macOS already schedule with the granularity a control loop needs
// (high-resolution timers are the default), so there is nothing to raise.
func raiseTimerResolution() bool { return false }

func releaseTimerResolution() {}

func yield() { runtime.Gosched() }
