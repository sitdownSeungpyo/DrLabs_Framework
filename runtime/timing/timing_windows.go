//go:build windows

package timing

import (
	"runtime"
	"syscall"
)

var (
	winmm               = syscall.NewLazyDLL("winmm.dll")
	procTimeBeginPeriod = winmm.NewProc("timeBeginPeriod")
	procTimeEndPeriod   = winmm.NewProc("timeEndPeriod")
)

// timerPeriodMS is the granularity we ask for.  1 ms is the finest the
// multimedia timer API offers and is enough for a 500 Hz loop.
const timerPeriodMS = 1

// raiseTimerResolution asks Windows for 1 ms timers.
//
// Without it, the scheduler's default 15.6 ms quantum makes any loop above
// ~60 Hz miss its period on most cycles.  The call affects the whole process,
// which is why it is reference counted and released on shutdown.
func raiseTimerResolution() bool {
	ret, _, _ := procTimeBeginPeriod.Call(timerPeriodMS)
	return ret == 0 // TIMERR_NOERROR
}

func releaseTimerResolution() {
	procTimeEndPeriod.Call(timerPeriodMS)
}

// yield hands the rest of this time slice to the scheduler.  During the final
// spin that is cheaper than a busy loop and precise enough, because the
// remaining wait is under a millisecond.
func yield() { runtime.Gosched() }
