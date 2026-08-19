// Package timing provides the scheduling accuracy a motor control loop needs.
//
// The problem it solves is concrete.  Windows' default timer granularity is
// 15.6 ms, so a naive sleep in a 100 Hz loop overshoots its 10 ms period by
// more than the period itself — the loop silently runs at 64 Hz with 15 ms of
// jitter, and every watchdog timeout derived from the nominal rate is wrong.
//
// Two things fix it: raising the multimedia timer resolution to 1 ms removes
// the coarse error, and a short spin at the end of each wait removes what is
// left.  The spin is bounded and only covers the final approach, so it costs a
// fraction of a core rather than a whole one.
package timing

import (
	"context"
	"sync"
	"time"
)

// DefaultSpin is how long before the deadline the wait switches from sleeping
// to spinning.  It is a little above the 1 ms granularity a raised timer
// gives, so the sleep never overshoots into the spin's territory.
const DefaultSpin = 1500 * time.Microsecond

var (
	resolutionMu    sync.Mutex
	resolutionCount int
)

// RaiseResolution asks the OS for the finest timer granularity it offers, and
// returns a function that releases the request.
//
// It is reference counted, so nested or concurrent callers each get a matching
// release.  On platforms where the request is unnecessary or unavailable it
// reports false and the returned function does nothing — a caller should carry
// on either way, since the spin still bounds the error.
func RaiseResolution() (raised bool, release func()) {
	resolutionMu.Lock()
	defer resolutionMu.Unlock()

	if resolutionCount == 0 {
		if !raiseTimerResolution() {
			return false, func() {}
		}
	}
	resolutionCount++

	var once sync.Once
	return true, func() {
		once.Do(func() {
			resolutionMu.Lock()
			defer resolutionMu.Unlock()
			resolutionCount--
			if resolutionCount == 0 {
				releaseTimerResolution()
			}
		})
	}
}

// SleepUntil waits until deadline, or until wake fires, or until ctx is done.
//
// It returns true when the wait was cut short by wake or by ctx — that is,
// when something happened that the caller should react to before running its
// next period.  The final approach spins rather than sleeping, so the return
// lands on the deadline instead of somewhere after it.
func SleepUntil(ctx context.Context, deadline time.Time, wake <-chan struct{}, spin time.Duration) bool {
	if spin <= 0 {
		spin = DefaultSpin
	}

	remaining := time.Until(deadline) - spin
	if remaining > 0 {
		timer := time.NewTimer(remaining)
		defer timer.Stop()
		select {
		case <-timer.C:
		case <-wake:
			return true
		case <-ctx.Done():
			return true
		}
	}

	// Final approach.  Yielding rather than sleeping keeps the landing
	// independent of OS timer granularity.
	for time.Now().Before(deadline) {
		select {
		case <-wake:
			return true
		case <-ctx.Done():
			return true
		default:
		}
		yield()
	}

	// Report an already-pending wake, so a signal that arrived during the
	// spin is not swallowed.
	select {
	case <-wake:
		return true
	default:
		return false
	}
}

// Statistics summarises a set of durations.
type Statistics struct {
	Count int
	Mean  time.Duration
	P95   time.Duration
	Max   time.Duration
}

// Summarise computes mean, 95th percentile and maximum.  A p95 far above the
// mean is the signature of an adapter that is mostly fast but occasionally
// stalls, which is exactly the case a mean alone hides.
func Summarise(samples []time.Duration) Statistics {
	if len(samples) == 0 {
		return Statistics{}
	}
	ordered := make([]time.Duration, len(samples))
	copy(ordered, samples)
	sortDurations(ordered)

	var total time.Duration
	for _, sample := range ordered {
		total += sample
	}
	index := int(float64(len(ordered)) * 0.95)
	if index >= len(ordered) {
		index = len(ordered) - 1
	}

	return Statistics{
		Count: len(ordered),
		Mean:  total / time.Duration(len(ordered)),
		P95:   ordered[index],
		Max:   ordered[len(ordered)-1],
	}
}

func sortDurations(values []time.Duration) {
	// Insertion sort: the sample windows here are a few hundred entries and
	// already nearly sorted, so this beats the allocation of a general sort.
	for i := 1; i < len(values); i++ {
		current := values[i]
		j := i - 1
		for j >= 0 && values[j] > current {
			values[j+1] = values[j]
			j--
		}
		values[j+1] = current
	}
}
