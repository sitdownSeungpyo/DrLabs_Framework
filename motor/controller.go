package motor

import "context"

// Controller manages a real-time control loop for one or more motors.
type Controller interface {
	// Start begins the control loop. Blocks until initialization is complete.
	Start(ctx context.Context) error

	// Stop gracefully shuts down the control loop.
	Stop() error

	// SetGoals sends position/velocity/current commands to motors.
	SetGoals(goals []Goal) error

	// Feedback returns the latest motor states.
	Feedback() ([]State, error)

	// Motors returns the list of managed motors.
	Motors() []Motor
}

// MultiController manages motors across different bus types.
type MultiController struct {
	controllers []Controller
}

// NewMultiController creates a controller that routes goals to the appropriate sub-controller.
func NewMultiController(controllers ...Controller) *MultiController {
	return &MultiController{controllers: controllers}
}

// Start starts all sub-controllers.
func (mc *MultiController) Start(ctx context.Context) error {
	for _, c := range mc.controllers {
		if err := c.Start(ctx); err != nil {
			// Stop already-started controllers on failure
			for _, started := range mc.controllers {
				if started == c {
					break
				}
				started.Stop()
			}
			return err
		}
	}
	return nil
}

// Stop stops all sub-controllers.
func (mc *MultiController) Stop() error {
	var lastErr error
	for _, c := range mc.controllers {
		if err := c.Stop(); err != nil {
			lastErr = err
		}
	}
	return lastErr
}
