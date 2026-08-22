//! One axis, driven properly.
//!
//! A session owns a [`Link`](drlabs_core::Link), a control loop that runs at a
//! declared rate, a safety monitor that sees every command and every reply, and
//! the state machine that decides what the axis is allowed to do next.
//!
//! Two sequencers live *inside* the loop rather than in an application on top of
//! it — the A↔B shuttle and host-side trajectories. That is deliberate: their
//! timing does not depend on a caller staying responsive, and they stop the
//! instant the axis is disarmed or faulted, because the code that decides
//! whether to command anything at all is the code that steps them.

pub mod control_loop;
pub mod record;
pub mod session;
pub mod shuttle;
pub mod state;
pub mod trajectory;

pub use record::{Event, Level, Sample};
pub use session::{Options, Session, Status};
pub use shuttle::Shuttle;
pub use state::{Connection, EStop, Params, Setpoint, TrajectoryStatus};
pub use trajectory::Trapezoidal;
