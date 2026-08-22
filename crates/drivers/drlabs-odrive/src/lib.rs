//! ODrive motor controllers.
//!
//! In the Go tree this package was eight lines of doc comment and no code — a
//! promise rather than a driver. Here the documented protocol is implemented
//! and pinned by tests: CANSimple's `(node_id << 5) | command_id` addressing
//! and message layouts, and the line-oriented ASCII protocol for a board on a
//! serial port.
//!
//! **No ODrive has been on this bench.** The tests check the encoding against
//! the published layouts, which is a real standard — it is how a genuine frame
//! ordering bug in the moteus driver was caught — but it is not the same as
//! having watched an axis move. [`link::OdriveLink`] reports its board status
//! as unknown and says so in the board name, so an operator is told rather than
//! left to assume.

pub mod link;
pub mod protocol;

pub use link::OdriveLink;
pub use protocol::{AxisState, Command, ControlMode, InputMode};
