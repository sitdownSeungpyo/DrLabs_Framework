//! ROBOTIS Dynamixel servos, over Protocol 2.0 on a serial bus.
//!
//! One driver, not two. The Go tree carried three layers that had accumulated
//! rather than been designed — `legacy/dxl`, `drivers/dynamixel/driver.go` and
//! the `Link` that replaced them — and every capability of the older two is
//! here: register reads and writes, ping, reboot, factory reset, and the sync
//! read and sync write that let a daisy chain move on one period instead of one
//! per servo.
//!
//! What is not here is the goroutine-and-channel calling convention the oldest
//! layer wrapped around them. That was a way of *calling* the driver, not a
//! thing the driver could do, and keeping a second copy of it would mean two
//! places to fix the next time the wire turns out to work differently than
//! anyone thought.

pub mod bus;
pub mod chain;
pub mod driver;
pub mod link;
pub mod profile;
pub mod protocol;

pub use bus::DynamixelBus;
pub use driver::Driver;
pub use link::DynamixelLink;
pub use profile::{Profile, XL330, X_SERIES};
