//! The surface every driver implements and every layer above them speaks.
//!
//! Nothing here touches a bus. It is the vocabulary: what a command is, what
//! telemetry is, which units they are in, and what a driver has to provide to
//! be driveable by the runtime.
//!
//! Two things are deliberately different from the Go framework this replaces,
//! and both come from bugs that reached hardware:
//!
//! * **Units carry their space.** [`units::Rev`] and [`units::Rad`] are
//!   different types, so a joint limit written in radians cannot be reported
//!   back in revolutions by accident.
//! * **"Not reported" is not a number.** Telemetry uses `Option`, not a NaN
//!   sentinel, so a missing torque cannot be read as zero torque.

pub mod command;
pub mod error;
pub mod link;
pub mod render;
pub mod telemetry;
pub mod units;

pub use command::{Command, ControlMode, Field, Mode, QueryProfile, TorqueLimit};
pub use error::{Error, Result};
pub use link::{BoardStatus, Capabilities, Envelope, Identity, Link};
pub use render::{render, MotorUnits, Units};
pub use telemetry::Telemetry;
pub use units::{Amps, Celsius, Nm, Rad, RadPerSec, RadPerSec2, Rev, RevPerSec, RevPerSec2, Volts};
