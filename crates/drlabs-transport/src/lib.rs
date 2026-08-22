//! The wires: serial ports and CAN buses.
//!
//! Everything above this crate speaks in commands and telemetry. Everything
//! here speaks in bytes and frames, and knows which operating system it is on.
//!
//! The two halves are shaped alike because they solve the same problem twice.
//! A serial port carries a Dynamixel daisy chain; a CAN bus carries several
//! moteus controllers. In both cases the unit that has to be atomic is the
//! *transaction* — write, then read the answer — not the individual call. Two
//! interleaved exchanges do not corrupt each other's bytes; they swap each
//! other's replies, and both sides time out holding a packet meant for the
//! other.
//!
//! The Go version solved that with a `Lock`/`Unlock` pair and a comment asking
//! callers to hold it for the whole exchange. Here the lock *is* the only route
//! to the port: [`serial::SharedPort::lock`] and [`can::SharedBus::lock`] hand
//! back a guard, and there is no way to reach a read or a write without one. A
//! rule that used to live in a doc comment is now checked by the compiler.

pub mod can;
pub mod fdcanusb;
pub mod serial;

pub use can::{Bus, Frame, SharedBus};
pub use serial::{PortInfo, SerialPort, SharedPort};
