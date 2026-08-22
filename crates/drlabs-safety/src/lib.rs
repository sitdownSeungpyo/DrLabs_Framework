//! The host-side protection layer.
//!
//! Motor controllers carry their own protections — over and under voltage, over
//! temperature, position bounds, a command watchdog. Those protect the *board*.
//!
//! This crate adds a second layer that protects the *machine*. The motor, the
//! gearbox and whatever they are bolted to are not the controller
//! manufacturer's products, so their envelope has to be declared by the
//! operator and enforced on the host, before anything reaches the bus.
//!
//! There are two kinds of check:
//!
//! * **clamps** applied to every outgoing command; they limit values silently,
//!   because a command that has been reduced is still a command
//! * **trips** observed in telemetry, which latch an emergency stop
//!
//! Nothing here is ever written to a controller's persistent configuration.

pub mod config;
pub mod monitor;

pub use config::{Config, Preset, PRESETS};
pub use monitor::{Monitor, Preflight, Trip};
