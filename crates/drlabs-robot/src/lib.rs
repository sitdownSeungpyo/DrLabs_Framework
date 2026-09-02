//! A machine: several joints, several buses, one JSON description.
//!
//! This is the layer the single-binary idea rests on. A robot is a description
//! and an executable: copy two files onto a machine, run it, and the axes are
//! under control.
//!
//! # Two spaces, one conversion point
//!
//! Everything below this crate works in **motor space**: revolutions at the
//! shaft, torque at the rotor. Everything above it is authored in **joint
//! space**: radians at the output, torque at the mechanism. A gear ratio, a
//! sign and a zero offset separate them.
//!
//! [`joint::Mapping`] is the only place that converts. Do not add a second one.
//! The same axis is legitimately `0.477 rev` and `3.000 rad` at the same
//! instant, so a number without its space is not a fact.

pub mod config;
pub mod joint;
pub mod learn;
pub mod robot;
pub mod units;

pub use config::{BusConfig, Config, JointConfig, JointLimits};
pub use joint::Mapping;
pub use learn::{LearnStatus, Learned, RangeLearner};
pub use robot::{Joint, JointState, Robot};
pub use units::JointUnits;
