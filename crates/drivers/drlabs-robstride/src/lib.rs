//! The RobStride (灵足时代) quasi-direct-drive actuator family: RS00 through
//! RS06.
//!
//! CAN 2.0B at 1 Mbit/s, 29-bit extended identifiers, an 8-byte payload. The
//! per-model scaling table is the part that differs between variants, and
//! getting it wrong is the difference between commanding 17 Nm and commanding
//! 120 — which is why the model is a required setting and never a default.

pub mod controller;
pub mod link;
pub mod models;
pub mod parameters;
pub mod protocol;

pub use controller::Controller;
pub use link::{PositionMode, RobstrideLink};
pub use models::{lookup_model, Model, RS00, RS01, RS02, RS03, RS04, RS05, RS06};
