//! The real-time runtime.
//!
//! One thread per bus. Each tick that thread takes the latest [`LowCmd`]
//! snapshot, decides per axis whether it may be driven, converts joint space
//! to motor space, makes one call into the bus, converts back, evaluates the
//! safety guard, and publishes a [`BusState`] snapshot. Nothing on that path
//! takes a lock or allocates; the snapshots cross threads through
//! [`exchange`], which is a triple buffer per reader.
//!
//! What the runtime deliberately does *not* do: plan trajectories, hold
//! poses, learn ranges, sequence burn-in. Those are clients, and they talk to
//! the runtime in the same two records as everything else.
//!
//! ```text
//!  client ──LowCmd──▶ [exchange] ──▶ BusWorker ──▶ Bus::cycle ──▶ wire
//!  client ◀─LowState─ [exchange] ◀── BusWorker ◀── replies    ◀── wire
//! ```

pub mod config;
pub mod daemon;
pub mod exchange;
pub mod gate;
pub mod mapping;
pub mod worker;

pub use config::{ApiConfig, BusConfig, Config, ConfigError, JointConfig, JointLimits, RtConfig};
pub use daemon::{Daemon, Handle};
pub use drlabs_core::lowlevel::{LowCmd, LowState, MotorCmd, MotorState, MAX_AXES};
pub use exchange::{Broadcast, Reader, Stamped, Writer};
pub use gate::{Estop, TimeoutPolicy};
pub use mapping::Mapping;
pub use worker::{AxisSpec, BusState, BusWorker, WorkerConfig};
