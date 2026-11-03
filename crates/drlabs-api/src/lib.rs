//! How a client talks to the runtime.
//!
//! Two records cross the boundary, [`LowCmd`] in and [`LowState`] out, as
//! CycloneDDS samples under ROS 2 names. A ROS 2 node using
//! `rmw_cyclonedds` on the same domain publishes and subscribes to them with
//! the `drlabs_msgs` package and no bridge; any other DDS participant can
//! do the same with the type names below. `tools/drlabs_client` is the
//! Python reference.
//!
//! | topic (ROS 2 name) | DDS type | QoS |
//! |---|---|---|
//! | `/<prefix>/lowcmd` | `drlabs_msgs::msg::dds_::LowCmd_` | best effort, volatile, keep last |
//! | `/<prefix>/lowstate` | `drlabs_msgs::msg::dds_::LowState_` | best effort, volatile, keep last 1 |
//! | `/<prefix>/describe` | `drlabs_msgs::msg::dds_::Describe_` | reliable, transient local, keep last 1 |
//!
//! `prefix` is `api.dds.prefix` in the robot description, `drlabs` by
//! default; on the DDS wire the topics carry ROS 2's `rt/` in front. The
//! sample bodies are plain XCDR1 (see [`cdr`]), little-endian, so a 64-joint
//! command is 1556 bytes and a state 1848 — every entry travels, as a ROS 2
//! fixed-size array does.
//!
//! The runtime publishes a state every `api.state_div` ticks and the
//! description once, latched, so a client that arrives later still receives
//! it. Late samples are dropped, not retried: a command older than the one
//! after it is worthless, which is what the best-effort QoS says.

pub mod cdr;
pub mod client;
pub mod dds;
pub mod describe;
pub mod server;

pub use cdr::{
    decode_cmd, decode_describe, decode_state, encode_cmd, encode_describe, encode_state, CdrError,
};
pub use client::{Client, Description, JointInfo};
pub use dds::DdsError;
pub use drlabs_core::lowlevel::{LowCmd, LowState, MotorCmd, MotorState, PROTOCOL_VERSION};
pub use server::DdsServer;
