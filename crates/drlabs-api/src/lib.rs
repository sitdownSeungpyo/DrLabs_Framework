//! How a client talks to the runtime.
//!
//! Two records cross the boundary, [`LowCmd`] in and [`LowState`] out, as
//! fixed-layout frames over UDP. Any language with a socket and a `struct`
//! module can speak it; `tools/drlabs_client` is the Python reference.
//!
//! The frame:
//!
//! ```text
//! 0   "DLRT"            magic
//! 4   u16 version       PROTOCOL_VERSION; a client built against another is refused
//! 6   u16 kind          1 CMD, 2 STATE, 3 DESCRIBE, 4 DESCRIBE_REPLY
//! 8   payload           CMD: LowCmd header (24 B) + n × MotorCmd (24 B)
//!                       STATE: LowState header (40 B) + n × MotorState (28 B)
//!                       DESCRIBE: empty
//!                       DESCRIBE_REPLY: UTF-8 JSON
//! end-4  u32 crc32      IEEE (zlib's), over everything before it
//! ```
//!
//! Little-endian throughout. Only the `n` entries in use travel, so a
//! 26-joint command is 24 + 8 + 26 × 24 + 4 = 660 bytes and a state 776 —
//! inside one datagram on any link.
//!
//! The runtime sends a STATE frame to every address it has heard a frame
//! from in the last two seconds, every `api.state_div` ticks. A client that
//! wants states and has nothing to command sends a DESCRIBE; the reply is
//! the machine's description (joint names in record order, limits, poses)
//! as JSON.

pub mod client;
pub mod crc;
pub mod describe;
pub mod frame;
pub mod udp;

pub use client::{Client, Description, JointInfo};
pub use drlabs_core::lowlevel::{LowCmd, LowState, MotorCmd, MotorState, PROTOCOL_VERSION};
pub use frame::{
    decode, encode_cmd, encode_describe, encode_describe_reply, encode_state, Frame, FrameError,
};
pub use udp::UdpServer;
