//! The fixed-size command and state records the runtime exchanges.
//!
//! Everything above the bus thread speaks in these two records: a client
//! writes a [`LowCmd`], the bus threads write a [`LowState`]. They are plain
//! `repr(C)` arrays of `f32` so that they can sit in a lock-free buffer, be
//! copied in one `memcpy`, and be read by a client written in any language
//! without a schema compiler. The layout is the wire layout; the sizes are
//! pinned by tests so that a field added in the wrong place fails to build
//! rather than silently moving every client's offsets.
//!
//! Values are in joint space — radians at the output, newton-metres at the
//! mechanism — because that is what every client thinks in. The runtime
//! converts to motor space on the way to the bus and back on the way out.
//!
//! One departure from the rest of this crate: a measurement that was not
//! taken is `NaN` here, not `None`. These records cross a process boundary
//! where `Option` has no representation, and a client has to know the rule
//! either way. It is written down once, here, and the in-process reply type
//! ([`crate::bus::AxisReply`]) keeps the `Option`s.

/// The most axes one runtime will ever carry. 26 is the robot this was built
/// for; 64 leaves room without making the record silly.
pub const MAX_AXES: usize = 64;

/// Bumped whenever the layout of [`LowCmd`] or [`LowState`] changes. A client
/// built against another version is refused at the frame boundary rather than
/// read with the wrong offsets.
pub const PROTOCOL_VERSION: u16 = 1;

/// What a client may ask of one motor.
pub mod mode {
    /// Torque off. The axis is observed, not driven. This is the state a
    /// motor starts in and the state a stop returns it to.
    pub const OFF: u8 = 0;
    /// Drive: follow `q`/`dq` with the gains and feed-forward given. The
    /// runtime only honours it once the axis passes its preflight check.
    pub const DRIVE: u8 = 1;
}

/// Robot-wide requests that ride on a [`LowCmd`].
pub mod flags {
    /// Latch an emergency stop on every axis now.
    pub const ESTOP: u32 = 1 << 0;
    /// Clear a latched emergency stop, if every axis is back inside its
    /// envelope. Cleared by the runtime once acted on; a client that keeps
    /// sending it does not keep resetting anything.
    pub const RESET_ESTOP: u32 = 1 << 1;
}

/// One motor's command for one cycle, in joint space.
///
/// The five numbers are the ones every impedance-controlled joint takes:
/// target position, target velocity, stiffness, damping and feed-forward
/// torque. A controller that implements them natively (moteus) gets all
/// five; a position servo (Dynamixel) gets what it can take and the driver
/// documents what it ignores.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCmd {
    pub mode: u8,
    pub reserved: [u8; 3],
    /// Target position, rad.
    pub q: f32,
    /// Target velocity, rad/s.
    pub dq: f32,
    /// Stiffness, Nm/rad. Zero with `kd` zero and `tau` zero means limp.
    pub kp: f32,
    /// Damping, Nm·s/rad.
    pub kd: f32,
    /// Feed-forward torque, Nm.
    pub tau: f32,
}

impl MotorCmd {
    /// Torque off, nothing asked.
    pub const OFF: Self = Self {
        mode: mode::OFF,
        reserved: [0; 3],
        q: 0.0,
        dq: 0.0,
        kp: 0.0,
        kd: 0.0,
        tau: 0.0,
    };
}

impl Default for MotorCmd {
    fn default() -> Self {
        Self::OFF
    }
}

/// One motor's reported state, in joint space. `NaN` means not reported.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorState {
    /// Position, rad.
    pub q: f32,
    /// Velocity, rad/s.
    pub dq: f32,
    /// Torque at the mechanism, Nm, as the driver estimates it.
    pub tau: f32,
    /// Motor current, A.
    pub current: f32,
    /// Bus voltage, V.
    pub voltage: f32,
    /// Hottest temperature the controller reports, °C.
    pub temp: f32,
    /// The controller's own fault register, raw. Zero is clear.
    pub fault: u16,
    /// An [`crate::bus::AxisStatus`] as a byte.
    pub status: u8,
    /// Why a drive request is not being honoured, as a
    /// [`crate::bus::Blocked`] byte. Zero when nothing blocks.
    pub blocked: u8,
}

impl MotorState {
    /// Nothing reported yet.
    pub const EMPTY: Self = Self {
        q: f32::NAN,
        dq: f32::NAN,
        tau: f32::NAN,
        current: f32::NAN,
        voltage: f32::NAN,
        temp: f32::NAN,
        fault: 0,
        status: 0,
        blocked: 0,
    };
}

impl Default for MotorState {
    fn default() -> Self {
        Self::EMPTY
    }
}

/// A client's command for every axis.
///
/// Only the first `n` entries mean anything; a frame carries only those.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LowCmd {
    /// Increments with every record the client sends. The runtime uses it to
    /// tell a fresh command from a repeat, and reports the last one it acted
    /// on in [`LowState::cmd_seq`].
    pub seq: u32,
    /// [`flags`].
    pub flags: u32,
    /// The client's clock when it built this record, ns. Informational.
    pub t_ns: u64,
    /// How many of `cmd` are in use.
    pub n: u16,
    pub reserved: [u8; 6],
    pub cmd: [MotorCmd; MAX_AXES],
}

impl LowCmd {
    pub const EMPTY: Self = Self {
        seq: 0,
        flags: 0,
        t_ns: 0,
        n: 0,
        reserved: [0; 6],
        cmd: [MotorCmd::OFF; MAX_AXES],
    };

    /// The entries in use.
    pub fn active(&self) -> &[MotorCmd] {
        &self.cmd[..(self.n as usize).min(MAX_AXES)]
    }
}

impl Default for LowCmd {
    fn default() -> Self {
        Self::EMPTY
    }
}

/// The runtime's report for every axis, written once per bus tick.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LowState {
    /// Increments with every record the runtime publishes.
    pub seq: u32,
    /// The [`LowCmd::seq`] this state was computed against.
    pub cmd_seq: u32,
    /// Bus ticks since the runtime started.
    pub tick: u64,
    /// The runtime's monotonic clock at the end of the tick, ns.
    pub t_ns: u64,
    /// The configured period, ns.
    pub period_ns: u32,
    /// Ticks that started late, since start.
    pub deadline_miss: u32,
    /// Non-zero while an emergency stop is latched: a [`crate::bus::TripCode`]
    /// byte in the low eight bits.
    pub estop_code: u16,
    /// 1 while an emergency stop is latched.
    pub estop: u8,
    pub reserved: u8,
    /// How many of `state` are in use.
    pub n: u16,
    pub reserved2: [u8; 2],
    pub state: [MotorState; MAX_AXES],
}

impl LowState {
    pub const EMPTY: Self = Self {
        seq: 0,
        cmd_seq: 0,
        tick: 0,
        t_ns: 0,
        period_ns: 0,
        deadline_miss: 0,
        estop_code: 0,
        estop: 0,
        reserved: 0,
        n: 0,
        reserved2: [0; 2],
        state: [MotorState::EMPTY; MAX_AXES],
    };

    /// The entries in use.
    pub fn active(&self) -> &[MotorState] {
        &self.state[..(self.n as usize).min(MAX_AXES)]
    }
}

impl Default for LowState {
    fn default() -> Self {
        Self::EMPTY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{align_of, size_of};

    /// The layout is the wire layout. A client in another language reads
    /// these offsets from a document, so they do not move.
    #[test]
    fn the_records_have_the_documented_sizes() {
        assert_eq!(size_of::<MotorCmd>(), 24);
        assert_eq!(size_of::<MotorState>(), 28);
        assert_eq!(size_of::<LowCmd>(), 24 + MAX_AXES * 24);
        assert_eq!(size_of::<LowState>(), 40 + MAX_AXES * 28);
        assert_eq!(align_of::<LowCmd>(), 8);
        assert_eq!(align_of::<LowState>(), 8);
    }

    /// The runtime copies these records into and out of lock-free buffers;
    /// they must be plain data with nothing to drop.
    #[test]
    fn the_records_are_plain_data() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<MotorCmd>();
        assert_copy::<MotorState>();
        assert_copy::<LowCmd>();
        assert_copy::<LowState>();
    }

    #[test]
    fn an_empty_state_reports_nothing_rather_than_zero() {
        let state = MotorState::EMPTY;
        assert!(state.q.is_nan());
        assert!(state.voltage.is_nan());
        assert_eq!(state.fault, 0);
    }

    #[test]
    fn only_the_entries_in_use_are_active() {
        let mut cmd = LowCmd::EMPTY;
        cmd.n = 3;
        assert_eq!(cmd.active().len(), 3);
        cmd.n = 1000;
        assert_eq!(cmd.active().len(), MAX_AXES, "n is clamped, never trusted");
    }
}
