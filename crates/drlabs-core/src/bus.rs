//! What a driver has to provide to be driven as a *bus*.
//!
//! [`crate::Link`] is one controller, one transaction. That is the right shape
//! for a bench with a single axis on it, and the wrong shape for a chain: a
//! Dynamixel bus answers sixteen servos in one packet, and sixteen `Link`s
//! taking turns on a mutex cannot express that. [`Bus`] is the group form —
//! every axis on one transport, commanded and read in one call, by one
//! thread.
//!
//! The contract is written for a real-time caller:
//!
//! * `cycle` is called once per period with one command per axis and one
//!   reply slot per axis. It fills every slot. An axis that did not answer
//!   gets [`AxisStatus::Silent`] and no readings, not an error — a dropped
//!   reply is a dropped cycle, and a loop that stops the machine for one of
//!   those stops machines that are working.
//! * `cycle` allocates nothing. Everything it needs is built in `open`.
//! * `Err` from `cycle` means the *transport* failed, not an axis.

use crate::command::{Command, QueryProfile};
use crate::error::Result;
use crate::link::{Capabilities, Envelope, Identity};
use crate::units::{Amps, Celsius, Nm, Rev, RevPerSec, Volts};

/// One axis's command for one cycle, in motor space.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AxisCmd {
    pub command: Command,
    pub profile: QueryProfile,
}

/// How an axis fared this cycle.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AxisStatus {
    /// Answered.
    Ok = 0,
    /// Did not answer this cycle.
    #[default]
    Silent = 1,
    /// Answered, and flags a hardware alert (the readings are still good).
    Alert = 2,
    /// Answered, and refused the command.
    Refused = 3,
    /// Not being driven: the runtime has it torque-off. Set by the runtime,
    /// never by a driver.
    Off = 4,
    /// An emergency stop is latched. Set by the runtime, never by a driver.
    Tripped = 5,
}

impl AxisStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            AxisStatus::Ok => "ok",
            AxisStatus::Silent => "silent",
            AxisStatus::Alert => "alert",
            AxisStatus::Refused => "refused",
            AxisStatus::Off => "off",
            AxisStatus::Tripped => "tripped",
        }
    }

    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0 => AxisStatus::Ok,
            1 => AxisStatus::Silent,
            2 => AxisStatus::Alert,
            3 => AxisStatus::Refused,
            4 => AxisStatus::Off,
            5 => AxisStatus::Tripped,
            _ => return None,
        })
    }

    /// Whether the readings in the reply were measured this cycle.
    pub fn answered(self) -> bool {
        matches!(
            self,
            AxisStatus::Ok | AxisStatus::Alert | AxisStatus::Refused
        )
    }
}

/// A controller's fault register, reduced to what the runtime decides on.
///
/// The name and the explanation stay with the driver, looked up from the
/// code when somebody asks; they are strings, and strings do not belong in
/// a reply that is written sixteen times a millisecond.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AxisFault {
    pub code: i32,
    /// Whether the controller has stopped driving because of it. A limiting
    /// state — "capping at max torque" — is a code with `hard == false`.
    pub hard: bool,
}

/// One axis's readings for one cycle, in motor space. `None` is not
/// reported, exactly as in [`crate::Telemetry`], and this is `Copy` with no
/// strings in it so that it can be written into a fixed array.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AxisReply {
    pub status: AxisStatus,
    /// The vendor's own mode register, raw.
    pub mode: Option<i32>,
    pub fault: Option<AxisFault>,
    pub position: Option<Rev>,
    pub velocity: Option<RevPerSec>,
    pub torque: Option<Nm>,
    pub current: Option<Amps>,
    pub voltage: Option<Volts>,
    pub board_temperature: Option<Celsius>,
    pub motor_temperature: Option<Celsius>,
}

impl AxisReply {
    /// Nothing heard.
    pub const SILENT: Self = Self {
        status: AxisStatus::Silent,
        mode: None,
        fault: None,
        position: None,
        velocity: None,
        torque: None,
        current: None,
        voltage: None,
        board_temperature: None,
        motor_temperature: None,
    };

    /// Overlays the fields `fresh` actually reported, keeping the rest.
    ///
    /// A fast query reads position and current; the voltage and temperature
    /// come round every tenth cycle. Merging keeps the last measured value
    /// for the fields this cycle did not ask about — and only for fields
    /// that were ever measured, so an unread field stays unread.
    pub fn merge(&mut self, fresh: &AxisReply) {
        self.status = fresh.status;
        if fresh.mode.is_some() {
            self.mode = fresh.mode;
        }
        if fresh.fault.is_some() {
            self.fault = fresh.fault;
        }
        merge(&mut self.position, fresh.position);
        merge(&mut self.velocity, fresh.velocity);
        merge(&mut self.torque, fresh.torque);
        merge(&mut self.current, fresh.current);
        merge(&mut self.voltage, fresh.voltage);
        merge(&mut self.board_temperature, fresh.board_temperature);
        merge(&mut self.motor_temperature, fresh.motor_temperature);
    }

    /// The hotter of the two temperatures a controller may report.
    pub fn hottest(&self) -> Option<Celsius> {
        match (self.board_temperature, self.motor_temperature) {
            (Some(a), Some(b)) => Some(if a.value() >= b.value() { a } else { b }),
            (a, b) => a.or(b),
        }
    }
}

#[inline]
fn merge<T: Copy>(destination: &mut Option<T>, source: Option<T>) {
    if source.is_some() {
        *destination = source;
    }
}

/// Every axis on one transport, driven by one thread.
pub trait Bus: Send {
    /// The driver name.
    fn kind(&self) -> &'static str;

    /// How many axes this bus carries. Fixed after construction.
    fn axes(&self) -> usize;

    /// Acquire the transport and leave every axis stopped. Returns what each
    /// axis said about itself, in axis order; `None` for one that answered
    /// nothing identifying. Allocation is fine here.
    fn open(&mut self) -> Result<Vec<Option<Identity>>>;

    /// Stop every axis and release the transport. Safe on a bus that never
    /// opened.
    fn close(&mut self) -> Result<()>;

    /// Transmit one command per axis and collect one reply per axis.
    ///
    /// `cmds.len()` and `out.len()` both equal [`Bus::axes`]; the bus may
    /// panic otherwise. Every `out[i]` is overwritten. No allocation.
    fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> Result<()>;

    /// What one axis can do.
    fn capabilities(&self, axis: usize) -> Capabilities;

    /// One axis's documented limits, when the driver knows them.
    fn envelope(&self, _axis: usize) -> Option<Envelope> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_status_byte_round_trips() {
        for status in [
            AxisStatus::Ok,
            AxisStatus::Silent,
            AxisStatus::Alert,
            AxisStatus::Refused,
            AxisStatus::Off,
            AxisStatus::Tripped,
        ] {
            assert_eq!(AxisStatus::from_u8(status as u8), Some(status));
        }
        assert_eq!(AxisStatus::from_u8(200), None);
    }

    #[test]
    fn merging_keeps_what_the_narrow_query_did_not_report() {
        let mut known = AxisReply {
            status: AxisStatus::Ok,
            position: Some(Rev(0.25)),
            voltage: Some(Volts(24.0)),
            ..AxisReply::SILENT
        };
        known.merge(&AxisReply {
            status: AxisStatus::Ok,
            position: Some(Rev(0.30)),
            ..AxisReply::SILENT
        });
        assert_eq!(known.position, Some(Rev(0.30)), "the fresh value wins");
        assert_eq!(known.voltage, Some(Volts(24.0)), "the old value survives");
        assert_eq!(known.torque, None, "an unread field stays unread");
    }

    /// A silent cycle keeps the last readings but says they are old.
    #[test]
    fn a_silent_reply_keeps_the_readings_and_changes_the_status() {
        let mut known = AxisReply {
            status: AxisStatus::Ok,
            position: Some(Rev(0.25)),
            ..AxisReply::SILENT
        };
        known.merge(&AxisReply::SILENT);
        assert_eq!(known.status, AxisStatus::Silent);
        assert_eq!(known.position, Some(Rev(0.25)));
        assert!(!known.status.answered());
    }

    #[test]
    fn the_hottest_temperature_is_the_one_that_trips() {
        let reply = AxisReply {
            board_temperature: Some(Celsius(40.0)),
            motor_temperature: Some(Celsius(65.0)),
            ..AxisReply::SILENT
        };
        assert_eq!(reply.hottest(), Some(Celsius(65.0)));
        assert_eq!(AxisReply::SILENT.hottest(), None);
    }
}
