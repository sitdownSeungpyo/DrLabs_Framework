//! What a driver has to provide to be driveable.
//!
//! One trait, and everything above it — the control loop, the safety monitor,
//! the machine — is written against this and never against a vendor.

use crate::command::{Command, ControlMode, QueryProfile};
use crate::error::Result;
use crate::telemetry::Telemetry;
use crate::units::{Amps, Rev, RevPerSec, Volts};

/// A connection to one controller.
///
/// `cycle` returning `Ok(None)` is a dropped cycle: the device did not answer
/// this period, which is not by itself fatal. That is deliberately different
/// from an error, because a loop that treats one missed reply as a failure
/// stops machines that are working.
pub trait Link: Send {
    /// The driver name this link was built from.
    fn kind(&self) -> &'static str;

    /// Connection details, for display and logging.
    fn describe(&self) -> Vec<(String, String)>;

    /// Acquire the transport and leave the controller stopped.
    fn open(&mut self) -> Result<()>;

    /// Stop the motor and release the transport. Safe on a link that never
    /// opened.
    fn close(&mut self) -> Result<()>;

    /// Read board identification. `Ok(None)` means the device answered but
    /// reports nothing identifying.
    fn identify(&mut self) -> Result<Option<Identity>>;

    /// Transmit one command and return whatever telemetry came back.
    fn cycle(&mut self, command: &Command, profile: QueryProfile) -> Result<Option<Telemetry>>;

    /// Which control modes this link can express.
    fn capabilities(&self) -> Capabilities;

    /// The hardware's own documented limits, when the driver knows them.
    fn envelope(&self) -> Option<Envelope> {
        None
    }
}

/// What a driver supports, so a UI can grey out what is unavailable instead of
/// failing at command time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Capabilities {
    pub modes: Vec<ControlMode>,

    /// The highest control rate the transport can sustain.
    pub max_rate_hz: u32,

    /// Whether the controller stops on its own when host commands stop
    /// arriving. Without it, the host loop is the only thing standing between
    /// a wedged process and a runaway axis.
    pub has_watchdog: bool,

    pub reports_temperature: bool,
    pub reports_current: bool,
    pub reports_voltage: bool,
}

impl Capabilities {
    pub fn supports(&self, mode: ControlMode) -> bool {
        self.modes.contains(&mode)
    }
}

/// The hardware's own ratings.
///
/// The host safety layer checks the operator's declared envelope against this,
/// so a host setting can never be looser than what the controller is rated for.
/// `None` means "not characterised by this driver", which is different from a
/// limit of zero — the Go version used 0 for both and had to say so in a
/// comment on every field.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Envelope {
    pub voltage_min: Option<Volts>,
    pub voltage_max: Option<Volts>,

    /// The supply the hardware is designed to run on.
    ///
    /// It is what tells a host-side voltage trip band whether it is plausible
    /// for this device at all: a band that excludes the rated supply is not a
    /// safety limit, it is a guaranteed nuisance trip. A 5 V servo against a
    /// band inherited from a 12 V bus cannot be armed at all, which is exactly
    /// what happened before the Dynamixel driver published this.
    pub voltage_rated: Option<Volts>,

    pub velocity_max: Option<RevPerSec>,
    pub peak_current: Option<Amps>,
}

/// How well this build knows the board in front of it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum BoardStatus {
    /// Recognised, and within what this build supports.
    Supported,
    /// Answered, but this build cannot confirm what it is.
    #[default]
    Unknown,
    /// Recognised, and known to be outside the supported range.
    Rejected,
}

impl BoardStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            BoardStatus::Supported => "supported",
            BoardStatus::Unknown => "unknown_revision",
            BoardStatus::Rejected => "rejected",
        }
    }
}

/// What a controller can say about itself.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Identity {
    pub vendor: &'static str,
    pub board: String,
    pub firmware: String,
    pub serial: String,
    pub family: Option<i32>,
    pub hardware_revision: Option<i32>,
    pub model_number: Option<u32>,

    /// The travel limits stored *in the controller*.
    ///
    /// Commands outside them are silently clamped or rejected by the firmware,
    /// which from the outside looks exactly like "the motor does not move", so
    /// the host reads them and explains the constraint up front.
    pub bounds: Option<(Rev, Rev)>,

    pub status: BoardStatus,
    pub simulated: bool,
}

impl Identity {
    pub fn new(vendor: &'static str) -> Self {
        Self {
            vendor,
            ..Default::default()
        }
    }

    pub fn is_supported(&self) -> bool {
        self.status == BoardStatus::Supported
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_uncharacterised_limit_is_not_a_limit_of_zero() {
        let envelope = Envelope::default();
        assert_eq!(envelope.voltage_rated, None);
        assert_ne!(envelope.voltage_rated, Some(Volts::ZERO));
    }

    #[test]
    fn capabilities_answer_what_a_ui_needs_to_ask() {
        let capabilities = Capabilities {
            modes: vec![ControlMode::Position, ControlMode::Velocity, ControlMode::Stop],
            ..Default::default()
        };
        assert!(capabilities.supports(ControlMode::Position));
        assert!(!capabilities.supports(ControlMode::Brake));
    }

    #[test]
    fn an_unidentified_board_is_not_supported() {
        let identity = Identity::new("moteus");
        assert_eq!(identity.status, BoardStatus::Unknown);
        assert!(!identity.is_supported());
    }
}
