//! The drive state, named the way CiA 402 names it.
//!
//! The session and the runtime each keep their own bookkeeping — a connection,
//! an operation-enabled flag, a fault latch — and this is the one word the
//! outside world gets for the combination. The names are the profile's own so
//! that an operator who knows servo drives reads them without a glossary:
//!
//! | CiA 402 state | here |
//! |---|---|
//! | Switch on disabled | no link, or the link is being opened |
//! | Switched on | connected, torque off, watching the axis |
//! | Operation enabled | torque on, following setpoints |
//! | Fault | a trip or a quick stop is latched; `fault_reset` clears it |
//!
//! The transitions keep the profile's verbs too: `enable_operation`,
//! `disable_operation`, `quick_stop`, `fault_reset`. Two of the profile's
//! states are deliberately absent. "Ready to switch on" has no separate
//! meaning when the high-level power is the controller's own, and "Quick stop
//! active" is not a resting state here: a quick stop latches a fault, because
//! a stop that undid itself would be worse than no stop at all.

/// Where a drive is in the CiA 402 state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveState {
    /// No link, or one still being opened. Nothing can be commanded.
    SwitchOnDisabled,
    /// Connected and read, torque off.
    SwitchedOn,
    /// Torque on; setpoints are followed.
    OperationEnabled,
    /// A trip or a quick stop is latched. Clears only by `fault_reset`, and
    /// clearing it lands in `SwitchedOn`, never back in `OperationEnabled`.
    Fault,
}

impl DriveState {
    /// The state the three flags amount to. A latched fault outranks the rest;
    /// a drive that is not connected cannot be anything but disabled.
    pub fn of(connected: bool, operation_enabled: bool, fault: bool) -> Self {
        if !connected {
            DriveState::SwitchOnDisabled
        } else if fault {
            DriveState::Fault
        } else if operation_enabled {
            DriveState::OperationEnabled
        } else {
            DriveState::SwitchedOn
        }
    }

    /// The wire spelling, as the JSON APIs carry it.
    pub fn as_str(&self) -> &'static str {
        match self {
            DriveState::SwitchOnDisabled => "switch_on_disabled",
            DriveState::SwitchedOn => "switched_on",
            DriveState::OperationEnabled => "operation_enabled",
            DriveState::Fault => "fault",
        }
    }

    /// The profile's own wording, for a human.
    pub fn label(&self) -> &'static str {
        match self {
            DriveState::SwitchOnDisabled => "switch on disabled",
            DriveState::SwitchedOn => "switched on",
            DriveState::OperationEnabled => "operation enabled",
            DriveState::Fault => "fault",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "switch_on_disabled" => Some(DriveState::SwitchOnDisabled),
            "switched_on" => Some(DriveState::SwitchedOn),
            "operation_enabled" => Some(DriveState::OperationEnabled),
            "fault" => Some(DriveState::Fault),
            _ => None,
        }
    }

    pub fn is_operation_enabled(&self) -> bool {
        matches!(self, DriveState::OperationEnabled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fault_outranks_an_enabled_drive_and_disconnection_outranks_both() {
        assert_eq!(
            DriveState::of(true, true, false),
            DriveState::OperationEnabled
        );
        assert_eq!(DriveState::of(true, true, true), DriveState::Fault);
        assert_eq!(DriveState::of(true, false, false), DriveState::SwitchedOn);
        assert_eq!(
            DriveState::of(false, true, true),
            DriveState::SwitchOnDisabled
        );
    }

    #[test]
    fn the_wire_spelling_round_trips() {
        for state in [
            DriveState::SwitchOnDisabled,
            DriveState::SwitchedOn,
            DriveState::OperationEnabled,
            DriveState::Fault,
        ] {
            assert_eq!(DriveState::parse(state.as_str()), Some(state));
        }
        assert_eq!(DriveState::parse("armed"), None);
    }
}
