//! One control cycle's worth of intent.
//!
//! The Go version carried every optional field as an `f64` that was NaN when
//! it should not be sent, which collapsed two different things into one
//! representation and needed a paragraph of documentation to keep them apart:
//!
//! > "send NaN" and "do not send" cannot share a representation. If they did,
//! > a velocity command would omit the position register, the controller would
//! > keep the previous position target, and the axis would servo back to it
//! > instead of running at the commanded velocity.
//!
//! Here they are different shapes. [`Field::Unchanged`] is not sent;
//! [`TorqueLimit::Controller`] is sent, as the protocol's own sentinel. The
//! trap is gone because the wrong state cannot be written down.

use crate::units::{Nm, Rev, RevPerSec, RevPerSec2};

/// What a driver should do with a sticky command register this cycle.
///
/// Command registers keep their value until they are written again, so
/// "leave it alone" is a real instruction and not the absence of one.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Field<T> {
    /// Do not transmit this register. Whatever was last written stays.
    #[default]
    Unchanged,
    /// Transmit this value.
    Set(T),
}

impl<T: Copy> Field<T> {
    #[inline]
    pub fn get(self) -> Option<T> {
        match self {
            Field::Unchanged => None,
            Field::Set(value) => Some(value),
        }
    }

    #[inline]
    pub fn is_set(self) -> bool {
        matches!(self, Field::Set(_))
    }

    #[inline]
    pub fn or(self, fallback: T) -> T {
        self.get().unwrap_or(fallback)
    }
}

impl<T> From<T> for Field<T> {
    fn from(value: T) -> Self {
        Field::Set(value)
    }
}

/// The three things a command can say about the torque ceiling.
///
/// This is the distinction `docs/MOTEUS.md` spends a section on. A number is a
/// number; handing control back to the controller is a *transmitted* NaN; and
/// saying nothing leaves whatever limit was last written in force. They are
/// three different frames on the wire, so they are three variants here.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum TorqueLimit {
    /// Not transmitted. The controller keeps the ceiling it already had.
    #[default]
    Unchanged,
    /// Transmitted as a value.
    Nm(Nm),
    /// Transmitted as the protocol's NaN sentinel: the controller applies its
    /// own configured limit, and the host ceiling stops applying until this is
    /// taken back.
    Controller,
}

impl TorqueLimit {
    /// The number, when there is one. `Controller` has no number by design —
    /// clamping it would turn "use your own limit" into "command zero torque",
    /// which is the failure the Go version guarded against by hand.
    #[inline]
    pub fn nm(self) -> Option<Nm> {
        match self {
            TorqueLimit::Nm(value) => Some(value),
            _ => None,
        }
    }

    /// Applies a ceiling, leaving the other two variants alone.
    #[inline]
    pub fn clamped_to(self, ceiling: Nm) -> Self {
        match self {
            TorqueLimit::Nm(value) if value > ceiling => TorqueLimit::Nm(ceiling),
            other => other,
        }
    }
}

/// The vendor-neutral control intent for one cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    /// Release the bridge: no torque, no holding.
    Stop,
    /// Read telemetry without changing the controller's mode.
    Query,
    /// Drive to a position with the configured limits.
    Position,
    /// Spin at a velocity, with no position target.
    Velocity,
    /// Command a feedforward torque directly.
    Torque,
    /// Servo velocity to zero without holding a position target.
    Hold,
    /// Short the phases for a passive, regenerative stop.
    Brake,
    /// Apply torque only outside a pair of bounds.
    StayWithin,
    /// Re-reference the output position. Runtime only: it must not touch the
    /// controller's stored configuration.
    SetOutput,
}

impl ControlMode {
    pub fn as_str(self) -> &'static str {
        match self {
            ControlMode::Stop => "stop",
            ControlMode::Query => "query",
            ControlMode::Position => "position",
            ControlMode::Velocity => "velocity",
            ControlMode::Torque => "torque",
            ControlMode::Hold => "hold",
            ControlMode::Brake => "brake",
            ControlMode::StayWithin => "within",
            ControlMode::SetOutput => "set_output",
        }
    }

    pub fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "stop" => ControlMode::Stop,
            "query" => ControlMode::Query,
            "position" => ControlMode::Position,
            "velocity" => ControlMode::Velocity,
            "torque" => ControlMode::Torque,
            "hold" => ControlMode::Hold,
            "brake" => ControlMode::Brake,
            "within" => ControlMode::StayWithin,
            "set_output" => ControlMode::SetOutput,
            _ => return None,
        })
    }
}

impl std::fmt::Display for ControlMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One cycle's command, in motor space.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Command {
    pub mode: Mode,

    pub position: Field<Rev>,
    pub velocity: Field<RevPerSec>,

    pub feedforward_torque: Field<Nm>,
    pub max_torque: TorqueLimit,
    pub stop_position: Field<Rev>,

    /// The controller stops on its own if no command arrives within this.
    pub watchdog: Field<f64>,

    pub velocity_limit: Field<RevPerSec>,
    pub accel_limit: Field<RevPerSec2>,

    pub kp_scale: Field<f64>,
    pub kd_scale: Field<f64>,
    pub ilimit_scale: Field<f64>,

    /// Bypass the controller's own stored travel limits, for this command only.
    pub ignore_position_bounds: bool,

    /// Send the complete parameter set this cycle rather than only what
    /// changed. Drivers with sticky registers use it to re-assert limits.
    pub full: bool,

    /// The motion the host expects to see. Not transmitted: the safety monitor
    /// uses it to tell a stall apart from a deliberate hold.
    pub velocity_target: RevPerSec,
}

/// The mode, plus the arguments that only make sense in one of them.
///
/// StayWithin's bounds used to be two more always-present fields that meant
/// nothing in eight of the nine modes. Attaching them to the variant that uses
/// them means a command in the wrong mode cannot carry them at all.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Mode {
    #[default]
    Stop,
    Query,
    Position,
    Velocity,
    Torque,
    Hold,
    Brake,
    StayWithin {
        lower: Rev,
        upper: Rev,
    },
    SetOutput {
        position: Rev,
    },
}

impl Mode {
    pub fn control(self) -> ControlMode {
        match self {
            Mode::Stop => ControlMode::Stop,
            Mode::Query => ControlMode::Query,
            Mode::Position => ControlMode::Position,
            Mode::Velocity => ControlMode::Velocity,
            Mode::Torque => ControlMode::Torque,
            Mode::Hold => ControlMode::Hold,
            Mode::Brake => ControlMode::Brake,
            Mode::StayWithin { .. } => ControlMode::StayWithin,
            Mode::SetOutput { .. } => ControlMode::SetOutput,
        }
    }
}

impl Command {
    /// A command in the given mode with every optional parameter unset.
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            ..Default::default()
        }
    }

    pub fn control_mode(&self) -> ControlMode {
        self.mode.control()
    }
}

/// Which telemetry a cycle asks for.
///
/// Asking for everything at full precision every period is what stops a loop
/// reaching its rate: a command shares its frame with the query it rides on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QueryProfile {
    /// The minimum needed to control and to trip safely.
    #[default]
    Fast,
    /// Adds the diagnostic registers.
    Full,
    /// Adds the controller's live command registers, so the host can check
    /// that the limits it believes it set are the ones in force.
    Verify,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The distinction the wire makes, made in the type system.
    #[test]
    fn a_deferred_torque_limit_is_not_an_absent_one() {
        assert_eq!(TorqueLimit::Unchanged.nm(), None);
        assert_eq!(TorqueLimit::Controller.nm(), None);
        assert_eq!(TorqueLimit::Nm(Nm(0.5)).nm(), Some(Nm(0.5)));

        // And they are not equal to each other, which is the whole point.
        assert_ne!(TorqueLimit::Unchanged, TorqueLimit::Controller);
    }

    /// Clamping a deferred limit must not turn it into zero torque.
    #[test]
    fn clamping_leaves_a_handed_back_ceiling_alone() {
        assert_eq!(
            TorqueLimit::Controller.clamped_to(Nm(0.3)),
            TorqueLimit::Controller
        );
        assert_eq!(
            TorqueLimit::Unchanged.clamped_to(Nm(0.3)),
            TorqueLimit::Unchanged
        );
        assert_eq!(
            TorqueLimit::Nm(Nm(2.0)).clamped_to(Nm(0.3)),
            TorqueLimit::Nm(Nm(0.3))
        );
        assert_eq!(
            TorqueLimit::Nm(Nm(0.1)).clamped_to(Nm(0.3)),
            TorqueLimit::Nm(Nm(0.1))
        );
    }

    #[test]
    fn an_unset_field_is_not_a_zero_one() {
        let unset: Field<Rev> = Field::Unchanged;
        let zero = Field::Set(Rev::ZERO);
        assert_ne!(unset, zero);
        assert_eq!(unset.get(), None);
        assert_eq!(zero.get(), Some(Rev::ZERO));
    }

    #[test]
    fn stay_within_carries_its_bounds_and_nothing_else_can() {
        let command = Command::new(Mode::StayWithin {
            lower: Rev(-0.1),
            upper: Rev(0.4),
        });
        assert_eq!(command.control_mode(), ControlMode::StayWithin);
        match command.mode {
            Mode::StayWithin { lower, upper } => {
                assert_eq!(lower, Rev(-0.1));
                assert_eq!(upper, Rev(0.4));
            }
            other => panic!("mode became {other:?}"),
        }
    }

    #[test]
    fn mode_names_round_trip() {
        for mode in [
            ControlMode::Stop,
            ControlMode::Query,
            ControlMode::Position,
            ControlMode::Velocity,
            ControlMode::Torque,
            ControlMode::Hold,
            ControlMode::Brake,
            ControlMode::StayWithin,
            ControlMode::SetOutput,
        ] {
            assert_eq!(ControlMode::parse(mode.as_str()), Some(mode));
        }
        assert_eq!(ControlMode::parse("nonsense"), None);
    }
}
