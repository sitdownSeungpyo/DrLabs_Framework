//! Whether an axis may be driven right now.
//!
//! This is the session's enable-operation / disable-operation / fault state machine, with the button
//! replaced by the `mode` byte of the client's [`drlabs_core::MotorCmd`]:
//!
//! * `OFF` — the axis is observed and not driven.
//! * `DRIVE` — honoured once the axis passes its preflight (inside the
//!   envelope, no hard fault, supply in band). Until then the axis stays off
//!   and the state record says why in `blocked`.
//!
//! An emergency stop is robot-wide. Any bus thread may latch it; every bus
//! thread reads it each tick and stops its axes. It clears only on a client's
//! explicit `FAULT_RESET`, and clearing it does not drive anything new: every
//! axis is back to `OFF` and has to be asked for again — except one its
//! [`EstopPolicy`] is holding, which keeps its place until the client's next
//! command after the reset says what it should do.

use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use std::sync::Arc;

use drlabs_core::bus::AxisReply;
use drlabs_safety::{Blocked, Guard, TripCode};

/// What to do with the axes when the client stops sending commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeoutPolicy {
    /// Keep driving to the last position asked for, with no velocity or
    /// feed-forward. A position servo holds; an impedance controller holds.
    Hold,
    /// Turn every axis off. Right for a machine that is safe limp and wrong
    /// for one that falls.
    Release,
}

impl TimeoutPolicy {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "hold" => Some(TimeoutPolicy::Hold),
            "release" => Some(TimeoutPolicy::Release),
            _ => None,
        }
    }
}

/// What a driven axis does when the robot is stopped.
///
/// The categories are the ones a machine-safety standard would name: 0 is
/// power removed at once, 1 is a controlled stop and then power removed, 2 is
/// a controlled stop with power kept. A joint that carries a load against
/// gravity falls under 0 — the arm on the bench fell 2.76 rad on a trip — so
/// it is the description's choice, not the runtime's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EstopPolicy {
    /// Torque off at once (category 0). The default, and the behaviour
    /// before the policy existed.
    #[default]
    Release,
    /// Decelerate within the joint's acceleration limit and hold that place,
    /// powered (category 2), until a client says otherwise after a reset.
    Hold,
    /// Decelerate to rest, then torque off (category 1).
    StopThenRelease,
}

impl EstopPolicy {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "release" => Some(EstopPolicy::Release),
            "hold" => Some(EstopPolicy::Hold),
            "stop_then_release" => Some(EstopPolicy::StopThenRelease),
            _ => None,
        }
    }

    pub const NAMES: &'static str = "\"release\", \"hold\" or \"stop_then_release\"";
}

/// Whether a stop for `code` may keep an axis powered.
///
/// A stop the operator asked for, a late loop, a joint moving too fast or
/// leaving its travel, another axis gone quiet: holding is the safer of the
/// two answers for a joint that would otherwise fall. A fault in the motor or
/// its supply — current, heat, voltage, a controller fault, a stall — is one
/// that holding makes worse, and a dead transport cannot carry a hold and
/// must not re-energise anything when it comes back. Those release.
pub fn trip_allows_hold(code: TripCode) -> bool {
    matches!(
        code,
        TripCode::Requested
            | TripCode::Deadline
            | TripCode::VelocityRunaway
            | TripCode::PositionEnvelope
            | TripCode::CommLoss
            | TripCode::FollowingError
    )
}

/// The robot-wide emergency stop latch, shared by every bus thread.
///
/// A reset is a robot-wide event too. Every bus thread sees the client's
/// reset flag, but only the first to run finds anything latched; so a clear
/// also bumps a generation and records the sequence number of the command
/// that carried it, and every thread acts on the generation changing rather
/// than on having been the one to clear.
#[derive(Debug, Clone, Default)]
pub struct Estop(Arc<EstopInner>);

#[derive(Debug, Default)]
struct EstopInner {
    code: AtomicU8,
    /// Bumped on every clear.
    generation: AtomicU32,
    /// The command that cleared it last.
    reset_seq: AtomicU32,
}

impl Estop {
    pub fn new() -> Self {
        Self::default()
    }

    /// Latches `code` unless something is already latched. The first reason
    /// is the one kept: it is the one that happened.
    pub fn latch(&self, code: TripCode) -> bool {
        self.0
            .code
            .compare_exchange(0, code as u8, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    pub fn code(&self) -> Option<TripCode> {
        TripCode::from_u8(self.0.code.load(Ordering::Acquire))
    }

    pub fn is_latched(&self) -> bool {
        self.0.code.load(Ordering::Acquire) != 0
    }

    /// Clears the latch unconditionally. For tests and shutdown; a client's
    /// reset goes through [`clear_by`](Self::clear_by).
    pub fn clear(&self) {
        if self.0.code.swap(0, Ordering::AcqRel) != 0 {
            self.0.generation.fetch_add(1, Ordering::AcqRel);
        }
    }

    /// Clears the latch on behalf of the command numbered `seq`, once per
    /// command however many bus threads see it: the first claims the
    /// number, and a thread that arrives later for the same command must
    /// not clear a stop that has latched since. Returns whether it cleared.
    pub fn clear_by(&self, seq: u32) -> bool {
        if self.0.reset_seq.swap(seq, Ordering::AcqRel) == seq {
            return false;
        }
        if self.0.code.load(Ordering::Acquire) == 0 {
            return false;
        }
        // The generation moves before the latch opens, so a thread that sees
        // the latch open (it reads the latch first) also sees the reset that
        // opened it. A non-zero code cannot change under us: `latch` only
        // writes over zero.
        self.0.generation.fetch_add(1, Ordering::AcqRel);
        self.0.code.store(0, Ordering::Release);
        true
    }

    /// The number of clears so far, and the command behind the last one.
    pub fn generation(&self) -> (u32, u32) {
        let generation = self.0.generation.load(Ordering::Acquire);
        (generation, self.0.reset_seq.load(Ordering::Acquire))
    }
}

/// One axis's gate.
#[derive(Debug, Clone, Copy, Default)]
pub struct AxisGate {
    /// Whether the axis is being driven this tick.
    pub driving: bool,
    /// Why a drive request is not honoured, if it is not.
    pub blocked: Blocked,
}

impl AxisGate {
    /// Decides for this tick. `want_drive` is the client's `mode`, `known` the
    /// last readings from the axis, `estop` whether the robot is stopped.
    ///
    /// Starting to drive resets the guard's trip timers, as enabling operation did: time
    /// spent off does not count towards a trip.
    pub fn decide(
        &mut self,
        want_drive: bool,
        known: &AxisReply,
        guard: &mut Guard,
        estop: bool,
    ) -> bool {
        if estop {
            self.driving = false;
            self.blocked = Blocked::Estop;
            return false;
        }
        if !want_drive {
            self.driving = false;
            self.blocked = Blocked::None;
            return false;
        }
        if self.driving {
            self.blocked = Blocked::None;
            return true;
        }
        match guard.preflight(known) {
            Blocked::None => {
                guard.reset();
                self.driving = true;
                self.blocked = Blocked::None;
                true
            }
            why => {
                self.blocked = why;
                false
            }
        }
    }

    /// Forces the axis off: an emergency stop, a reset, a shutdown.
    pub fn release(&mut self) {
        self.driving = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::bus::{AxisFault, AxisStatus};
    use drlabs_core::units::{Amps, Celsius, Rev, RevPerSec, Volts};

    fn healthy() -> AxisReply {
        AxisReply {
            status: AxisStatus::Ok,
            mode: Some(0),
            position: Some(Rev(0.0)),
            velocity: Some(RevPerSec(0.0)),
            voltage: Some(Volts(24.0)),
            current: Some(Amps(0.0)),
            board_temperature: Some(Celsius(30.0)),
            ..AxisReply::SILENT
        }
    }

    #[test]
    fn an_axis_is_off_until_asked_and_passing_preflight() {
        let mut gate = AxisGate::default();
        let mut guard = Guard::default();

        assert!(!gate.decide(false, &healthy(), &mut guard, false));
        assert_eq!(gate.blocked, Blocked::None);

        // Asked, but nothing heard from the axis yet.
        assert!(!gate.decide(true, &AxisReply::SILENT, &mut guard, false));
        assert_eq!(gate.blocked, Blocked::NoTelemetry);

        // Asked, and healthy.
        assert!(gate.decide(true, &healthy(), &mut guard, false));
        assert!(gate.driving);
        assert_eq!(gate.blocked, Blocked::None);

        // Asked off: off at once.
        assert!(!gate.decide(false, &healthy(), &mut guard, false));
        assert!(!gate.driving);
    }

    /// Once driving, the preflight is not re-run every tick: the running
    /// guard is what watches a driven axis, and it allows the trip margin.
    #[test]
    fn a_driving_axis_is_not_re_checked_by_the_preflight() {
        let mut gate = AxisGate::default();
        let mut guard = Guard::default();
        assert!(gate.decide(true, &healthy(), &mut guard, false));

        let faulted = AxisReply {
            fault: Some(AxisFault {
                code: 5,
                hard: true,
            }),
            ..healthy()
        };
        assert!(
            gate.decide(true, &faulted, &mut guard, false),
            "the guard, not the gate, trips on this"
        );
    }

    #[test]
    fn an_estop_overrides_everything_and_names_itself() {
        let mut gate = AxisGate::default();
        let mut guard = Guard::default();
        assert!(gate.decide(true, &healthy(), &mut guard, false));
        assert!(!gate.decide(true, &healthy(), &mut guard, true));
        assert_eq!(gate.blocked, Blocked::Estop);
        assert!(!gate.driving);
    }

    #[test]
    fn the_first_trip_is_the_one_kept() {
        let estop = Estop::new();
        assert!(!estop.is_latched());
        assert!(estop.latch(TripCode::OverCurrent));
        assert!(!estop.latch(TripCode::Stall), "already latched");
        assert_eq!(estop.code(), Some(TripCode::OverCurrent));
        estop.clear();
        assert_eq!(estop.code(), None);
    }

    #[test]
    fn the_estop_policy_is_spelled_as_in_the_config() {
        assert_eq!(EstopPolicy::parse("release"), Some(EstopPolicy::Release));
        assert_eq!(EstopPolicy::parse("hold"), Some(EstopPolicy::Hold));
        assert_eq!(
            EstopPolicy::parse("stop_then_release"),
            Some(EstopPolicy::StopThenRelease)
        );
        assert_eq!(EstopPolicy::parse("brake"), None);
        assert_eq!(EstopPolicy::default(), EstopPolicy::Release);
    }

    /// Holding is allowed for the stops it makes safer, never for a fault
    /// it makes worse.
    #[test]
    fn only_the_stops_holding_helps_may_hold() {
        for code in [
            TripCode::Requested,
            TripCode::Deadline,
            TripCode::VelocityRunaway,
            TripCode::PositionEnvelope,
            TripCode::CommLoss,
            TripCode::FollowingError,
        ] {
            assert!(trip_allows_hold(code), "{code:?}");
        }
        for code in [
            TripCode::ControllerFault,
            TripCode::OverVoltage,
            TripCode::UnderVoltage,
            TripCode::OverCurrent,
            TripCode::BoardOverTemperature,
            TripCode::MotorOverTemperature,
            TripCode::Stall,
            TripCode::Transport,
        ] {
            assert!(!trip_allows_hold(code), "{code:?}");
        }
    }

    #[test]
    fn the_timeout_policy_is_spelled_as_in_the_config() {
        assert_eq!(TimeoutPolicy::parse("hold"), Some(TimeoutPolicy::Hold));
        assert_eq!(
            TimeoutPolicy::parse("release"),
            Some(TimeoutPolicy::Release)
        );
        assert_eq!(TimeoutPolicy::parse("limp"), None);
    }
}
