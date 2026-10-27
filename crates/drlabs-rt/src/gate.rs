//! Whether an axis may be driven right now.
//!
//! This is the session's arm/disarm/e-stop state machine, with the button
//! replaced by the `mode` byte of the client's [`drlabs_core::MotorCmd`]:
//!
//! * `OFF` — the axis is observed and not driven.
//! * `DRIVE` — honoured once the axis passes its preflight (inside the
//!   envelope, no hard fault, supply in band). Until then the axis stays off
//!   and the state record says why in `blocked`.
//!
//! An emergency stop is robot-wide. Any bus thread may latch it; every bus
//! thread reads it each tick and stops its axes. It clears only on a client's
//! explicit `RESET_ESTOP`, and clearing it does not drive anything: every
//! axis is back to `OFF` and has to be asked for again.

use std::sync::atomic::{AtomicU8, Ordering};
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

/// The robot-wide emergency stop latch, shared by every bus thread.
#[derive(Debug, Clone, Default)]
pub struct Estop(Arc<AtomicU8>);

impl Estop {
    pub fn new() -> Self {
        Self::default()
    }

    /// Latches `code` unless something is already latched. The first reason
    /// is the one kept: it is the one that happened.
    pub fn latch(&self, code: TripCode) -> bool {
        self.0
            .compare_exchange(0, code as u8, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    pub fn code(&self) -> Option<TripCode> {
        TripCode::from_u8(self.0.load(Ordering::Acquire))
    }

    pub fn is_latched(&self) -> bool {
        self.0.load(Ordering::Acquire) != 0
    }

    pub fn clear(&self) {
        self.0.store(0, Ordering::Release);
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
    /// Starting to drive resets the guard's trip timers, as arming did: time
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
    fn the_timeout_policy_is_spelled_as_in_the_config() {
        assert_eq!(TimeoutPolicy::parse("hold"), Some(TimeoutPolicy::Hold));
        assert_eq!(
            TimeoutPolicy::parse("release"),
            Some(TimeoutPolicy::Release)
        );
        assert_eq!(TimeoutPolicy::parse("limp"), None);
    }
}
