//! The trip logic of [`crate::Monitor`], without the strings.
//!
//! `Monitor` builds a sentence for every warning and every trip, which is
//! what an operator reading a log wants and what a thread that runs sixteen
//! axes every two milliseconds cannot afford: each sentence is an allocation,
//! and an allocation is a lock shared with every other thread in the process.
//!
//! `Guard` makes the same decisions from the same [`Config`] and reports them
//! as codes. The thresholds, the debounce timers and the armed/disarmed
//! asymmetry are the ones in `Monitor`, and the tests at the bottom are the
//! `Monitor` tests re-run against this type so the two cannot drift apart
//! quietly. The words are put back by whoever reads the code, off the
//! real-time path.

use std::time::Instant;

use drlabs_core::bus::AxisReply;
use drlabs_core::units::RevPerSec;

use crate::config::Config;
use crate::monitor::Persistence;

/// Why the machine was stopped. The low byte of a status record.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripCode {
    ControllerFault = 1,
    OverVoltage = 2,
    UnderVoltage = 3,
    OverCurrent = 4,
    BoardOverTemperature = 5,
    MotorOverTemperature = 6,
    PositionEnvelope = 7,
    VelocityRunaway = 8,
    Stall = 9,
    /// An axis stopped answering for longer than the configured number of
    /// cycles. Raised by the runtime, not by `evaluate`.
    CommLoss = 10,
    /// The bus thread missed its deadline too many times in a row. Raised by
    /// the runtime.
    Deadline = 11,
    /// A client asked for it.
    Requested = 12,
    /// The transport itself failed.
    Transport = 13,
}

impl TripCode {
    /// The same stable identifiers [`crate::Trip::code`] uses.
    pub fn code(self) -> &'static str {
        match self {
            TripCode::ControllerFault => "controller_fault",
            TripCode::OverVoltage => "over_voltage",
            TripCode::UnderVoltage => "under_voltage",
            TripCode::OverCurrent => "over_current",
            TripCode::BoardOverTemperature => "board_over_temperature",
            TripCode::MotorOverTemperature => "motor_over_temperature",
            TripCode::PositionEnvelope => "position_envelope",
            TripCode::VelocityRunaway => "velocity_runaway",
            TripCode::Stall => "stall",
            TripCode::CommLoss => "comm_loss",
            TripCode::Deadline => "deadline",
            TripCode::Requested => "estop_requested",
            TripCode::Transport => "transport",
        }
    }

    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            1 => TripCode::ControllerFault,
            2 => TripCode::OverVoltage,
            3 => TripCode::UnderVoltage,
            4 => TripCode::OverCurrent,
            5 => TripCode::BoardOverTemperature,
            6 => TripCode::MotorOverTemperature,
            7 => TripCode::PositionEnvelope,
            8 => TripCode::VelocityRunaway,
            9 => TripCode::Stall,
            10 => TripCode::CommLoss,
            11 => TripCode::Deadline,
            12 => TripCode::Requested,
            13 => TripCode::Transport,
            _ => return None,
        })
    }
}

/// Why an axis may not be driven yet. The `blocked` byte of a motor state.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Blocked {
    #[default]
    None = 0,
    /// Nothing heard from the controller yet.
    NoTelemetry = 1,
    VoltageLow = 2,
    VoltageHigh = 3,
    ControllerFault = 4,
    /// The controller reports no usable position.
    NoPosition = 5,
    /// Outside the envelope plus margin: move it back by hand, the drive is
    /// off.
    OutsideEnvelope = 6,
    /// An emergency stop is latched robot-wide.
    Estop = 7,
}

impl Blocked {
    pub fn as_str(self) -> &'static str {
        match self {
            Blocked::None => "",
            Blocked::NoTelemetry => "no telemetry received from the controller",
            Blocked::VoltageLow => "bus voltage below the configured minimum",
            Blocked::VoltageHigh => "bus voltage above the configured maximum",
            Blocked::ControllerFault => "controller reports a hard fault",
            Blocked::NoPosition => "output position is invalid",
            Blocked::OutsideEnvelope => "outside the safety envelope; move the axis back by hand",
            Blocked::Estop => "emergency stop is latched",
        }
    }
}

/// The non-fatal observations from the last evaluation, as bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Warnings(pub u16);

impl Warnings {
    /// The controller is in a limiting state (its fault code is non-zero and
    /// not hard).
    pub const LIMITING: Warnings = Warnings(1 << 0);
    pub const VOLTAGE_NEAR_LIMIT: Warnings = Warnings(1 << 1);
    pub const CURRENT_HIGH: Warnings = Warnings(1 << 2);
    pub const BOARD_TEMP_HIGH: Warnings = Warnings(1 << 3);
    pub const MOTOR_TEMP_HIGH: Warnings = Warnings(1 << 4);
    pub const OUTSIDE_SOFT_LIMITS: Warnings = Warnings(1 << 5);

    pub const NONE: Warnings = Warnings(0);

    pub fn contains(self, other: Warnings) -> bool {
        self.0 & other.0 == other.0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    fn set(&mut self, other: Warnings) {
        self.0 |= other.0;
    }
}

/// Evaluates one axis's replies against a [`Config`], allocating nothing.
#[derive(Debug, Clone, Copy)]
pub struct Guard {
    pub config: Config,
    current: Persistence,
    runaway: Persistence,
    stall: Persistence,
    /// The non-fatal observations from the last evaluation.
    pub warnings: Warnings,
}

impl Guard {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            current: Persistence::default(),
            runaway: Persistence::default(),
            stall: Persistence::default(),
            warnings: Warnings::NONE,
        }
    }

    /// Clears the trip timers.
    ///
    /// Called when an axis starts driving and after an emergency stop is
    /// cleared, so time spent stopped does not count towards a trip.
    pub fn reset(&mut self) {
        self.current = Persistence::default();
        self.runaway = Persistence::default();
        self.stall = Persistence::default();
        self.warnings = Warnings::NONE;
    }

    /// The check performed before an axis may be driven: the blocking half
    /// of [`crate::Monitor::preflight`]. The advisories — unidentified
    /// board, warm board, high torque limit — are for an operator to read
    /// and belong in the runtime's log, not here.
    pub fn preflight(&self, reply: &AxisReply) -> Blocked {
        let config = &self.config;

        if reply.mode.is_none() && reply.position.is_none() {
            return Blocked::NoTelemetry;
        }
        if let Some(voltage) = reply.voltage {
            if voltage < config.bus_voltage_min {
                return Blocked::VoltageLow;
            }
            if voltage > config.bus_voltage_max {
                return Blocked::VoltageHigh;
            }
        }
        if reply.fault.is_some_and(|fault| fault.hard) {
            return Blocked::ControllerFault;
        }
        match reply.position {
            None => return Blocked::NoPosition,
            Some(position) if config.position_limit_enabled => {
                // The same margin the running check allows, so the two agree:
                // an axis may be driven wherever it would be allowed to keep
                // running.
                let margin = config.position_trip_margin;
                if position < config.position_min - margin
                    || position > config.position_max + margin
                {
                    return Blocked::OutsideEnvelope;
                }
            }
            Some(_) => {}
        }
        Blocked::None
    }

    /// Examines one cycle's reply and returns the reason to stop, if there
    /// is one. Refreshes [`Guard::warnings`].
    ///
    /// `driving` is [`crate::Monitor::evaluate`]'s `armed`: several
    /// conditions — being outside the position envelope in particular — are
    /// a reason to refuse driving but not a reason to latch while the drive
    /// is off, because with the drive off the operator's only way to fix it
    /// is to move the axis by hand.
    ///
    /// `velocity_target` is the motion the host expects to see, which is what
    /// tells a stall apart from a deliberate hold.
    pub fn evaluate(
        &mut self,
        reply: &AxisReply,
        velocity_target: RevPerSec,
        driving: bool,
        now: Instant,
    ) -> Option<TripCode> {
        let config = self.config;
        let mut warnings = Warnings::NONE;
        let mut trip: Option<TripCode> = None;

        // The first reason found is the one reported, as in `Monitor`.
        let mut set_trip = |code: TripCode| {
            if trip.is_none() {
                trip = Some(code);
            }
        };

        // --- controller-reported problems ---------------------------------
        if let Some(fault) = reply.fault {
            if fault.hard {
                set_trip(TripCode::ControllerFault);
            } else if fault.code > 0 {
                warnings.set(Warnings::LIMITING);
            }
        }

        // --- electrical ----------------------------------------------------
        if let Some(voltage) = reply.voltage {
            if voltage > config.bus_voltage_max {
                set_trip(TripCode::OverVoltage);
            } else if voltage < config.bus_voltage_min && driving {
                set_trip(TripCode::UnderVoltage);
            } else if voltage.value() > config.bus_voltage_max.value() * 0.95 {
                warnings.set(Warnings::VOLTAGE_NEAR_LIMIT);
            }
        }

        if let Some(current) = reply.current {
            let over = current.abs() > config.current_trip;
            if self.current.held(over, now, config.current_trip_time) {
                set_trip(TripCode::OverCurrent);
            } else if over {
                warnings.set(Warnings::CURRENT_HIGH);
            }
        }

        // --- thermal -------------------------------------------------------
        if let Some(temperature) = reply.board_temperature {
            if temperature.value() > config.board_temp_trip {
                set_trip(TripCode::BoardOverTemperature);
            } else if temperature.value() > config.board_temp_warn {
                warnings.set(Warnings::BOARD_TEMP_HIGH);
            }
        }
        if let Some(temperature) = reply.motor_temperature {
            if temperature.value() > config.motor_temp_trip {
                set_trip(TripCode::MotorOverTemperature);
            } else if temperature.value() > config.motor_temp_warn {
                warnings.set(Warnings::MOTOR_TEMP_HIGH);
            }
        }

        // --- motion --------------------------------------------------------
        if let (Some(position), true) = (reply.position, config.position_limit_enabled) {
            let margin = config.position_trip_margin;
            let outside =
                position > config.position_max + margin || position < config.position_min - margin;
            if outside && !driving {
                warnings.set(Warnings::OUTSIDE_SOFT_LIMITS);
            } else if outside {
                set_trip(TripCode::PositionEnvelope);
            } else if position > config.position_max || position < config.position_min {
                warnings.set(Warnings::OUTSIDE_SOFT_LIMITS);
            }
        }

        if driving {
            if let Some(velocity) = reply.velocity {
                let ceiling = RevPerSec(
                    (config.max_velocity.value() * config.runaway_velocity_factor).max(0.05),
                );
                if self
                    .runaway
                    .held(velocity.abs() > ceiling, now, config.runaway_time)
                {
                    set_trip(TripCode::VelocityRunaway);
                }
            }

            if config.stall_detect_enabled {
                // A stall is only meaningful against an intent to move: current
                // with no motion is exactly what holding a load looks like.
                let wants_motion = velocity_target.abs() > config.stall_velocity;
                let stalled = wants_motion
                    && reply
                        .velocity
                        .is_some_and(|velocity| velocity.abs() < config.stall_velocity)
                    && reply
                        .current
                        .is_some_and(|current| current.abs() > config.stall_current);
                if self.stall.held(stalled, now, config.stall_time) {
                    set_trip(TripCode::Stall);
                }
            }
        }

        self.warnings = warnings;
        trip
    }
}

impl Default for Guard {
    fn default() -> Self {
        Self::new(Config::default())
    }
}

#[cfg(test)]
mod tests {
    //! The `Monitor` tests, against `Guard`. Same inputs, same verdicts.

    use super::*;
    use drlabs_core::bus::{AxisFault, AxisStatus};
    use drlabs_core::units::{Amps, Celsius, Rev, Volts};
    use std::time::Duration;

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

    const STILL: RevPerSec = RevPerSec(0.0);

    #[test]
    fn a_healthy_axis_does_not_trip() {
        let mut guard = Guard::default();
        assert_eq!(
            guard.evaluate(&healthy(), STILL, true, Instant::now()),
            None
        );
        assert!(guard.warnings.is_empty());
    }

    /// A momentary excursion is not a trip. The whole point of the timers is
    /// that a machine that spikes and recovers keeps running.
    #[test]
    fn a_brief_over_current_does_not_latch() {
        let mut guard = Guard::default();
        let reply = AxisReply {
            current: Some(Amps(50.0)),
            ..healthy()
        };
        let now = Instant::now();

        assert_eq!(guard.evaluate(&reply, STILL, true, now), None);
        assert!(guard.warnings.contains(Warnings::CURRENT_HIGH));

        let soon = now + Duration::from_millis(200);
        assert_eq!(guard.evaluate(&reply, STILL, true, soon), None);

        let later = now + Duration::from_millis(500);
        assert_eq!(
            guard.evaluate(&reply, STILL, true, later),
            Some(TripCode::OverCurrent)
        );
    }

    /// And a condition that clears restarts the clock.
    #[test]
    fn a_condition_that_clears_restarts_its_timer() {
        let mut guard = Guard::default();
        let now = Instant::now();
        let hot = AxisReply {
            current: Some(Amps(50.0)),
            ..healthy()
        };

        guard.evaluate(&hot, STILL, true, now);
        guard.evaluate(&healthy(), STILL, true, now + Duration::from_millis(300));
        assert_eq!(
            guard.evaluate(&hot, STILL, true, now + Duration::from_millis(500)),
            None,
            "the clock should have restarted"
        );
    }

    /// The distinction that keeps a fault recoverable: with the drive off, the
    /// operator's only way back inside the envelope is to move the shaft by
    /// hand, and tripping on that would make it unrecoverable.
    #[test]
    fn leaving_the_envelope_while_not_driving_warns_rather_than_trips() {
        let mut guard = Guard::default();
        let reply = AxisReply {
            position: Some(Rev(2.0)),
            ..healthy()
        };

        assert_eq!(guard.evaluate(&reply, STILL, false, Instant::now()), None);
        assert!(guard.warnings.contains(Warnings::OUTSIDE_SOFT_LIMITS));

        assert_eq!(
            guard.evaluate(&reply, STILL, true, Instant::now()),
            Some(TripCode::PositionEnvelope)
        );
    }

    /// Inside the margin is a warning, not a trip — ordinary overshoot at the
    /// edge of the range would otherwise trip on every approach.
    #[test]
    fn overshoot_within_the_margin_is_only_a_warning() {
        let mut guard = Guard::default(); // ±0.5 rev, 0.05 margin
        let reply = AxisReply {
            position: Some(Rev(0.52)),
            ..healthy()
        };
        assert_eq!(guard.evaluate(&reply, STILL, true, Instant::now()), None);
        assert!(guard.warnings.contains(Warnings::OUTSIDE_SOFT_LIMITS));
    }

    /// A stall is current with no motion *while motion was asked for*. Current
    /// with no motion and no command is what holding a load looks like.
    #[test]
    fn holding_a_load_is_not_a_stall() {
        let mut guard = Guard::default();
        let reply = AxisReply {
            velocity: Some(RevPerSec(0.0)),
            current: Some(Amps(10.0)),
            ..healthy()
        };
        let now = Instant::now();

        guard.evaluate(&reply, STILL, true, now);
        assert_eq!(
            guard.evaluate(&reply, STILL, true, now + Duration::from_secs(5)),
            None
        );

        let moving = RevPerSec(1.0);
        guard.evaluate(&reply, moving, true, now);
        assert_eq!(
            guard.evaluate(&reply, moving, true, now + Duration::from_secs(3)),
            Some(TripCode::Stall)
        );
    }

    /// A hard fault from the controller is a trip immediately: there is nothing
    /// to wait out.
    #[test]
    fn a_controller_fault_trips_at_once() {
        let mut guard = Guard::default();
        let reply = AxisReply {
            fault: Some(AxisFault {
                code: 39,
                hard: true,
            }),
            ..healthy()
        };
        assert_eq!(
            guard.evaluate(&reply, STILL, true, Instant::now()),
            Some(TripCode::ControllerFault)
        );
    }

    /// A limiting code is not a fault. It is the most useful thing to show
    /// somebody whose motor accepts commands and does not move.
    #[test]
    fn a_limiting_state_is_a_warning_not_a_trip() {
        let mut guard = Guard::default();
        let reply = AxisReply {
            fault: Some(AxisFault {
                code: 102,
                hard: false,
            }),
            ..healthy()
        };
        assert_eq!(guard.evaluate(&reply, STILL, true, Instant::now()), None);
        assert!(guard.warnings.contains(Warnings::LIMITING));
    }

    /// Resetting clears the timers, so time spent stopped does not count
    /// towards a trip when the axis is driven again.
    #[test]
    fn resetting_forgets_how_long_a_condition_has_held() {
        let mut guard = Guard::default();
        let now = Instant::now();
        let hot = AxisReply {
            current: Some(Amps(50.0)),
            ..healthy()
        };

        guard.evaluate(&hot, STILL, true, now);
        guard.reset();
        assert_eq!(
            guard.evaluate(&hot, STILL, true, now + Duration::from_millis(450)),
            None
        );
    }

    /// The asymmetry that stopped an axis re-arming: the preflight has to
    /// allow the same margin the trip check does, or the layer's own clamped
    /// target leaves the axis unable to drive.
    #[test]
    fn preflight_allows_the_same_margin_the_trip_check_does() {
        let guard = Guard::default(); // ±0.5 rev, 0.05 margin
        let just_outside = AxisReply {
            position: Some(Rev(0.52)),
            ..healthy()
        };
        assert_eq!(guard.preflight(&just_outside), Blocked::None);

        let well_outside = AxisReply {
            position: Some(Rev(0.8)),
            ..healthy()
        };
        assert_eq!(guard.preflight(&well_outside), Blocked::OutsideEnvelope);
    }

    #[test]
    fn preflight_names_the_first_blocker() {
        let guard = Guard::default();
        assert_eq!(guard.preflight(&AxisReply::SILENT), Blocked::NoTelemetry);

        let low = AxisReply {
            voltage: Some(Volts(3.0)),
            ..healthy()
        };
        assert_eq!(guard.preflight(&low), Blocked::VoltageLow);

        let faulted = AxisReply {
            fault: Some(AxisFault {
                code: 1,
                hard: true,
            }),
            ..healthy()
        };
        assert_eq!(guard.preflight(&faulted), Blocked::ControllerFault);

        let blind = AxisReply {
            position: None,
            ..healthy()
        };
        assert_eq!(guard.preflight(&blind), Blocked::NoPosition);
        assert_eq!(guard.preflight(&healthy()), Blocked::None);
    }

    #[test]
    fn trip_codes_round_trip_through_a_byte() {
        for value in 1..=13u8 {
            let code = TripCode::from_u8(value).expect("every value in range is a code");
            assert_eq!(code as u8, value);
            assert!(!code.code().is_empty());
        }
        assert_eq!(TripCode::from_u8(0), None);
    }
}
