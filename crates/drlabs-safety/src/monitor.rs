//! Watching telemetry, and deciding when the machine has to stop.

use std::time::Instant;

use drlabs_core::command::{Command, TorqueLimit};
use drlabs_core::link::{BoardStatus, Identity};
use drlabs_core::render::{render, Units};
use drlabs_core::telemetry::Telemetry;
use drlabs_core::units::{Nm, RevPerSec, RevPerSec2};

use crate::config::Config;

/// A latched reason to stop.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Trip {
    /// A stable identifier, suitable for matching on.
    pub code: &'static str,
    /// One sentence stating what happened.
    pub message: String,
    /// Why it happened and what to do about it. The difference between an
    /// operator who reaches for the manual and one who fixes the machine.
    pub detail: String,
}

impl std::fmt::Display for Trip {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for Trip {}

/// Measures how long a condition has held continuously, so a momentary
/// excursion does not latch a stop. Shared with [`crate::Guard`].
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Persistence {
    since: Option<Instant>,
}

impl Persistence {
    /// Whether `condition` has been true without interruption for `seconds`.
    pub(crate) fn held(&mut self, condition: bool, now: Instant, seconds: f64) -> bool {
        if !condition {
            self.since = None;
            return false;
        }
        let since = *self.since.get_or_insert(now);
        now.duration_since(since).as_secs_f64() >= seconds
    }
}

/// What a preflight check found.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Preflight {
    /// Reasons that must be cleared before arming.
    pub blocking: Vec<String>,
    /// Observations the operator should read but may proceed past.
    ///
    /// Keeping these apart from blockers matters: a system that treats every
    /// observation as a blocker teaches people to bypass it.
    pub advisory: Vec<String>,
}

impl Preflight {
    pub fn may_arm(&self) -> bool {
        self.blocking.is_empty()
    }
}

/// Evaluates telemetry against a [`Config`].
///
/// The control loop owns it and is the only thing that calls
/// [`Monitor::evaluate`].
pub struct Monitor {
    pub config: Config,

    /// Renders the positions, speeds and torques that appear in the messages
    /// this monitor produces.
    ///
    /// `None` means motor space, which is right for a session driving one
    /// controller directly; the robot layer supplies a joint renderer so a
    /// limit written in radians is not reported back in revolutions.
    units: Option<Box<dyn Units>>,

    current: Persistence,
    runaway: Persistence,
    stall: Persistence,

    /// The non-fatal observations from the last evaluation.
    pub warnings: Vec<String>,
}

impl Monitor {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            units: None,
            current: Persistence::default(),
            runaway: Persistence::default(),
            stall: Persistence::default(),
            warnings: Vec::new(),
        }
    }

    /// Renders this monitor's messages in the caller's space.
    pub fn with_units(mut self, units: Box<dyn Units>) -> Self {
        self.units = Some(units);
        self
    }

    fn units(&self) -> &dyn Units {
        render(self.units.as_deref())
    }

    /// Clears the trip timers.
    ///
    /// Called on arm and after an emergency stop is cleared, so time spent
    /// stopped does not count towards a trip.
    pub fn reset(&mut self) {
        self.current = Persistence::default();
        self.runaway = Persistence::default();
        self.stall = Persistence::default();
        self.warnings.clear();
    }

    /// The check performed once, before the operator may arm.
    pub fn preflight(
        &self,
        identity: Option<&Identity>,
        telemetry: &Telemetry,
        connected: bool,
    ) -> Preflight {
        let config = &self.config;
        let units = self.units();
        let mut out = Preflight::default();

        if !connected {
            out.blocking.push("Not connected.".into());
        }

        match identity {
            None => out.advisory.push(
                "Board identity could not be read; verify manually what is attached before arming."
                    .into(),
            ),
            Some(identity) => match identity.status {
                BoardStatus::Rejected => out.blocking.push(format!(
                    "Connected board is {:?}, which this build knows to be outside the supported \
                     range.",
                    identity.board
                )),
                BoardStatus::Unknown => out.advisory.push(format!(
                    "Board {:?} was not positively identified; its electrical limits are assumed, \
                     not verified.",
                    identity.board
                )),
                BoardStatus::Supported => {}
            },
        }

        if telemetry.mode.is_none() && telemetry.position.is_none() {
            out.blocking
                .push("No telemetry received from the controller.".into());
            return out;
        }

        match telemetry.voltage {
            None => out
                .advisory
                .push("Bus voltage is not being reported.".into()),
            Some(voltage) if voltage < config.bus_voltage_min => out.blocking.push(format!(
                "Bus voltage {:.1} V is below the configured minimum {:.1} V.",
                voltage.value(),
                config.bus_voltage_min.value()
            )),
            Some(voltage) if voltage > config.bus_voltage_max => out.blocking.push(format!(
                "Bus voltage {:.1} V is above the configured maximum {:.1} V.",
                voltage.value(),
                config.bus_voltage_max.value()
            )),
            Some(_) => {}
        }

        if let Some(fault) = &telemetry.fault {
            if fault.hard {
                out.blocking.push(format!(
                    "Controller reports fault {} ({}). {}",
                    fault.code, fault.name, fault.detail
                ));
            }
        }

        match telemetry.position {
            None => out.blocking.push(
                "Output position is invalid — the encoder or the motor calibration is not usable."
                    .into(),
            ),
            Some(position) if config.position_limit_enabled => {
                // The same margin the running check allows, so the two agree:
                // an axis may be armed wherever it would be allowed to keep
                // running.
                //
                // Without it the arm check is stricter than the trip check, and
                // the gap is not theoretical — a target clamped to the envelope
                // limit is commanded exactly at the bound, and the axis settles
                // a fraction of a degree outside it through ordinary tracking
                // error. That is a position this layer itself asked for, and it
                // left the axis unable to re-arm while the running check would
                // only have warned.
                let margin = config.position_trip_margin;
                if position < config.position_min - margin
                    || position > config.position_max + margin
                {
                    out.blocking.push(format!(
                        "Current position {} is outside the host safety envelope [{}, {}] plus {} \
                         of margin. Widen the envelope, or move the axis back inside — the drive \
                         is off, so the shaft is free.",
                        units.position(position),
                        units.position(config.position_min),
                        units.position(config.position_max),
                        units.span(margin)
                    ));
                }
            }
            Some(_) => {}
        }

        if let Some(temperature) = telemetry.board_temperature {
            if temperature.value() > config.board_temp_warn {
                out.advisory.push(format!(
                    "Board is already warm ({:.0} °C).",
                    temperature.value()
                ));
            }
        }
        if config.max_torque > Nm(2.0) {
            out.advisory.push(format!(
                "Torque limit is set to {}. Start low with an unfamiliar motor.",
                units.torque(config.max_torque)
            ));
        }

        out
    }

    /// Examines one cycle's telemetry and returns a [`Trip`] when the machine
    /// must be stopped. It also refreshes [`Monitor::warnings`].
    ///
    /// The `armed` flag matters more than it looks. Several conditions — being
    /// outside the position envelope in particular — are a reason to refuse
    /// arming but not a reason to latch while disarmed: with the drive off, the
    /// operator's only way to fix it is to move the axis by hand, and tripping
    /// on that would make the fault unrecoverable.
    pub fn evaluate(
        &mut self,
        telemetry: &Telemetry,
        commanded: &Command,
        armed: bool,
        now: Instant,
    ) -> Option<Trip> {
        let config = self.config;
        let units = render(self.units.as_deref());
        let mut warnings = Vec::new();
        let mut trip: Option<Trip> = None;

        macro_rules! set_trip {
            ($code:expr, $message:expr, $detail:expr) => {
                if trip.is_none() {
                    trip = Some(Trip {
                        code: $code,
                        message: $message,
                        detail: $detail,
                    });
                }
            };
        }

        // --- controller-reported problems ---------------------------------
        if let Some(fault) = &telemetry.fault {
            if fault.hard {
                set_trip!(
                    "controller_fault",
                    format!("Controller fault {}: {}", fault.code, fault.name),
                    fault.detail.clone()
                );
            } else if fault.code > 0 {
                // A limiting state, not a fault. It is the single most useful
                // thing to surface when a motor accepts commands but does not
                // move.
                warnings.push(fault.name.clone());
            }
        }

        // --- electrical ----------------------------------------------------
        if let Some(voltage) = telemetry.voltage {
            if voltage > config.bus_voltage_max {
                set_trip!(
                    "over_voltage",
                    format!(
                        "Bus voltage {:.1} V above limit {:.1} V",
                        voltage.value(),
                        config.bus_voltage_max.value()
                    ),
                    "Regeneration with no battery on the bus is the usual cause.".into()
                );
            } else if voltage < config.bus_voltage_min && armed {
                set_trip!(
                    "under_voltage",
                    format!(
                        "Bus voltage {:.1} V below limit {:.1} V",
                        voltage.value(),
                        config.bus_voltage_min.value()
                    ),
                    "The supply is sagging under load.".into()
                );
            } else if voltage.value() > config.bus_voltage_max.value() * 0.95 {
                warnings.push("bus voltage near limit".into());
            }
        }

        if let Some(current) = telemetry.q_current {
            let over = current.abs() > config.current_trip;
            if self.current.held(over, now, config.current_trip_time) {
                set_trip!(
                    "over_current",
                    format!(
                        "Q current {:.1} A above limit {:.1} A",
                        current.value(),
                        config.current_trip.value()
                    ),
                    format!(
                        "Sustained for more than {:.0} ms.",
                        config.current_trip_time * 1e3
                    )
                );
            } else if over {
                warnings.push("current above limit".into());
            }
        }

        // --- thermal -------------------------------------------------------
        if let Some(temperature) = telemetry.board_temperature {
            if temperature.value() > config.board_temp_trip {
                set_trip!(
                    "board_over_temperature",
                    format!(
                        "Board temperature {:.0} °C above limit {:.0} °C",
                        temperature.value(),
                        config.board_temp_trip
                    ),
                    String::new()
                );
            } else if temperature.value() > config.board_temp_warn {
                warnings.push("board temperature high".into());
            }
        }
        if let Some(temperature) = telemetry.motor_temperature {
            if temperature.value() > config.motor_temp_trip {
                set_trip!(
                    "motor_over_temperature",
                    format!(
                        "Motor temperature {:.0} °C above limit {:.0} °C",
                        temperature.value(),
                        config.motor_temp_trip
                    ),
                    String::new()
                );
            } else if temperature.value() > config.motor_temp_warn {
                warnings.push("motor temperature high".into());
            }
        }

        // --- motion --------------------------------------------------------
        if let (Some(position), true) = (telemetry.position, config.position_limit_enabled) {
            let margin = config.position_trip_margin;
            let outside =
                position > config.position_max + margin || position < config.position_min - margin;
            if outside && !armed {
                warnings.push("outside soft position limits".into());
            } else if outside {
                set_trip!(
                    "position_envelope",
                    format!(
                        "Position {} left the soft envelope",
                        units.position(position)
                    ),
                    format!(
                        "Envelope [{}, {}] plus {} margin.",
                        units.position(config.position_min),
                        units.position(config.position_max),
                        units.span(margin)
                    )
                );
            } else if position > config.position_max || position < config.position_min {
                warnings.push("outside soft position limits".into());
            }
        }

        if armed {
            if let Some(velocity) = telemetry.velocity {
                let ceiling = RevPerSec(
                    (config.max_velocity.value() * config.runaway_velocity_factor).max(0.05),
                );
                if self
                    .runaway
                    .held(velocity.abs() > ceiling, now, config.runaway_time)
                {
                    set_trip!(
                        "velocity_runaway",
                        format!(
                            "Velocity {} exceeded {}",
                            units.velocity(velocity),
                            units.velocity(ceiling)
                        ),
                        format!(
                            "Held for {:.1} s. The ceiling is the velocity limit {} × {:.1}. Raise \
                             the velocity ceiling, lower kp, or pick a looser preset if this was \
                             normal overshoot.",
                            config.runaway_time,
                            units.velocity(config.max_velocity),
                            config.runaway_velocity_factor
                        )
                    );
                }
            }

            if config.stall_detect_enabled {
                // A stall is only meaningful against an intent to move: current
                // with no motion is exactly what holding a load looks like.
                let wants_motion = commanded.velocity_target.abs() > config.stall_velocity;
                let stalled = wants_motion
                    && telemetry
                        .velocity
                        .is_some_and(|velocity| velocity.abs() < config.stall_velocity)
                    && telemetry
                        .q_current
                        .is_some_and(|current| current.abs() > config.stall_current);
                if self.stall.held(stalled, now, config.stall_time) {
                    let current = telemetry.q_current.map(|c| c.value()).unwrap_or(0.0);
                    set_trip!(
                        "stall",
                        format!("Stall detected: {current:.1} A with no motion"),
                        format!(
                            "Motion was commanded but the axis moved less than {} while drawing \
                             more than {:.1} A for {:.1} s.",
                            units.velocity(config.stall_velocity),
                            config.stall_current.value(),
                            config.stall_time
                        )
                    );
                }
            }
        }

        self.warnings = warnings;
        trip
    }

    /// Applies every command clamp and returns notes describing what was
    /// reduced.
    ///
    /// This is the last thing between an application's intent and the bus, so
    /// it runs on every cycle rather than only when a setpoint changes: a limit
    /// tightened mid-motion has to take effect immediately, not at the next
    /// user action.
    pub fn clamp_command(&self, mut command: Command) -> (Command, Vec<String>) {
        let config = &self.config;
        let mut notes = Vec::new();

        if let Some(position) = command.position.get() {
            let (clamped, changed) = config.clamp_position(position);
            command.position = clamped.into();
            if changed {
                notes.push("position clamped to the soft envelope".into());
            }
        }
        if let Some(velocity) = command.velocity.get() {
            let (clamped, changed) = config.clamp_velocity(velocity);
            command.velocity = clamped.into();
            if changed {
                notes.push("velocity clamped to the velocity ceiling".into());
            }
        }

        // The torque ceiling is bypassed on purpose when the controller's own
        // limit has been requested; the current, stall and runaway guards are
        // what protect the machine in that mode.
        command.max_torque = match command.max_torque {
            TorqueLimit::Nm(value) => {
                let (clamped, changed) = config.clamp_torque(value);
                if changed {
                    notes.push("torque limit clamped to the torque ceiling".into());
                }
                TorqueLimit::Nm(clamped)
            }
            // A command that forgets to state a limit still gets one.
            TorqueLimit::Unchanged => TorqueLimit::Unchanged,
            TorqueLimit::Controller => TorqueLimit::Controller,
        };

        if let Some(limit) = command.velocity_limit.get() {
            command.velocity_limit = limit.clamp_to(RevPerSec(0.0), config.max_velocity).into();
        }
        if let Some(limit) = command.accel_limit.get() {
            let ceiling = RevPerSec2(config.accel_limit.value().max(0.01));
            command.accel_limit = limit.clamp_to(RevPerSec2(0.01), ceiling).into();
        }
        if let Some(scale) = command.kp_scale.get() {
            command.kp_scale = scale.clamp(0.0, config.max_kp_scale).into();
        }
        if let Some(scale) = command.kd_scale.get() {
            command.kd_scale = scale.clamp(0.0, config.max_kd_scale).into();
        }
        if let Some(scale) = command.ilimit_scale.get() {
            command.ilimit_scale = scale.clamp(0.0, 1.0).into();
        }
        if let Some(torque) = command.feedforward_torque.get() {
            command.feedforward_torque = torque
                .clamp_to(-config.max_torque, config.max_torque)
                .into();
        }
        if let drlabs_core::command::Mode::StayWithin { lower, upper } = command.mode {
            command.mode = drlabs_core::command::Mode::StayWithin {
                lower: config.clamp_position(lower).0,
                upper: config.clamp_position(upper).0,
            };
        }

        (command, notes)
    }
}

/// A convenience for the common case of a monitor with no renderer.
impl Default for Monitor {
    fn default() -> Self {
        Self::new(Config::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::command::{Field, Mode};
    use drlabs_core::telemetry::Fault;
    use drlabs_core::units::{Amps, Celsius, Rev, Volts};
    use std::time::Duration;

    fn healthy() -> Telemetry {
        Telemetry {
            mode: Some(0),
            position: Some(Rev(0.0)),
            velocity: Some(RevPerSec(0.0)),
            voltage: Some(Volts(24.0)),
            q_current: Some(Amps(0.0)),
            board_temperature: Some(Celsius(30.0)),
            ..Default::default()
        }
    }

    fn armed_command() -> Command {
        Command::new(Mode::Position)
    }

    #[test]
    fn a_healthy_axis_does_not_trip() {
        let mut monitor = Monitor::default();
        assert_eq!(
            monitor.evaluate(&healthy(), &armed_command(), true, Instant::now()),
            None
        );
        assert!(monitor.warnings.is_empty());
    }

    /// A momentary excursion is not a trip. The whole point of the timers is
    /// that a machine that spikes and recovers keeps running.
    #[test]
    fn a_brief_over_current_does_not_latch() {
        let mut monitor = Monitor::default();
        let telemetry = Telemetry {
            q_current: Some(Amps(50.0)),
            ..healthy()
        };
        let now = Instant::now();

        assert_eq!(
            monitor.evaluate(&telemetry, &armed_command(), true, now),
            None
        );
        assert!(monitor.warnings.iter().any(|w| w.contains("current")));

        // Still under the trip time.
        let soon = now + Duration::from_millis(200);
        assert_eq!(
            monitor.evaluate(&telemetry, &armed_command(), true, soon),
            None
        );

        // Past it.
        let later = now + Duration::from_millis(500);
        let trip = monitor
            .evaluate(&telemetry, &armed_command(), true, later)
            .expect("a sustained over-current trips");
        assert_eq!(trip.code, "over_current");
    }

    /// And a condition that clears restarts the clock.
    #[test]
    fn a_condition_that_clears_restarts_its_timer() {
        let mut monitor = Monitor::default();
        let now = Instant::now();
        let hot = Telemetry {
            q_current: Some(Amps(50.0)),
            ..healthy()
        };

        monitor.evaluate(&hot, &armed_command(), true, now);
        monitor.evaluate(
            &healthy(),
            &armed_command(),
            true,
            now + Duration::from_millis(300),
        );
        assert_eq!(
            monitor.evaluate(
                &hot,
                &armed_command(),
                true,
                now + Duration::from_millis(500)
            ),
            None,
            "the clock should have restarted"
        );
    }

    /// The distinction that keeps a fault recoverable: with the drive off, the
    /// operator's only way back inside the envelope is to move the shaft by
    /// hand, and tripping on that would make it unrecoverable.
    #[test]
    fn leaving_the_envelope_while_disarmed_warns_rather_than_trips() {
        let mut monitor = Monitor::default();
        let telemetry = Telemetry {
            position: Some(Rev(2.0)),
            ..healthy()
        };

        assert_eq!(
            monitor.evaluate(&telemetry, &armed_command(), false, Instant::now()),
            None
        );
        assert!(monitor
            .warnings
            .iter()
            .any(|w| w.contains("outside soft position limits")));

        let trip = monitor
            .evaluate(&telemetry, &armed_command(), true, Instant::now())
            .expect("armed, it trips");
        assert_eq!(trip.code, "position_envelope");
    }

    /// Inside the margin is a warning, not a trip — ordinary overshoot at the
    /// edge of the range would otherwise trip on every approach.
    #[test]
    fn overshoot_within_the_margin_is_only_a_warning() {
        let mut monitor = Monitor::default(); // ±0.5 rev, 0.05 margin
        let telemetry = Telemetry {
            position: Some(Rev(0.52)),
            ..healthy()
        };
        assert_eq!(
            monitor.evaluate(&telemetry, &armed_command(), true, Instant::now()),
            None
        );
        assert!(monitor
            .warnings
            .iter()
            .any(|w| w.contains("outside soft position limits")));
    }

    /// A stall is current with no motion *while motion was asked for*. Current
    /// with no motion and no command is what holding a load looks like.
    #[test]
    fn holding_a_load_is_not_a_stall() {
        let mut monitor = Monitor::default();
        let telemetry = Telemetry {
            velocity: Some(RevPerSec(0.0)),
            q_current: Some(Amps(10.0)),
            ..healthy()
        };
        let now = Instant::now();

        // No commanded motion: not a stall however long it lasts.
        let holding = Command {
            velocity_target: RevPerSec(0.0),
            ..armed_command()
        };
        monitor.evaluate(&telemetry, &holding, true, now);
        assert_eq!(
            monitor.evaluate(&telemetry, &holding, true, now + Duration::from_secs(5)),
            None
        );

        // Motion asked for, and none happening.
        let moving = Command {
            velocity_target: RevPerSec(1.0),
            ..armed_command()
        };
        monitor.evaluate(&telemetry, &moving, true, now);
        let trip = monitor
            .evaluate(&telemetry, &moving, true, now + Duration::from_secs(3))
            .expect("a sustained stall trips");
        assert_eq!(trip.code, "stall");
    }

    /// A hard fault from the controller is a trip immediately: there is nothing
    /// to wait out.
    #[test]
    fn a_controller_fault_trips_at_once() {
        let mut monitor = Monitor::default();
        let telemetry = Telemetry {
            fault: Some(Fault {
                code: 39,
                name: "Outside limit".into(),
                detail: "Turn the shaft back inside by hand.".into(),
                hard: true,
            }),
            ..healthy()
        };
        let trip = monitor
            .evaluate(&telemetry, &armed_command(), true, Instant::now())
            .expect("a hard fault trips");
        assert_eq!(trip.code, "controller_fault");
        assert!(trip.detail.contains("by hand"));
    }

    /// A limiting code is not a fault. It is the most useful thing to show
    /// somebody whose motor accepts commands and does not move.
    #[test]
    fn a_limiting_state_is_a_warning_not_a_trip() {
        let mut monitor = Monitor::default();
        let telemetry = Telemetry {
            fault: Some(Fault {
                code: 102,
                name: "Limiting: max torque".into(),
                detail: String::new(),
                hard: false,
            }),
            ..healthy()
        };
        assert_eq!(
            monitor.evaluate(&telemetry, &armed_command(), true, Instant::now()),
            None
        );
        assert!(monitor.warnings.iter().any(|w| w.contains("max torque")));
    }

    /// Preflight separates what must be fixed from what should be read.
    #[test]
    fn preflight_keeps_blockers_and_advisories_apart() {
        let monitor = Monitor::default();
        let identity = Identity {
            board: "moteus r4.11".into(),
            status: BoardStatus::Supported,
            ..Identity::new("moteus")
        };

        let result = monitor.preflight(Some(&identity), &healthy(), true);
        assert!(result.may_arm(), "{:?}", result.blocking);

        // An unidentified board is worth saying, and not worth blocking on.
        let unknown = Identity {
            board: "moteus family 0, hwrev 9".into(),
            status: BoardStatus::Unknown,
            ..Identity::new("moteus")
        };
        let result = monitor.preflight(Some(&unknown), &healthy(), true);
        assert!(result.may_arm());
        assert!(result.advisory.iter().any(|a| a.contains("not positively")));
    }

    /// A board this build knows to be outside the supported range is a blocker.
    #[test]
    fn a_rejected_board_blocks_arming() {
        let monitor = Monitor::default();
        let identity = Identity {
            board: "moteus r3".into(),
            status: BoardStatus::Rejected,
            ..Identity::new("moteus")
        };
        let result = monitor.preflight(Some(&identity), &healthy(), true);
        assert!(!result.may_arm());
    }

    /// The asymmetry that stopped an axis re-arming: the arm check has to allow
    /// the same margin the running check does, or the layer's own clamped
    /// target leaves the axis unable to arm.
    #[test]
    fn arming_allows_the_same_margin_the_trip_check_does() {
        let monitor = Monitor::default(); // ±0.5 rev, 0.05 margin
        let just_outside = Telemetry {
            position: Some(Rev(0.52)),
            ..healthy()
        };
        let result = monitor.preflight(None, &just_outside, true);
        assert!(
            result.may_arm(),
            "inside the margin, so it must be armable: {:?}",
            result.blocking
        );

        let well_outside = Telemetry {
            position: Some(Rev(0.8)),
            ..healthy()
        };
        let result = monitor.preflight(None, &well_outside, true);
        assert!(!result.may_arm());
        assert!(result.blocking[0].contains("the shaft is free"));
    }

    #[test]
    fn clamping_reports_what_it_reduced() {
        let monitor = Monitor::default();
        let (command, notes) = monitor.clamp_command(Command {
            position: Field::Set(Rev(5.0)),
            velocity: Field::Set(RevPerSec(20.0)),
            max_torque: TorqueLimit::Nm(Nm(9.0)),
            ..Command::new(Mode::Position)
        });

        assert_eq!(command.position.get(), Some(Rev(0.5)));
        assert_eq!(command.velocity.get(), Some(RevPerSec(2.0)));
        assert_eq!(command.max_torque, TorqueLimit::Nm(Nm(1.0)));
        assert_eq!(notes.len(), 3, "{notes:?}");
    }

    /// A deliberate hand-back to the controller's own limit is not clamped —
    /// clamping it would turn "use your configured limit" into a number.
    #[test]
    fn a_deferred_torque_limit_passes_through_the_clamp() {
        let monitor = Monitor::default();
        let (command, notes) = monitor.clamp_command(Command {
            max_torque: TorqueLimit::Controller,
            ..Command::new(Mode::Position)
        });
        assert_eq!(command.max_torque, TorqueLimit::Controller);
        assert!(notes.is_empty());
    }

    /// A stay-within command's bounds are places on the axis, so they are
    /// clamped like one.
    #[test]
    fn stay_within_bounds_are_clamped_to_the_envelope() {
        let monitor = Monitor::default();
        let (command, _) = monitor.clamp_command(Command::new(Mode::StayWithin {
            lower: Rev(-3.0),
            upper: Rev(3.0),
        }));
        assert_eq!(
            command.mode,
            Mode::StayWithin {
                lower: Rev(-0.5),
                upper: Rev(0.5)
            }
        );
    }

    /// Resetting clears the timers, so time spent stopped does not count
    /// towards a trip when the axis is armed again.
    #[test]
    fn resetting_forgets_how_long_a_condition_has_held() {
        let mut monitor = Monitor::default();
        let now = Instant::now();
        let hot = Telemetry {
            q_current: Some(Amps(50.0)),
            ..healthy()
        };

        monitor.evaluate(&hot, &armed_command(), true, now);
        monitor.reset();
        assert_eq!(
            monitor.evaluate(
                &hot,
                &armed_command(),
                true,
                now + Duration::from_millis(450)
            ),
            None
        );
    }
}
