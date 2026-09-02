//! The operator-declared envelope.

use drlabs_core::link::Envelope;
use drlabs_core::units::{Amps, Nm, Rev, RevPerSec, RevPerSec2, Volts};

/// What the machine is allowed to do.
///
/// Defaults are deliberately timid — half a turn either side of zero, one
/// newton-metre, two revolutions per second — because the first power-up of an
/// unfamiliar motor is exactly when a generous default does damage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    // --- command clamps ---------------------------------------------------
    pub max_torque: Nm,
    pub max_velocity: RevPerSec,
    pub accel_limit: RevPerSec2,
    /// 0..1.
    pub max_kp_scale: f64,
    pub max_kd_scale: f64,

    // --- soft position envelope -------------------------------------------
    pub position_limit_enabled: bool,
    pub position_min: Rev,
    pub position_max: Rev,
    /// How far past the envelope the axis may travel before the stop latches.
    ///
    /// Without it, ordinary overshoot at the edge of the range would trip on
    /// every approach.
    pub position_trip_margin: Rev,

    // --- electrical trips -------------------------------------------------
    pub bus_voltage_min: Volts,
    pub bus_voltage_max: Volts,

    /// Whether the voltage band above was chosen for this machine, or is just
    /// the default nobody picked.
    ///
    /// It decides who wins against the motor's own datasheet. A band somebody
    /// set is a deliberate margin and stands; a default is not a fact about
    /// any particular motor, and deferring to it means a 12 V servo protected
    /// by a number that came from a struct initialiser.
    pub voltage_band_declared: bool,
    pub current_trip: Amps,
    /// Seconds the current must stay over the limit.
    pub current_trip_time: f64,

    // --- thermal trips ----------------------------------------------------
    pub board_temp_warn: f64,
    pub board_temp_trip: f64,
    pub motor_temp_warn: f64,
    pub motor_temp_trip: f64,

    // --- motion trips -----------------------------------------------------
    /// Multiplies `max_velocity` to get the ceiling above which motion counts
    /// as a runaway. A factor rather than an absolute value, so raising the
    /// velocity limit raises the trip with it.
    pub runaway_velocity_factor: f64,
    pub runaway_time: f64,

    pub stall_detect_enabled: bool,
    pub stall_current: Amps,
    /// The speed below which the axis counts as still.
    pub stall_velocity: RevPerSec,
    pub stall_time: f64,

    // --- communication ----------------------------------------------------
    pub max_consecutive_timeouts: u32,

    /// Sets the controller's command timeout as a multiple of the control
    /// period, so it scales with the loop rate instead of needing to be
    /// retuned every time the rate changes.
    pub watchdog_periods: f64,
    /// A floor chosen from measured USB behaviour: a USB-CAN transaction is
    /// well under a millisecond typically but can reach ~20 ms on a loaded
    /// Windows host, and a watchdog below that would fault the controller on an
    /// ordinary scheduling hiccup.
    pub watchdog_min: f64,
    pub watchdog_max: f64,

    /// Stops the axis when the supervising client stops talking to the
    /// session. Zero disables it.
    pub deadman_timeout: f64,

    // --- operator confirmations -------------------------------------------
    /// The move size beyond which a UI should ask for confirmation. Advisory:
    /// the runtime does not enforce it.
    pub large_move: Rev,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            max_torque: Nm(1.00),
            max_velocity: RevPerSec(2.0),
            accel_limit: RevPerSec2(5.0),
            max_kp_scale: 1.0,
            max_kd_scale: 1.0,

            position_limit_enabled: true,
            position_min: Rev(-0.5),         // -180°
            position_max: Rev(0.5),          // +180°
            position_trip_margin: Rev(0.05), // 18° of overshoot

            bus_voltage_min: Volts(11.0),
            bus_voltage_max: Volts(42.0),
            voltage_band_declared: false,
            current_trip: Amps(12.0),
            current_trip_time: 0.40,

            board_temp_warn: 60.0,
            board_temp_trip: 80.0,
            motor_temp_warn: 70.0,
            motor_temp_trip: 100.0,

            runaway_velocity_factor: 2.5,
            runaway_time: 0.50,
            stall_detect_enabled: true,
            stall_current: Amps(8.0),
            stall_velocity: RevPerSec(0.05),
            stall_time: 2.00,

            max_consecutive_timeouts: 5,
            watchdog_periods: 4.0,
            watchdog_min: 0.08,
            watchdog_max: 0.50,
            deadman_timeout: 2.0,

            large_move: Rev(0.25), // 90°
        }
    }
}

impl Config {
    /// Brings the configuration into a self-consistent, physically possible
    /// state and reports every adjustment it made.
    ///
    /// It returns notes rather than silently correcting, because an operator
    /// who asked for a limit and got a different one needs to know — the whole
    /// point of this layer is that the declared envelope is the one in force.
    pub fn normalise(&mut self, envelope: Option<&Envelope>) -> Vec<String> {
        let mut notes = Vec::new();

        if let Some(envelope) = envelope {
            if let Some(max) = envelope.velocity_max {
                if max.value() > 0.0 && self.max_velocity > max {
                    self.max_velocity = max;
                    notes.push(
                        "velocity limit clamped to the controller's documented maximum".into(),
                    );
                }
            }
            if let Some(peak) = envelope.peak_current {
                if peak.value() > 0.0 && self.current_trip > peak {
                    self.current_trip = peak;
                    notes
                        .push("current trip clamped to the controller's peak phase current".into());
                }
            }
            if let Some(min) = envelope.voltage_min {
                if min.value() > 0.0 && self.bus_voltage_min < min {
                    self.bus_voltage_min = min;
                    notes.push("bus voltage minimum raised to the controller's minimum".into());
                }
            }
            if let Some(max) = envelope.voltage_max {
                if max.value() > 0.0 && self.bus_voltage_max > max {
                    self.bus_voltage_max = max;
                    notes.push("bus voltage maximum lowered to the controller's maximum".into());
                }
            }
        }

        self.max_velocity = RevPerSec(self.max_velocity.value().max(0.0));
        self.max_torque = Nm(self.max_torque.value().max(0.0));
        self.accel_limit = RevPerSec2(self.accel_limit.value().max(0.0));
        self.max_kp_scale = self.max_kp_scale.clamp(0.0, 1.0);
        self.max_kd_scale = self.max_kd_scale.clamp(0.0, 1.0);

        if self.bus_voltage_max <= self.bus_voltage_min {
            self.bus_voltage_max = Volts(self.bus_voltage_min.value() + 1.0);
            notes.push("bus voltage maximum raised above the minimum".into());
        }
        if self.position_max < self.position_min {
            std::mem::swap(&mut self.position_min, &mut self.position_max);
            notes.push("position limits were swapped into order".into());
        }

        self.max_consecutive_timeouts = self.max_consecutive_timeouts.max(1);
        self.watchdog_periods = self.watchdog_periods.clamp(2.0, 50.0);
        if self.watchdog_min <= 0.0 {
            self.watchdog_min = 0.08;
        }
        if self.watchdog_max < self.watchdog_min {
            self.watchdog_max = self.watchdog_min;
        }
        if self.deadman_timeout > 0.0 {
            self.deadman_timeout = self.deadman_timeout.clamp(0.5, 30.0);
        }

        notes
    }

    /// Widens the bus voltage trip band when it does not contain the supply the
    /// connected hardware is rated for, and reports what it changed.
    ///
    /// This is the one place the safety layer relaxes a limit rather than
    /// tightening it, and it is deliberately narrow. The band's purpose is to
    /// catch a supply that has sagged or a bus that is regenerating; a band
    /// that excludes the motor's own rated voltage catches neither, it just
    /// trips the instant the machine is powered. A 5 V servo against a band
    /// inherited from a 12 V controller is not a protected machine, it is one
    /// that cannot be armed — and an operator who meets that on every connect
    /// learns to widen the band by hand, past where they would have stopped.
    ///
    /// A band that already contains the rated supply is left exactly as
    /// declared, margins and all.
    pub fn fit_voltage_to(&mut self, envelope: &Envelope) -> Vec<String> {
        let (rated, min, max) = match (
            envelope.voltage_rated,
            envelope.voltage_min,
            envelope.voltage_max,
        ) {
            (Some(rated), Some(min), Some(max))
                if rated.value() > 0.0 && min.value() > 0.0 && max.value() > 0.0 =>
            {
                (rated, min, max)
            }
            _ => return Vec::new(),
        };
        let before = (self.bus_voltage_min, self.bus_voltage_max);

        // Nobody chose the band, so the datasheet is the better authority.
        if !self.voltage_band_declared {
            if before == (min, max) {
                return Vec::new();
            }
            self.bus_voltage_min = min;
            self.bus_voltage_max = max;
            return vec![format!(
                "bus voltage band taken from the hardware's rated range [{}, {}] V, replacing the default [{}, {}] V",
                min.value(),
                max.value(),
                before.0.value(),
                before.1.value(),
            )];
        }

        if rated >= self.bus_voltage_min && rated <= self.bus_voltage_max {
            return Vec::new();
        }

        self.bus_voltage_min = min;
        self.bus_voltage_max = max;

        vec![format!(
            "bus voltage band widened from [{}, {}] V to the hardware's rated range [{}, {}] V, \
             because the declared band excluded this motor's {} V rated supply",
            before.0.value(),
            before.1.value(),
            min.value(),
            max.value(),
            rated.value()
        )]
    }

    /// The command timeout to send the controller for a loop running at the
    /// given period, in seconds.
    pub fn watchdog_timeout(&self, period: f64) -> f64 {
        (self.watchdog_periods * period).clamp(self.watchdog_min, self.watchdog_max)
    }

    /// Constrains a position target to the soft envelope.
    pub fn clamp_position(&self, value: Rev) -> (Rev, bool) {
        if !self.position_limit_enabled {
            return (value, false);
        }
        value.clamp_changed(self.position_min, self.position_max)
    }

    /// Constrains a velocity command to ±`max_velocity`.
    pub fn clamp_velocity(&self, value: RevPerSec) -> (RevPerSec, bool) {
        value.clamp_changed(-self.max_velocity, self.max_velocity)
    }

    /// Constrains a torque limit to the operator's ceiling.
    pub fn clamp_torque(&self, value: Nm) -> (Nm, bool) {
        value.clamp_changed(Nm(0.0), self.max_torque)
    }
}

/// A one-click set of motion-trip sensitivities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    /// First power-up with an unknown motor. Trips early.
    Conservative,
    /// Normal use. Tolerates overshoot and stiff joints.
    Standard,
    /// Only clear faults stop the axis. Watch it closely.
    Relaxed,
}

impl Preset {
    pub fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "conservative" => Preset::Conservative,
            "standard" => Preset::Standard,
            "relaxed" => Preset::Relaxed,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            Preset::Conservative => "conservative",
            Preset::Standard => "standard",
            Preset::Relaxed => "relaxed",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Preset::Conservative => "Conservative",
            Preset::Standard => "Standard",
            Preset::Relaxed => "Relaxed",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Preset::Conservative => "First power-up with an unknown motor. Trips early.",
            Preset::Standard => "Normal use. Tolerates overshoot and stiff joints.",
            Preset::Relaxed => "Only clear faults stop the axis. Watch it closely.",
        }
    }

    /// Tightens or loosens every motion trip together.
    pub fn apply(self, config: &mut Config) {
        let (
            runaway_factor,
            runaway_time,
            stall_current,
            stall_time,
            current,
            current_time,
            board,
            motor,
        ) = match self {
            Preset::Conservative => (1.5, 0.30, 4.0, 1.0, 6.0, 0.20, 70.0, 90.0),
            Preset::Standard => (2.5, 0.50, 8.0, 2.0, 12.0, 0.40, 80.0, 100.0),
            Preset::Relaxed => (4.0, 1.00, 15.0, 4.0, 20.0, 1.00, 90.0, 110.0),
        };
        config.runaway_velocity_factor = runaway_factor;
        config.runaway_time = runaway_time;
        config.stall_current = Amps(stall_current);
        config.stall_time = stall_time;
        config.current_trip = Amps(current);
        config.current_trip_time = current_time;
        config.board_temp_trip = board;
        config.motor_temp_trip = motor;
    }
}

pub const PRESETS: &[Preset] = &[Preset::Conservative, Preset::Standard, Preset::Relaxed];

#[cfg(test)]
mod tests {
    use super::*;

    /// The default is what an unfamiliar motor meets first, and it is meant to
    /// be too small rather than too generous.
    #[test]
    fn the_default_envelope_is_timid() {
        let config = Config::default();
        assert_eq!(config.position_min, Rev(-0.5));
        assert_eq!(config.position_max, Rev(0.5));
        assert_eq!(config.max_torque, Nm(1.0));
        assert_eq!(config.max_velocity, RevPerSec(2.0));
    }

    #[test]
    fn normalising_reports_what_it_changed() {
        let mut config = Config {
            max_velocity: RevPerSec(500.0),
            ..Default::default()
        };
        let envelope = Envelope {
            velocity_max: Some(RevPerSec(4.0)),
            ..Default::default()
        };
        let notes = config.normalise(Some(&envelope));

        assert_eq!(config.max_velocity, RevPerSec(4.0));
        assert_eq!(notes.len(), 1);
        assert!(notes[0].contains("velocity limit clamped"), "{notes:?}");
    }

    /// A limit the operator got right is left alone, and says nothing.
    #[test]
    fn a_consistent_configuration_produces_no_notes() {
        let mut config = Config::default();
        assert!(config.normalise(None).is_empty());
    }

    #[test]
    fn position_limits_written_backwards_are_put_in_order() {
        let mut config = Config {
            position_min: Rev(0.5),
            position_max: Rev(-0.5),
            ..Default::default()
        };
        let notes = config.normalise(None);
        assert_eq!(config.position_min, Rev(-0.5));
        assert_eq!(config.position_max, Rev(0.5));
        assert!(
            notes.iter().any(|note| note.contains("swapped")),
            "{notes:?}"
        );
    }

    /// With nothing declared, the motor's own rated range becomes the band.
    ///
    /// The XL330 case this started with: a healthy servo reporting 4.8 V
    /// against a band inherited from a 12 V bus could not be armed at all.
    #[test]
    fn an_undeclared_band_is_taken_from_the_hardware() {
        let mut config = Config::default(); // 11..42 V, nobody's choice
        let envelope = Envelope {
            voltage_min: Some(Volts(3.7)),
            voltage_max: Some(Volts(6.0)),
            voltage_rated: Some(Volts(5.0)),
            ..Default::default()
        };

        let notes = config.fit_voltage_to(&envelope);
        assert_eq!(config.bus_voltage_min, Volts(3.7));
        assert_eq!(config.bus_voltage_max, Volts(6.0));
        assert_eq!(notes.len(), 1);
        assert!(notes[0].contains("rated range"), "{}", notes[0]);
    }

    /// The MX-64 case, and the reason this rule changed. Its rated 12 V sits
    /// inside the default 11..42 V band, so the old rule left the default in
    /// place and tripped the motor at 10.9 V — a voltage its own e-Manual
    /// permits, the datasheet range being 10.0 to 14.8 V.
    #[test]
    fn a_twelve_volt_motor_is_judged_by_its_datasheet_not_the_default() {
        let mut config = Config::default();
        let envelope = Envelope {
            voltage_min: Some(Volts(10.0)),
            voltage_max: Some(Volts(14.8)),
            voltage_rated: Some(Volts(12.0)),
            ..Default::default()
        };

        let notes = config.fit_voltage_to(&envelope);
        assert_eq!(config.bus_voltage_min, Volts(10.0), "10.9 V must not trip");
        assert_eq!(config.bus_voltage_max, Volts(14.8));
        assert_eq!(notes.len(), 1, "the change is reported: {notes:?}");
    }

    /// A band the description declared is a deliberate margin and stands, even
    /// though it is tighter than the datasheet allows.
    #[test]
    fn a_declared_band_is_left_exactly_as_declared() {
        let mut config = Config {
            bus_voltage_min: Volts(11.5),
            bus_voltage_max: Volts(13.0),
            voltage_band_declared: true,
            ..Default::default()
        };
        let envelope = Envelope {
            voltage_min: Some(Volts(10.0)),
            voltage_max: Some(Volts(14.8)),
            voltage_rated: Some(Volts(12.0)),
            ..Default::default()
        };

        assert!(config.fit_voltage_to(&envelope).is_empty());
        assert_eq!(config.bus_voltage_min, Volts(11.5), "the margin survives");
        assert_eq!(config.bus_voltage_max, Volts(13.0));
    }

    /// Except when it excludes the rated supply outright: such a band protects
    /// nothing and only makes the machine unarmable, so it is widened even
    /// though somebody chose it.
    #[test]
    fn a_declared_band_that_excludes_the_rated_supply_is_still_widened() {
        let mut config = Config {
            bus_voltage_min: Volts(20.0),
            bus_voltage_max: Volts(30.0),
            voltage_band_declared: true,
            ..Default::default()
        };
        let envelope = Envelope {
            voltage_min: Some(Volts(3.7)),
            voltage_max: Some(Volts(6.0)),
            voltage_rated: Some(Volts(5.0)),
            ..Default::default()
        };

        let notes = config.fit_voltage_to(&envelope);
        assert_eq!(config.bus_voltage_min, Volts(3.7));
        assert_eq!(config.bus_voltage_max, Volts(6.0));
        assert!(notes[0].contains("5 V rated supply"), "{}", notes[0]);
    }

    /// A driver that does not know its own rated supply cannot ask for this.
    #[test]
    fn an_envelope_without_a_rated_supply_changes_nothing() {
        let mut config = Config::default();
        let envelope = Envelope {
            voltage_min: Some(Volts(3.7)),
            voltage_max: Some(Volts(6.0)),
            voltage_rated: None,
            ..Default::default()
        };
        assert!(config.fit_voltage_to(&envelope).is_empty());
        assert_eq!(config.bus_voltage_min, Volts(11.0));
    }

    /// The watchdog scales with the loop rate, between a floor and a ceiling.
    /// The floor is what stops an ordinary Windows scheduling hiccup faulting
    /// the controller.
    #[test]
    fn the_watchdog_scales_with_the_period_but_not_past_its_bounds() {
        let config = Config::default(); // 4 periods, 0.08..0.50 s
        assert!((config.watchdog_timeout(0.05) - 0.20).abs() < 1e-9);
        assert_eq!(config.watchdog_timeout(0.001), 0.08, "the floor holds");
        assert_eq!(config.watchdog_timeout(1.0), 0.50, "and the ceiling");
    }

    #[test]
    fn clamping_reports_whether_it_changed_anything() {
        let config = Config::default();
        assert_eq!(config.clamp_position(Rev(0.25)), (Rev(0.25), false));
        assert_eq!(config.clamp_position(Rev(2.0)), (Rev(0.5), true));
        assert_eq!(
            config.clamp_velocity(RevPerSec(-9.0)),
            (RevPerSec(-2.0), true)
        );
        assert_eq!(config.clamp_torque(Nm(5.0)), (Nm(1.0), true));
    }

    /// With the soft envelope off, a position passes through untouched — the
    /// operator has said the mechanism has no bound this layer knows about.
    #[test]
    fn a_disabled_envelope_clamps_nothing() {
        let config = Config {
            position_limit_enabled: false,
            ..Default::default()
        };
        assert_eq!(config.clamp_position(Rev(100.0)), (Rev(100.0), false));
    }

    #[test]
    fn the_presets_move_every_motion_trip_together() {
        let mut conservative = Config::default();
        Preset::Conservative.apply(&mut conservative);
        let mut relaxed = Config::default();
        Preset::Relaxed.apply(&mut relaxed);

        assert!(conservative.runaway_velocity_factor < relaxed.runaway_velocity_factor);
        assert!(conservative.current_trip < relaxed.current_trip);
        assert!(conservative.stall_time < relaxed.stall_time);
        assert!(conservative.board_temp_trip < relaxed.board_temp_trip);
    }

    #[test]
    fn every_preset_can_be_named_and_parsed_back() {
        for preset in PRESETS {
            assert_eq!(Preset::parse(preset.name()), Some(*preset));
            assert!(!preset.description().is_empty());
        }
        assert_eq!(Preset::parse("nonsense"), None);
    }
}
