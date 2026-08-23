//! A motor that is not there.
//!
//! One simulator, not one per driver. The Go tree had a `sim.go` in the moteus
//! package and another in the robstride one; the physics in both was the same
//! rotor, the same friction and the same inner loop, differing only in what
//! they called their modes. Two copies of an integrator is two places for it to
//! drift.
//!
//! What it is for: running the whole stack — state machine, control loop,
//! safety monitor, driver surface — with nothing plugged in. It is how the
//! session and robot layers get exercised in a test that does not need a bench,
//! and how somebody new to the framework sees an axis move before they own one.
//!
//! What it is not: a model of any real controller. The gains are plausible and
//! the thermal model is a first-order fit with no basis beyond looking right on
//! a plot. Nothing about a machine should be concluded from it.

use std::time::Instant;

use drlabs_core::command::{Command, ControlMode, Mode, QueryProfile, TorqueLimit};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity, Link};
use drlabs_core::telemetry::{Fault, Telemetry};
use drlabs_core::units::{Amps, Celsius, Nm, Rev, RevPerSec, RevPerSec2, Volts};

/// What the simulated axis is made of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Physics {
    /// kg·m², rotor plus load.
    pub inertia: f64,
    /// Nm per rev/s.
    pub friction: f64,
    /// Nm/A.
    pub torque_constant: f64,
    /// The integrator's step. Small enough that a 500 Hz control loop is not
    /// integrating its own period.
    pub substep_seconds: f64,
    /// What the firmware's own torque limit would be, for a command that hands
    /// the ceiling back to the controller.
    pub default_torque: Nm,
    pub supply: Volts,
}

impl Default for Physics {
    fn default() -> Self {
        Self {
            inertia: 0.006,
            friction: 0.02,
            torque_constant: 0.06,
            substep_seconds: 0.0005, // 2 kHz
            default_torque: Nm(1.0),
            supply: Volts(24.0),
        }
    }
}

/// The controller's own loop, as the simulator models it.
///
/// Deliberately separate from the physics: this is the profile generator and
/// the position loop a real controller runs at its own rate, and the axis
/// follows it rather than the command.
#[derive(Debug, Clone, Copy, Default)]
struct ControlState {
    position: f64,
    velocity: f64,
}

/// The sticky command registers, which a simulated controller keeps like a real
/// one — so a session that stops re-sending them still gets the last values.
#[derive(Debug, Clone, Copy)]
struct SimParams {
    max_torque: Option<f64>,
    velocity_limit: f64,
    accel_limit: f64,
    kp_scale: f64,
    kd_scale: f64,
    watchdog: f64,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            max_torque: None,
            velocity_limit: 1.0,
            accel_limit: 5.0,
            kp_scale: 1.0,
            kd_scale: 1.0,
            watchdog: 0.1,
        }
    }
}

/// A simulated axis behind the ordinary [`Link`] surface.
pub struct SimLink {
    id: u8,
    kind: &'static str,
    pub physics: Physics,

    position: f64,
    velocity: f64,
    control: ControlState,

    board_temp: f64,
    motor_temp: f64,
    voltage: f64,
    mode: ControlMode,

    params: SimParams,
    last: Option<Instant>,
    open: bool,
    elapsed: f64,
}

impl SimLink {
    pub fn new(id: u8) -> Self {
        Self::with_physics(id, "sim", Physics::default())
    }

    /// A simulator that reports itself as a particular family, so a config that
    /// names `moteus-sim` gets something that says "moteus" in its status.
    pub fn with_physics(id: u8, kind: &'static str, physics: Physics) -> Self {
        Self {
            id,
            kind,
            board_temp: 24.0,
            motor_temp: 24.0,
            voltage: physics.supply.value(),
            physics,
            position: 0.0,
            velocity: 0.0,
            control: ControlState::default(),
            mode: ControlMode::Stop,
            params: SimParams::default(),
            last: None,
            open: false,
            elapsed: 0.0,
        }
    }

    /// Places the axis somewhere, for a test that needs it to start away from
    /// zero.
    pub fn at(mut self, position: Rev) -> Self {
        self.position = position.value();
        self.control.position = position.value();
        self
    }

    pub fn position(&self) -> Rev {
        Rev(self.position)
    }

    /// Records the sticky registers a command carries.
    fn apply_params(&mut self, command: &Command) {
        match command.max_torque {
            TorqueLimit::Nm(value) => self.params.max_torque = Some(value.value()),
            // The controller's own limit, which is the firmware default here.
            TorqueLimit::Controller => self.params.max_torque = None,
            TorqueLimit::Unchanged => {}
        }
        if let Some(limit) = command.velocity_limit.get() {
            self.params.velocity_limit = limit.value();
        }
        if let Some(limit) = command.accel_limit.get() {
            self.params.accel_limit = limit.value();
        }
        if let Some(scale) = command.kp_scale.get() {
            self.params.kp_scale = scale;
        }
        if let Some(scale) = command.kd_scale.get() {
            self.params.kd_scale = scale;
        }
        if let Some(watchdog) = command.watchdog.get() {
            self.params.watchdog = watchdog;
        }
    }

    /// One integration step, returning the torque the controller asked for.
    fn step(&mut self, command: &Command, max_torque: f64, h: f64) -> f64 {
        let torque = match command.mode {
            Mode::Position | Mode::Velocity | Mode::Torque | Mode::StayWithin { .. } => {
                self.mode = if matches!(command.mode, Mode::StayWithin { .. }) {
                    ControlMode::StayWithin
                } else {
                    ControlMode::Position
                };

                let velocity_limit = self.params.velocity_limit.max(0.001);
                let accel_limit = self.params.accel_limit.max(0.01);
                let target_velocity = command.velocity.get().map(|v| v.value()).unwrap_or(0.0);

                let desired = match command.position.get() {
                    // No position target: hold the commanded speed. This is how
                    // velocity mode is expressed on the real wire too.
                    None => target_velocity.clamp(-velocity_limit, velocity_limit),
                    Some(target) => {
                        let error = target.value() - self.control.position;
                        // The fastest we may travel and still brake to a stop
                        // exactly on the target under the acceleration limit.
                        let stopping = (2.0 * accel_limit * error.abs()).max(0.0).sqrt();
                        if error.abs() < 1e-4 {
                            0.0
                        } else {
                            velocity_limit.min(stopping).copysign(error)
                        }
                    }
                };

                let step = accel_limit * h;
                self.control.velocity += (desired - self.control.velocity).clamp(-step, step);
                self.control.position += self.control.velocity * h;

                let kp = 120.0 * self.params.kp_scale;
                let kd = 6.0 * self.params.kd_scale;
                let mut torque = kp * (self.control.position - self.position)
                    + kd * (self.control.velocity - self.velocity);
                if let Some(feedforward) = command.feedforward_torque.get() {
                    torque += feedforward.value();
                }
                torque.clamp(-max_torque, max_torque)
            }
            Mode::Hold => {
                self.mode = ControlMode::Hold;
                self.control.position = self.position;
                self.control.velocity = 0.0;
                (-6.0 * self.velocity).clamp(-max_torque, max_torque)
            }
            Mode::Brake => {
                self.mode = ControlMode::Brake;
                self.control.position = self.position;
                self.control.velocity = 0.0;
                -0.2 * self.velocity
            }
            _ => {
                self.mode = ControlMode::Stop;
                self.control.position = self.position;
                self.control.velocity = 0.0;
                0.0
            }
        };

        let accel = (torque - self.physics.friction * self.velocity) / self.physics.inertia;
        self.velocity += accel * h;
        self.position += self.velocity * h;
        torque
    }

    fn telemetry(&self, torque: f64, current: f64, profile: QueryProfile) -> Telemetry {
        let mut telemetry = Telemetry {
            mode: Some(self.mode as i32),
            mode_name: self.mode.as_str().to_string(),
            position: Some(Rev(self.position)),
            velocity: Some(RevPerSec(self.velocity)),
            torque: Some(Nm(torque)),
            q_current: Some(Amps(current)),
            voltage: Some(Volts(self.voltage)),
            board_temperature: Some(Celsius(self.board_temp)),
            fault: Some(Fault::none()),
            ..Default::default()
        };

        if profile != QueryProfile::Fast {
            telemetry.motor_temperature = Some(Celsius(self.motor_temp));
            telemetry.abs_position = Some(Rev(self.position));
            telemetry.power_w = Some(torque * self.velocity * std::f64::consts::TAU);
        }
        if profile == QueryProfile::Verify {
            // A real controller answers with what it is actually holding, and
            // the session compares that against what it believes it set. A
            // simulator that answered with the session's own numbers would make
            // that check pass without testing it.
            telemetry.active = drlabs_core::telemetry::ActiveLimits {
                max_torque: Some(Nm(self
                    .params
                    .max_torque
                    .unwrap_or(self.physics.default_torque.value()))),
                watchdog: Some(self.params.watchdog),
                velocity_limit: Some(RevPerSec(self.params.velocity_limit)),
                accel_limit: Some(RevPerSec2(self.params.accel_limit)),
            };
        }
        telemetry
    }
}

impl Link for SimLink {
    fn kind(&self) -> &'static str {
        self.kind
    }

    fn describe(&self) -> Vec<(String, String)> {
        vec![
            ("driver".into(), self.kind.into()),
            ("id".into(), self.id.to_string()),
            ("transport".into(), "simulated".into()),
            ("inertia".into(), format!("{} kg·m²", self.physics.inertia)),
        ]
    }

    fn open(&mut self) -> Result<()> {
        self.open = true;
        self.last = None;
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.open = false;
        Ok(())
    }

    /// Says plainly that it is a simulation.
    ///
    /// `BoardStatus::Supported` would be a lie of exactly the kind this
    /// framework spends its effort avoiding: an operator glancing at a status
    /// line should never have to work out whether the machine is real.
    fn identify(&mut self) -> Result<Option<Identity>> {
        Ok(Some(Identity {
            board: format!("simulated axis ({})", self.kind),
            firmware: "simulation".into(),
            status: BoardStatus::Unknown,
            simulated: true,
            ..Identity::new("sim")
        }))
    }

    fn capabilities(&self) -> Capabilities {
        Capabilities {
            modes: vec![
                ControlMode::Stop,
                ControlMode::Query,
                ControlMode::Position,
                ControlMode::Velocity,
                ControlMode::Torque,
                ControlMode::Hold,
                ControlMode::Brake,
                ControlMode::StayWithin,
                ControlMode::SetOutput,
            ],
            max_rate_hz: 1000,
            has_watchdog: true,
            reports_temperature: true,
            reports_current: true,
            reports_voltage: true,
        }
    }

    fn envelope(&self) -> Option<Envelope> {
        Some(Envelope {
            voltage_min: Some(Volts(10.0)),
            voltage_max: Some(Volts(44.0)),
            voltage_rated: Some(self.physics.supply),
            velocity_max: Some(RevPerSec(100.0)),
            peak_current: Some(Amps(40.0)),
        })
    }

    fn cycle(&mut self, command: &Command, profile: QueryProfile) -> Result<Option<Telemetry>> {
        if !self.open {
            return Err(Error::Closed);
        }
        let now = Instant::now();
        // Bounded, because a debugger pause or a laptop resuming from sleep
        // would otherwise integrate a minute of motion in one step.
        let dt = self
            .last
            .map(|last| now.duration_since(last).as_secs_f64().clamp(1e-4, 0.05))
            .unwrap_or(0.001);
        self.last = Some(now);
        self.elapsed += dt;

        self.apply_params(command);
        let max_torque = self
            .params
            .max_torque
            .unwrap_or(self.physics.default_torque.value());

        if let Mode::SetOutput { position } = command.mode {
            // Re-referencing moves the *label*, not the shaft.
            let offset = position.value() - self.position;
            self.position += offset;
            self.control.position = self.position;
        }

        let steps = (dt / self.physics.substep_seconds).round().max(1.0) as usize;
        let h = dt / steps as f64;
        let mut torque = 0.0;
        for _ in 0..steps {
            torque = self.step(command, max_torque, h);
        }

        let current = torque / self.physics.torque_constant;
        // A first-order thermal fit. It looks right on a plot and means nothing
        // about any real motor.
        self.motor_temp += (0.6 * current * current - 0.05 * (self.motor_temp - 24.0)) * dt;
        self.board_temp += (0.3 * current * current - 0.04 * (self.board_temp - 24.0)) * dt;
        self.voltage =
            self.physics.supply.value() - 0.05 * current.abs() + 0.02 * (self.elapsed).sin();

        Ok(Some(self.telemetry(torque, current, profile)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::command::Field;

    fn opened() -> SimLink {
        let mut link = SimLink::new(1);
        link.open().unwrap();
        link
    }

    /// Drives for a real interval, not a step count.
    ///
    /// The simulator integrates wall-clock time, so a loop that counts
    /// iterations measures how fast this machine runs tests rather than how
    /// the axis behaves.
    fn drive(link: &mut SimLink, command: &Command, seconds: f64) {
        let deadline = Instant::now() + std::time::Duration::from_secs_f64(seconds);
        while Instant::now() < deadline {
            link.cycle(command, QueryProfile::Fast).unwrap();
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
    }

    #[test]
    fn a_simulated_axis_reaches_a_position_target() {
        let mut link = opened();
        let command = Command {
            position: Field::Set(Rev(0.5)),
            velocity_limit: Field::Set(RevPerSec(4.0)),
            accel_limit: Field::Set(RevPerSec2(20.0)),
            max_torque: TorqueLimit::Nm(Nm(2.0)),
            ..Command::new(Mode::Position)
        };
        drive(&mut link, &command, 2.0);

        let reached = link.position().value();
        assert!(
            (reached - 0.5).abs() < 0.02,
            "expected to arrive near 0.5 rev, got {reached}"
        );
    }

    /// The acceleration limit has to bind, or a simulator makes a profile
    /// generator look correct that is not.
    #[test]
    fn the_acceleration_limit_bounds_the_approach() {
        let mut link = opened();
        let command = Command {
            position: Field::Set(Rev(10.0)),
            velocity_limit: Field::Set(RevPerSec(50.0)),
            accel_limit: Field::Set(RevPerSec2(1.0)),
            max_torque: TorqueLimit::Nm(Nm(5.0)),
            ..Command::new(Mode::Position)
        };
        // A quarter second at 1 rev/s² cannot have reached more than about
        // 0.25 rev/s, whatever the velocity limit says.
        drive(&mut link, &command, 0.25);
        let reply = link.cycle(&command, QueryProfile::Fast).unwrap().unwrap();
        let velocity = reply.velocity.unwrap().value();
        assert!(
            velocity < 1.0,
            "the accel limit did not bind: {velocity} rev/s"
        );
    }

    /// A torque ceiling is the operator's, and a simulator that ignores it
    /// hides every bug in the layer that sets it.
    #[test]
    fn the_torque_ceiling_binds() {
        let mut link = opened();
        let command = Command {
            position: Field::Set(Rev(100.0)),
            velocity_limit: Field::Set(RevPerSec(50.0)),
            accel_limit: Field::Set(RevPerSec2(500.0)),
            max_torque: TorqueLimit::Nm(Nm(0.05)),
            ..Command::new(Mode::Position)
        };
        drive(&mut link, &command, 0.2);
        let reply = link.cycle(&command, QueryProfile::Fast).unwrap().unwrap();
        let torque = reply.torque.unwrap().value().abs();
        assert!(
            torque <= 0.0501,
            "torque was {torque} Nm against a 0.05 ceiling"
        );
    }

    /// Handing the ceiling back to the controller must not put a NaN into the
    /// physics, where it would poison every value downstream.
    #[test]
    fn deferring_the_torque_limit_uses_the_firmware_default() {
        let mut link = opened();
        let command = Command {
            position: Field::Set(Rev(1.0)),
            max_torque: TorqueLimit::Controller,
            ..Command::new(Mode::Position)
        };
        let reply = link.cycle(&command, QueryProfile::Fast).unwrap().unwrap();
        assert!(reply.torque.unwrap().value().is_finite());
        assert!(reply.position.unwrap().value().is_finite());
    }

    /// Re-referencing moves the label, not the shaft.
    #[test]
    fn setting_the_output_moves_the_number_and_not_the_axis() {
        let mut link = opened().at(Rev(0.3));
        let before = link.position();
        link.cycle(
            &Command::new(Mode::SetOutput { position: Rev(0.0) }),
            QueryProfile::Fast,
        )
        .unwrap();

        assert_eq!(link.position(), Rev(0.0));
        assert_ne!(before, Rev(0.0), "the axis really was somewhere else");
    }

    /// A hold resists motion without pulling towards a target.
    #[test]
    fn a_hold_damps_rather_than_servos() {
        let mut link = opened().at(Rev(0.4));
        drive(&mut link, &Command::new(Mode::Hold), 0.5);
        let settled = link.position().value();
        assert!(
            (settled - 0.4).abs() < 0.05,
            "a hold should stay where it was, not travel: {settled}"
        );
    }

    /// The verify profile answers with what the simulated controller is
    /// actually holding. Echoing the session's own numbers back would make the
    /// mismatch check pass without ever testing it.
    #[test]
    fn the_verify_profile_reports_the_controllers_own_registers() {
        let mut link = opened();
        let command = Command {
            velocity_limit: Field::Set(RevPerSec(3.5)),
            max_torque: TorqueLimit::Nm(Nm(0.75)),
            ..Command::new(Mode::Position)
        };
        link.cycle(&command, QueryProfile::Fast).unwrap();

        // A later cycle that does not re-send them still gets them back: the
        // registers are sticky, as they are on real hardware.
        let bare = Command::new(Mode::Position);
        let reply = link.cycle(&bare, QueryProfile::Verify).unwrap().unwrap();
        assert_eq!(reply.active.velocity_limit, Some(RevPerSec(3.5)));
        assert_eq!(reply.active.max_torque, Some(Nm(0.75)));
    }

    /// An operator glancing at a status line should never have to work out
    /// whether the machine is real.
    #[test]
    fn a_simulation_says_that_it_is_one() {
        let mut link = opened();
        let identity = link.identify().unwrap().unwrap();
        assert!(identity.simulated);
        assert_ne!(identity.status, BoardStatus::Supported);
        assert!(identity.board.contains("simulated"), "{}", identity.board);
    }

    #[test]
    fn a_closed_link_refuses_to_cycle() {
        let mut link = SimLink::new(1);
        assert!(matches!(
            link.cycle(&Command::new(Mode::Query), QueryProfile::Fast),
            Err(Error::Closed)
        ));
    }
}
