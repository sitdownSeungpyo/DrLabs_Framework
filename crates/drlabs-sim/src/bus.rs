//! A row of simulated axes behind the [`Bus`] surface.
//!
//! One clock for the whole row. A [`crate::SimLink`] reads the time on every
//! call, which is right for one axis and wrong for twenty-six of them cycled
//! in sequence: each would integrate a slightly different `dt`, and the last
//! would always be a few microseconds ahead of the first. The bus reads the
//! clock once per tick and hands every axis the same interval.

use std::time::Instant;

use drlabs_core::bus::{AxisCmd, AxisReply, Bus};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{Capabilities, Envelope, Identity};
use drlabs_core::units::Rev;

use crate::{capabilities, elapsed, envelope, identity, Physics, SimAxis};

pub struct SimBus {
    kind: &'static str,
    axes: Vec<SimAxis>,
    last: Option<Instant>,
    /// Integrate this many seconds per cycle instead of the wall clock. For
    /// a test that drives the bus with its own clock.
    fixed_step: Option<f64>,
    open: bool,
}

impl SimBus {
    pub fn new(axes: usize) -> Self {
        Self::with_physics(axes, "sim", Physics::default())
    }

    pub fn with_physics(axes: usize, kind: &'static str, physics: Physics) -> Self {
        Self {
            kind,
            axes: (0..axes).map(|_| SimAxis::new(physics)).collect(),
            last: None,
            fixed_step: None,
            open: false,
        }
    }

    /// Makes every [`Bus::cycle`] integrate `seconds`, whatever the wall
    /// clock says. A test that calls the runtime's tick with a synthetic
    /// clock gets a simulation that advances with it.
    pub fn fixed_step(mut self, seconds: f64) -> Self {
        self.fixed_step = Some(seconds);
        self
    }

    /// Places one axis somewhere, for a test that needs it to start away from
    /// zero.
    pub fn at(mut self, axis: usize, position: Rev) -> Self {
        self.axes[axis] = self.axes[axis].at(position);
        self
    }

    pub fn axis(&self, axis: usize) -> &SimAxis {
        &self.axes[axis]
    }

    /// Advances every axis by `dt` seconds. The deterministic form of
    /// [`Bus::cycle`], for a test that wants to count steps rather than wait
    /// for a clock. Allocates nothing.
    pub fn step(&mut self, dt: f64, cmds: &[AxisCmd], out: &mut [AxisReply]) {
        assert_eq!(cmds.len(), self.axes.len(), "one command per axis");
        assert_eq!(out.len(), self.axes.len(), "one reply slot per axis");
        for ((axis, cmd), slot) in self.axes.iter_mut().zip(cmds).zip(out.iter_mut()) {
            let (torque, current) = axis.advance(&cmd.command, dt);
            *slot = axis.reply(torque, current, cmd.profile);
        }
    }
}

impl Bus for SimBus {
    fn kind(&self) -> &'static str {
        self.kind
    }

    fn axes(&self) -> usize {
        self.axes.len()
    }

    fn open(&mut self) -> Result<Vec<Option<Identity>>> {
        self.open = true;
        self.last = None;
        Ok(self
            .axes
            .iter()
            .map(|_| Some(identity(self.kind)))
            .collect())
    }

    fn close(&mut self) -> Result<()> {
        self.open = false;
        Ok(())
    }

    fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> Result<()> {
        if !self.open {
            return Err(Error::Closed);
        }
        let dt = match self.fixed_step {
            Some(seconds) => seconds,
            None => elapsed(&mut self.last, Instant::now()),
        };
        self.step(dt, cmds, out);
        Ok(())
    }

    fn capabilities(&self, _axis: usize) -> Capabilities {
        capabilities()
    }

    fn envelope(&self, axis: usize) -> Option<Envelope> {
        self.axes.get(axis).map(|a| envelope(&a.physics))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::bus::AxisStatus;
    use drlabs_core::command::{Command, Field, Mode, QueryProfile, TorqueLimit};
    use drlabs_core::units::{Nm, RevPerSec, RevPerSec2};

    fn drive_to(target: Rev) -> AxisCmd {
        AxisCmd {
            command: Command {
                position: Field::Set(target),
                velocity_limit: Field::Set(RevPerSec(4.0)),
                accel_limit: Field::Set(RevPerSec2(20.0)),
                max_torque: TorqueLimit::Nm(Nm(2.0)),
                ..Command::new(Mode::Position)
            },
            profile: QueryProfile::Fast,
        }
    }

    /// Every axis gets its own command and its own reply, in order.
    #[test]
    fn each_axis_follows_its_own_target() {
        let mut bus = SimBus::new(3);
        bus.open().unwrap();
        let cmds = [drive_to(Rev(0.5)), drive_to(Rev(-0.25)), drive_to(Rev(0.0))];
        let mut out = [AxisReply::SILENT; 3];
        for _ in 0..1000 {
            bus.step(0.002, &cmds, &mut out);
        }
        assert!(
            (out[0].position.unwrap().value() - 0.5).abs() < 0.02,
            "{out:?}"
        );
        assert!(
            (out[1].position.unwrap().value() + 0.25).abs() < 0.02,
            "{out:?}"
        );
        assert!(out[2].position.unwrap().value().abs() < 0.02, "{out:?}");
        assert!(out.iter().all(|r| r.status == AxisStatus::Ok));
    }

    #[test]
    fn a_closed_bus_refuses_to_cycle() {
        let mut bus = SimBus::new(2);
        let cmds = [AxisCmd::default(); 2];
        let mut out = [AxisReply::SILENT; 2];
        assert!(matches!(bus.cycle(&cmds, &mut out), Err(Error::Closed)));
    }

    #[test]
    fn a_simulated_bus_says_so_for_every_axis() {
        let mut bus = SimBus::new(4);
        let identities = bus.open().unwrap();
        assert_eq!(identities.len(), 4);
        assert!(identities.iter().all(|i| i.as_ref().unwrap().simulated));
    }
}
