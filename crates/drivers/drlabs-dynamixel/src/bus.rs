//! Every Dynamixel on one port, driven by one thread.
//!
//! The [`crate::chain::Chain`] shares a port between N links on N threads,
//! and whichever thread asks first does the bus's work for the rest. This is
//! the other shape: one owner, one tick, every servo. There is no mutex to
//! queue on because there is nobody to queue; the port lock inside the
//! [`Driver`] is taken and released by the same thread, uncontended.
//!
//! One tick on the wire, in order:
//!
//! 1. torque off, for servos leaving drive — one sync write;
//! 2. the limits that changed (watchdog, acceleration, velocity) — one sync
//!    write per register, only when something changed;
//! 3. torque on, for servos entering drive — one sync write;
//! 4. every goal position — one sync write;
//! 5. every present-value block — one *fast* sync read for the servos whose
//!    firmware has it, and chunked sync reads for the rest;
//! 6. once a second, the hardware-error and bus-watchdog registers.
//!
//! In steady state that is one write and one read, however many servos.
//! Nothing allocates: the grouping buffers and the packet buffers are built
//! in [`DynamixelBus::new`] and reused.
//!
//! What this bus does not do inside a tick: EEPROM writes (the operating
//! mode is set at open and never changed), individual recovery reads of a
//! silent servo (a silent servo is reported silent and asked again next
//! tick), and anything that waits longer than the driver's reply budget.

use std::time::Duration;

use drlabs_core::bus::{AxisCmd, AxisFault, AxisReply, AxisStatus, Bus};
use drlabs_core::command::{ControlMode, Mode};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{BoardStatus, Capabilities, Envelope, Identity};
use drlabs_core::units::{Amps, Celsius, Nm, Rev, RevPerSec, Volts};
use drlabs_transport::serial::SharedPort;

use crate::driver::{Driver, FastOutcome, Scratch};
use crate::link::{watchdog_counts, EEPROM_SETTLE_TIME, RETURN_DELAY_COUNTS};
use crate::profile::{known_model, profile_named, Profile, OP_MODE_EXTENDED_POSITION};
use crate::protocol::MAX_VALID_MOTOR_ID;

/// The most servos asked for in one plain sync read. See
/// [`crate::chain`] for the measurement behind it.
const MAX_SYNC_READ: usize = 8;

/// The wide block (through voltage and temperature) goes out every this
/// many ticks instead of the ten-byte fast one.
const WIDE_EVERY: u64 = 10;

/// The largest present-value block any profile reads.
const BLOCK_CAPACITY: usize = 32;

/// A latched bus watchdog, as a fault code. Above every real
/// hardware-error bit.
pub const FAULT_WATCHDOG_LATCHED: i32 = 0x100;
/// The alert bit seen before the error register has been read.
pub const FAULT_ALERT: i32 = 0x80;

struct Axis {
    id: u8,
    profile: Profile,
    declared: bool,
    model: Option<u32>,
    model_name: Option<&'static str>,

    /// Whether the firmware answers a fast sync read. Found out at open.
    fast: bool,
    torque: bool,
    velocity_limit_counts: Option<u32>,
    last_velocity: Option<u32>,
    last_accel: Option<u32>,
    last_watchdog: Option<u8>,

    // What this tick asked for.
    want_torque: bool,
    goal: Option<i32>,
    velocity: Option<u32>,
    accel: Option<u32>,
    watchdog: Option<u8>,
    refused: bool,

    // What came back.
    block: [u8; BLOCK_CAPACITY],
    block_len: usize,
    answered: bool,
    alert: bool,
    fault: Option<u8>,
    watchdog_byte: Option<u8>,
}

impl Axis {
    fn new(id: u8, profile: Option<Profile>) -> Self {
        Self {
            id,
            profile: profile.unwrap_or(crate::profile::X_SERIES),
            declared: profile.is_some(),
            model: None,
            model_name: None,
            fast: false,
            torque: false,
            velocity_limit_counts: None,
            last_velocity: None,
            last_accel: None,
            last_watchdog: None,
            want_torque: false,
            goal: None,
            velocity: None,
            accel: None,
            watchdog: None,
            refused: false,
            block: [0; BLOCK_CAPACITY],
            block_len: 0,
            answered: false,
            alert: false,
            fault: None,
            watchdog_byte: None,
        }
    }

    /// The wide present-value block: current through temperature.
    fn wide_len(&self) -> u16 {
        self.profile.addr_present_temperature + 1 - self.profile.addr_present_current
    }

    /// The fast block: current, velocity, position.
    fn fast_len(&self) -> u16 {
        self.profile.addr_present_position + 4 - self.profile.addr_present_current
    }

    fn store(&mut self, data: &[u8], alert: bool) {
        let n = data.len().min(BLOCK_CAPACITY);
        self.block[..n].copy_from_slice(&data[..n]);
        self.block_len = n;
        self.answered = true;
        self.alert = alert;
    }

    /// Same reasoning as `DynamixelLink::resolve_profile`.
    fn resolve_profile(&mut self, model: u32) -> Result<()> {
        self.model = Some(model);
        let Some(known) = known_model(model) else {
            return Ok(());
        };
        self.model_name = Some(known.name);
        let Some(expected) = profile_named(known.profile) else {
            return Ok(());
        };
        if !self.declared {
            let torque_constant = self.profile.torque_constant;
            self.profile = expected;
            self.profile.torque_constant = torque_constant.or(expected.torque_constant);
            return Ok(());
        }
        if self.profile.name != expected.name {
            return Err(Error::Config(format!(
                "servo id {} reports model {model} ({}), which needs the {:?} profile, but {:?} \
                 was configured; the profiles disagree on current scale and supply range",
                self.id, known.name, known.profile, self.profile.name
            )));
        }
        Ok(())
    }

    fn reply(&self, stopped_for_watchdog: bool) -> AxisReply {
        let mut reply = AxisReply::SILENT;
        reply.status = if !self.answered {
            AxisStatus::Silent
        } else if self.refused {
            AxisStatus::Refused
        } else if self.alert {
            AxisStatus::Alert
        } else {
            AxisStatus::Ok
        };
        reply.mode = Some(i32::from(self.torque));

        // The error register as last read, the alert bit as last seen, or a
        // latched watchdog found this tick: whichever is the most serious
        // thing known.
        reply.fault = if stopped_for_watchdog {
            Some(AxisFault {
                code: FAULT_WATCHDOG_LATCHED,
                hard: true,
            })
        } else {
            match (self.fault, self.alert && self.answered) {
                (Some(bits), _) if bits != 0 => Some(AxisFault {
                    code: bits as i32,
                    hard: true,
                }),
                (_, true) => Some(AxisFault {
                    code: FAULT_ALERT,
                    hard: true,
                }),
                (Some(_), false) => Some(AxisFault::default()),
                (None, false) => None,
            }
        };

        if !self.answered || self.block_len < 10 {
            return reply;
        }
        let data = &self.block[..self.block_len];
        let profile = &self.profile;

        // Present Current on a family with the sensor; Present Load on one
        // without, which is not a current by any scaling.
        let current = profile.has_current_sense.then(|| {
            i16::from_le_bytes([data[0], data[1]]) as f64 * profile.current_amps_per_count
        });
        let velocity = i32::from_le_bytes([data[2], data[3], data[4], data[5]]) as f64
            * profile.velocity_rpm_per_count
            / 60.0;
        let position = i32::from_le_bytes([data[6], data[7], data[8], data[9]]) as f64
            / profile.counts_per_rev;

        reply.current = current.map(Amps);
        reply.velocity = Some(RevPerSec(velocity));
        reply.position = Some(Rev(position));
        reply.torque = current
            .zip(profile.torque_constant)
            .map(|(current, constant)| Nm(current * constant));

        let voltage_offset = (profile.addr_present_voltage - profile.addr_present_current) as usize;
        let temp_offset =
            (profile.addr_present_temperature - profile.addr_present_current) as usize;
        if data.len() > temp_offset {
            let voltage = u16::from_le_bytes([data[voltage_offset], data[voltage_offset + 1]])
                as f64
                * profile.voltage_volts_per_count;
            reply.voltage = Some(Volts(voltage));
            // An integrated servo has one temperature, and it stands for both.
            let temperature = Celsius(data[temp_offset] as f64);
            reply.motor_temperature = Some(temperature);
            reply.board_temperature = Some(temperature);
        }
        reply
    }
}

/// A [`Bus`] of Dynamixel servos.
pub struct DynamixelBus {
    driver: Driver,
    port_name: String,
    baud: u32,
    axes: Vec<Axis>,
    /// Axis index by servo id, 255 for none.
    by_id: [u8; 256],
    scratch: Scratch,
    ids: Vec<u8>,
    values: Vec<u8>,
    group: Vec<u8>,
    open: bool,
    tick: u64,
    /// Ticks between error-register reads.
    fault_every: u64,
}

impl DynamixelBus {
    /// The servos on `port`, in axis order, each with its declared profile
    /// or `None` to let the model number choose.
    pub fn new(port: SharedPort, servos: &[(u8, Option<Profile>)]) -> Self {
        let mut driver = Driver::new(port);
        // A bus thread owns its core; it spins for a reply rather than
        // sleeping a scheduler quantum for it.
        driver.poll_interval = Duration::ZERO;
        let mut by_id = [255u8; 256];
        let axes: Vec<Axis> = servos
            .iter()
            .enumerate()
            .map(|(index, (id, profile))| {
                by_id[*id as usize] = index as u8;
                Axis::new(*id, *profile)
            })
            .collect();
        let n = axes.len();
        Self {
            driver,
            port_name: String::new(),
            baud: 1_000_000,
            axes,
            by_id,
            scratch: Scratch::new(),
            ids: Vec::with_capacity(n),
            values: Vec::with_capacity(n * 4),
            group: Vec::with_capacity(n),
            open: false,
            tick: 0,
            fault_every: 500,
        }
    }

    pub fn with_port_name(mut self, name: &str, baud: u32) -> Self {
        self.port_name = name.to_string();
        self.baud = baud;
        self
    }

    /// Tells the bus its period, so the once-a-second reads are once a
    /// second whatever the rate.
    pub fn with_period(mut self, period: Duration) -> Self {
        let ticks = (1.0 / period.as_secs_f64().max(1e-6)).round() as u64;
        self.fault_every = ticks.max(1);
        self
    }

    /// Whether each axis answers a fast sync read, as found at open. For a
    /// report line.
    pub fn fast_axes(&self) -> Vec<(u8, bool)> {
        self.axes.iter().map(|axis| (axis.id, axis.fast)).collect()
    }

    /// Opens one servo: identify, clear what the last session left, make
    /// sure the mode is the one this bus drives, leave it free.
    fn open_axis(&mut self, index: usize) -> Result<Option<Identity>> {
        let id = self.axes[index].id;
        if id > MAX_VALID_MOTOR_ID {
            return Err(Error::Config(format!(
                "servo id {id} is out of range (0..{MAX_VALID_MOTOR_ID})"
            )));
        }
        let model = self.driver.ping(id).map_err(|error| {
            Error::Refused(format!(
                "no reply from servo id {id} on {} at {} baud: {error}",
                self.port_name, self.baud
            ))
        })?;
        self.axes[index].resolve_profile(model as u32)?;
        let profile = self.axes[index].profile;

        // A latched watchdog refuses every goal register; clear it first.
        self.driver
            .write_u8(id, profile.addr_bus_watchdog, 0)
            .map_err(|error| {
                Error::Refused(format!("servo id {id}: clearing the bus watchdog: {error}"))
            })?;

        // Return delay: read, write only when wrong (EEPROM).
        if profile.addr_return_delay != 0 {
            if let Ok(data) = self.driver.read(id, profile.addr_return_delay, 1) {
                if data.len() == 1 && data[0] != RETURN_DELAY_COUNTS {
                    self.driver.write_u8(id, profile.addr_torque_enable, 0)?;
                    self.driver
                        .write_u8(id, profile.addr_return_delay, RETURN_DELAY_COUNTS)
                        .map_err(|error| {
                            Error::Refused(format!(
                                "servo id {id}: normalising the return delay: {error}"
                            ))
                        })?;
                    std::thread::sleep(EEPROM_SETTLE_TIME);
                }
            }
        }

        // The one mode this bus drives. Set now, in EEPROM, so the tick never
        // has to; read first so an already-right servo costs no write.
        let mode = self
            .driver
            .read(id, profile.addr_operating_mode, 1)
            .ok()
            .and_then(|data| data.first().copied());
        if mode != Some(OP_MODE_EXTENDED_POSITION) {
            self.driver.write_u8(id, profile.addr_torque_enable, 0)?;
            self.driver
                .write_u8(id, profile.addr_operating_mode, OP_MODE_EXTENDED_POSITION)
                .map_err(|error| {
                    Error::Refused(format!(
                        "servo id {id}: setting extended position mode: {error}"
                    ))
                })?;
            std::thread::sleep(EEPROM_SETTLE_TIME);
        }

        if profile.addr_velocity_limit != 0 {
            if let Ok(data) = self.driver.read(id, profile.addr_velocity_limit, 4) {
                if data.len() == 4 {
                    let counts = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                    if counts > 0 {
                        self.axes[index].velocity_limit_counts = Some(counts);
                    }
                }
            }
        }

        // Free until a client asks for it.
        self.driver.write_u8(id, profile.addr_torque_enable, 0)?;
        self.axes[index].torque = false;

        // Does the firmware answer a fast sync read? Asked alone, so the
        // answer is unambiguous.
        let fast_len = self.axes[index].fast_len();
        let outcome = self.driver.fast_sync_read_into(
            &mut self.scratch,
            profile.addr_present_current,
            fast_len,
            &[id],
            &mut |_, _, _| {},
        )?;
        self.axes[index].fast = matches!(outcome, FastOutcome::Answered(1));

        let mut identity = Identity::new("dynamixel");
        identity.model_number = Some(model as u32);
        identity.board = match self.axes[index].model_name {
            Some(name) => format!("Dynamixel {name} ({} control table)", profile.name),
            None => format!(
                "Dynamixel model {model} ({} control table, not confirmed)",
                profile.name
            ),
        };
        if let Ok(data) = self.driver.read(id, profile.addr_firmware_version, 1) {
            if data.len() == 1 {
                identity.firmware = data[0].to_string();
            }
        }
        identity.status = if self.axes[index].model_name.is_some() || self.axes[index].declared {
            BoardStatus::Supported
        } else {
            BoardStatus::Unknown
        };
        Ok(Some(identity))
    }

    /// Emits one sync write per distinct register among the axes `pick`
    /// selects, `width` bytes each. `pick` returns the register and writes
    /// the value's bytes into the buffer it is given.
    fn emit(
        &mut self,
        width: u16,
        mut pick: impl FnMut(&Axis, &mut Vec<u8>) -> Option<u16>,
    ) -> Result<()> {
        // Repeated passes, one per distinct address. Profiles on one bus
        // almost always share a control table, so this is one pass.
        let mut done = [false; 256];
        loop {
            let mut address: Option<u16> = None;
            self.ids.clear();
            self.values.clear();
            for (index, axis) in self.axes.iter().enumerate() {
                if done[index] {
                    continue;
                }
                let before = self.values.len();
                let Some(register) = pick(axis, &mut self.values) else {
                    done[index] = true;
                    continue;
                };
                match address {
                    None => address = Some(register),
                    Some(current) if current != register => {
                        // Another register: next pass.
                        self.values.truncate(before);
                        continue;
                    }
                    Some(_) => {}
                }
                self.ids.push(axis.id);
                done[index] = true;
            }
            let Some(register) = address else {
                return Ok(());
            };
            self.driver.sync_write_into(
                &mut self.scratch,
                register,
                width,
                &self.ids,
                &self.values,
            )?;
        }
    }

    /// Reads one register block from every axis `pick` selects, grouped by
    /// the register and length the axis's profile gives, fast or chunked as
    /// the axis allows.
    fn read_all(
        &mut self,
        pick: impl Fn(&Axis) -> Option<(u16, u16)>,
        mut store: impl FnMut(&mut Axis, &[u8], bool),
    ) -> Result<()> {
        let mut done = [false; 256];
        loop {
            // One (register, length, fast) group per pass.
            let mut key: Option<(u16, u16, bool)> = None;
            self.group.clear();
            for (index, axis) in self.axes.iter().enumerate() {
                if done[index] {
                    continue;
                }
                let Some((register, length)) = pick(axis) else {
                    done[index] = true;
                    continue;
                };
                let this = (register, length, axis.fast);
                match key {
                    None => key = Some(this),
                    Some(current) if current != this => continue,
                    Some(_) => {}
                }
                self.group.push(axis.id);
                done[index] = true;
            }
            let Some((register, length, fast)) = key else {
                return Ok(());
            };

            let Self {
                driver,
                scratch,
                axes,
                by_id,
                group,
                ..
            } = self;
            let mut sink = |id: u8, alert: bool, data: &[u8]| {
                let index = by_id[id as usize];
                if index != 255 {
                    store(&mut axes[index as usize], data, alert);
                }
            };
            if fast {
                match driver.fast_sync_read_into(scratch, register, length, group, &mut sink)? {
                    FastOutcome::Answered(_) | FastOutcome::Silent => {}
                    FastOutcome::Unsupported(id) => {
                        // Its firmware does not have the instruction after
                        // all; it reads the slow way from now on.
                        let index = by_id[id as usize];
                        if index != 255 {
                            axes[index as usize].fast = false;
                        }
                    }
                }
            } else {
                for chunk in group.chunks(MAX_SYNC_READ) {
                    driver.sync_read_into(scratch, register, length, chunk, &mut sink)?;
                }
            }
        }
    }
}

impl Bus for DynamixelBus {
    fn kind(&self) -> &'static str {
        "dynamixel"
    }

    fn axes(&self) -> usize {
        self.axes.len()
    }

    fn open(&mut self) -> Result<Vec<Option<Identity>>> {
        self.driver.tune_read_timeout();
        let mut identities = Vec::with_capacity(self.axes.len());
        for index in 0..self.axes.len() {
            identities.push(self.open_axis(index)?);
        }
        self.open = true;
        self.tick = 0;
        Ok(identities)
    }

    fn close(&mut self) -> Result<()> {
        // Straight to the wire, one servo at a time: there is no next tick
        // to carry a queued write.
        for axis in &mut self.axes {
            let _ = self
                .driver
                .write_u8(axis.id, axis.profile.addr_torque_enable, 0);
            axis.torque = false;
        }
        self.open = false;
        Ok(())
    }

    fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> Result<()> {
        if !self.open {
            return Err(Error::Closed);
        }
        assert_eq!(cmds.len(), self.axes.len(), "one command per axis");
        assert_eq!(out.len(), self.axes.len(), "one reply slot per axis");
        self.tick += 1;

        // --- what each axis wants this tick ---------------------------------
        for (axis, cmd) in self.axes.iter_mut().zip(cmds) {
            let command = &cmd.command;
            axis.goal = None;
            axis.velocity = None;
            axis.accel = None;
            axis.watchdog = None;
            axis.refused = false;
            match command.mode {
                Mode::Position => {
                    axis.want_torque = true;
                    if let Some(position) = command.position.get() {
                        axis.goal =
                            Some((position.value() * axis.profile.counts_per_rev).round() as i32);
                    }
                    if let Some(limit) = command.velocity_limit.get() {
                        let mut counts = (limit.value() * 60.0
                            / axis.profile.velocity_rpm_per_count)
                            .round()
                            .max(0.0) as u32;
                        if let Some(ceiling) = axis.velocity_limit_counts {
                            counts = counts.min(ceiling);
                        }
                        if command.full || axis.last_velocity != Some(counts) {
                            axis.velocity = Some(counts);
                        }
                    }
                    if let Some(limit) = command.accel_limit.get() {
                        let counts = (limit.value() * 3600.0
                            / axis.profile.accel_rev_per_min2_per_count)
                            .round()
                            .max(0.0) as u32;
                        if command.full || axis.last_accel != Some(counts) {
                            axis.accel = Some(counts);
                        }
                    }
                    if axis.profile.addr_bus_watchdog != 0 {
                        if let Some(timeout) = command.watchdog.get() {
                            let counts = watchdog_counts(timeout, &axis.profile);
                            if command.full || axis.last_watchdog != Some(counts) {
                                axis.watchdog = Some(counts);
                            }
                        }
                    }
                }
                Mode::Stop | Mode::Query => axis.want_torque = false,
                _ => {
                    axis.want_torque = false;
                    axis.refused = true;
                }
            }
        }

        // --- writes, in arming order ----------------------------------------
        self.emit(1, |axis, out| {
            (axis.torque && !axis.want_torque).then(|| {
                out.push(0);
                axis.profile.addr_torque_enable
            })
        })?;
        for axis in &mut self.axes {
            if axis.torque && !axis.want_torque {
                axis.torque = false;
                // Limits are re-sent on the next drive.
                axis.last_velocity = None;
                axis.last_accel = None;
                axis.last_watchdog = None;
            }
        }
        self.emit(1, |axis, out| {
            axis.watchdog.map(|counts| {
                out.push(counts);
                axis.profile.addr_bus_watchdog
            })
        })?;
        self.emit(4, |axis, out| {
            axis.accel.map(|counts| {
                out.extend_from_slice(&counts.to_le_bytes());
                axis.profile.addr_profile_accel
            })
        })?;
        self.emit(4, |axis, out| {
            axis.velocity.map(|counts| {
                out.extend_from_slice(&counts.to_le_bytes());
                axis.profile.addr_profile_velocity
            })
        })?;
        for axis in &mut self.axes {
            if let Some(counts) = axis.watchdog {
                axis.last_watchdog = Some(counts);
            }
            if let Some(counts) = axis.accel {
                axis.last_accel = Some(counts);
            }
            if let Some(counts) = axis.velocity {
                axis.last_velocity = Some(counts);
            }
        }
        self.emit(1, |axis, out| {
            (!axis.torque && axis.want_torque).then(|| {
                out.push(1);
                axis.profile.addr_torque_enable
            })
        })?;
        for axis in &mut self.axes {
            if !axis.torque && axis.want_torque {
                axis.torque = true;
            }
        }
        self.emit(4, |axis, out| {
            axis.goal.filter(|_| axis.torque).map(|counts| {
                out.extend_from_slice(&counts.to_le_bytes());
                axis.profile.addr_goal_position
            })
        })?;

        // --- the readings ----------------------------------------------------
        let wide = self.tick % WIDE_EVERY == 0;
        for axis in &mut self.axes {
            axis.answered = false;
            axis.alert = false;
        }
        self.read_all(
            |axis| {
                let length = if wide {
                    axis.wide_len()
                } else {
                    axis.fast_len()
                };
                Some((axis.profile.addr_present_current, length))
            },
            |axis, data, alert| axis.store(data, alert),
        )?;

        // --- faults and the watchdog, once a second --------------------------
        let mut latched = [false; 256];
        if self.tick % self.fault_every == 1 || self.fault_every == 1 {
            self.read_all(
                |axis| {
                    (axis.profile.addr_hardware_error != 0)
                        .then_some((axis.profile.addr_hardware_error, 1))
                },
                |axis, data, _| {
                    if let Some(&bits) = data.first() {
                        axis.fault = Some(bits);
                    }
                },
            )?;
            self.read_all(
                |axis| {
                    (axis.profile.addr_bus_watchdog != 0)
                        .then_some((axis.profile.addr_bus_watchdog, 1))
                },
                |axis, data, _| {
                    if let Some(&byte) = data.first() {
                        axis.watchdog_byte = Some(byte);
                    }
                },
            )?;
            // A latched watchdog refuses every goal silently. Clear it, say
            // so loudly this tick, and forget the cached timeout so the next
            // drive re-arms it.
            for index in 0..self.axes.len() {
                if self.axes[index].watchdog_byte == Some(0xFF) {
                    let id = self.axes[index].id;
                    let address = self.axes[index].profile.addr_bus_watchdog;
                    let _ = self.driver.write_u8(id, address, 0);
                    self.axes[index].watchdog_byte = None;
                    self.axes[index].last_watchdog = None;
                    self.axes[index].torque = false;
                    latched[index] = true;
                }
            }
        }

        for (index, (axis, slot)) in self.axes.iter().zip(out.iter_mut()).enumerate() {
            *slot = axis.reply(latched[index]);
        }
        Ok(())
    }

    fn capabilities(&self, axis: usize) -> Capabilities {
        let Some(axis) = self.axes.get(axis) else {
            return Capabilities::default();
        };
        // The wire budget of one tick: a goal sync write and one block read,
        // fast or chunked. Ten bits a byte.
        let n = self.axes.len() as f64;
        let write = 14.0 + n * 5.0;
        let read = if self.axes.iter().all(|a| a.fast) {
            8.0 + n * (axis.fast_len() as f64 + 4.0) + 12.0 + n
        } else {
            n * (axis.fast_len() as f64 + 11.0) + 14.0 + n
        };
        let seconds = (write + read) * 10.0 / self.baud as f64;
        Capabilities {
            modes: vec![ControlMode::Stop, ControlMode::Query, ControlMode::Position],
            // Half the wire, leaving the rest for turnarounds and the host.
            max_rate_hz: (0.5 / seconds).clamp(1.0, 2000.0) as u32,
            has_watchdog: axis.profile.addr_bus_watchdog != 0,
            reports_temperature: true,
            reports_current: axis.profile.has_current_sense,
            reports_voltage: true,
        }
    }

    fn envelope(&self, axis: usize) -> Option<Envelope> {
        let axis = self.axes.get(axis)?;
        let profile = &axis.profile;
        Some(Envelope {
            voltage_min: Some(Volts(profile.voltage_min)),
            voltage_max: Some(Volts(profile.voltage_max)),
            voltage_rated: Some(Volts(profile.voltage_rated)),
            velocity_max: Some(RevPerSec(match axis.velocity_limit_counts {
                Some(counts) => (counts as f64 * profile.velocity_rpm_per_count / 60.0)
                    .min(profile.max_velocity_rev_per_sec),
                None => profile.max_velocity_rev_per_sec,
            })),
            peak_current: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{
        update_crc, BROADCAST_ID, INST_FAST_SYNC_READ, INST_PING, INST_READ, INST_STATUS,
        INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
    };
    use drlabs_core::command::{Command, Field, QueryProfile};
    use drlabs_transport::serial::SerialPort;
    use std::collections::BTreeMap;
    use std::sync::{Arc, Mutex};

    /// A chain of servos that behave like servos: they answer pings with a
    /// model number, reads from a register map, acknowledge writes, and
    /// answer sync reads in order — fast or one by one as configured.
    #[derive(Clone)]
    struct FakeServos {
        inner: Arc<Mutex<Inner>>,
    }

    struct Servo {
        model: u16,
        fast: bool,
        registers: BTreeMap<u16, u8>,
    }

    #[derive(Default)]
    struct Inner {
        servos: BTreeMap<u8, Servo>,
        pending: Vec<u8>,
        /// Every instruction packet received, as (id, instruction, params).
        log: Vec<(u8, u8, Vec<u8>)>,
    }

    impl FakeServos {
        fn new() -> Self {
            Self {
                inner: Arc::new(Mutex::new(Inner::default())),
            }
        }

        fn add(&self, id: u8, model: u16, fast: bool) {
            let mut registers = BTreeMap::new();
            registers.insert(6, 48); // firmware
            registers.insert(9, RETURN_DELAY_COUNTS);
            registers.insert(11, OP_MODE_EXTENDED_POSITION);
            for (offset, byte) in 1000u32.to_le_bytes().iter().enumerate() {
                registers.insert(44 + offset as u16, *byte); // velocity limit
            }
            // Present block: current 100, velocity 0, position 2048, …, voltage 120, temp 31.
            for (offset, byte) in 100i16.to_le_bytes().iter().enumerate() {
                registers.insert(126 + offset as u16, *byte);
            }
            for (offset, byte) in 2048i32.to_le_bytes().iter().enumerate() {
                registers.insert(132 + offset as u16, *byte);
            }
            registers.insert(144, 120);
            registers.insert(145, 0);
            registers.insert(146, 31);
            self.inner.lock().unwrap().servos.insert(
                id,
                Servo {
                    model,
                    fast,
                    registers,
                },
            );
        }

        fn register(&self, id: u8, address: u16) -> Option<u8> {
            self.inner
                .lock()
                .unwrap()
                .servos
                .get(&id)
                .and_then(|s| s.registers.get(&address).copied())
        }

        fn u32_register(&self, id: u8, address: u16) -> u32 {
            u32::from_le_bytes([
                self.register(id, address).unwrap_or(0),
                self.register(id, address + 1).unwrap_or(0),
                self.register(id, address + 2).unwrap_or(0),
                self.register(id, address + 3).unwrap_or(0),
            ])
        }

        fn log(&self) -> Vec<(u8, u8, Vec<u8>)> {
            self.inner.lock().unwrap().log.clone()
        }

        fn clear_log(&self) {
            self.inner.lock().unwrap().log.clear();
        }
    }

    fn status(id: u8, error: u8, params: &[u8]) -> Vec<u8> {
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
        let length = (1 + 1 + params.len() + 2) as u16;
        packet.extend_from_slice(&length.to_le_bytes());
        packet.push(INST_STATUS);
        packet.push(error);
        packet.extend_from_slice(params);
        let crc = update_crc(0, &packet);
        packet.extend_from_slice(&crc.to_le_bytes());
        packet
    }

    impl Inner {
        fn block(&self, id: u8, address: u16, length: u16) -> Vec<u8> {
            let servo = &self.servos[&id];
            (0..length)
                .map(|offset| {
                    servo
                        .registers
                        .get(&(address + offset))
                        .copied()
                        .unwrap_or(0)
                })
                .collect()
        }

        fn handle(&mut self, packet: &[u8]) {
            // An instruction packet: header, id, length, instruction,
            // parameters, CRC. A ping is ten bytes.
            if packet.len() < 10 || packet[0..3] != [0xFF, 0xFF, 0xFD] {
                return;
            }
            let length = u16::from_le_bytes([packet[5], packet[6]]) as usize;
            if packet.len() != 7 + length {
                return;
            }
            let received = u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);
            if received != update_crc(0, &packet[..packet.len() - 2]) {
                return;
            }
            let id = packet[4];
            let instruction = packet[7];
            let params = packet[8..packet.len() - 2].to_vec();
            self.log.push((id, instruction, params.clone()));

            match instruction {
                INST_PING => {
                    if let Some(servo) = self.servos.get(&id) {
                        let mut p = servo.model.to_le_bytes().to_vec();
                        p.push(48);
                        let reply = status(id, 0, &p);
                        self.pending.extend_from_slice(&reply);
                    }
                }
                INST_READ => {
                    if self.servos.contains_key(&id) {
                        let address = u16::from_le_bytes([params[0], params[1]]);
                        let length = u16::from_le_bytes([params[2], params[3]]);
                        let data = self.block(id, address, length);
                        let reply = status(id, 0, &data);
                        self.pending.extend_from_slice(&reply);
                    }
                }
                INST_WRITE => {
                    if let Some(servo) = self.servos.get_mut(&id) {
                        let address = u16::from_le_bytes([params[0], params[1]]);
                        for (offset, byte) in params[2..].iter().enumerate() {
                            servo.registers.insert(address + offset as u16, *byte);
                        }
                        let reply = status(id, 0, &[]);
                        self.pending.extend_from_slice(&reply);
                    }
                }
                INST_SYNC_WRITE => {
                    let address = u16::from_le_bytes([params[0], params[1]]);
                    let width = u16::from_le_bytes([params[2], params[3]]) as usize;
                    for entry in params[4..].chunks(1 + width) {
                        if let Some(servo) = self.servos.get_mut(&entry[0]) {
                            for (offset, byte) in entry[1..].iter().enumerate() {
                                servo.registers.insert(address + offset as u16, *byte);
                            }
                        }
                    }
                }
                INST_SYNC_READ => {
                    let address = u16::from_le_bytes([params[0], params[1]]);
                    let length = u16::from_le_bytes([params[2], params[3]]);
                    for &id in &params[4..] {
                        if self.servos.contains_key(&id) {
                            let data = self.block(id, address, length);
                            let reply = status(id, 0, &data);
                            self.pending.extend_from_slice(&reply);
                        }
                    }
                }
                INST_FAST_SYNC_READ => {
                    let address = u16::from_le_bytes([params[0], params[1]]);
                    let length = u16::from_le_bytes([params[2], params[3]]);
                    let ids: Vec<u8> = params[4..].to_vec();
                    if let Some(&slow) = ids
                        .iter()
                        .find(|id| self.servos.get(id).is_some_and(|s| !s.fast))
                    {
                        let reply = status(slow, 0x02, &[]);
                        self.pending.extend_from_slice(&reply);
                        return;
                    }
                    let present: Vec<u8> = ids
                        .iter()
                        .copied()
                        .filter(|id| self.servos.contains_key(id))
                        .collect();
                    if present.is_empty() {
                        return;
                    }
                    let total = 1 + present.len() * (length as usize + 4);
                    let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID];
                    packet.extend_from_slice(&(total as u16).to_le_bytes());
                    packet.push(INST_STATUS);
                    for id in present {
                        packet.push(0);
                        packet.push(id);
                        packet.extend_from_slice(&self.block(id, address, length));
                        let crc = update_crc(0, &packet);
                        packet.extend_from_slice(&crc.to_le_bytes());
                    }
                    self.pending.extend_from_slice(&packet);
                }
                _ => {}
            }
        }
    }

    impl SerialPort for FakeServos {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut inner = self.inner.lock().unwrap();
            let n = inner.pending.len().min(buf.len());
            buf[..n].copy_from_slice(&inner.pending[..n]);
            inner.pending.drain(..n);
            Ok(n)
        }

        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.inner.lock().unwrap().handle(buf);
            Ok(buf.len())
        }
    }

    fn bus(servos: &FakeServos, ids: &[u8]) -> DynamixelBus {
        let shared = SharedPort::new(Box::new(servos.clone()));
        let axes: Vec<(u8, Option<Profile>)> = ids.iter().map(|&id| (id, None)).collect();
        let mut bus = DynamixelBus::new(shared, &axes).with_period(Duration::from_millis(2));
        bus.driver.timeout = Duration::from_millis(20);
        bus.driver.reply_timeout = Duration::from_millis(20);
        bus
    }

    fn drive(position: Rev) -> AxisCmd {
        AxisCmd {
            command: Command {
                position: Field::Set(position),
                velocity_limit: Field::Set(RevPerSec(1.0)),
                accel_limit: Field::Set(drlabs_core::units::RevPerSec2(10.0)),
                watchdog: Field::Set(0.1),
                ..Command::new(Mode::Position)
            },
            profile: QueryProfile::Fast,
        }
    }

    /// Open finds out which servos answer a fast sync read, picks their
    /// profiles from their model numbers, and leaves every one free.
    #[test]
    fn open_identifies_each_servo_and_probes_for_fast_sync_read() {
        let servos = FakeServos::new();
        servos.add(1, 1020, true); // XM430: fast
        servos.add(2, 311, false); // MX-64: not
        let mut bus = bus(&servos, &[1, 2]);

        let identities = bus.open().unwrap();
        assert_eq!(identities.len(), 2);
        assert!(identities[0].as_ref().unwrap().board.contains("XM430"));
        assert!(identities[1].as_ref().unwrap().board.contains("MX-64"));
        assert_eq!(bus.fast_axes(), vec![(1, true), (2, false)]);
        assert_eq!(bus.axes[1].profile.name, "MX64");
        assert_eq!(servos.register(1, 64), Some(0), "torque off");
        assert_eq!(servos.register(1, 98), Some(0), "watchdog cleared");
    }

    /// A tick in steady state is one goal sync write and one read; arming
    /// adds the torque and limit writes once, in order.
    #[test]
    fn a_tick_is_one_write_and_one_read_in_steady_state() {
        let servos = FakeServos::new();
        for id in 1..=12 {
            servos.add(id, 1020, true);
        }
        let ids: Vec<u8> = (1..=12).collect();
        let mut bus = bus(&servos, &ids);
        bus.open().unwrap();
        let mut out = vec![AxisReply::SILENT; 12];
        // The first tick also reads the error and watchdog registers; let it
        // go by observing, so the arming tick is seen on its own.
        bus.cycle(&vec![AxisCmd::default(); 12], &mut out).unwrap();
        servos.clear_log();

        let cmds = vec![drive(Rev(0.25)); 12];
        bus.cycle(&cmds, &mut out).unwrap();
        let log = servos.log();
        let instructions: Vec<(u8, u16)> = log
            .iter()
            .map(|(_, inst, params)| (*inst, u16::from_le_bytes([params[0], params[1]])))
            .collect();
        // watchdog, accel, velocity, torque on, goal — then the fast read.
        assert_eq!(
            instructions,
            vec![
                (INST_SYNC_WRITE, 98),
                (INST_SYNC_WRITE, 108),
                (INST_SYNC_WRITE, 112),
                (INST_SYNC_WRITE, 64),
                (INST_SYNC_WRITE, 116),
                (INST_FAST_SYNC_READ, 126),
            ],
            "{log:?}"
        );
        assert_eq!(servos.u32_register(7, 116), 1024, "0.25 rev is 1024 counts");
        assert_eq!(servos.register(7, 64), Some(1), "torque on");

        // Steady state: nothing changed but the goal.
        servos.clear_log();
        let cmds = vec![drive(Rev(0.5)); 12];
        bus.cycle(&cmds, &mut out).unwrap();
        let instructions: Vec<u8> = servos.log().iter().map(|(_, inst, _)| *inst).collect();
        assert_eq!(instructions, vec![INST_SYNC_WRITE, INST_FAST_SYNC_READ]);
        assert!(out.iter().all(|r| r.status == AxisStatus::Ok), "{out:?}");
        assert!((out[0].position.unwrap().value() - 0.5).abs() < 1e-9);
        assert_eq!(out[0].current, Some(Amps(100.0 * 0.00269)));
    }

    /// Servos whose firmware lacks the instruction are read the slow way,
    /// in chunks, on the same tick as the fast ones.
    #[test]
    fn a_mixed_bus_reads_fast_and_slow_groups_together() {
        let servos = FakeServos::new();
        for id in 1..=5 {
            servos.add(id, 1020, true);
        }
        for id in 6..=15 {
            servos.add(id, 311, false);
        }
        let ids: Vec<u8> = (1..=15).collect();
        let mut bus = bus(&servos, &ids);
        bus.open().unwrap();
        let cmds = vec![AxisCmd::default(); 15]; // Stop: observe only
        let mut out = vec![AxisReply::SILENT; 15];
        bus.cycle(&cmds, &mut out).unwrap(); // the once-a-second reads
        servos.clear_log();
        bus.cycle(&cmds, &mut out).unwrap();

        let reads: Vec<(u8, Vec<u8>)> = servos
            .log()
            .iter()
            .filter(|(_, inst, _)| *inst == INST_SYNC_READ || *inst == INST_FAST_SYNC_READ)
            .map(|(_, inst, params)| (*inst, params[4..].to_vec()))
            .collect();
        assert_eq!(reads.len(), 3, "one fast read and two chunks: {reads:?}");
        assert_eq!(reads[0], (INST_FAST_SYNC_READ, vec![1, 2, 3, 4, 5]));
        assert_eq!(reads[1], (INST_SYNC_READ, (6..=13).collect()));
        assert_eq!(reads[2], (INST_SYNC_READ, vec![14, 15]));
        assert!(out.iter().all(|r| r.status == AxisStatus::Ok), "{out:?}");
        assert!(out.iter().all(|r| r.mode == Some(0)), "nobody is driving");
    }

    /// A servo that does not answer is silent, not an error, and its
    /// neighbours still report.
    #[test]
    fn a_missing_servo_is_silent_and_the_rest_report() {
        let servos = FakeServos::new();
        servos.add(1, 1020, true);
        servos.add(3, 1020, true);
        let mut bus = bus(&servos, &[1, 3]);
        bus.open().unwrap();
        // Servo 3 goes away.
        servos.inner.lock().unwrap().servos.remove(&3);

        let cmds = vec![AxisCmd::default(); 2];
        let mut out = vec![AxisReply::SILENT; 2];
        bus.cycle(&cmds, &mut out).unwrap();
        assert_eq!(out[0].status, AxisStatus::Ok);
        assert_eq!(out[1].status, AxisStatus::Silent);
        assert_eq!(out[1].position, None);
    }

    /// Every tenth tick reads the wide block; voltage and temperature come
    /// with it and not otherwise.
    #[test]
    fn the_wide_block_rides_every_tenth_tick() {
        let servos = FakeServos::new();
        servos.add(1, 1020, true);
        let mut bus = bus(&servos, &[1]);
        bus.open().unwrap();
        let cmds = vec![AxisCmd::default(); 1];
        let mut out = vec![AxisReply::SILENT; 1];
        for tick in 1..=10u64 {
            bus.cycle(&cmds, &mut out).unwrap();
            let wide = tick % 10 == 0;
            assert_eq!(out[0].voltage.is_some(), wide, "tick {tick}");
        }
        assert_eq!(out[0].voltage, Some(Volts(12.0)));
        assert_eq!(out[0].motor_temperature, Some(Celsius(31.0)));
    }

    /// Closing writes torque off straight to the wire, one servo at a time.
    #[test]
    fn closing_frees_every_servo() {
        let servos = FakeServos::new();
        servos.add(1, 1020, true);
        servos.add(2, 1020, true);
        let mut bus = bus(&servos, &[1, 2]);
        bus.open().unwrap();
        let cmds = vec![drive(Rev(0.0)); 2];
        let mut out = vec![AxisReply::SILENT; 2];
        bus.cycle(&cmds, &mut out).unwrap();
        assert_eq!(servos.register(2, 64), Some(1));
        bus.close().unwrap();
        assert_eq!(servos.register(1, 64), Some(0));
        assert_eq!(servos.register(2, 64), Some(0));
        assert!(matches!(bus.cycle(&cmds, &mut out), Err(Error::Closed)));
    }
}
