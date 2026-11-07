//! What every subcommand that drives a machine shares: building the buses
//! this binary carries, waiting for the runtime to see its axes, and the
//! sticky command record a client keeps.
//!
//! `run`, `bringup` and `rt` all go through here, so the one-axis tool and
//! the whole-machine tool open hardware the same way and read the same
//! state record. What the CLI can do is a lower bound on what the runtime
//! can do.

use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::bus::AxisStatus;
use drlabs_core::lowlevel::{mode, LowCmd, LowState, MotorCmd};
use drlabs_rt::{BusConfig, Config, Daemon, Handle, JointConfig};
use drlabs_safety::{Blocked, TripCode};

/// Starts the runtime on every bus this binary can open.
pub fn start(config: &Config) -> Result<Handle, String> {
    Daemon::start(config, &factory)
}

/// Makes the bus for one group of joints. The daemon does not know the
/// drivers; this binary does.
pub fn factory(bus: &BusConfig, joints: &[&JointConfig]) -> Result<Box<dyn drlabs_core::Bus>, String> {
    match bus.transport.as_str() {
        "sim" => drlabs_rt::daemon::sim_factory(bus, joints),
        "serial" => {
            if let Some(joint) = joints.iter().find(|joint| joint.driver != "dynamixel") {
                return Err(format!(
                    "bus {:?}: joint {:?} is a {:?}; a serial bus carries dynamixel joints only",
                    bus.name, joint.name, joint.driver
                ));
            }
            if bus.port.is_empty() {
                return Err(format!("bus {:?}: a serial bus needs a port", bus.name));
            }
            let baud = if bus.baud == 0 { 1_000_000 } else { bus.baud };
            let port = drlabs_transport::serial::open(&bus.port, baud)
                .map_err(|error| format!("bus {:?}: open {}: {error}", bus.name, bus.port))?;
            let shared = drlabs_transport::serial::SharedPort::new(Box::new(port));
            let mut servos = Vec::with_capacity(joints.len());
            let mut overrides = Vec::with_capacity(joints.len());
            for joint in joints {
                let profile = match joint.options.get("profile") {
                    Some(name) => Some(drlabs_dynamixel::profile::profile_named(name).ok_or_else(
                        || format!("joint {:?}: unknown profile {name:?}", joint.name),
                    )?),
                    None => None,
                };
                servos.push((joint.id as u8, profile));
                overrides.push(
                    drlabs_dynamixel::torque_override(&joint.options)
                        .map_err(|error| format!("joint {:?}: {error}", joint.name))?,
                );
            }
            let period = Duration::from_secs_f64(1.0 / bus.rate_hz.max(1) as f64);
            Ok(Box::new(
                drlabs_dynamixel::DynamixelBus::new(shared, &servos)
                    .with_port_name(&bus.port, baud)
                    .with_period(period)
                    .with_torque_overrides(&overrides),
            ))
        }
        "fdcanusb" => {
            if let Some(joint) = joints.iter().find(|joint| joint.driver != "moteus") {
                return Err(format!(
                    "bus {:?}: joint {:?} is a {:?}; an fdcanusb bus carries moteus joints only \
                     in this version (robstride and odrive have no bus driver yet)",
                    bus.name, joint.name, joint.driver
                ));
            }
            let port = if bus.port.is_empty() {
                crate::probe::auto_detect()?
            } else {
                bus.port.clone()
            };
            let device = drlabs_transport::fdcanusb::open(&port)
                .map_err(|error| format!("bus {:?}: open {port}: {error}", bus.name))?;
            let shared = Arc::new(drlabs_transport::can::SharedBus::new(Box::new(device)));
            let ids: Vec<u8> = joints
                .iter()
                .map(|joint| {
                    u8::try_from(joint.id).map_err(|_| {
                        format!(
                            "joint {:?}: CAN id {} is out of range (1..126)",
                            joint.name, joint.id
                        )
                    })
                })
                .collect::<Result<_, _>>()?;
            Ok(Box::new(drlabs_moteus::MoteusBus::new(shared, &ids)))
        }
        other => Err(format!(
            "bus {:?}: transport {other:?} is not carried by the runtime yet",
            bus.name
        )),
    }
}

/// Waits until every joint has reported a position, or says which have not.
///
/// A bus that failed to open latches a transport stop rather than failing
/// the start, so this is also where that is noticed.
pub fn wait_for_telemetry(
    handle: &mut Handle,
    config: &Config,
    budget: Duration,
) -> Result<LowState, String> {
    let deadline = Instant::now() + budget;
    loop {
        let state = handle.state();
        if state.estop != 0 {
            return Err(format!(
                "stopped before any joint reported: {}",
                trip_name(state.estop_code)
            ));
        }
        let silent: Vec<&str> = config
            .joints
            .iter()
            .enumerate()
            .filter(|(index, _)| !state.state[*index].q.is_finite())
            .map(|(_, joint)| joint.name.as_str())
            .collect();
        if silent.is_empty() {
            return Ok(state);
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "no telemetry from {} within {:.0} s",
                silent.join(", "),
                budget.as_secs_f64()
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Waits until every joint asked to drive is being driven, or says why
/// each one is not. The caller keeps the command fresh meanwhile.
pub fn wait_until_driving(
    handle: &mut Handle,
    config: &Config,
    commander: &mut Commander,
    budget: Duration,
) -> Result<(), String> {
    let deadline = Instant::now() + budget;
    loop {
        commander.send(handle, 0);
        let state = handle.state();
        if state.estop != 0 {
            return Err(format!("stopped: {}", trip_name(state.estop_code)));
        }
        let waiting: Vec<String> = config
            .joints
            .iter()
            .enumerate()
            .filter(|(index, _)| mode::drives(commander.cmd.cmd[*index].mode))
            .filter(|(index, _)| !is_driven(state.state[*index].status))
            .map(|(index, joint)| {
                let motor = state.state[index];
                match motor.blocked {
                    0 => format!("{} ({})", joint.name, status_name(motor.status)),
                    why => format!("{} ({})", joint.name, blocked_name(why)),
                }
            })
            .collect();
        if waiting.is_empty() {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("did not energise: {}", waiting.join(", ")));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Whether a status byte says the axis is being driven and answering.
pub fn is_driven(status: u8) -> bool {
    matches!(
        AxisStatus::from_u8(status),
        Some(AxisStatus::Ok | AxisStatus::Alert | AxisStatus::Refused)
    )
}

pub fn status_name(status: u8) -> &'static str {
    AxisStatus::from_u8(status)
        .map(AxisStatus::as_str)
        .unwrap_or("unknown")
}

pub fn blocked_name(byte: u8) -> &'static str {
    match byte {
        0 => "",
        1 => Blocked::NoTelemetry.as_str(),
        2 => Blocked::VoltageLow.as_str(),
        3 => Blocked::VoltageHigh.as_str(),
        4 => Blocked::ControllerFault.as_str(),
        5 => Blocked::NoPosition.as_str(),
        6 => Blocked::OutsideEnvelope.as_str(),
        7 => Blocked::Estop.as_str(),
        _ => "blocked",
    }
}

pub fn trip_name(code: u16) -> &'static str {
    TripCode::from_u8(code as u8)
        .map(TripCode::code)
        .unwrap_or("unknown")
}

/// The command record a client keeps and re-sends.
///
/// Entries are sticky: changing one and sending ships the whole record,
/// and sending again without a change keeps the runtime from seeing the
/// command go stale.
pub struct Commander {
    pub cmd: LowCmd,
    seq: u32,
}

impl Commander {
    pub fn new(joints: usize) -> Self {
        let mut cmd = LowCmd::EMPTY;
        cmd.n = joints as u16;
        Self { cmd, seq: 0 }
    }

    pub fn set(&mut self, index: usize, entry: MotorCmd) {
        if let Some(slot) = self.cmd.cmd.get_mut(index) {
            *slot = entry;
        }
    }

    /// Position `q` with velocity feed-forward `dq`, rad and rad/s.
    pub fn drive(&mut self, index: usize, q: f64, dq: f64) {
        self.set(
            index,
            MotorCmd {
                mode: mode::DRIVE,
                q: q as f32,
                dq: dq as f32,
                ..MotorCmd::OFF
            },
        );
    }

    pub fn release_all(&mut self) {
        for entry in self.cmd.cmd.iter_mut() {
            *entry = MotorCmd::OFF;
        }
    }

    pub fn send(&mut self, handle: &mut Handle, flags: u32) {
        self.seq = self.seq.wrapping_add(1).max(1);
        self.cmd.seq = self.seq;
        self.cmd.flags = flags;
        self.cmd.t_ns = 0;
        handle.send(self.cmd);
    }
}

/// Turns every axis off and stops the runtime, giving the bus threads the
/// stop burst before the threads are joined.
pub fn shutdown(mut handle: Handle, commander: &mut Commander) {
    commander.release_all();
    commander.send(&mut handle, 0);
    std::thread::sleep(Duration::from_millis(50));
    handle.stop();
}

/// One line of timekeeping from the runtime, for a report.
pub struct Timing {
    last_tick: u64,
    last_at: Instant,
}

impl Timing {
    pub fn new(state: &LowState) -> Self {
        Self {
            last_tick: state.tick,
            last_at: Instant::now(),
        }
    }

    /// The bus rate since the last call, Hz.
    pub fn rate(&mut self, state: &LowState) -> f64 {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_at).as_secs_f64();
        let rate = if elapsed > 0.0 {
            (state.tick - self.last_tick) as f64 / elapsed
        } else {
            0.0
        };
        self.last_tick = state.tick;
        self.last_at = now;
        rate
    }
}

/// The state of every joint as a table.
pub fn state_table(config: &Config, state: &LowState) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "{:<12}{:<10}{:<14}{:<15}{:<12}{:<8}NOTE\n",
        "JOINT", "STATUS", "POSITION", "VELOCITY", "TORQUE", "FAULT"
    ));
    for (index, joint) in config.joints.iter().enumerate() {
        let motor = state.state[index];
        let number = |value: f32, unit: &str, digits: usize| {
            if value.is_finite() {
                format!("{value:+.digits$} {unit}")
            } else {
                "—".into()
            }
        };
        let note = if state.estop != 0 {
            trip_name(state.estop_code).to_string()
        } else if motor.blocked != 0 {
            blocked_name(motor.blocked).to_string()
        } else {
            String::new()
        };
        out.push_str(&format!(
            "{:<12}{:<10}{:<14}{:<15}{:<12}{:<8}{note}\n",
            joint.name,
            status_name(motor.status),
            number(motor.q, "rad", 4),
            number(motor.dq, "rad/s", 3),
            number(motor.tau, "Nm", 3),
            motor.fault
        ));
    }
    out
}
