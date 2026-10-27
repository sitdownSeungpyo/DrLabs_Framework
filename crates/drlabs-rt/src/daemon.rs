//! Assembling a machine from its description and running it.
//!
//! The daemon owns the threads. A [`Handle`] is how the rest of the process
//! talks to them: publish a command, take a state snapshot, stop. Transports
//! that carry those records out of the process (UDP, later DDS) are clients
//! of the handle, not part of it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use drlabs_core::bus::Bus;
use drlabs_core::lowlevel::{LowCmd, LowState, MAX_AXES};
use drlabs_safety::Config as SafetyConfig;

use crate::config::{BusConfig, Config, JointConfig};
use crate::exchange::{triple, Broadcast, Reader, Stamped};
use crate::gate::{Estop, TimeoutPolicy};
use crate::mapping::Mapping;
use crate::worker::{AxisSpec, BusState, BusWorker, WorkerConfig};

/// Makes the bus for one group of joints. The daemon does not know the
/// drivers; the binary that links them supplies this.
pub type BusFactory = dyn Fn(&BusConfig, &[&JointConfig]) -> Result<Box<dyn Bus>, String>;

/// The buses this crate can build on its own: the simulator.
pub fn sim_factory(bus: &BusConfig, joints: &[&JointConfig]) -> Result<Box<dyn Bus>, String> {
    match bus.transport.as_str() {
        "sim" => Ok(Box::new(drlabs_sim::SimBus::new(joints.len()))),
        other => Err(format!(
            "bus {:?}: this build cannot open a {other:?} transport",
            bus.name
        )),
    }
}

/// One bus's threads-eye view, kept by the handle to assemble the state.
struct BusTap {
    joints: Vec<usize>,
    reader: Reader<BusState>,
}

/// The running machine.
pub struct Handle {
    cmd: Broadcast<Stamped<LowCmd>>,
    taps: Vec<BusTap>,
    estop: Estop,
    running: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
    joints: usize,
    period: Duration,
    epoch: Instant,
    seq: u32,
}

impl Handle {
    /// Publishes a command to every bus. The sequence number is the
    /// client's; a client that sends the same number twice is sending the
    /// same command twice.
    pub fn send(&mut self, cmd: LowCmd) {
        self.seq = cmd.seq;
        self.cmd.publish(Stamped::now(cmd));
    }

    /// The latest state of every joint, assembled from the buses.
    pub fn state(&mut self) -> LowState {
        let mut out = LowState::EMPTY;
        out.n = self.joints as u16;
        out.period_ns = self.period.as_nanos().min(u32::MAX as u128) as u32;
        for tap in &mut self.taps {
            let (bus, _) = tap.reader.latest();
            for &joint in &tap.joints {
                out.state[joint] = bus.state[joint];
            }
            out.tick = out.tick.max(bus.tick);
            out.t_ns = out.t_ns.max(bus.t_ns);
            out.deadline_miss = out.deadline_miss.saturating_add(bus.deadline_miss);
            out.cmd_seq = out.cmd_seq.max(bus.cmd_seq);
        }
        if let Some(code) = self.estop.code() {
            out.estop = 1;
            out.estop_code = code as u16;
        }
        out.seq = out.tick.min(u32::MAX as u64) as u32;
        out
    }

    /// The worst lateness any bus has seen, ns. For a report line.
    pub fn late_max_ns(&mut self) -> u32 {
        self.taps
            .iter_mut()
            .map(|tap| tap.reader.latest().0.late_max_ns)
            .max()
            .unwrap_or(0)
    }

    pub fn estop(&self) -> &Estop {
        &self.estop
    }

    pub fn epoch(&self) -> Instant {
        self.epoch
    }

    pub fn joints(&self) -> usize {
        self.joints
    }

    /// Stops every bus thread and waits for it to close its bus.
    pub fn stop(mut self) {
        self.running.store(false, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

impl Drop for Handle {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

/// Builds and starts the machine.
pub struct Daemon;

impl Daemon {
    /// Starts one thread per bus. Returns once every thread has been
    /// spawned; a bus that fails to open latches a transport stop rather
    /// than failing this call, so the state record says what happened.
    pub fn start(config: &Config, factory: &BusFactory) -> Result<Handle, String> {
        if config.joints.len() > MAX_AXES {
            return Err(format!(
                "{} joints, but a runtime carries at most {MAX_AXES}",
                config.joints.len()
            ));
        }
        let on_timeout = TimeoutPolicy::parse(&config.api.on_cmd_timeout)
            .ok_or_else(|| format!("on_cmd_timeout {:?}", config.api.on_cmd_timeout))?;

        let mut cmd = Broadcast::new();
        let estop = Estop::new();
        let running = Arc::new(AtomicBool::new(true));
        let epoch = Instant::now();
        let mut taps = Vec::new();
        let mut workers = Vec::new();

        for bus_name in config.bus_names() {
            let bus_config = config.bus(&bus_name).cloned().unwrap_or_else(|| BusConfig {
                name: bus_name.clone(),
                transport: implicit_transport(config),
                port: String::new(),
                baud: 0,
                rate_hz: 0,
            });
            let members: Vec<(usize, &JointConfig)> = config
                .joints
                .iter()
                .enumerate()
                .filter(|(_, joint)| joint.bus == bus_name)
                .collect();
            let joints: Vec<&JointConfig> = members.iter().map(|(_, j)| *j).collect();
            let bus = factory(&bus_config, &joints)?;
            if bus.axes() != joints.len() {
                return Err(format!(
                    "bus {bus_name:?}: the driver built {} axes for {} joints",
                    bus.axes(),
                    joints.len()
                ));
            }

            let specs: Vec<AxisSpec> = members
                .iter()
                .map(|(index, joint)| {
                    let mapping = Mapping::of(joint);
                    AxisSpec {
                        joint: *index,
                        mapping,
                        safety: mapping.safety_config(&joint.limits, SafetyConfig::default()),
                    }
                })
                .collect();

            let rate = config.rate_for_bus(&bus_name);
            let period = Duration::from_secs_f64(1.0 / rate as f64);
            let worker_config = WorkerConfig {
                name: bus_name.clone(),
                period,
                cmd_timeout: Duration::from_millis(config.api.cmd_timeout_ms as u64),
                on_timeout,
                deadline_miss_trip: config.rt.deadline_miss_trip,
                full_every: 10,
                resend_every: 50,
                priority: config.rt.priority,
                cpu: if config.rt.cpus.is_empty() {
                    None
                } else {
                    Some(config.rt.cpus[taps.len() % config.rt.cpus.len()])
                },
            };

            let cmd_rx = cmd.subscribe(Stamped::now(LowCmd::EMPTY));
            let (state_tx, state_rx) = triple(BusState::EMPTY);
            taps.push(BusTap {
                joints: members.iter().map(|(index, _)| *index).collect(),
                reader: state_rx,
            });
            workers.push(BusWorker::new(
                worker_config,
                bus,
                &specs,
                cmd_rx,
                state_tx,
                estop.clone(),
                Arc::clone(&running),
            ));
        }

        let mut threads = Vec::new();
        for (worker, name) in workers.into_iter().zip(config.bus_names()) {
            let thread = std::thread::Builder::new()
                .name(format!("drlabs-bus-{name}"))
                .spawn(move || worker.run(epoch))
                .map_err(|error| format!("cannot start bus thread {name:?}: {error}"))?;
            threads.push(thread);
        }

        Ok(Handle {
            cmd,
            taps,
            estop,
            running,
            threads,
            joints: config.joints.len(),
            period: Duration::from_secs_f64(1.0 / config.rate_hz as f64),
            epoch,
            seq: 0,
        })
    }
}

/// A description with no `buses` section puts every joint on one unnamed
/// bus; its transport is whatever the joints' driver implies.
fn implicit_transport(config: &Config) -> String {
    if config.joints.iter().all(|joint| joint.driver == "sim") {
        "sim".into()
    } else {
        "serial".into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::bus::AxisStatus;
    use drlabs_core::lowlevel::{mode, MotorCmd};

    fn sim_config(axes: usize, buses: usize) -> Config {
        let mut text = String::from(r#"{"robot":"sim","rate_hz":500,"buses":["#);
        for b in 0..buses {
            if b > 0 {
                text.push(',');
            }
            text.push_str(&format!(r#"{{"name":"sim{b}","transport":"sim"}}"#));
        }
        text.push_str(r#"],"joints":["#);
        for i in 0..axes {
            if i > 0 {
                text.push(',');
            }
            text.push_str(&format!(
                r#"{{"name":"j{i}","bus":"sim{}","driver":"sim","id":{},"gear_ratio":1,"direction":1,
                   "limits":{{"min_rad":-1.5,"max_rad":1.5,"max_torque_nm":1,"max_velocity_rad_s":6,"max_accel_rad_s2":30}}}}"#,
                i % buses,
                i + 1
            ));
        }
        text.push_str("]}");
        Config::parse(&text).expect("a valid sim description")
    }

    /// Twenty-six simulated joints on two buses, driven for a moment: every
    /// one reports, every one goes where it is sent, and the threads stop
    /// when asked.
    #[test]
    fn a_two_bus_machine_runs_and_follows_a_command() {
        let config = sim_config(26, 2);
        let mut handle = Daemon::start(&config, &sim_factory).unwrap();

        let mut cmd = LowCmd::EMPTY;
        cmd.n = 26;
        for (i, motor) in cmd.cmd.iter_mut().enumerate().take(26) {
            *motor = MotorCmd {
                mode: mode::DRIVE,
                q: 0.2 + 0.01 * i as f32,
                ..MotorCmd::OFF
            };
        }
        let started = Instant::now();
        let mut seq = 0;
        while started.elapsed() < Duration::from_millis(1500) {
            seq += 1;
            cmd.seq = seq;
            handle.send(cmd);
            std::thread::sleep(Duration::from_millis(10));
        }

        let state = handle.state();
        assert_eq!(state.n, 26);
        assert_eq!(state.estop, 0, "code {}", state.estop_code);
        assert!(state.tick > 100, "ticks: {}", state.tick);
        for (i, s) in state.active().iter().enumerate() {
            assert_eq!(s.status, AxisStatus::Ok as u8, "joint {i}: {s:?}");
            let want = 0.2 + 0.01 * i as f32;
            assert!((s.q - want).abs() < 0.05, "joint {i}: {s:?}");
        }
        handle.stop();
    }

    #[test]
    fn a_transport_this_build_cannot_open_is_refused_by_name() {
        let mut config = sim_config(2, 1);
        config.buses[0].transport = "serial".into();
        let error = match Daemon::start(&config, &sim_factory) {
            Ok(_) => panic!("a serial bus must not open in a sim-only build"),
            Err(error) => error,
        };
        assert!(error.contains("serial"), "{error}");
    }
}
