//! A Rust client of the runtime, for the tools in this repository that sit
//! above it (the panel). Mirrors `tools/drlabs_client`.
//!
//! States arrive on Cyclone's receive thread and the latest is kept. The
//! command record is sticky: [`Client::set`] changes entries, [`Client::send`]
//! ships the whole record, and a background keep-alive re-sends it so the
//! runtime never sees the command go stale while the client is alive.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use drlabs_core::lowlevel::{flags, mode, LowCmd, LowState, MotorCmd, MAX_AXES};
use serde::Deserialize;

use crate::cdr::{decode_describe, decode_state, encode_cmd, names};
use crate::dds::{DdsError, Participant, Qos, Reader, Topic, Writer};

/// One joint, as the runtime describes it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct JointInfo {
    pub index: usize,
    pub name: String,
    pub bus: String,
    pub driver: String,
    pub id: i32,
    pub min_rad: f64,
    pub max_rad: f64,
    pub max_torque_nm: f64,
    pub max_velocity_rad_s: f64,
    pub max_accel_rad_s2: f64,
}

/// The machine, as the runtime describes it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Description {
    pub protocol: u16,
    #[serde(default)]
    pub robot: String,
    pub rate_hz: u32,
    pub period_ns: u64,
    #[serde(default)]
    pub cmd_timeout_ms: u32,
    #[serde(default)]
    pub on_cmd_timeout: String,
    pub joints: Vec<JointInfo>,
    #[serde(default)]
    pub poses: BTreeMap<String, BTreeMap<String, f64>>,
}

impl Description {
    pub fn index(&self, name: &str) -> Option<usize> {
        self.joints
            .iter()
            .find(|joint| joint.name == name)
            .map(|joint| joint.index)
    }

    pub fn joint(&self, name: &str) -> Option<&JointInfo> {
        self.joints.iter().find(|joint| joint.name == name)
    }
}

struct Shared {
    latest: Mutex<Option<(LowState, Instant)>>,
    description: Mutex<Option<Description>>,
    cmd: Mutex<LowCmd>,
    running: AtomicBool,
    seq: AtomicU32,
}

pub struct Client {
    shared: Arc<Shared>,
    keeper: Option<JoinHandle<()>>,
    // Drop order: readers stop first, then the writer, then the participant.
    _state_reader: Reader,
    _describe_reader: Reader,
    cmd_writer: Arc<Writer>,
    _participant: Participant,
    endpoint: String,
}

impl Client {
    /// Joins `domain` and listens under `rt/<prefix>/…`. Does not wait for
    /// the runtime to be there; [`Client::describe`] does that.
    pub fn connect(domain: u32, prefix: &str, keepalive: Duration) -> Result<Self, DdsError> {
        let participant = Participant::new(domain)?;
        let cmd_topic = Topic::blob(
            &participant,
            &names::topic(prefix, "lowcmd"),
            names::TYPE_LOWCMD,
        )?;
        let state_topic = Topic::blob(
            &participant,
            &names::topic(prefix, "lowstate"),
            names::TYPE_LOWSTATE,
        )?;
        let describe_topic = Topic::blob(
            &participant,
            &names::topic(prefix, "describe"),
            names::TYPE_DESCRIBE,
        )?;

        let shared = Arc::new(Shared {
            latest: Mutex::new(None),
            description: Mutex::new(None),
            cmd: Mutex::new(LowCmd::EMPTY),
            running: AtomicBool::new(true),
            seq: AtomicU32::new(0),
        });

        let state_reader = {
            let shared = Arc::clone(&shared);
            Reader::new(
                &participant,
                &state_topic,
                &Qos::best_effort(1),
                Arc::new(move |bytes: &[u8]| {
                    if let Ok(state) = decode_state(bytes) {
                        *shared.latest.lock().unwrap() = Some((state, Instant::now()));
                    }
                }),
            )?
        };
        let describe_reader = {
            let shared = Arc::clone(&shared);
            Reader::new(
                &participant,
                &describe_topic,
                &Qos::latched(),
                Arc::new(move |bytes: &[u8]| {
                    if let Ok(json) = decode_describe(bytes) {
                        if let Ok(description) = serde_json::from_str::<Description>(&json) {
                            *shared.description.lock().unwrap() = Some(description);
                        }
                    }
                }),
            )?
        };
        let cmd_writer = Arc::new(Writer::new(&participant, &cmd_topic, &Qos::best_effort(1))?);

        let keeper = {
            let shared = Arc::clone(&shared);
            let writer = Arc::clone(&cmd_writer);
            std::thread::Builder::new()
                .name("drlabs-client-keepalive".into())
                .spawn(move || {
                    let mut out = Vec::new();
                    while shared.running.load(Ordering::Relaxed) {
                        std::thread::sleep(keepalive);
                        let cmd = {
                            let mut cmd = shared.cmd.lock().unwrap();
                            if cmd.n == 0 {
                                // Nothing commanded yet: nothing to keep fresh.
                                continue;
                            }
                            cmd.seq = next_seq(&shared.seq);
                            cmd.flags = 0;
                            *cmd
                        };
                        encode_cmd(&cmd, &mut out);
                        let _ = writer.write(&out);
                    }
                })
                .map_err(|error| DdsError(format!("keepalive thread: {error}")))?
        };

        Ok(Self {
            shared,
            keeper: Some(keeper),
            _state_reader: state_reader,
            _describe_reader: describe_reader,
            cmd_writer,
            _participant: participant,
            endpoint: format!("dds domain {domain}, rt/{prefix}"),
        })
    }

    /// Waits for the latched description.
    pub fn describe(&self, timeout: Duration) -> Result<Description, String> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(description) = self.shared.description.lock().unwrap().clone() {
                return Ok(description);
            }
            if Instant::now() >= deadline {
                return Err(format!("no description on {}", self.endpoint));
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// The latest state and how old it is.
    pub fn latest(&self) -> Option<(LowState, Duration)> {
        self.shared
            .latest
            .lock()
            .unwrap()
            .map(|(state, at)| (state, at.elapsed()))
    }

    /// Changes one joint's entry without sending.
    pub fn set(&self, index: usize, entry: MotorCmd) {
        if index >= MAX_AXES {
            return;
        }
        let mut cmd = self.shared.cmd.lock().unwrap();
        cmd.cmd[index] = entry;
        cmd.n = cmd.n.max(index as u16 + 1);
    }

    /// Sets the record's length, so entries never touched still travel as
    /// OFF.
    pub fn set_axes(&self, n: usize) {
        let mut cmd = self.shared.cmd.lock().unwrap();
        cmd.n = cmd.n.max(n.min(MAX_AXES) as u16);
    }

    /// Drives one joint to `q`.
    pub fn drive(&self, index: usize, q: f32) {
        self.set(
            index,
            MotorCmd {
                mode: mode::DRIVE,
                q,
                ..MotorCmd::OFF
            },
        );
    }

    /// Asks for one joint in any mode, with the full entry.
    pub fn set_mode(&self, index: usize, mode: u8, q: f32, dq: f32, tau: f32) {
        self.set(
            index,
            MotorCmd {
                mode,
                q,
                dq,
                tau,
                ..MotorCmd::OFF
            },
        );
    }

    /// Re-references one joint: its current output position becomes `q`.
    /// The runtime acts once, on the change into `ZERO`, so the entry has
    /// to go back to `OFF` (via [`Client::release`]) before a second one.
    pub fn zero(&self, index: usize, q: f32) {
        self.set(
            index,
            MotorCmd {
                mode: mode::ZERO,
                q,
                ..MotorCmd::OFF
            },
        );
    }

    /// Turns one joint off.
    pub fn release(&self, index: usize) {
        if let Some(entry) = self.shared.cmd.lock().unwrap().cmd.get_mut(index) {
            entry.mode = mode::OFF;
        }
    }

    pub fn release_all(&self) {
        for entry in self.shared.cmd.lock().unwrap().cmd.iter_mut() {
            entry.mode = mode::OFF;
        }
    }

    /// The entry as it stands.
    pub fn entry(&self, index: usize) -> Option<MotorCmd> {
        self.shared.cmd.lock().unwrap().cmd.get(index).copied()
    }

    /// Ships the record with `flags`.
    pub fn send(&self, flags: u32) -> Result<(), DdsError> {
        let cmd = {
            let mut cmd = self.shared.cmd.lock().unwrap();
            cmd.seq = next_seq(&self.shared.seq);
            cmd.flags = flags;
            *cmd
        };
        let mut out = Vec::new();
        encode_cmd(&cmd, &mut out);
        self.cmd_writer.write(&out)
    }

    /// Latches the robot-wide stop (CiA 402: a quick stop that lands in
    /// Fault).
    pub fn quick_stop(&self) -> Result<(), DdsError> {
        self.send(flags::QUICK_STOP)
    }

    /// Clears a latched stop. Every joint is off afterwards.
    pub fn fault_reset(&self) -> Result<(), DdsError> {
        self.release_all();
        self.send(flags::FAULT_RESET)
    }

    /// Where this client is listening, for a log line.
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Relaxed);
        if let Some(thread) = self.keeper.take() {
            let _ = thread.join();
        }
    }
}

fn next_seq(seq: &AtomicU32) -> u32 {
    let next = seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    if next == 0 {
        1
    } else {
        next
    }
}
