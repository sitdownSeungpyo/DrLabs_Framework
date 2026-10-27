//! A Rust client of the runtime, for the tools in this repository that sit
//! above it (the panel, a bridge). Mirrors `tools/drlabs_client`.
//!
//! States arrive on a reader thread and the latest is kept. The command
//! record is sticky: [`Client::set`] changes entries, [`Client::send`] ships
//! the whole record, and a background keep-alive re-sends it so the runtime
//! never sees the command go stale while the client is alive.

use std::collections::BTreeMap;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use drlabs_core::lowlevel::{flags, mode, LowCmd, LowState, MotorCmd, MAX_AXES};
use serde::Deserialize;

use crate::frame::{decode, encode_cmd, encode_describe, Frame};

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
    socket: UdpSocket,
    server: SocketAddr,
    shared: Arc<Shared>,
    threads: Vec<JoinHandle<()>>,
}

impl Client {
    /// Opens a socket to the runtime at `server` and starts listening. Does
    /// not wait for the runtime to answer; [`Client::describe`] does that.
    pub fn connect(server: impl ToSocketAddrs, keepalive: Duration) -> std::io::Result<Self> {
        let server = server
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "no address"))?;
        let socket = UdpSocket::bind(if server.is_ipv4() {
            "0.0.0.0:0"
        } else {
            "[::]:0"
        })?;
        socket.set_read_timeout(Some(Duration::from_millis(200)))?;
        let shared = Arc::new(Shared {
            latest: Mutex::new(None),
            description: Mutex::new(None),
            cmd: Mutex::new(LowCmd::EMPTY),
            running: AtomicBool::new(true),
            seq: AtomicU32::new(0),
        });

        let reader = {
            let socket = socket.try_clone()?;
            let shared = Arc::clone(&shared);
            std::thread::Builder::new()
                .name("drlabs-client-rx".into())
                .spawn(move || {
                    let mut buf = vec![0u8; 65536];
                    while shared.running.load(Ordering::Relaxed) {
                        let n = match socket.recv_from(&mut buf) {
                            Ok((n, _)) => n,
                            Err(_) => continue,
                        };
                        match decode(&buf[..n]) {
                            Ok(Frame::State(state)) => {
                                *shared.latest.lock().unwrap() = Some((state, Instant::now()));
                            }
                            Ok(Frame::DescribeReply(json)) => {
                                if let Ok(description) = serde_json::from_str::<Description>(json) {
                                    *shared.description.lock().unwrap() = Some(description);
                                }
                            }
                            _ => {}
                        }
                    }
                })?
        };

        let keeper = {
            let socket = socket.try_clone()?;
            let shared = Arc::clone(&shared);
            std::thread::Builder::new()
                .name("drlabs-client-keepalive".into())
                .spawn(move || {
                    let mut out = Vec::new();
                    while shared.running.load(Ordering::Relaxed) {
                        std::thread::sleep(keepalive);
                        let cmd = {
                            let mut cmd = shared.cmd.lock().unwrap();
                            if cmd.n == 0 {
                                // Nothing commanded yet: just keep the states
                                // coming.
                                encode_describe(&mut out);
                                let _ = socket.send_to(&out, server);
                                continue;
                            }
                            cmd.seq = next_seq(&shared.seq);
                            cmd.flags = 0;
                            *cmd
                        };
                        encode_cmd(&cmd, &mut out);
                        let _ = socket.send_to(&out, server);
                    }
                })?
        };

        Ok(Self {
            socket,
            server,
            shared,
            threads: vec![reader, keeper],
        })
    }

    /// Asks for, and waits for, the description.
    pub fn describe(&self, timeout: Duration) -> Result<Description, String> {
        if let Some(description) = self.shared.description.lock().unwrap().clone() {
            return Ok(description);
        }
        let deadline = Instant::now() + timeout;
        let mut out = Vec::new();
        encode_describe(&mut out);
        while Instant::now() < deadline {
            let _ = self.socket.send_to(&out, self.server);
            std::thread::sleep(Duration::from_millis(100));
            if let Some(description) = self.shared.description.lock().unwrap().clone() {
                return Ok(description);
            }
        }
        Err(format!("no description from {}", self.server))
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
    pub fn send(&self, flags: u32) -> std::io::Result<()> {
        let cmd = {
            let mut cmd = self.shared.cmd.lock().unwrap();
            cmd.seq = next_seq(&self.shared.seq);
            cmd.flags = flags;
            *cmd
        };
        let mut out = Vec::new();
        encode_cmd(&cmd, &mut out);
        self.socket.send_to(&out, self.server).map(|_| ())
    }

    pub fn estop(&self) -> std::io::Result<()> {
        self.send(flags::ESTOP)
    }

    /// Clears a latched stop. Every joint is off afterwards.
    pub fn reset_estop(&self) -> std::io::Result<()> {
        self.release_all();
        self.send(flags::RESET_ESTOP)
    }

    pub fn server(&self) -> SocketAddr {
        self.server
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

impl Drop for Client {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}
