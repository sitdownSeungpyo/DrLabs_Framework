//! The UDP endpoint.
//!
//! Two threads, neither real-time. The receiver blocks on the socket and
//! hands every valid command to the daemon at once; the publisher wakes on
//! the state cadence, takes a snapshot, and sends it to everyone who has
//! spoken in the last two seconds. A client that falls silent simply stops
//! being sent to; a client that sends garbage is ignored, frame by frame.

use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use drlabs_rt::Handle;
use drlabs_timing::sleep_until;

use crate::frame::{decode, encode_describe_reply, encode_state, Frame, MAX_FRAME};

/// How long after its last frame a client keeps receiving states.
const SUBSCRIBER_TTL: Duration = Duration::from_secs(2);

pub struct UdpServer {
    running: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
    local: SocketAddr,
}

impl UdpServer {
    /// Binds `bind` and starts serving `handle`. `state_period` is how often
    /// a state frame goes out.
    pub fn start(
        bind: &str,
        handle: Arc<Mutex<Handle>>,
        describe: String,
        state_period: Duration,
    ) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(bind)?;
        let local = socket.local_addr()?;
        // The receiver wakes every so often to notice a stop request.
        socket.set_read_timeout(Some(Duration::from_millis(200)))?;
        let sender = socket.try_clone()?;

        let running = Arc::new(AtomicBool::new(true));
        let subscribers: Arc<Mutex<HashMap<SocketAddr, Instant>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let rx = {
            let running = Arc::clone(&running);
            let handle = Arc::clone(&handle);
            let subscribers = Arc::clone(&subscribers);
            std::thread::Builder::new()
                .name("drlabs-api-rx".into())
                .spawn(move || {
                    let mut buf = vec![0u8; MAX_FRAME.max(2048)];
                    let mut reply = Vec::new();
                    while running.load(Ordering::Relaxed) {
                        let (n, from) = match socket.recv_from(&mut buf) {
                            Ok(received) => received,
                            Err(_) => continue,
                        };
                        match decode(&buf[..n]) {
                            Ok(Frame::Cmd(cmd)) => {
                                subscribers.lock().unwrap().insert(from, Instant::now());
                                if let Ok(mut handle) = handle.lock() {
                                    handle.send(cmd);
                                }
                            }
                            Ok(Frame::Describe) => {
                                subscribers.lock().unwrap().insert(from, Instant::now());
                                encode_describe_reply(&describe, &mut reply);
                                let _ = socket.send_to(&reply, from);
                            }
                            // A client's own state or description echoed back:
                            // not ours to act on.
                            Ok(_) => {}
                            Err(_) => {}
                        }
                    }
                })?
        };

        let tx = {
            let running = Arc::clone(&running);
            std::thread::Builder::new()
                .name("drlabs-api-tx".into())
                .spawn(move || {
                    let mut out = Vec::with_capacity(MAX_FRAME);
                    let mut next = Instant::now() + state_period;
                    let spin = Duration::from_micros(200);
                    while running.load(Ordering::Relaxed) {
                        sleep_until(next, None, spin);
                        next += state_period;
                        if next + state_period < Instant::now() {
                            next = Instant::now() + state_period;
                        }
                        let state = match handle.lock() {
                            Ok(mut handle) => handle.state(),
                            Err(_) => continue,
                        };
                        encode_state(&state, &mut out);
                        let now = Instant::now();
                        let mut subscribers = subscribers.lock().unwrap();
                        subscribers.retain(|_, seen| now.duration_since(*seen) < SUBSCRIBER_TTL);
                        for address in subscribers.keys() {
                            let _ = sender.send_to(&out, address);
                        }
                    }
                })?
        };

        Ok(Self {
            running,
            threads: vec![rx, tx],
            local,
        })
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.local
    }

    pub fn stop(mut self) {
        self.running.store(false, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

impl Drop for UdpServer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{encode_cmd, encode_describe};
    use drlabs_core::lowlevel::{mode, LowCmd, MotorCmd};
    use drlabs_core::AxisStatus;
    use drlabs_rt::{daemon::sim_factory, Config, Daemon};

    fn sim_config() -> Config {
        Config::parse(
            r#"{"robot":"t","rate_hz":500,"buses":[{"name":"s","transport":"sim"}],
                "joints":[
                 {"name":"a","bus":"s","driver":"sim","id":1,"gear_ratio":1,"direction":1,
                  "limits":{"min_rad":-1.5,"max_rad":1.5,"max_torque_nm":1,"max_velocity_rad_s":6,"max_accel_rad_s2":30}},
                 {"name":"b","bus":"s","driver":"sim","id":2,"gear_ratio":1,"direction":1,
                  "limits":{"min_rad":-1.5,"max_rad":1.5,"max_torque_nm":1,"max_velocity_rad_s":6,"max_accel_rad_s2":30}}]}"#,
        )
        .unwrap()
    }

    /// A client over the loopback: describe, command, and see the state come
    /// back with the axes driving where they were sent.
    #[test]
    fn a_client_can_describe_command_and_observe() {
        let config = sim_config();
        let handle = Arc::new(Mutex::new(Daemon::start(&config, &sim_factory).unwrap()));
        let server = UdpServer::start(
            "127.0.0.1:0",
            Arc::clone(&handle),
            crate::describe::describe_json(&config),
            Duration::from_millis(2),
        )
        .unwrap();
        let server_addr = server.local_addr();

        let client = UdpSocket::bind("127.0.0.1:0").unwrap();
        client
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();
        let mut out = Vec::new();
        let mut buf = vec![0u8; MAX_FRAME];

        encode_describe(&mut out);
        client.send_to(&out, server_addr).unwrap();
        let described = loop {
            let (n, _) = client.recv_from(&mut buf).unwrap();
            if let Ok(Frame::DescribeReply(json)) = decode(&buf[..n]) {
                break json.to_string();
            }
        };
        assert!(described.contains("\"name\":\"b\""), "{described}");

        let mut cmd = LowCmd::EMPTY;
        cmd.n = 2;
        cmd.cmd[1] = MotorCmd {
            mode: mode::DRIVE,
            q: 0.3,
            ..MotorCmd::OFF
        };
        let started = Instant::now();
        let mut last = None;
        while started.elapsed() < Duration::from_millis(1500) {
            cmd.seq += 1;
            encode_cmd(&cmd, &mut out);
            client.send_to(&out, server_addr).unwrap();
            if let Ok((n, _)) = client.recv_from(&mut buf) {
                if let Ok(Frame::State(state)) = decode(&buf[..n]) {
                    last = Some(state);
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let state = last.expect("states were published");
        assert_eq!(state.n, 2);
        assert_eq!(state.state[0].status, AxisStatus::Off as u8);
        assert_eq!(
            state.state[1].status,
            AxisStatus::Ok as u8,
            "{:?}",
            state.state[1]
        );
        assert!(
            (state.state[1].q - 0.3).abs() < 0.05,
            "{:?}",
            state.state[1]
        );
        assert!(state.cmd_seq > 0);

        server.stop();
        match Arc::try_unwrap(handle) {
            Ok(mutex) => mutex.into_inner().unwrap().stop(),
            Err(_) => panic!("the server kept a handle"),
        }
    }
}
