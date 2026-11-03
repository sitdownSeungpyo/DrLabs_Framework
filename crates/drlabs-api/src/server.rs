//! The runtime's DDS endpoint.
//!
//! Three topics on one participant. The description is written once,
//! latched, so a client that arrives later still gets it. States go out on a
//! publisher thread at the state cadence. Commands arrive on Cyclone's
//! receive thread and are handed to the daemon at once — the handle's
//! `send` is a triple-buffer publish, so the bus threads never wait on it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use drlabs_rt::Handle;
use drlabs_timing::sleep_until;

use crate::cdr::{decode_cmd, encode_describe, encode_state, names};
use crate::dds::{DdsError, Participant, Qos, Reader, Topic, Writer};

/// How many commands a reader keeps if the callback falls behind a burst.
const CMD_DEPTH: i32 = 4;

pub struct DdsServer {
    running: Arc<AtomicBool>,
    publisher: Option<JoinHandle<()>>,
    // Dropped in this order: the reader first (no more callbacks into the
    // handle), then the writers, then the participant that owns them all.
    _cmd_reader: Reader,
    _state_writer: Arc<Writer>,
    _describe_writer: Writer,
    _participant: Participant,
    endpoint: String,
}

impl DdsServer {
    /// Joins `domain`, serves `handle` under `rt/<prefix>/…`, publishes
    /// `describe` once, and a state every `state_period`.
    pub fn start(
        domain: u32,
        prefix: &str,
        handle: Arc<Mutex<Handle>>,
        describe: String,
        state_period: Duration,
    ) -> Result<Self, DdsError> {
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

        let describe_writer = Writer::new(&participant, &describe_topic, &Qos::latched())?;
        let mut out = Vec::new();
        encode_describe(&describe, &mut out);
        describe_writer.write(&out)?;

        let state_writer = Arc::new(Writer::new(
            &participant,
            &state_topic,
            &Qos::best_effort(1),
        )?);

        let cmd_reader = {
            let handle = Arc::clone(&handle);
            Reader::new(
                &participant,
                &cmd_topic,
                &Qos::best_effort(CMD_DEPTH),
                Arc::new(move |bytes: &[u8]| {
                    // A sample that does not decode is a client built against
                    // another layout; it is dropped, sample by sample.
                    if let Ok(cmd) = decode_cmd(bytes) {
                        if let Ok(mut handle) = handle.lock() {
                            handle.send(cmd);
                        }
                    }
                }),
            )?
        };

        let running = Arc::new(AtomicBool::new(true));
        let publisher = {
            let running = Arc::clone(&running);
            let writer = Arc::clone(&state_writer);
            std::thread::Builder::new()
                .name("drlabs-api-state".into())
                .spawn(move || {
                    let mut out = Vec::new();
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
                        let _ = writer.write(&out);
                    }
                })
                .map_err(|error| DdsError(format!("state thread: {error}")))?
        };

        Ok(Self {
            running,
            publisher: Some(publisher),
            _cmd_reader: cmd_reader,
            _state_writer: state_writer,
            _describe_writer: describe_writer,
            _participant: participant,
            endpoint: format!("dds domain {domain}, rt/{prefix}/{{lowcmd,lowstate,describe}}"),
        })
    }

    /// Where a client finds this runtime, for a log line.
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    pub fn stop(mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(thread) = self.publisher.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for DdsServer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(thread) = self.publisher.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::client::Client;
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

    /// A client on the same domain: the latched description arrives
    /// unasked, a command drives an axis, and the states say so. Domain 200
    /// so a runtime on the default domain is not disturbed.
    #[test]
    fn a_client_can_describe_command_and_observe_over_dds() {
        let config = sim_config();
        let handle = Arc::new(Mutex::new(Daemon::start(&config, &sim_factory).unwrap()));
        let server = DdsServer::start(
            200,
            "drlabs_test",
            Arc::clone(&handle),
            crate::describe::describe_json(&config),
            Duration::from_millis(2),
        )
        .unwrap();

        let client = Client::connect(200, "drlabs_test", Duration::from_millis(20)).unwrap();
        let described = client.describe(Duration::from_secs(5)).unwrap();
        assert_eq!(described.joints.len(), 2);
        assert_eq!(described.joints[1].name, "b");

        client.set_axes(2);
        client.drive(1, 0.3);
        let started = Instant::now();
        let mut last = None;
        while started.elapsed() < Duration::from_millis(1500) {
            client.send(0).unwrap();
            if let Some((state, _)) = client.latest() {
                last = Some(state);
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

        drop(client);
        server.stop();
        match Arc::try_unwrap(handle) {
            Ok(mutex) => mutex.into_inner().unwrap().stop(),
            Err(_) => panic!("the server kept a handle"),
        }
    }
}
