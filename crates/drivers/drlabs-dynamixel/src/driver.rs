//! Transactions on the wire.
//!
//! One exchange is flush, write, then read until a packet arrives. That whole
//! sequence has to be atomic on a daisy chain: the reads are not addressed —
//! whatever arrives next is taken — so two interleaved exchanges would each
//! consume the other's reply. The lock on [`SharedPort`] is what holds it
//! together, and there is no way to reach the port without taking it.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};
use drlabs_transport::serial::SharedPort;

use crate::profile::packet_error_text;
use crate::protocol::{
    build_packet, find_packet_start, parse_packet, update_crc, StatusPacket, INST_FACTORY_RESET,
    INST_PING, INST_READ, INST_REBOOT, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
    MAX_PACKET_SIZE, MIN_HEADER_SIZE, READ_BUFFER_SIZE,
};

/// How long one transaction may take.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// The pause between reads when the port has nothing yet.
pub const DEFAULT_POLL_INTERVAL: Duration = Duration::from_micros(500);

/// How long a single serial read may block waiting for the servo.
///
/// This one number was worth 13 times the loop rate. A port opens with a plain
/// total read timeout, which means a read blocks until it has the *whole*
/// buffer it asked for or the timeout expires. The packet reader asks for a
/// buffer far larger than any status packet, so it never fills, and every read
/// costs the full timeout however quickly the servo answered. Measured on an
/// XL330 at 1 Mbaud that was 15.0 ms a cycle — 67 Hz, whatever rate the caller
/// configured — against 1.1 ms once tuned.
pub const READ_TIMEOUT: Duration = Duration::from_millis(1);

/// The packet-level interface to a bus.
pub struct Driver {
    port: SharedPort,
    pub timeout: Duration,
    pub poll_interval: Duration,
}

impl Driver {
    pub fn new(port: SharedPort) -> Self {
        Self {
            port,
            timeout: DEFAULT_TIMEOUT,
            poll_interval: DEFAULT_POLL_INTERVAL,
        }
    }

    /// Asks the port to return as soon as bytes arrive. See [`READ_TIMEOUT`].
    pub fn tune_read_timeout(&self) {
        let _ = self.port.lock().set_read_timeout(READ_TIMEOUT);
    }

    /// Sends a packet and waits for the reply.
    pub fn transfer(&self, packet: &[u8]) -> Result<Vec<u8>> {
        let mut port = self.port.lock();

        // Whatever is left over from a previous exchange is not an answer to
        // this one.
        let _ = port.flush_buffers();
        port.write(packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        let deadline = Instant::now() + self.timeout;
        let mut buf: Vec<u8> = Vec::with_capacity(READ_BUFFER_SIZE);
        let mut scratch = [0u8; READ_BUFFER_SIZE];

        while Instant::now() < deadline {
            let n = port.read(&mut scratch).map_err(|error| Error::Transport {
                operation: "read",
                source: Box::new(error),
            })?;
            if n == 0 {
                std::thread::sleep(self.poll_interval);
                continue;
            }
            buf.extend_from_slice(&scratch[..n]);

            while buf.len() >= MIN_HEADER_SIZE {
                match find_packet_start(&buf) {
                    Some(0) => {}
                    Some(start) => {
                        buf.drain(..start);
                        continue;
                    }
                    None => {
                        // Keep the last two bytes: a header could be split
                        // across this read and the next.
                        if buf.len() > 2 {
                            buf.drain(..buf.len() - 2);
                        }
                        break;
                    }
                }
                if buf.len() < MIN_HEADER_SIZE {
                    break;
                }

                let body = u16::from_le_bytes([buf[5], buf[6]]) as usize;
                let total = MIN_HEADER_SIZE + body;
                if total > MAX_PACKET_SIZE {
                    // A length this large is noise that happened to look like a
                    // header. Step past it and keep scanning.
                    buf.drain(..3);
                    continue;
                }
                if buf.len() < total {
                    break;
                }

                let candidate = &buf[..total];
                let received = u16::from_le_bytes([candidate[total - 2], candidate[total - 1]]);
                if received == update_crc(0, &candidate[..total - 2]) {
                    return Ok(candidate.to_vec());
                }
                // A header-shaped run of noise. The real packet may start
                // inside what we just skipped.
                buf.drain(..3);
            }
        }

        Err(Error::Timeout(self.timeout))
    }

    /// One transaction, with the servo's own error code turned into a sentence.
    fn exchange(&self, id: u8, instruction: u8, params: &[u8]) -> Result<StatusPacket> {
        let reply = self.transfer(&build_packet(id, instruction, params))?;
        let status = parse_packet(&reply)
            .map_err(|error| Error::Malformed(format!("servo id {id}: {error}")))?;
        if status.error != 0 {
            let (name, detail) = packet_error_text(status.error);
            return Err(Error::Refused(format!(
                "servo id {id}: {name} (0x{:02X}). {detail}",
                status.error
            )));
        }
        Ok(status)
    }

    /// Writes bytes to a register.
    pub fn write(&self, id: u8, address: u16, data: &[u8]) -> Result<()> {
        let mut params = Vec::with_capacity(2 + data.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(data);
        self.exchange(id, INST_WRITE, &params).map(|_| ())
    }

    /// Reads `length` bytes from a register.
    pub fn read(&self, id: u8, address: u16, length: u16) -> Result<Vec<u8>> {
        let mut params = Vec::with_capacity(4);
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&length.to_le_bytes());
        Ok(self.exchange(id, INST_READ, &params)?.params)
    }

    pub fn write_u32(&self, id: u8, address: u16, value: u32) -> Result<()> {
        self.write(id, address, &value.to_le_bytes())
    }

    pub fn write_u16(&self, id: u8, address: u16, value: u16) -> Result<()> {
        self.write(id, address, &value.to_le_bytes())
    }

    pub fn write_u8(&self, id: u8, address: u16, value: u8) -> Result<()> {
        self.write(id, address, &[value])
    }

    pub fn read_u32(&self, id: u8, address: u16) -> Result<u32> {
        let data = self.read(id, address, 4)?;
        if data.len() < 4 {
            return Err(Error::Malformed(format!(
                "servo id {id} answered a 4-byte read with {} bytes",
                data.len()
            )));
        }
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    /// Confirms the servo answers, and returns the model number it announces.
    pub fn ping(&self, id: u8) -> Result<u16> {
        let status = self.exchange(id, INST_PING, &[])?;
        if status.params.len() >= 2 {
            Ok(u16::from_le_bytes([status.params[0], status.params[1]]))
        } else {
            Ok(0)
        }
    }

    pub fn reboot(&self, id: u8) -> Result<()> {
        self.exchange(id, INST_REBOOT, &[]).map(|_| ())
    }

    /// Resets the servo to factory settings.
    ///
    /// Level 0xFF resets everything including the id, which is how a bus with
    /// two servos at the same address becomes a bus with no working servos.
    pub fn factory_reset(&self, id: u8, level: u8) -> Result<()> {
        self.exchange(id, INST_FACTORY_RESET, &[level]).map(|_| ())
    }

    /// Writes the same register on several servos in one transaction.
    ///
    /// This is what makes a daisy chain worth having. Writing five goal
    /// positions individually is five round trips, each with its own reply; a
    /// sync write is one packet and no reply at all, so the whole chain moves
    /// on the same period instead of over five of them.
    pub fn sync_write(
        &self,
        address: u16,
        data_length: u16,
        values: &BTreeMap<u8, Vec<u8>>,
    ) -> Result<()> {
        if values.is_empty() {
            return Ok(());
        }
        for (id, data) in values {
            if data.len() != data_length as usize {
                return Err(Error::Config(format!(
                    "sync write to servo id {id} carries {} bytes, but the register is {data_length}",
                    data.len()
                )));
            }
        }

        let mut params = Vec::with_capacity(4 + values.len() * (1 + data_length as usize));
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        for (&id, data) in values {
            params.push(id);
            params.extend_from_slice(data);
        }

        // Id 0xFE is the broadcast address, and a broadcast is not answered.
        let packet = build_packet(0xFE, INST_SYNC_WRITE, &params);
        let mut port = self.port.lock();
        let _ = port.flush_buffers();
        port.write(&packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;
        Ok(())
    }

    /// Reads the same register from several servos.
    ///
    /// Unlike a sync write this *is* answered — once per servo, in the order
    /// they were asked. A servo that does not answer is left out of the result
    /// rather than reported as zero, because a missing reply and a reading of
    /// zero are very different things to a control loop.
    pub fn sync_read(
        &self,
        address: u16,
        data_length: u16,
        ids: &[u8],
    ) -> Result<BTreeMap<u8, Vec<u8>>> {
        if ids.is_empty() {
            return Ok(BTreeMap::new());
        }

        let mut params = Vec::with_capacity(4 + ids.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        params.extend_from_slice(ids);

        let packet = build_packet(0xFE, INST_SYNC_READ, &params);
        let mut port = self.port.lock();
        let _ = port.flush_buffers();
        port.write(&packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;
        drop(port);

        let mut out = BTreeMap::new();
        for _ in ids {
            let reply = match self.read_one_reply() {
                Ok(reply) => reply,
                // A chain where one servo has gone quiet still reports the
                // others. Stopping here would lose readings that arrived.
                Err(_) => break,
            };
            if let Ok(status) = parse_packet(&reply) {
                if status.error == 0 && ids.contains(&status.id) {
                    out.insert(status.id, status.params);
                }
            }
        }
        Ok(out)
    }

    /// Reads one packet without writing anything first, for the replies to a
    /// sync read.
    fn read_one_reply(&self) -> Result<Vec<u8>> {
        let mut port = self.port.lock();
        let deadline = Instant::now() + self.timeout;
        let mut buf: Vec<u8> = Vec::with_capacity(READ_BUFFER_SIZE);
        let mut scratch = [0u8; READ_BUFFER_SIZE];

        while Instant::now() < deadline {
            let n = port.read(&mut scratch).map_err(|error| Error::Transport {
                operation: "read",
                source: Box::new(error),
            })?;
            if n == 0 {
                std::thread::sleep(self.poll_interval);
                continue;
            }
            buf.extend_from_slice(&scratch[..n]);

            if let Some(start) = find_packet_start(&buf) {
                if buf.len() >= start + MIN_HEADER_SIZE {
                    let body = u16::from_le_bytes([buf[start + 5], buf[start + 6]]) as usize;
                    let total = MIN_HEADER_SIZE + body;
                    if start + total <= buf.len() && total <= MAX_PACKET_SIZE {
                        return Ok(buf[start..start + total].to_vec());
                    }
                }
            }
        }
        Err(Error::Timeout(self.timeout))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_transport::serial::SerialPort;
    use std::sync::{Arc, Mutex};

    /// A servo made of a script: it answers whatever the test queued, and
    /// records what it was sent.
    #[derive(Clone, Default)]
    struct FakeServo {
        written: Arc<Mutex<Vec<u8>>>,
        replies: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl FakeServo {
        fn reply_with(&self, id: u8, error: u8, params: &[u8]) {
            let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
            let length = 1 + 1 + params.len() + 2; // instruction, error, params, CRC
            packet.push((length & 0xFF) as u8);
            packet.push(((length >> 8) & 0xFF) as u8);
            packet.push(0x55); // status
            packet.push(error);
            packet.extend_from_slice(params);
            let crc = update_crc(0, &packet);
            packet.extend_from_slice(&crc.to_le_bytes());
            self.replies.lock().unwrap().push(packet);
        }

        fn sent(&self) -> Vec<u8> {
            self.written.lock().unwrap().clone()
        }
    }

    impl SerialPort for FakeServo {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut replies = self.replies.lock().unwrap();
            if replies.is_empty() {
                return Ok(0);
            }
            let reply = replies.remove(0);
            let n = reply.len().min(buf.len());
            buf[..n].copy_from_slice(&reply[..n]);
            Ok(n)
        }

        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.written.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
    }

    fn driver(servo: &FakeServo) -> Driver {
        let mut driver = Driver::new(SharedPort::new(Box::new(servo.clone())));
        driver.timeout = Duration::from_millis(20);
        driver.poll_interval = Duration::from_micros(100);
        driver
    }

    #[test]
    fn a_ping_reports_the_model_the_servo_announces() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[0xB0, 0x04, 46]); // model 1200
        assert_eq!(driver(&servo).ping(1).unwrap(), 1200);
    }

    /// The error path that matters most: the servo answered, and said no. That
    /// is information, and it should arrive as the servo's own words.
    #[test]
    fn a_refusal_is_reported_with_its_cause() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0x07, &[]);

        let error = driver(&servo).write_u32(1, 116, 2048).unwrap_err();
        let text = format!("{error}");
        assert!(text.contains("Access error"), "{text}");
        assert!(text.contains("bus watchdog has latched"), "{text}");
    }

    /// A servo that says nothing is a dropped cycle, not a broken bus — the
    /// caller decides which, so this only has to be distinguishable.
    #[test]
    fn silence_is_a_timeout_rather_than_a_hang() {
        let servo = FakeServo::default();
        let started = Instant::now();
        let error = driver(&servo).ping(1).unwrap_err();
        assert!(matches!(error, Error::Timeout(_)), "{error}");
        assert!(started.elapsed() < Duration::from_millis(200));
    }

    /// Line noise before a packet is skipped rather than parsed. At 1 Mbaud on
    /// a half-duplex bus there is always some.
    #[test]
    fn noise_before_a_packet_is_skipped() {
        let servo = FakeServo::default();
        servo.replies.lock().unwrap().push(vec![0x00, 0xFF, 0x12]);
        servo.reply_with(1, 0, &[0xB0, 0x04, 46]);
        assert_eq!(driver(&servo).ping(1).unwrap(), 1200);
    }

    #[test]
    fn a_register_read_returns_the_parameters() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[0x00, 0x08, 0x00, 0x00]); // 2048 counts
        assert_eq!(driver(&servo).read_u32(1, 132).unwrap(), 2048);
    }

    /// A sync write is one packet for the whole chain, and it is not answered.
    #[test]
    fn a_sync_write_addresses_every_servo_in_one_packet() {
        let servo = FakeServo::default();
        let mut values = BTreeMap::new();
        values.insert(1u8, 1000u32.to_le_bytes().to_vec());
        values.insert(2u8, 2000u32.to_le_bytes().to_vec());

        driver(&servo).sync_write(116, 4, &values).unwrap();

        let sent = servo.sent();
        assert_eq!(sent[4], 0xFE, "sent to the broadcast id");
        assert_eq!(sent[7], INST_SYNC_WRITE);
        // Address, length, then id-and-data per servo.
        assert_eq!(&sent[8..12], &[116, 0, 4, 0]);
        assert_eq!(sent[12], 1);
        assert_eq!(sent[17], 2);
    }

    /// Getting the width wrong would write half a value into every servo on the
    /// chain, so it is caught before anything reaches the wire.
    #[test]
    fn a_sync_write_with_the_wrong_width_is_refused() {
        let servo = FakeServo::default();
        let mut values = BTreeMap::new();
        values.insert(1u8, vec![0x00, 0x08]);

        let error = driver(&servo).sync_write(116, 4, &values).unwrap_err();
        assert!(matches!(error, Error::Config(_)), "{error}");
        assert!(servo.sent().is_empty(), "nothing should have been written");
    }

    /// A chain where one servo has gone quiet still reports the others.
    #[test]
    fn a_sync_read_returns_the_servos_that_answered() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[0x00, 0x08, 0x00, 0x00]);
        // Nothing from id 2.

        let values = driver(&servo).sync_read(132, 4, &[1, 2]).unwrap();
        assert_eq!(values.len(), 1);
        assert_eq!(values[&1], vec![0x00, 0x08, 0x00, 0x00]);
        assert!(
            !values.contains_key(&2),
            "a silent servo is absent, not zero"
        );
    }
}
