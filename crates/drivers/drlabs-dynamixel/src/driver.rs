//! Transactions on the wire.
//!
//! One exchange is flush, write, then read until a packet arrives. That whole
//! sequence has to be atomic on a daisy chain: the reads are not addressed —
//! whatever arrives next is taken — so two interleaved exchanges would each
//! consume the other's reply. The lock on [`SharedPort`] is what holds it
//! together, and there is no way to reach the port without taking it.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};
use drlabs_transport::serial::SharedPort;

use crate::profile::packet_error_text;
use crate::protocol::{
    build_packet, destuff_into, find_packet_start, for_each_fast_segment, parse_fast_sync_read,
    parse_packet, parse_packet_borrowed, update_crc, write_packet, StatusPacket, BROADCAST_ID,
    INST_FACTORY_RESET, INST_FAST_SYNC_READ, INST_PING, INST_READ, INST_REBOOT, INST_SYNC_READ,
    INST_SYNC_WRITE, INST_WRITE, MAX_PACKET_SIZE, MIN_HEADER_SIZE, READ_BUFFER_SIZE,
};

/// Buffers a real-time caller keeps from one transaction to the next, so
/// that building a packet and collecting its replies allocate nothing once
/// they have grown to the bus's largest exchange.
pub struct Scratch {
    params: Vec<u8>,
    packet: Vec<u8>,
    rx: Vec<u8>,
    data: Vec<u8>,
}

impl Scratch {
    pub fn new() -> Self {
        Self {
            params: Vec::with_capacity(MAX_PACKET_SIZE),
            packet: Vec::with_capacity(MAX_PACKET_SIZE),
            rx: Vec::with_capacity(READ_BUFFER_SIZE * 2),
            data: Vec::with_capacity(64),
        }
    }
}

impl Default for Scratch {
    fn default() -> Self {
        Self::new()
    }
}

/// How a fast sync read ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastOutcome {
    /// The combined reply arrived; this many segments were handed to the sink.
    Answered(usize),
    /// This servo answered alone with an Instruction Error: its firmware does
    /// not know the instruction, and the caller should read it the slow way.
    Unsupported(u8),
    /// Nothing usable came back within the reply budget.
    Silent,
}

/// How long one transaction may take.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// How long a sync read waits for each reply, and how long the chain's
/// recovery read of one silent servo may take.
///
/// Shorter than [`DEFAULT_TIMEOUT`] on purpose. A single transaction may
/// legitimately be slow — an EEPROM write settles for tens of milliseconds —
/// but the replies to a sync read arrive back to back: eight ten-byte
/// readings at 3 Mbaud are on the wire in about a millisecond, and the only
/// way one of them is 20 ms late is that it is not coming. Waiting the full
/// transaction timeout for it cost the chain 100 ms per silent servo per
/// period, which is what turned one dropped reply into a stalled bus and a
/// comm-loss e-stop. 20 ms is the worst host-side scheduling hiccup the
/// session's watchdog floor was measured for; a reply later than that is
/// treated as lost and asked for again, alone.
pub const DEFAULT_REPLY_TIMEOUT: Duration = Duration::from_millis(20);

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

/// Bit 7 of a status packet's error field: the servo's hardware-error
/// register is non-zero. It rides on every reply while the error stands and
/// does not mean the instruction was refused.
pub const ALERT_BIT: u8 = 0x80;

/// How many packets a sync read will skip, per servo asked, before giving
/// up: stale replies from an earlier broadcast, replies that refuse the
/// instruction, or packets from servos not on the list.
const MAX_SKIPPED_REPLIES: usize = 2;

/// The packet-level interface to a bus.
pub struct Driver {
    port: SharedPort,
    pub timeout: Duration,
    /// Per-reply budget inside a sync read, and the budget of a recovery
    /// read. See [`DEFAULT_REPLY_TIMEOUT`].
    pub reply_timeout: Duration,
    pub poll_interval: Duration,

    /// Whether the receive buffer may hold leftovers from a transaction that
    /// went wrong — a timed-out reply, a sync read cut short. Only then is a
    /// flush worth its price: `PurgeComm`/`tcflush` is a driver ioctl on
    /// every exchange otherwise, paid twice a period on the hot path, while
    /// leftovers from *successful* exchanges cannot exist — a transaction
    /// holds the port until it has collected its whole answer, and the
    /// `expect_id` filter and resync scanning stand guard regardless.
    needs_flush: AtomicBool,
}

impl Driver {
    pub fn new(port: SharedPort) -> Self {
        Self {
            port,
            timeout: DEFAULT_TIMEOUT,
            reply_timeout: DEFAULT_REPLY_TIMEOUT,
            poll_interval: DEFAULT_POLL_INTERVAL,
            needs_flush: AtomicBool::new(true),
        }
    }

    /// Asks the port to return as soon as bytes arrive. See [`READ_TIMEOUT`].
    pub fn tune_read_timeout(&self) {
        let _ = self.port.lock().set_read_timeout(READ_TIMEOUT);
    }

    /// Waits a moment for the port to have something.
    ///
    /// A zero `poll_interval` yields instead of sleeping. On Windows a sleep
    /// of any length is a sleep of at least the timer period, a millisecond,
    /// which is half of a 500 Hz budget; a bus thread that owns its core
    /// would rather spin.
    fn pause(&self) {
        if self.poll_interval.is_zero() {
            std::thread::yield_now();
        } else {
            std::thread::sleep(self.poll_interval);
        }
    }

    /// Sends a packet and waits for the reply.
    /// One request and the answer to *it*.
    ///
    /// `expect_id` is what makes the second half of that true. A reply left on
    /// the wire by an earlier transaction — a sync read that timed out part way
    /// through its answers, most often — is a well formed packet with a valid
    /// CRC, and taking the first one that arrives means reading another servo's
    /// answer as this one's. What that looks like on a bench is a servo
    /// rejecting a value its neighbour has just accepted, intermittently, with
    /// an error code that belongs to neither.
    pub fn transfer(&self, packet: &[u8], expect_id: u8) -> Result<Vec<u8>> {
        self.transfer_within(packet, expect_id, self.timeout)
    }

    /// [`transfer`](Self::transfer) with its own reply budget, for a request
    /// whose answer is either prompt or not coming.
    pub fn transfer_within(
        &self,
        packet: &[u8],
        expect_id: u8,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let result = self.transfer_inner(packet, expect_id, timeout);
        if result.is_err() {
            // The reply this transaction gave up on may still arrive, and it
            // must not be read as the answer to the next one.
            self.needs_flush.store(true, Ordering::Relaxed);
        }
        result
    }

    fn transfer_inner(&self, packet: &[u8], expect_id: u8, timeout: Duration) -> Result<Vec<u8>> {
        let mut port = self.port.lock();

        // Leftovers exist only after a transaction went wrong; see
        // `needs_flush`.
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        let deadline = Instant::now() + timeout;
        let mut buf: Vec<u8> = Vec::with_capacity(READ_BUFFER_SIZE);
        let mut scratch = [0u8; READ_BUFFER_SIZE];

        while Instant::now() < deadline {
            let n = port.read(&mut scratch).map_err(|error| Error::Transport {
                operation: "read",
                source: Box::new(error),
            })?;
            if n == 0 {
                self.pause();
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
                    // A whole, valid packet — but not necessarily ours.
                    if candidate[4] == expect_id {
                        return Ok(candidate.to_vec());
                    }
                    let stale = total;
                    buf.drain(..stale);
                    continue;
                }
                // A header-shaped run of noise. The real packet may start
                // inside what we just skipped.
                buf.drain(..3);
            }
        }

        Err(Error::Timeout(timeout))
    }

    /// One transaction, with the servo's own error code turned into a sentence.
    fn exchange(&self, id: u8, instruction: u8, params: &[u8]) -> Result<StatusPacket> {
        self.exchange_within(id, instruction, params, self.timeout)
    }

    fn exchange_within(
        &self,
        id: u8,
        instruction: u8,
        params: &[u8],
        timeout: Duration,
    ) -> Result<StatusPacket> {
        let reply = self.transfer_within(&build_packet(id, instruction, params), id, timeout)?;
        let status = parse_packet(&reply)
            .map_err(|error| Error::Malformed(format!("servo id {id}: {error}")))?;
        // Bit 7 is the hardware alert, raised on every reply while the
        // servo's hardware-error register is non-zero. It says nothing about
        // this instruction, which was carried out; the fault itself is read
        // from that register. Only the low bits refuse the instruction.
        if status.error & !ALERT_BIT != 0 {
            let (name, detail) = packet_error_text(status.error & !ALERT_BIT);
            let alert = if status.error & ALERT_BIT != 0 {
                " The servo also flags a hardware error."
            } else {
                ""
            };
            return Err(Error::Refused(format!(
                "servo id {id}: {name} (0x{:02X}). {detail}{alert}",
                status.error
            )));
        }
        Ok(status)
    }

    /// Writes bytes to a register.
    ///
    /// The address travels with the failure. A servo answering "data range
    /// error" says the value was wrong but not which register was asked for,
    /// and on a driver that writes a dozen of them that is the difference
    /// between a diagnosis and a search.
    pub fn write(&self, id: u8, address: u16, data: &[u8]) -> Result<()> {
        let mut params = Vec::with_capacity(2 + data.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(data);
        self.exchange(id, INST_WRITE, &params)
            .map(|_| ())
            .map_err(|error| match error {
                Error::Refused(reason) => Error::Refused(format!(
                    "{reason} (writing {} to register {address})",
                    data.iter()
                        .map(|byte| byte.to_string())
                        .collect::<Vec<_>>()
                        .join(",")
                )),
                other => other,
            })
    }

    /// Reads `length` bytes from a register.
    pub fn read(&self, id: u8, address: u16, length: u16) -> Result<Vec<u8>> {
        let mut params = Vec::with_capacity(4);
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&length.to_le_bytes());
        Ok(self.exchange(id, INST_READ, &params)?.params)
    }

    /// Reads `length` bytes from one servo within the reply budget, and says
    /// whether the reply carried the hardware alert bit.
    ///
    /// This is the chain's recovery read: a servo that stayed silent on a
    /// broadcast is asked once more, alone. Alone matters — in a sync read
    /// every servo after the silent one waits for a reply that never comes,
    /// so one dropped packet takes the rest of the chunk with it; a direct
    /// read has nobody to wait for. And it is bounded by `reply_timeout`, not
    /// the transaction timeout, because a servo that has not answered in
    /// 20 ms is not about to.
    pub fn read_quick(&self, id: u8, address: u16, length: u16) -> Result<(Vec<u8>, bool)> {
        let mut params = Vec::with_capacity(4);
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&length.to_le_bytes());
        let status = self.exchange_within(id, INST_READ, &params, self.reply_timeout)?;
        let alert = status.error & ALERT_BIT != 0;
        Ok((status.params, alert))
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
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
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
        self.sync_read_alerts(address, data_length, ids)
            .map(|(answers, _)| answers)
    }

    /// `sync_read`, also naming the servos whose replies carried the hardware
    /// alert bit.
    ///
    /// A servo with a non-zero hardware-error register sets bit 7 of the
    /// error field on every status packet it sends, for as long as the error
    /// stands — until it is rebooted, on most models. The reply is otherwise
    /// complete and correct. Treating it as a refusal made such a servo look
    /// silent to every sync read, including the one that reads the error
    /// register itself: the session e-stopped on "no reply" and the fault
    /// never got its name.
    pub fn sync_read_alerts(
        &self,
        address: u16,
        data_length: u16,
        ids: &[u8],
    ) -> Result<(BTreeMap<u8, Vec<u8>>, BTreeSet<u8>)> {
        if ids.is_empty() {
            return Ok((BTreeMap::new(), BTreeSet::new()));
        }

        let mut params = Vec::with_capacity(4 + ids.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        params.extend_from_slice(ids);

        let packet = build_packet(0xFE, INST_SYNC_READ, &params);

        // The port is held across the request *and* every reply. A sync read
        // is one transaction with N answers, and releasing the port between
        // them lets another thread's request land in the middle of the reply
        // stream — after which both transactions read each other's answers and
        // both time out. On a chain that is the difference between one
        // transaction and a cascade of them.
        let mut port = self.port.lock();
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(&packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        // One buffer for the whole reply stream. The servos answer back to
        // back, so a single read very often returns two or three replies at
        // once; a buffer that started empty for each of them would hand back
        // the first and throw the rest away.
        let mut buf: Vec<u8> = Vec::with_capacity(READ_BUFFER_SIZE);

        // Collect until every servo asked for has answered, or the wire goes
        // quiet. A reply from a servo *not* on this list — one that answered a
        // previous, cut-short broadcast late — is skipped and does not use up
        // a slot: counted the other way, three stale replies at the head of
        // the stream made the last three servos of this read look silent,
        // every cycle their neighbours' leftovers arrived first.
        //
        // Skipped packets are counted, and the read gives up after a bound of
        // them: every packet read takes a fresh reply timeout, so an unbounded
        // loop lets a stream of stale or refused replies hold the bus for far
        // longer than one timeout.
        let mut out = BTreeMap::new();
        let mut alerts = BTreeSet::new();
        let mut skipped = 0usize;
        while out.len() < ids.len() {
            let reply = match self.read_one_reply_locked(&mut port, &mut buf) {
                Ok(reply) => reply,
                // A servo that has gone quiet still lets the others report;
                // its reply may yet arrive, so the next transaction flushes.
                Err(_) => {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    break;
                }
            };
            let mut taken = false;
            if let Ok(status) = parse_packet(&reply) {
                if status.error & !ALERT_BIT == 0 && ids.contains(&status.id) {
                    if status.error & ALERT_BIT != 0 {
                        alerts.insert(status.id);
                    }
                    out.insert(status.id, status.params);
                    taken = true;
                }
            }
            if !taken {
                skipped += 1;
                if skipped > MAX_SKIPPED_REPLIES * ids.len() {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    break;
                }
            }
        }
        Ok((out, alerts))
    }

    /// Reads the same register from several servos in *one* reply.
    ///
    /// A fast sync read (0x8A) is the instruction that makes a long chain
    /// affordable at a high rate: the servos answer in one combined packet
    /// instead of one packet each, so the per-servo cost drops from eleven
    /// bytes of framing plus a bus turnaround to four bytes and no
    /// turnaround at all. Sixteen servos reading ten bytes is 232 bytes on
    /// the wire instead of 336 and sixteen gaps.
    ///
    /// Not every servo speaks it — X-series firmware gained it around v45,
    /// and older models never did. A servo that does not recognise the
    /// instruction answers *alone*, with an Instruction Error from its own
    /// id, and that is reported as a refusal naming the servo so the caller
    /// can read that servo the slow way. A servo that stays silent leaves
    /// its segment out of the combined reply, exactly as in a plain sync
    /// read, and is absent from the result rather than zero.
    ///
    /// Returns the readings and the ids whose segment carried the hardware
    /// alert bit, like [`sync_read_alerts`](Self::sync_read_alerts).
    pub fn fast_sync_read(
        &self,
        address: u16,
        data_length: u16,
        ids: &[u8],
    ) -> Result<(BTreeMap<u8, Vec<u8>>, BTreeSet<u8>)> {
        if ids.is_empty() {
            return Ok((BTreeMap::new(), BTreeSet::new()));
        }

        let mut params = Vec::with_capacity(4 + ids.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        params.extend_from_slice(ids);
        let packet = build_packet(BROADCAST_ID, INST_FAST_SYNC_READ, &params);

        let mut port = self.port.lock();
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(&packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        // One packet is the answer; anything else on the wire first is a
        // leftover or a refusal. The combined reply carries the broadcast id.
        let mut buf: Vec<u8> = Vec::with_capacity(READ_BUFFER_SIZE);
        let mut skipped = 0usize;
        loop {
            let reply = match self.read_one_reply_locked(&mut port, &mut buf) {
                Ok(reply) => reply,
                Err(error) => {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    return Err(error);
                }
            };
            if reply.len() > 4 && reply[4] == BROADCAST_ID {
                let segments = parse_fast_sync_read(&reply, data_length as usize)
                    .map_err(|error| Error::Malformed(format!("fast sync read: {error}")))?;
                let mut out = BTreeMap::new();
                let mut alerts = BTreeSet::new();
                for segment in segments {
                    if segment.error & !ALERT_BIT != 0 || !ids.contains(&segment.id) {
                        continue;
                    }
                    if segment.error & ALERT_BIT != 0 {
                        alerts.insert(segment.id);
                    }
                    out.insert(segment.id, segment.data);
                }
                return Ok((out, alerts));
            }
            // A single-servo status packet in answer to a broadcast read:
            // the servo does not know the instruction. Say which one, and
            // let the rest of its replies drain into the next flush.
            if let Ok(status) = parse_packet(&reply) {
                if ids.contains(&status.id) && status.error & !ALERT_BIT != 0 {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    let (name, _) = packet_error_text(status.error & !ALERT_BIT);
                    return Err(Error::Refused(format!(
                        "servo id {}: {name} (0x{:02X}) in answer to a fast sync read: the firmware does not support it",
                        status.id, status.error
                    )));
                }
            }
            skipped += 1;
            if skipped > MAX_SKIPPED_REPLIES * ids.len() {
                self.needs_flush.store(true, Ordering::Relaxed);
                return Err(Error::Timeout(self.reply_timeout));
            }
        }
    }

    /// Takes the next packet out of `buf`, reading more from the port only when
    /// what is already buffered does not hold a whole one.
    ///
    /// `buf` carries across the replies to one sync read, and the bytes of the
    /// packet returned are drained from it. That is the whole point: a read
    /// that returns three concatenated replies must yield three packets, not
    /// one packet and two discarded.
    fn read_one_reply_locked(
        &self,
        port: &mut std::sync::MutexGuard<'_, Box<dyn drlabs_transport::serial::SerialPort>>,
        buf: &mut Vec<u8>,
    ) -> Result<Vec<u8>> {
        let deadline = Instant::now() + self.reply_timeout;
        let mut scratch = [0u8; READ_BUFFER_SIZE];

        loop {
            if let Some(packet) = take_packet(buf) {
                return Ok(packet);
            }
            if Instant::now() >= deadline {
                return Err(Error::Timeout(self.reply_timeout));
            }
            let n = port.read(&mut scratch).map_err(|error| Error::Transport {
                operation: "read",
                source: Box::new(error),
            })?;
            if n == 0 {
                self.pause();
                continue;
            }
            buf.extend_from_slice(&scratch[..n]);
        }
    }

    /// Like [`read_one_reply_locked`](Self::read_one_reply_locked), but
    /// leaves the packet in `buf` and returns where it is. The caller reads
    /// it in place and then drains `..end`. Nothing is allocated.
    fn read_one_reply_range_locked(
        &self,
        port: &mut std::sync::MutexGuard<'_, Box<dyn drlabs_transport::serial::SerialPort>>,
        buf: &mut Vec<u8>,
    ) -> Result<std::ops::Range<usize>> {
        let deadline = Instant::now() + self.reply_timeout;
        let mut scratch = [0u8; READ_BUFFER_SIZE];

        loop {
            if let Some(range) = next_packet_range(buf) {
                return Ok(range);
            }
            if Instant::now() >= deadline {
                return Err(Error::Timeout(self.reply_timeout));
            }
            let n = port.read(&mut scratch).map_err(|error| Error::Transport {
                operation: "read",
                source: Box::new(error),
            })?;
            if n == 0 {
                self.pause();
                continue;
            }
            buf.extend_from_slice(&scratch[..n]);
        }
    }

    /// [`sync_write`](Self::sync_write) without the allocation.
    ///
    /// `values` holds `ids.len()` runs of `width` bytes, in id order. The
    /// packet is built in `scratch`.
    pub fn sync_write_into(
        &self,
        scratch: &mut Scratch,
        address: u16,
        width: u16,
        ids: &[u8],
        values: &[u8],
    ) -> Result<()> {
        if ids.is_empty() {
            return Ok(());
        }
        if values.len() != ids.len() * width as usize {
            return Err(Error::Config(format!(
                "sync write of {} servos at {width} bytes each needs {} bytes, got {}",
                ids.len(),
                ids.len() * width as usize,
                values.len()
            )));
        }
        let Scratch { params, packet, .. } = scratch;
        params.clear();
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&width.to_le_bytes());
        for (id, value) in ids.iter().zip(values.chunks(width as usize)) {
            params.push(*id);
            params.extend_from_slice(value);
        }
        write_packet(packet, BROADCAST_ID, INST_SYNC_WRITE, params);

        let mut port = self.port.lock();
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;
        Ok(())
    }

    /// [`sync_read_alerts`](Self::sync_read_alerts) without the allocation.
    ///
    /// `sink` is called once per servo that answered, with its id, whether
    /// its reply carried the alert bit, and its data. Returns how many
    /// answered. A servo that stays silent costs one reply budget, as before.
    pub fn sync_read_into(
        &self,
        scratch: &mut Scratch,
        address: u16,
        data_length: u16,
        ids: &[u8],
        sink: &mut dyn FnMut(u8, bool, &[u8]),
    ) -> Result<usize> {
        if ids.is_empty() {
            return Ok(0);
        }
        let Scratch {
            params,
            packet,
            rx,
            data,
        } = scratch;
        params.clear();
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        params.extend_from_slice(ids);
        write_packet(packet, BROADCAST_ID, INST_SYNC_READ, params);

        let mut port = self.port.lock();
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        rx.clear();
        let mut answered = 0usize;
        let mut skipped = 0usize;
        // The ids answered so far, as a bitmap, so a duplicate reply is not
        // counted twice. 253 valid ids fit in four words.
        let mut seen = [0u64; 4];
        while answered < ids.len() {
            let range = match self.read_one_reply_range_locked(&mut port, rx) {
                Ok(range) => range,
                Err(_) => {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    break;
                }
            };
            let mut taken = false;
            if let Ok((id, error, raw)) = parse_packet_borrowed(&rx[range.clone()]) {
                let bit = (seen[(id / 64) as usize] >> (id % 64)) & 1;
                if error & !ALERT_BIT == 0 && ids.contains(&id) && bit == 0 {
                    seen[(id / 64) as usize] |= 1 << (id % 64);
                    destuff_into(data, raw);
                    sink(id, error & ALERT_BIT != 0, data);
                    answered += 1;
                    taken = true;
                }
            }
            rx.drain(..range.end);
            if !taken {
                skipped += 1;
                if skipped > MAX_SKIPPED_REPLIES * ids.len() {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    break;
                }
            }
        }
        Ok(answered)
    }

    /// [`fast_sync_read`](Self::fast_sync_read) without the allocation.
    ///
    /// `sink` is called once per segment of the combined reply whose servo
    /// is on the list and did not refuse, with its id, alert flag and data.
    pub fn fast_sync_read_into(
        &self,
        scratch: &mut Scratch,
        address: u16,
        data_length: u16,
        ids: &[u8],
        sink: &mut dyn FnMut(u8, bool, &[u8]),
    ) -> Result<FastOutcome> {
        if ids.is_empty() {
            return Ok(FastOutcome::Answered(0));
        }
        let Scratch {
            params, packet, rx, ..
        } = scratch;
        params.clear();
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&data_length.to_le_bytes());
        params.extend_from_slice(ids);
        write_packet(packet, BROADCAST_ID, INST_FAST_SYNC_READ, params);

        let mut port = self.port.lock();
        if self.needs_flush.swap(false, Ordering::Relaxed) {
            let _ = port.flush_buffers();
        }
        port.write(packet).map_err(|error| Error::Transport {
            operation: "write",
            source: Box::new(error),
        })?;

        rx.clear();
        let mut skipped = 0usize;
        loop {
            let range = match self.read_one_reply_range_locked(&mut port, rx) {
                Ok(range) => range,
                Err(_) => {
                    self.needs_flush.store(true, Ordering::Relaxed);
                    return Ok(FastOutcome::Silent);
                }
            };
            let reply = &rx[range.clone()];
            if reply.len() > 4 && reply[4] == BROADCAST_ID {
                let mut answered = 0usize;
                let parsed =
                    for_each_fast_segment(reply, data_length as usize, |id, error, data| {
                        if error & !ALERT_BIT == 0 && ids.contains(&id) {
                            sink(id, error & ALERT_BIT != 0, data);
                            answered += 1;
                        }
                    });
                let end = range.end;
                rx.drain(..end);
                return match parsed {
                    Ok(()) => Ok(FastOutcome::Answered(answered)),
                    Err(error) => {
                        self.needs_flush.store(true, Ordering::Relaxed);
                        Err(Error::Malformed(format!("fast sync read: {error}")))
                    }
                };
            }
            let refused = match parse_packet_borrowed(reply) {
                Ok((id, error, _)) if ids.contains(&id) && error & !ALERT_BIT != 0 => Some(id),
                _ => None,
            };
            let end = range.end;
            rx.drain(..end);
            if let Some(id) = refused {
                self.needs_flush.store(true, Ordering::Relaxed);
                return Ok(FastOutcome::Unsupported(id));
            }
            skipped += 1;
            if skipped > MAX_SKIPPED_REPLIES * ids.len() {
                self.needs_flush.store(true, Ordering::Relaxed);
                return Ok(FastOutcome::Silent);
            }
        }
    }
}

/// Where the first whole packet in `buf` is, after discarding anything
/// before it. `None` means what is buffered is not yet a complete packet.
fn next_packet_range(buf: &mut Vec<u8>) -> Option<std::ops::Range<usize>> {
    let start = find_packet_start(buf)?;
    if start > 0 {
        buf.drain(..start);
    }
    if buf.len() < MIN_HEADER_SIZE {
        return None;
    }
    let body = u16::from_le_bytes([buf[5], buf[6]]) as usize;
    let total = MIN_HEADER_SIZE + body;
    if total > MAX_PACKET_SIZE {
        // Not a length this protocol can produce; drop the header and resync.
        buf.drain(..1);
        return None;
    }
    if total > buf.len() {
        return None;
    }
    Some(0..total)
}

/// Removes and returns the first whole packet in `buf`, discarding anything
/// before it. `None` means what is buffered is not yet a complete packet.
fn take_packet(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    let start = find_packet_start(buf)?;
    if buf.len() < start + MIN_HEADER_SIZE {
        return None;
    }
    let body = u16::from_le_bytes([buf[start + 5], buf[start + 6]]) as usize;
    let total = MIN_HEADER_SIZE + body;
    if total > MAX_PACKET_SIZE {
        // Not a length this protocol can produce; drop the header and resync.
        buf.drain(..start + 1);
        return None;
    }
    if start + total > buf.len() {
        return None;
    }
    let packet = buf[start..start + total].to_vec();
    buf.drain(..start + total);
    Some(packet)
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
        flushes: Arc<Mutex<u64>>,
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

        fn flush_buffers(&mut self) -> std::io::Result<()> {
            *self.flushes.lock().unwrap() += 1;
            Ok(())
        }
    }

    fn driver(servo: &FakeServo) -> Driver {
        let mut driver = Driver::new(SharedPort::new(Box::new(servo.clone())));
        driver.timeout = Duration::from_millis(20);
        driver.reply_timeout = Duration::from_millis(20);
        driver.poll_interval = Duration::from_micros(100);
        driver
    }

    /// A silent servo costs a sync read the reply budget, not the transaction
    /// timeout: the difference between a dropped reading and a stalled bus.
    #[test]
    fn a_silent_servo_costs_a_sync_read_the_reply_budget_only() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[1; 10]); // id 2 says nothing
        let mut driver = driver(&servo);
        driver.timeout = Duration::from_millis(500);
        driver.reply_timeout = Duration::from_millis(10);

        let started = Instant::now();
        let answers = driver.sync_read(126, 10, &[1, 2]).unwrap();
        let took = started.elapsed();

        assert_eq!(answers.len(), 1, "id 1 answered, id 2 did not");
        assert!(
            took < Duration::from_millis(200),
            "the silent servo held the read for {took:?}: the transaction timeout, not the reply budget"
        );
    }

    /// The recovery read is one direct READ, bounded by the reply budget,
    /// and it reports the alert bit rather than refusing on it.
    #[test]
    fn a_quick_read_is_a_direct_read_within_the_reply_budget() {
        let servo = FakeServo::default();
        servo.reply_with(2, ALERT_BIT, &[5; 10]);
        let mut driver = driver(&servo);
        driver.timeout = Duration::from_millis(500);
        driver.reply_timeout = Duration::from_millis(10);

        let (data, alert) = driver.read_quick(2, 126, 10).unwrap();
        assert_eq!(data, vec![5; 10]);
        assert!(alert, "the alert bit rides on the reply and is reported");
        let sent = servo.sent();
        assert_eq!(sent[4], 2, "addressed to the silent servo alone");
        assert_eq!(sent[7], INST_READ);

        let started = Instant::now();
        driver
            .read_quick(3, 126, 10)
            .expect_err("nobody answers for id 3");
        assert!(started.elapsed() < Duration::from_millis(200));
    }

    /// The flush is paid where it earns something: once when the port is
    /// fresh, and again after a transaction that went wrong — never on the
    /// hot path between clean exchanges.
    #[test]
    fn a_flush_happens_after_a_failure_and_not_between_clean_exchanges() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[1]);
        servo.reply_with(1, 0, &[2]);
        let driver = driver(&servo);

        // A fresh driver flushes once, on its first transaction.
        driver.read(1, 0, 1).expect("first reply queued");
        assert_eq!(*servo.flushes.lock().unwrap(), 1);

        // A clean exchange leaves nothing behind; the next does not flush.
        driver.read(1, 0, 1).expect("second reply queued");
        assert_eq!(*servo.flushes.lock().unwrap(), 1);

        // A timeout may leave a late reply in the buffer; the transaction
        // after it flushes again.
        driver
            .read(1, 0, 1)
            .expect_err("no reply queued: a timeout");
        assert_eq!(
            *servo.flushes.lock().unwrap(),
            1,
            "the failure itself does not flush"
        );
        servo.reply_with(1, 0, &[3]);
        driver.read(1, 0, 1).expect("third reply queued");
        assert_eq!(*servo.flushes.lock().unwrap(), 2);
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
    /// The defect that made a chain report only its first servo. A daisy chain
    /// answers a sync read back to back, so the OS very often hands over two or
    /// three replies in a single read. Anything that keeps only the first
    /// packet and drops the tail of the buffer loses every servo but one — and
    /// it does not look like a bug, it looks like servos that went quiet.
    /// The defect this guard exists for. A sync read that gave up part way
    /// through its answers leaves the rest on the wire; the next individual
    /// transaction then reads one of them, and reports another servo's error
    /// as its own. On a chain that shows up as a servo rejecting a value its
    /// neighbour has just accepted.
    #[test]
    fn a_reply_from_another_servo_is_not_taken_as_this_ones() {
        let servo = FakeServo::default();
        // Left over from an earlier transaction, and carrying an error.
        servo.reply_with(9, 0x04, &[]);
        // The answer actually being waited for.
        servo.reply_with(3, 0, &[42]);

        let status = driver(&servo)
            .read(3, 132, 1)
            .expect("the stale reply must be skipped, not returned");
        assert_eq!(status, vec![42]);
    }

    /// And the guard must not swallow a genuine refusal from the right servo.
    #[test]
    fn an_error_from_the_addressed_servo_still_surfaces() {
        let servo = FakeServo::default();
        servo.reply_with(3, 0x04, &[]);
        let error = driver(&servo).read(3, 132, 1).unwrap_err();
        assert!(format!("{error}").contains("Data range error"), "{error}");
    }

    #[test]
    fn replies_arriving_in_one_read_are_all_returned() {
        /// A port that returns every queued reply concatenated, in one read.
        #[derive(Clone, Default)]
        struct Burst(Arc<Mutex<Vec<u8>>>);
        impl SerialPort for Burst {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                let mut pending = self.0.lock().unwrap();
                let n = pending.len().min(buf.len());
                buf[..n].copy_from_slice(&pending[..n]);
                pending.drain(..n);
                Ok(n)
            }
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                Ok(buf.len())
            }
        }

        let burst = Burst::default();
        {
            let mut pending = burst.0.lock().unwrap();
            for (id, value) in [(1u8, 10u8), (2, 20), (3, 30)] {
                let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
                let params = [value, 0, 0, 0];
                let length = 1 + 1 + params.len() + 2;
                packet.push((length & 0xFF) as u8);
                packet.push(((length >> 8) & 0xFF) as u8);
                packet.push(0x55);
                packet.push(0);
                packet.extend_from_slice(&params);
                let crc = update_crc(0, &packet);
                packet.extend_from_slice(&crc.to_le_bytes());
                pending.extend_from_slice(&packet);
            }
        }

        let mut driver = Driver::new(SharedPort::new(Box::new(burst)));
        driver.timeout = Duration::from_millis(20);
        driver.poll_interval = Duration::from_micros(100);

        let values = driver.sync_read(132, 4, &[1, 2, 3]).unwrap();
        assert_eq!(values.len(), 3, "all three replies were in the buffer");
        assert_eq!(values[&1][0], 10);
        assert_eq!(values[&2][0], 20);
        assert_eq!(values[&3][0], 30);
    }

    /// The extractor drains exactly the packet it returns, and skips leading
    /// noise rather than choking on it.
    #[test]
    fn taking_a_packet_leaves_the_rest_of_the_buffer() {
        let mut buf = vec![0x00, 0xAA]; // noise ahead of the header

        // header(4) id(1) length(2) instruction(1) error(1) crc(2)
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, 7, 4, 0, 0x55, 0];
        let crc = update_crc(0, &packet);
        packet.extend_from_slice(&crc.to_le_bytes());

        buf.extend_from_slice(&packet);
        buf.extend_from_slice(&[0xDE, 0xAD]); // the start of the next reply

        let taken = take_packet(&mut buf).expect("a whole packet is buffered");
        assert_eq!(taken, packet);
        assert_eq!(buf, vec![0xDE, 0xAD], "only the packet was consumed");
    }

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

    /// A stale reply from a servo that is not on this read's list — left on
    /// the wire by an earlier broadcast that was cut short — is skipped
    /// without costing one of the servos that *are* on the list its answer.
    #[test]
    fn a_stale_reply_from_another_servo_does_not_cost_a_slot() {
        let servo = FakeServo::default();
        servo.reply_with(13, 0, &[0x00]); // leftover from a previous read
        servo.reply_with(14, 0, &[0x00]);
        servo.reply_with(1, 0, &[0x01]);
        servo.reply_with(2, 0, &[0x02]);

        let values = driver(&servo).sync_read(70, 1, &[1, 2]).unwrap();
        assert_eq!(
            values.len(),
            2,
            "both servos on the list answered: {values:?}"
        );
        assert_eq!(values[&2], vec![0x02]);
    }

    /// A servo whose hardware-error register is set raises the alert bit on
    /// every reply. The reply is still an answer — the servo is there and
    /// its data is good — and the alert is reported alongside so the fault
    /// can be read and named, instead of the servo looking silent.
    #[test]
    fn an_alert_reply_is_an_answer_and_is_reported() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0, &[0x01]);
        servo.reply_with(2, ALERT_BIT, &[0x02]);

        let (values, alerts) = driver(&servo).sync_read_alerts(132, 1, &[1, 2]).unwrap();
        assert_eq!(values.len(), 2, "the alerting servo answered: {values:?}");
        assert_eq!(values[&2], vec![0x02]);
        assert_eq!(alerts.into_iter().collect::<Vec<_>>(), vec![2]);
    }

    /// The low bits are a refusal of the instruction itself; a reply that
    /// carries one has no data worth keeping, alert or not.
    #[test]
    fn a_refused_reply_is_not_an_answer() {
        let servo = FakeServo::default();
        servo.reply_with(1, 0x07, &[0x01]);
        servo.reply_with(2, ALERT_BIT | 0x07, &[0x02]);

        let (values, alerts) = driver(&servo).sync_read_alerts(132, 1, &[1, 2]).unwrap();
        assert!(values.is_empty(), "refusals are not readings: {values:?}");
        assert!(alerts.is_empty());
    }

    /// A stream of packets that fill no slot cannot hold the read forever:
    /// each one restarts the reply timeout, so the loop is bounded by count.
    #[test]
    fn a_sync_read_gives_up_after_a_bound_of_skipped_packets() {
        let servo = FakeServo::default();
        for _ in 0..(MAX_SKIPPED_REPLIES * 2 + 3) {
            servo.reply_with(9, 0, &[0x00]); // never on the list
        }
        servo.reply_with(1, 0, &[0x01]); // behind the pile

        let driver = driver(&servo);
        let values = driver.sync_read(132, 1, &[1, 2]).unwrap();
        assert!(values.is_empty(), "the bound was reached first: {values:?}");
        assert!(
            driver.needs_flush.load(Ordering::Relaxed),
            "what was left on the wire is flushed by the next transaction"
        );
    }

    /// What a chain puts on the wire in answer to a fast sync read: one
    /// packet under the broadcast id, a segment per servo, closed by the last
    /// servo's CRC.
    fn fast_reply(servo: &FakeServo, data_length: usize, segments: &[(u8, u8, Vec<u8>)]) {
        let length = 1 + segments.len() * (data_length + 4);
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID];
        packet.extend_from_slice(&(length as u16).to_le_bytes());
        packet.push(0x55);
        for (id, error, data) in segments {
            packet.push(*error);
            packet.push(*id);
            packet.extend_from_slice(data);
            let crc = update_crc(0, &packet);
            packet.extend_from_slice(&crc.to_le_bytes());
        }
        servo.replies.lock().unwrap().push(packet);
    }

    /// A fast sync read is one request and one reply for the whole list;
    /// each servo's segment becomes its reading, and the alert bit rides
    /// along as it does in a plain sync read.
    #[test]
    fn a_fast_sync_read_takes_every_reading_from_one_packet() {
        let servo = FakeServo::default();
        fast_reply(
            &servo,
            10,
            &[
                (1, 0, vec![1; 10]),
                (2, ALERT_BIT, vec![2; 10]),
                (3, 0, vec![3; 10]),
            ],
        );

        let (values, alerts) = driver(&servo).fast_sync_read(126, 10, &[1, 2, 3]).unwrap();
        assert_eq!(values.len(), 3, "{values:?}");
        assert_eq!(values[&2], vec![2; 10]);
        assert_eq!(alerts.into_iter().collect::<Vec<_>>(), vec![2]);

        let sent = servo.sent();
        assert_eq!(sent[4], BROADCAST_ID);
        assert_eq!(sent[7], INST_FAST_SYNC_READ);
        assert_eq!(&sent[8..12], &[126, 0, 10, 0]);
        assert_eq!(&sent[12..15], &[1, 2, 3]);
    }

    /// A servo whose firmware predates the instruction answers alone with an
    /// Instruction Error. That is a refusal that names the servo — the
    /// caller's cue to read it the slow way — not a silent chain.
    #[test]
    fn a_servo_without_fast_sync_read_is_named_in_the_refusal() {
        let servo = FakeServo::default();
        servo.reply_with(7, 0x02, &[]);

        let error = driver(&servo).fast_sync_read(126, 10, &[1, 7]).unwrap_err();
        let text = error.to_string();
        assert!(matches!(error, Error::Refused(_)), "{text}");
        assert!(text.contains("servo id 7"), "{text}");
        assert!(text.contains("Instruction error"), "{text}");
    }

    /// The allocation-free forms say the same things as the allocating ones.
    #[test]
    fn the_scratch_forms_agree_with_the_allocating_ones() {
        let servo = FakeServo::default();
        let driver = driver(&servo);
        let mut scratch = Scratch::new();

        // Sync write: identical bytes on the wire.
        let mut values = BTreeMap::new();
        values.insert(1u8, 1000u32.to_le_bytes().to_vec());
        values.insert(2u8, 2000u32.to_le_bytes().to_vec());
        driver.sync_write(116, 4, &values).unwrap();
        let allocating = servo.sent();
        servo.written.lock().unwrap().clear();
        let mut flat = Vec::new();
        flat.extend_from_slice(&1000u32.to_le_bytes());
        flat.extend_from_slice(&2000u32.to_le_bytes());
        driver
            .sync_write_into(&mut scratch, 116, 4, &[1, 2], &flat)
            .unwrap();
        assert_eq!(servo.sent(), allocating);

        // Sync read: the same answers, with a stuffed payload in one of them.
        servo.reply_with(1, 0, &[0xFF, 0xFF, 0xFD, 0xFD, 0x01]);
        servo.reply_with(2, ALERT_BIT, &[2; 5]);
        servo.reply_with(2, 0, &[9; 5]); // a duplicate, not counted twice
        let mut got = Vec::new();
        let answered = driver
            .sync_read_into(&mut scratch, 126, 4, &[1, 2], &mut |id, alert, data| {
                got.push((id, alert, data.to_vec()))
            })
            .unwrap();
        assert_eq!(answered, 2);
        assert_eq!(got[0], (1, false, vec![0xFF, 0xFF, 0xFD, 0x01]));
        assert_eq!(got[1], (2, true, vec![2; 5]));

        // Fast sync read: one packet, every segment.
        fast_reply(
            &servo,
            10,
            &[(1, 0, vec![1; 10]), (3, ALERT_BIT, vec![3; 10])],
        );
        let mut got = Vec::new();
        let outcome = driver
            .fast_sync_read_into(&mut scratch, 126, 10, &[1, 2, 3], &mut |id, alert, data| {
                got.push((id, alert, data.len()))
            })
            .unwrap();
        assert_eq!(outcome, FastOutcome::Answered(2));
        assert_eq!(got, vec![(1, false, 10), (3, true, 10)]);

        // And a refusal names the servo.
        servo.reply_with(7, 0x02, &[]);
        let outcome = driver
            .fast_sync_read_into(&mut scratch, 126, 10, &[1, 7], &mut |_, _, _| {})
            .unwrap();
        assert_eq!(outcome, FastOutcome::Unsupported(7));

        // And silence is silence.
        let outcome = driver
            .fast_sync_read_into(&mut scratch, 126, 10, &[1], &mut |_, _, _| {})
            .unwrap();
        assert_eq!(outcome, FastOutcome::Silent);
    }

    /// A silent servo leaves its segment out; the others still report.
    #[test]
    fn a_fast_sync_read_reports_the_servos_that_answered() {
        let servo = FakeServo::default();
        fast_reply(&servo, 4, &[(1, 0, vec![1; 4]), (3, 0, vec![3; 4])]);

        let (values, _) = driver(&servo).fast_sync_read(132, 4, &[1, 2, 3]).unwrap();
        assert_eq!(values.keys().copied().collect::<Vec<_>>(), vec![1, 3]);
    }

    /// A single exchange treats the alert bit the same way: the instruction
    /// went through, and the caller gets the reply.
    #[test]
    fn an_exchange_accepts_a_reply_that_only_carries_the_alert() {
        let servo = FakeServo::default();
        servo.reply_with(1, ALERT_BIT, &[0x21]);
        assert_eq!(driver(&servo).read(1, 70, 1).unwrap(), vec![0x21]);

        servo.reply_with(1, ALERT_BIT | 0x07, &[]);
        let error = driver(&servo).write(1, 116, &[0, 8, 0, 0]).unwrap_err();
        let text = error.to_string();
        assert!(text.contains("Access error"), "{text}");
        assert!(text.contains("hardware error"), "{text}");
    }
}
