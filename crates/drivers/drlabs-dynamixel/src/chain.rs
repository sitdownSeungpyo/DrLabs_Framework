//! The servos on one daisy chain, cycled together.
//!
//! A Dynamixel bus is one wire, and a link that owns a servo does not own the
//! wire. With one link per joint and a transaction per link per cycle, the cost
//! of a control period is N round trips — and because each one holds the port
//! for its whole request-and-reply, the joints do not share the bus so much as
//! queue for it. Measured on a 16-servo chain at 1 Mbaud that came to about 22
//! transactions a second in total, against several hundred the wire can carry,
//! and joints past the eighth never energised at all: the loop could not run
//! often enough for a joint to arm.
//!
//! Protocol 2.0 already has the answer. A sync write puts every goal position
//! in one unanswered packet, and a sync read asks every servo for the same
//! register block and takes the replies back to back. So the chain — not the
//! link — owns the period: one write, one read, however many joints.
//!
//! The links still do everything else themselves. Arming, mode changes, the
//! bus watchdog and identification are rare, per-servo, and stay exactly where
//! they were; only the hot path moves here.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};

use crate::driver::Driver;

/// One servo's place in the chain.
#[derive(Debug, Default, Clone)]
struct Member {
    /// Where this servo's present-value block starts, and how long it is.
    /// Carried per member because the profile decides it, and a chain is
    /// allowed to mix families. `read_len` is the wide block, through voltage
    /// and temperature; `fast_len` the ten-byte one, through position, which
    /// is what most periods actually read.
    read_base: u16,
    read_len: u16,
    fast_len: u16,

    /// The goal registers this servo is being commanded on, register to
    /// little-endian bytes. Position (116/4), velocity (104/4) or current
    /// (102/2) — whichever the mode uses. Empty means this servo is left out
    /// of the goal sync writes (stopped, or in a mode with no goal this cycle).
    goals: BTreeMap<u16, Vec<u8>>,

    /// Torque enable/disable queued for the next cycle, with the register it
    /// lives in. Emitted in its own phase — off before the limits, on after —
    /// so the arm sequence is preserved. Consumed after emission.
    torque: Option<(u16, bool)>,

    /// Profile velocity, acceleration and bus watchdog queued for the next
    /// cycle, register to little-endian bytes. A BTreeMap so they emit in
    /// address order (watchdog 98, accel 108, velocity 112). Consumed after
    /// emission; the link re-queues only on change or a full cycle.
    limits: BTreeMap<u16, Vec<u8>>,

    /// The hardware-error register this servo's profile reports faults in.
    fault_addr: u16,

    /// The block as it last came back, and whether the last cycle heard from
    /// this servo at all.
    block: Option<Vec<u8>>,
    answered: bool,

    /// How many bytes this cycle's block read asked this servo for; zero when
    /// it was left out. The recovery read asks for the same block.
    asked: u16,

    /// Consecutive cycles in which this servo answered neither the broadcast
    /// nor the recovery read. Past `RECOVERY_GIVE_UP` the recovery read is
    /// skipped until the servo answers a broadcast again: a servo that is
    /// gone should not cost the chain a reply budget every period.
    misses: u32,

    /// Whether the last reply carried the hardware alert bit: the servo's
    /// error register is non-zero, whether or not that register has been
    /// read yet on the slower fault cadence.
    alert: bool,

    /// The hardware-error byte as it last came back. Faults latch in the
    /// servo until power-cycled, so a slightly stale byte is still the truth.
    fault: Option<u8>,

    /// The bus-watchdog register, and the byte it last read as. Zero means
    /// this servo has no watchdog register and is left out of that read.
    ///
    /// Read on the fault cadence for the same reason faults are: a fired
    /// watchdog latches at 0xFF, and while it is latched the servo answers
    /// every goal write with a data-range or access error — which a sync
    /// write, being unanswered, never hears. Without this read a latched
    /// servo looks like one that accepts every goal and moves to none.
    watchdog_addr: u16,
    watchdog: Option<u8>,
}

#[derive(Debug, Default)]
struct Inner {
    members: BTreeMap<u8, Member>,
    read_at: Option<Instant>,
    wide_read_at: Option<Instant>,
    faults_read_at: Option<Instant>,
}

/// Every servo on one port, sharing one transaction per period.
pub struct Chain {
    driver: Arc<Driver>,
    inner: Mutex<Inner>,

    /// How old a reading may be before the next servo to ask for one refreshes
    /// the whole chain. This is the bus period: a link asking faster than the
    /// bus runs gets the current reading rather than a transaction of its own.
    period: Duration,

    /// How often the wide block — through voltage and temperature — is read
    /// in place of the ten-byte fast one. Eleven more bytes per servo is
    /// cheap on one joint and 1.8 ms a period on sixteen, and voltage and
    /// temperature move on thermal timescales; reading them every tenth
    /// period is the same bargain the session's own full-query cadence makes
    /// for a servo off a chain.
    wide_period: Duration,

    /// How often the hardware-error registers are refreshed. Reading them one
    /// servo at a time used to cost a round trip per joint on every wide
    /// query, which undid the chain's whole point; here they travel as one
    /// more broadcast, and rarely. Faults latch in the servo, so a second of
    /// staleness costs nothing but the moment of discovery — the same bargain
    /// the session's verify cadence already makes.
    fault_period: Duration,
}

/// About the session's verify cadence at the default rate (97 cycles at
/// 100 Hz), without this crate having to know either number.
const DEFAULT_FAULT_PERIOD: Duration = Duration::from_secs(1);

/// The session reads a full query every tenth cycle; the chain's wide read
/// keeps the same cadence.
const WIDE_EVERY: u32 = 10;

/// The most servos to ask for in one sync read. A longer reply stream than
/// this loses its tail intermittently on a sixteen-servo bus; kept short, the
/// whole chain answers.
const MAX_SYNC_READ: usize = 8;

/// After this many consecutive silent cycles a servo is no longer asked
/// alone. Matches the session's comm-loss threshold: by then the joint is
/// e-stopped, and the recovery read would only be paying a reply budget a
/// period to confirm it. The first broadcast it answers resets the count.
const RECOVERY_GIVE_UP: u32 = 5;

impl Chain {
    pub fn new(driver: Arc<Driver>, period: Duration) -> Self {
        Self {
            driver,
            inner: Mutex::new(Inner::default()),
            period,
            wide_period: period.saturating_mul(WIDE_EVERY),
            fault_period: DEFAULT_FAULT_PERIOD,
        }
    }

    /// Overrides how often the wide block is read in place of the fast one.
    #[allow(dead_code)]
    pub fn with_wide_period(mut self, period: Duration) -> Self {
        self.wide_period = period;
        self
    }

    /// Overrides how often hardware-error registers are refreshed.
    #[allow(dead_code)]
    pub fn with_fault_period(mut self, period: Duration) -> Self {
        self.fault_period = period;
        self
    }

    /// The one Driver that owns this bus's port. A link on the chain adopts it
    /// as its own, so that every transaction on the bus — the chain's sync
    /// read/write and a link's individual mode/limit/torque writes alike —
    /// goes through a single `needs_flush` discipline. Without this each link
    /// built its own Driver over a clone of the same port, and a failure on
    /// one left stale bytes the next Driver would not flush before reading.
    pub fn driver(&self) -> Arc<Driver> {
        self.driver.clone()
    }

    /// Adds a servo to the chain, with the wide and fast register blocks its
    /// profile reads and the register its faults are reported in.
    pub fn join(&self, id: u8, read_base: u16, read_len: u16, fast_len: u16, fault_addr: u16) {
        let mut inner = self.lock();
        let member = inner.members.entry(id).or_default();
        member.read_base = read_base;
        member.read_len = read_len;
        member.fast_len = fast_len;
        member.fault_addr = fault_addr;
    }

    /// Names the register this servo's bus watchdog lives in, so the chain
    /// can watch for it latching.
    pub fn set_watchdog_addr(&self, id: u8, address: u16) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.watchdog_addr = address;
        }
    }

    pub fn leave(&self, id: u8) {
        self.lock().members.remove(&id);
    }

    /// Records a 4-byte goal (position or velocity) for the next sync write.
    pub fn set_goal(&self, id: u8, address: u16, counts: i32) {
        self.queue_goal(id, address, counts.to_le_bytes().to_vec());
    }

    /// Records a goal on an arbitrary register, as its little-endian bytes:
    /// position/velocity are four bytes, goal current two. Whichever this
    /// servo's mode uses this cycle.
    pub fn queue_goal(&self, id: u8, address: u16, bytes: Vec<u8>) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.goals.insert(address, bytes);
        }
    }

    /// Says this servo is not goal-commanded this period, so that a stale
    /// target is not written to a servo that has since been stopped or put in
    /// another mode. Drops every goal register for it.
    pub fn clear_goal(&self, id: u8) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.goals.clear();
        }
    }

    /// Queues torque enable/disable for the next cycle. Off is emitted before
    /// the limits and on after, so a joint arms in the right order.
    pub fn queue_torque(&self, id: u8, address: u16, enabled: bool) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.torque = Some((address, enabled));
        }
    }

    /// Queues a profile-velocity / profile-accel / bus-watchdog write for the
    /// next cycle, as the register's little-endian bytes.
    pub fn queue_limit(&self, id: u8, address: u16, bytes: Vec<u8>) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.limits.insert(address, bytes);
        }
    }

    /// Brings the chain's readings up to date if they have aged out, and
    /// returns this servo's present-value block.
    ///
    /// Whichever link asks first does the transaction for everyone; the rest
    /// block on the mutex for as long as it takes and then find the answer
    /// already waiting. `Ok(None)` means this servo did not answer the last
    /// sync read — a dropped cycle, which the loop counts and carries on from.
    pub fn refresh(&self, id: u8) -> Result<Option<Vec<u8>>> {
        let mut inner = self.lock();

        let stale = match inner.read_at {
            Some(at) => at.elapsed() >= self.period,
            None => true,
        };
        if stale {
            self.cycle(&mut inner)?;
        }

        Ok(match inner.members.get(&id) {
            Some(member) if member.answered => member.block.clone(),
            _ => None,
        })
    }

    /// This servo's hardware-error byte, as the chain last heard it. Costs
    /// nothing on the wire: the byte was taken in `refresh`'s broadcast, on
    /// the fault cadence. `None` means the servo has not answered a fault
    /// read yet.
    pub fn fault(&self, id: u8) -> Option<u8> {
        self.lock().members.get(&id).and_then(|member| member.fault)
    }

    /// Whether this servo's last reply carried the hardware alert bit. Set on
    /// every reply while its error register is non-zero, so it is known a
    /// cycle after the fault happens, before the register itself is read.
    pub fn alert(&self, id: u8) -> bool {
        self.lock()
            .members
            .get(&id)
            .map(|member| member.answered && member.alert)
            .unwrap_or(false)
    }

    /// The bus-watchdog byte as the chain last read it, on the fault cadence.
    /// `Some(0xFF)` is a latched watchdog. `None` means not read yet, or the
    /// servo has no such register.
    pub fn watchdog(&self, id: u8) -> Option<u8> {
        self.lock().members.get(&id).and_then(|member| member.watchdog)
    }

    /// Forgets a latched reading once the link has cleared the register, so
    /// the same latch is not reported again before the next fault read.
    pub fn forget_watchdog(&self, id: u8) {
        if let Some(member) = self.lock().members.get_mut(&id) {
            member.watchdog = None;
        }
    }

    /// One period on the wire: every goal out in one packet, every reading back
    /// in one more.
    fn cycle(&self, inner: &mut Inner) -> Result<()> {
        if trace_enabled() {
            if let Some(at) = inner.read_at {
                let gap = at.elapsed();
                if gap > self.period.saturating_mul(2) {
                    eprintln!(
                        "[dxl-chain] {:.0} ms since the last cycle (period {:.0} ms)",
                        gap.as_secs_f64() * 1e3,
                        self.period.as_secs_f64() * 1e3
                    );
                }
            }
        }
        // Every write on this bus rides one of a few broadcast sync writes,
        // emitted in a fixed order so a joint arms correctly: torque off, then
        // the limits, then torque on, then the goals. A broadcast is not
        // answered, so none of these can be misread as another transaction's
        // reply — which is what an individual write per joint per period was,
        // and what turned sixteen joints into a storm of data-range errors.
        //
        // Emitting: gather the phase's writes across every member, grouped by
        // register, and send one sync write per register.
        let emit = |driver: &Driver,
                    groups: BTreeMap<u16, BTreeMap<u8, Vec<u8>>>|
         -> Result<()> {
            for (address, values) in groups {
                let width = values.values().next().map(|b| b.len()).unwrap_or(0) as u16;
                driver.sync_write(address, width, &values)?;
            }
            Ok(())
        };

        // Phase 1 — torque off (a servo being disarmed or about to change a
        // limit that needs torque down first, though mode changes stay off the
        // cycle entirely).
        let mut torque_off: BTreeMap<u16, BTreeMap<u8, Vec<u8>>> = BTreeMap::new();
        let mut torque_on: BTreeMap<u16, BTreeMap<u8, Vec<u8>>> = BTreeMap::new();
        for (&id, member) in inner.members.iter() {
            if let Some((address, enabled)) = member.torque {
                let group = if enabled { &mut torque_on } else { &mut torque_off };
                group.entry(address).or_default().insert(id, vec![u8::from(enabled)]);
            }
        }
        emit(&self.driver, torque_off)?;

        // Phase 2 — limits, in register order (watchdog 98, accel 108,
        // velocity 112): a BTreeMap keys them that way for free.
        let mut limits: BTreeMap<u16, BTreeMap<u8, Vec<u8>>> = BTreeMap::new();
        for (&id, member) in inner.members.iter() {
            for (&address, bytes) in member.limits.iter() {
                limits.entry(address).or_default().insert(id, bytes.clone());
            }
        }
        emit(&self.driver, limits)?;

        // Phase 3 — torque on.
        emit(&self.driver, torque_on)?;

        // Phase 4 — goals.
        let mut goals: BTreeMap<u16, BTreeMap<u8, Vec<u8>>> = BTreeMap::new();
        for (&id, member) in inner.members.iter() {
            for (&address, bytes) in member.goals.iter() {
                goals.entry(address).or_default().insert(id, bytes.clone());
            }
        }
        emit(&self.driver, goals)?;

        // The one-shot queues are spent; goals persist until re-set or cleared,
        // matching the position goal's long-standing behaviour.
        for member in inner.members.values_mut() {
            member.torque = None;
            member.limits.clear();
        }

        // Then the readings, grouped the same way. Most periods take the
        // ten-byte fast block; on the wide cadence the whole block travels,
        // and voltage and temperature come with it.
        let wide = match inner.wide_read_at {
            Some(at) => at.elapsed() >= self.wide_period,
            None => true,
        };
        let mut reads: BTreeMap<(u16, u16), Vec<u8>> = BTreeMap::new();
        for (&id, member) in inner.members.iter_mut() {
            let len = if wide || member.fast_len == 0 {
                member.read_len
            } else {
                member.fast_len
            };
            member.asked = len;
            member.answered = false;
            if len > 0 {
                reads.entry((member.read_base, len)).or_default().push(id);
            }
        }

        // Split large groups into chunks: a single sync read of sixteen servos
        // sees its tail drop intermittently on this bench (the twelfth reply
        // is reliably the last), so a shorter reply stream per transaction
        // keeps every servo answering. Still O(1) in transactions per period
        // as far as joint count on a chain of this size.
        let read_started = Instant::now();
        for ((base, len), ids) in reads {
            for chunk in ids.chunks(MAX_SYNC_READ) {
                let (answers, alerts) = self.driver.sync_read_alerts(base, len, chunk)?;
                for (id, data) in answers {
                    if let Some(member) = inner.members.get_mut(&id) {
                        member.answered = true;
                        member.alert = alerts.contains(&id);
                        member.block = Some(data);
                        member.misses = 0;
                    }
                }
            }
        }

        self.recover_silent(inner, read_started)?;

        if wide {
            inner.wide_read_at = Some(Instant::now());
        }

        // Faults travel the same way, one broadcast for the whole chain, but
        // on their own slower cadence. A servo that stays silent keeps its
        // last answer: the register latches, so old news is still news.
        let faults_stale = match inner.faults_read_at {
            Some(at) => at.elapsed() >= self.fault_period,
            None => true,
        };
        if faults_stale {
            let mut reads: BTreeMap<u16, Vec<u8>> = BTreeMap::new();
            for (&id, member) in inner.members.iter() {
                if member.fault_addr > 0 {
                    reads.entry(member.fault_addr).or_default().push(id);
                }
            }
            // Chunked like the block reads, and for a sharper reason: asked
            // of fifteen servos in one packet, a one-byte read lost every
            // reply after the eleventh about half the time on this bench,
            // and each loss cost the chain the full reply timeout — a
            // 100 ms stall in the middle of a 100 ms period, once a second.
            for (addr, ids) in reads {
                for chunk in ids.chunks(MAX_SYNC_READ) {
                    let answers = self.driver.sync_read(addr, 1, chunk)?;
                    for (id, data) in answers {
                        if let Some(member) = inner.members.get_mut(&id) {
                            if let Some(&bits) = data.first() {
                                member.fault = Some(bits);
                            }
                        }
                    }
                }
            }

            // The watchdog register rides the same cadence, one more
            // broadcast a second, grouped by address like the faults.
            let mut reads: BTreeMap<u16, Vec<u8>> = BTreeMap::new();
            for (&id, member) in inner.members.iter() {
                if member.watchdog_addr > 0 {
                    reads.entry(member.watchdog_addr).or_default().push(id);
                }
            }
            for (addr, ids) in reads {
                for chunk in ids.chunks(MAX_SYNC_READ) {
                    let answers = self.driver.sync_read(addr, 1, chunk)?;
                    for (id, data) in answers {
                        if let Some(member) = inner.members.get_mut(&id) {
                            if let Some(&byte) = data.first() {
                                member.watchdog = Some(byte);
                            }
                        }
                    }
                }
            }
            if trace_enabled() {
                let latched: Vec<u8> = inner
                    .members
                    .iter()
                    .filter(|(_, m)| m.watchdog == Some(0xFF))
                    .map(|(&id, _)| id)
                    .collect();
                if !latched.is_empty() {
                    eprintln!("[dxl-chain] watchdog latched on ids {latched:?}");
                }
            }
            inner.faults_read_at = Some(Instant::now());
        }

        inner.read_at = Some(Instant::now());
        Ok(())
    }

    /// Asks each servo that stayed silent on the block broadcast once more,
    /// alone, before the cycle reports it as unanswered.
    ///
    /// Why this exists: on the sixteen-servo bench a sync read of eight loses
    /// its tail under load — ids 5 through 8 of a chunk, together, a few times
    /// an hour. Servos answer a sync read in turn, each after the one before
    /// it, so a single dropped reply silences everyone behind it; asked
    /// directly, the same servos answer. Before this pass one lost packet
    /// became four missed cycles for four joints, five of those in a row
    /// became a comm-loss e-stop, and the elbow dropped.
    ///
    /// Bounded two ways. Each read has the driver's reply budget, not the
    /// transaction timeout. And a servo silent for `RECOVERY_GIVE_UP` cycles
    /// running is left to the broadcast alone until it answers one: a servo
    /// that is gone must not cost every other joint a reply budget a period.
    fn recover_silent(&self, inner: &mut Inner, read_started: Instant) -> Result<()> {
        let silent: Vec<u8> = inner
            .members
            .iter()
            .filter(|(_, m)| !m.answered && m.asked > 0)
            .map(|(&id, _)| id)
            .collect();
        if silent.is_empty() {
            return Ok(());
        }

        let mut recovered = Vec::new();
        let mut lost = Vec::new();
        let mut given_up = Vec::new();
        for id in &silent {
            let Some(member) = inner.members.get_mut(id) else {
                continue;
            };
            if member.misses >= RECOVERY_GIVE_UP {
                member.misses = member.misses.saturating_add(1);
                given_up.push(*id);
                continue;
            }
            match self.driver.read_quick(*id, member.read_base, member.asked) {
                Ok((data, alert)) if data.len() >= member.asked as usize => {
                    member.answered = true;
                    member.alert = alert;
                    member.block = Some(data);
                    member.misses = 0;
                    recovered.push(*id);
                }
                // The port itself failing is the chain's failure, not one
                // servo's silence.
                Err(error @ Error::Transport { .. }) => return Err(error),
                Ok(_) | Err(_) => {
                    member.misses = member.misses.saturating_add(1);
                    lost.push(*id);
                }
            }
        }

        if trace_enabled() {
            eprintln!(
                "[dxl-chain] silent on the broadcast {silent:?}: recovered {recovered:?}, \
                 lost {lost:?}, given up {given_up:?} ({:.1} ms for the read phase)",
                read_started.elapsed().as_secs_f64() * 1e3
            );
        }
        Ok(())
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        // A poisoned lock means another link panicked mid-transaction. The
        // chain's own state is a cache of what the servos said, so carrying on
        // with it is safer than taking the whole machine down.
        self.inner.lock().unwrap_or_else(|error| error.into_inner())
    }
}

/// Bench tracing of the chain's timing and watchdog reads, to stderr, when
/// `DRLABS_DXL_TRACE` is set. Off it costs one environment lookup a cycle.
fn trace_enabled() -> bool {
    std::env::var_os("DRLABS_DXL_TRACE").is_some()
}

impl std::fmt::Debug for Chain {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Chain")
            .field("period", &self.period)
            .field("members", &self.lock().members.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_transport::serial::{SerialPort, SharedPort};
    use std::sync::Mutex as StdMutex;

    /// A chain of servos that answer a sync read in order, and record what was
    /// written to them.
    #[derive(Clone, Default)]
    struct FakeChain {
        written: Arc<StdMutex<Vec<u8>>>,
        replies: Arc<StdMutex<Vec<Vec<u8>>>>,
        flushes: Arc<StdMutex<u64>>,
        /// What a servo answers when read *directly* (a READ addressed to
        /// its id), as opposed to the scripted broadcast replies above. A
        /// servo in this map is one that drops out of the sync read but is
        /// still there when asked alone.
        direct: Arc<StdMutex<BTreeMap<u8, Vec<u8>>>>,
    }

    impl FakeChain {
        fn status_packet(id: u8, params: &[u8]) -> Vec<u8> {
            let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
            let length = 1 + 1 + params.len() + 2;
            packet.push((length & 0xFF) as u8);
            packet.push(((length >> 8) & 0xFF) as u8);
            packet.push(0x55);
            packet.push(0);
            packet.extend_from_slice(params);
            let crc = crate::protocol::update_crc(0, &packet);
            packet.extend_from_slice(&crc.to_le_bytes());
            packet
        }
        fn answer(&self, id: u8, params: &[u8]) {
            self.replies
                .lock()
                .unwrap()
                .push(Self::status_packet(id, params));
        }
        fn answer_direct(&self, id: u8, params: &[u8]) {
            self.direct.lock().unwrap().insert(id, params.to_vec());
        }
        fn sent(&self) -> Vec<u8> {
            self.written.lock().unwrap().clone()
        }
        /// The ids that were asked with a direct READ (instruction 0x02),
        /// in the order they were asked.
        fn direct_reads(&self) -> Vec<u8> {
            let sent = self.sent();
            let mut ids = Vec::new();
            let mut i = 0;
            while i + 8 <= sent.len() {
                if sent[i..i + 4] == [0xFF, 0xFF, 0xFD, 0x00] {
                    let id = sent[i + 4];
                    let length = u16::from_le_bytes([sent[i + 5], sent[i + 6]]) as usize;
                    if sent[i + 7] == 0x02 && id != 0xFE {
                        ids.push(id);
                    }
                    i += 7 + length;
                } else {
                    i += 1;
                }
            }
            ids
        }
    }

    impl SerialPort for FakeChain {
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
            // A direct READ to a servo with a scripted direct answer is
            // answered on the spot; a broadcast is answered by the script.
            if buf.len() >= 8 && buf[..4] == [0xFF, 0xFF, 0xFD, 0x00] && buf[7] == 0x02 {
                if let Some(params) = self.direct.lock().unwrap().get(&buf[4]) {
                    self.replies
                        .lock()
                        .unwrap()
                        .push(Self::status_packet(buf[4], params));
                }
            }
            Ok(buf.len())
        }
        fn flush_buffers(&mut self) -> std::io::Result<()> {
            *self.flushes.lock().unwrap() += 1;
            Ok(())
        }
    }

    fn chain(servo: &FakeChain, period: Duration) -> Chain {
        let mut driver = Driver::new(SharedPort::new(Box::new(servo.clone())));
        driver.timeout = Duration::from_millis(20);
        driver.reply_timeout = Duration::from_millis(5);
        driver.poll_interval = Duration::from_micros(100);
        // The behaviour tests below are about the present-value cycle. The
        // fault broadcast still fires once, on the first cycle, but a long
        // cadence keeps it from firing again mid-test; it has tests of its
        // own below.
        Chain::new(Arc::new(driver), period).with_fault_period(Duration::from_secs(3600))
    }

    /// The point of the whole module: three servos, one period, a fixed
    /// number of packets. A sync write, a sync read, and — first cycle only —
    /// the fault broadcast; never a packet per servo.
    #[test]
    fn a_period_is_one_write_and_one_read_however_many_servos() {
        let servo = FakeChain::default();
        for id in [1u8, 2, 3] {
            servo.answer(id, &[0; 10]);
        }
        let chain = chain(&servo, Duration::from_millis(0));
        for id in [1u8, 2, 3] {
            chain.join(id, 126, 10, 10, 70);
            chain.set_goal(id, 116, 2048);
        }

        chain.refresh(1).expect("the chain answered");

        // Two broadcast packets went out — a sync write and a sync read — and
        // not one packet per servo.
        let sent = servo.sent();
        let broadcasts = sent
            .windows(5)
            .filter(|w| w[..4] == [0xFF, 0xFF, 0xFD, 0x00] && w[4] == 0xFE)
            .count();
        assert_eq!(
            broadcasts, 3,
            "expected exactly a sync write, a sync read, and the first-cycle              fault broadcast"
        );

        // The sync write carries all three ids and their counts.
        assert!(sent.contains(&0x83), "no sync write instruction");
        assert!(sent.contains(&0x82), "no sync read instruction");
    }

    /// Every member gets its own reply, and the reply goes to the right one.
    #[test]
    fn each_servo_gets_its_own_reading() {
        let servo = FakeChain::default();
        servo.answer(1, &[11; 10]);
        servo.answer(2, &[22; 10]);
        // A long period, so the second link reads what the first fetched.
        let chain = chain(&servo, Duration::from_secs(60));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        assert_eq!(chain.refresh(1).unwrap(), Some(vec![11; 10]));
        // The second asks within the period, so it reads the cache rather than
        // starting a transaction of its own.
        assert_eq!(chain.refresh(2).unwrap(), Some(vec![22; 10]));
    }

    /// A servo that does not answer is a dropped cycle for that joint only.
    /// The others still get their readings.
    #[test]
    fn a_silent_servo_does_not_take_the_chain_down() {
        let servo = FakeChain::default();
        servo.answer(1, &[7; 10]); // id 2 says nothing
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        assert_eq!(chain.refresh(1).unwrap(), Some(vec![7; 10]));
        assert_eq!(
            servo.direct_reads(),
            vec![2],
            "the silent servo was asked once, alone"
        );
        // A zero period, so this asks for a fresh cycle in which nobody
        // answers; id 2 is still a dropped cycle, not an error.
        assert_eq!(chain.refresh(2).unwrap(), None, "id 2 never answered");
    }

    /// The bench failure this guards against: a servo drops out of the sync
    /// read but answers when asked alone. The cycle asks, and the joint gets
    /// its reading instead of a missed cycle.
    #[test]
    fn a_servo_silent_on_the_broadcast_is_recovered_by_a_direct_read() {
        let servo = FakeChain::default();
        servo.answer(1, &[7; 10]); // id 2 drops out of the broadcast…
        servo.answer_direct(2, &[9; 10]); // …but answers a direct read
        let chain = chain(&servo, Duration::from_secs(60));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        assert_eq!(chain.refresh(1).unwrap(), Some(vec![7; 10]));
        assert_eq!(chain.refresh(2).unwrap(), Some(vec![9; 10]), "recovered");
        assert_eq!(servo.direct_reads(), vec![2]);
    }

    /// A servo that is really gone is asked alone for a few cycles, then left
    /// to the broadcast — until it answers one, when the asking resumes.
    #[test]
    fn recovery_gives_up_on_a_servo_that_stays_silent_and_resumes_when_it_returns() {
        let servo = FakeChain::default();
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        for _ in 0..(RECOVERY_GIVE_UP + 3) {
            servo.answer(1, &[1; 10]); // id 2 silent every cycle
            assert_eq!(chain.refresh(2).unwrap(), None);
        }
        assert_eq!(
            servo.direct_reads().len(),
            RECOVERY_GIVE_UP as usize,
            "asked alone until the give-up count, then left to the broadcast"
        );

        // It answers a broadcast again: the next silence is asked about.
        servo.answer(1, &[1; 10]);
        servo.answer(2, &[2; 10]);
        assert_eq!(chain.refresh(2).unwrap(), Some(vec![2; 10]));
        servo.answer(1, &[1; 10]);
        assert_eq!(chain.refresh(2).unwrap(), None);
        assert_eq!(servo.direct_reads().len(), RECOVERY_GIVE_UP as usize + 1);
    }

    /// A reading inside the period is the one already taken. This is what stops
    /// sixteen links from turning one bus period into sixteen transactions.
    #[test]
    fn a_reading_within_the_period_does_not_start_a_transaction() {
        let servo = FakeChain::default();
        servo.answer(1, &[1; 10]);
        let chain = chain(&servo, Duration::from_secs(60));
        chain.join(1, 126, 10, 10, 70);

        chain.refresh(1).unwrap();
        let after_first = servo.sent().len();
        for _ in 0..10 {
            chain.refresh(1).unwrap();
        }
        assert_eq!(
            servo.sent().len(),
            after_first,
            "a fresh reading must not go back to the wire"
        );
    }

    /// Faults ride one broadcast for the whole chain, are cached between
    /// fault periods, and go out encoded as a one-byte sync read of the
    /// hardware-error register.
    #[test]
    fn faults_travel_in_one_broadcast_and_are_cached() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]); // block replies
        servo.answer(2, &[0; 10]);
        servo.answer(1, &[0x00]); // fault replies: id 1 clean,
        servo.answer(2, &[0x01]); // id 2 with an input-voltage error latched
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        chain.refresh(1).expect("the chain answered");
        assert_eq!(chain.fault(1), Some(0x00));
        assert_eq!(chain.fault(2), Some(0x01));

        // Pinned encoding: sync read (0x82), address 70, length 1, ids 1 2.
        let sent = servo.sent();
        let fault_read = [0x82, 70, 0, 1, 0, 1, 2];
        assert!(
            sent.windows(fault_read.len()).any(|w| w == fault_read),
            "no one-byte sync read of register 70 went out: {sent:02X?}"
        );

        // Within the fault period the next cycle re-reads blocks but not
        // faults: the byte count of fault reads on the wire stays put.
        let fault_reads_before = sent
            .windows(fault_read.len())
            .filter(|w| *w == fault_read)
            .count();
        servo.answer(1, &[0; 10]);
        servo.answer(2, &[0; 10]);
        chain.refresh(1).expect("the chain answered");
        let sent = servo.sent();
        let fault_reads_after = sent
            .windows(fault_read.len())
            .filter(|w| *w == fault_read)
            .count();
        assert_eq!(
            fault_reads_before, fault_reads_after,
            "a fault read went out inside the fault period"
        );
        // And the cache still answers.
        assert_eq!(chain.fault(2), Some(0x01));
    }

    /// Helper: the addresses of the sync writes (instruction 0x83) that went
    /// out, in order.
    fn sync_write_addrs(sent: &[u8]) -> Vec<u16> {
        let mut out = Vec::new();
        let mut i = 0;
        while i + 10 <= sent.len() {
            if sent[i..i + 5] == [0xFF, 0xFF, 0xFD, 0x00, 0xFE] && sent[i + 7] == 0x83 {
                out.push(u16::from_le_bytes([sent[i + 8], sent[i + 9]]));
            }
            i += 1;
        }
        out
    }

    /// The writes emit in arm order regardless of the order they were queued:
    /// torque-off, then limits ascending (watchdog 98, accel 108, velocity
    /// 112), then torque-on (64), then the goal (116). This is the invariant
    /// that protects a joint's arm sequence now that every write is a
    /// broadcast phase rather than an individual transaction.
    #[test]
    fn writes_emit_in_arm_order_not_queue_order() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]);
        servo.answer(2, &[0; 10]);
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        // Queue in a deliberately scrambled order.
        chain.queue_goal(1, 116, vec![0; 4]);
        chain.queue_limit(1, 112, vec![63, 0, 0, 0]);
        chain.queue_torque(2, 64, false); // a disarm on id 2
        chain.queue_torque(1, 64, true); // an arm on id 1
        chain.queue_limit(1, 98, vec![5]);
        chain.queue_limit(1, 108, vec![0, 0, 0, 0]);

        chain.refresh(1).expect("the chain answered");

        let addrs = sync_write_addrs(&servo.sent());
        // torque-off(64), limits 98/108/112, torque-on(64), goal(116).
        assert_eq!(
            addrs,
            vec![64, 98, 108, 112, 64, 116],
            "sync writes did not emit in arm order: {addrs:?}"
        );
    }

    /// A queued limit is emitted once and then consumed: the next cycle with
    /// nothing newly queued writes no limit register.
    #[test]
    fn a_queued_limit_is_consumed_after_one_cycle() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]);
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.queue_limit(1, 112, vec![63, 0, 0, 0]);

        chain.refresh(1).expect("cycle 1");
        assert!(
            sync_write_addrs(&servo.sent()).contains(&112),
            "the velocity limit was not written on the cycle it was queued"
        );
        let after_first = servo.sent().len();

        servo.answer(1, &[0; 10]);
        chain.refresh(1).expect("cycle 2");
        let second = &servo.sent()[after_first..];
        assert!(
            !sync_write_addrs(second).contains(&112),
            "a consumed limit was re-sent on the next cycle"
        );
    }

    /// Helper: how many sync reads (instruction 0x82) of the given register
    /// went out.
    fn sync_read_count(sent: &[u8], address: u16) -> usize {
        let mut n = 0;
        let mut i = 0;
        while i + 10 <= sent.len() {
            if sent[i..i + 5] == [0xFF, 0xFF, 0xFD, 0x00, 0xFE]
                && sent[i + 7] == 0x82
                && u16::from_le_bytes([sent[i + 8], sent[i + 9]]) == address
            {
                n += 1;
            }
            i += 1;
        }
        n
    }

    /// A group larger than `MAX_SYNC_READ` is split across several sync reads:
    /// a single long reply stream loses its tail on a full bus, so the chain
    /// asks in chunks. Nine members become two reads of the same register.
    #[test]
    fn a_read_larger_than_the_chunk_is_split() {
        let servo = FakeChain::default();
        for id in 1..=9u8 {
            servo.answer(id, &[0; 10]);
        }
        let chain = chain(&servo, Duration::from_millis(0));
        for id in 1..=9u8 {
            chain.join(id, 126, 10, 10, 70);
        }

        chain.refresh(1).expect("the chain answered");

        assert_eq!(
            sync_read_count(&servo.sent(), 126),
            2,
            "nine members over a chunk of eight should be two sync reads"
        );
    }

    /// One Driver per bus: after a sync read leaves the wire dirty (a servo
    /// went silent mid-stream), the very next transaction on the same Driver
    /// flushes before it reads. This is what Move 1 buys — the link's writes
    /// and the chain's reads now share one `needs_flush`.
    #[test]
    fn a_partial_read_makes_the_next_transaction_flush() {
        let servo = FakeChain::default();
        // Two ids joined, but only one answers: the sync read is partial and
        // arms needs_flush on the shared Driver.
        servo.answer(1, &[0; 10]);
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        let driver = chain.driver();
        let flushes_before = *servo.flushes.lock().unwrap();
        chain.refresh(1).expect("the chain answered for id 1");
        // The partial read (id 2 silent) should have set needs_flush.
        // Now a direct write on the SAME Driver must flush before writing.
        servo.answer(1, &[]); // status for the write
        let flushes_mid = *servo.flushes.lock().unwrap();
        let _ = driver.write_u8(1, 64, 1);
        let flushes_after = *servo.flushes.lock().unwrap();
        assert!(
            flushes_after > flushes_mid,
            "a direct write after a partial read did not flush (before={flushes_before},              mid={flushes_mid}, after={flushes_after})"
        );
    }

    /// The wide block travels on its own cadence; every other period takes
    /// the ten-byte fast block. Pinned by the sync read's length field.
    #[test]
    fn the_wide_block_travels_on_its_own_cadence() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 21]);
        let chain = chain(&servo, Duration::from_millis(0))
            .with_wide_period(Duration::from_secs(3600));
        chain.join(1, 126, 21, 10, 70);

        // First cycle: nothing has been read wide yet, so the wide block goes.
        chain.refresh(1).expect("the chain answered");
        let wide_read = [0x82, 126, 0, 21, 0, 1];
        assert!(
            servo.sent().windows(6).any(|w| w == wide_read),
            "the first cycle did not read the wide block: {:02X?}",
            servo.sent()
        );

        // Inside the wide period: the fast block, eleven bytes shorter.
        servo.answer(1, &[0; 10]);
        chain.refresh(1).expect("the chain answered");
        let fast_read = [0x82, 126, 0, 10, 0, 1];
        assert!(
            servo.sent().windows(6).any(|w| w == fast_read),
            "the second cycle did not read the fast block: {:02X?}",
            servo.sent()
        );
    }

    /// A servo that never answers a fault read has no fault byte — not a
    /// fabricated clean one.
    #[test]
    fn a_silent_servo_has_no_fault_byte() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]);
        servo.answer(2, &[0; 10]);
        servo.answer(1, &[0x04]); // only id 1 answers the fault read
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);

        chain.refresh(1).expect("the chain answered");
        assert_eq!(chain.fault(1), Some(0x04));
        assert_eq!(chain.fault(2), None, "id 2 never answered the fault read");
    }

    /// A servo that has stopped being position-commanded must not have its last
    /// target written to it again.
    #[test]
    fn a_cleared_goal_leaves_the_sync_write() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]);
        servo.answer(2, &[0; 10]);
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);
        chain.set_goal(1, 116, 100);
        chain.set_goal(2, 116, 200);
        chain.clear_goal(2);

        chain.refresh(1).unwrap();

        // 200 little-endian would appear as 0xC8; id 1's 100 as 0x64.
        let sent = servo.sent();
        assert!(sent.contains(&0x64), "id 1's target should have gone out");
        assert!(
            !sent.contains(&0xC8),
            "id 2 was cleared and must not be commanded"
        );
    }

    /// The bus-watchdog register is read with the faults, one more broadcast
    /// on the same cadence, and a latched servo (0xFF) is reported as such.
    /// This is what lets a sync-written goal that the servo silently refused
    /// become a visible failure.
    #[test]
    fn the_watchdog_register_rides_the_fault_broadcast() {
        let servo = FakeChain::default();
        servo.answer(1, &[0; 10]); // block replies
        servo.answer(2, &[0; 10]);
        servo.answer(1, &[0x00]); // fault replies
        servo.answer(2, &[0x00]);
        servo.answer(1, &[20]); // watchdog replies: id 1 armed at 400 ms,
        servo.answer(2, &[0xFF]); // id 2 latched
        let chain = chain(&servo, Duration::from_millis(0));
        chain.join(1, 126, 10, 10, 70);
        chain.join(2, 126, 10, 10, 70);
        chain.set_watchdog_addr(1, 98);
        chain.set_watchdog_addr(2, 98);

        chain.refresh(1).expect("the chain answered");
        assert_eq!(chain.watchdog(1), Some(20));
        assert_eq!(chain.watchdog(2), Some(0xFF));

        // Pinned encoding: sync read (0x82), address 98, length 1, ids 1 2.
        let sent = servo.sent();
        let watchdog_read = [0x82, 98, 0, 1, 0, 1, 2];
        assert!(
            sent.windows(watchdog_read.len()).any(|w| w == watchdog_read),
            "no one-byte sync read of register 98 went out: {sent:02X?}"
        );

        // Once the link has cleared the latch the reading is forgotten, so
        // the same latch is not reported twice before the next fault read.
        chain.forget_watchdog(2);
        assert_eq!(chain.watchdog(2), None);
    }

    /// The one-byte fault and watchdog broadcasts are chunked like the block
    /// reads: nine members become two sync reads of each register.
    #[test]
    fn fault_and_watchdog_reads_are_chunked() {
        let servo = FakeChain::default();
        for id in 1..=9u8 {
            servo.answer(id, &[0; 10]);
        }
        for id in 1..=9u8 {
            servo.answer(id, &[0]); // faults
        }
        for id in 1..=9u8 {
            servo.answer(id, &[20]); // watchdogs
        }
        let chain = chain(&servo, Duration::from_millis(0));
        for id in 1..=9u8 {
            chain.join(id, 126, 10, 10, 70);
            chain.set_watchdog_addr(id, 98);
        }

        chain.refresh(1).expect("the chain answered");

        assert_eq!(sync_read_count(&servo.sent(), 70), 2, "fault reads not chunked");
        assert_eq!(sync_read_count(&servo.sent(), 98), 2, "watchdog reads not chunked");
        assert_eq!(chain.watchdog(9), Some(20));
    }
}
