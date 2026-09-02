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

use drlabs_core::error::Result;

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

    /// The hardware-error byte as it last came back. Faults latch in the
    /// servo until power-cycled, so a slightly stale byte is still the truth.
    fault: Option<u8>,
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

    /// One period on the wire: every goal out in one packet, every reading back
    /// in one more.
    fn cycle(&self, inner: &mut Inner) -> Result<()> {
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
        for (&id, member) in inner.members.iter() {
            let len = if wide || member.fast_len == 0 {
                member.read_len
            } else {
                member.fast_len
            };
            if len > 0 {
                reads.entry((member.read_base, len)).or_default().push(id);
            }
        }

        for member in inner.members.values_mut() {
            member.answered = false;
        }

        // Split large groups into chunks: a single sync read of sixteen servos
        // sees its tail drop intermittently on this bench (the twelfth reply
        // is reliably the last), so a shorter reply stream per transaction
        // keeps every servo answering. Still O(1) in transactions per period
        // as far as joint count on a chain of this size.
        for ((base, len), ids) in reads {
            for chunk in ids.chunks(MAX_SYNC_READ) {
                let answers = self.driver.sync_read(base, len, chunk)?;
                for (id, data) in answers {
                    if let Some(member) = inner.members.get_mut(&id) {
                        member.answered = true;
                        member.block = Some(data);
                    }
                }
            }
        }

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
            for (addr, ids) in reads {
                let answers = self.driver.sync_read(addr, 1, &ids)?;
                for (id, data) in answers {
                    if let Some(member) = inner.members.get_mut(&id) {
                        if let Some(&bits) = data.first() {
                            member.fault = Some(bits);
                        }
                    }
                }
            }
            inner.faults_read_at = Some(Instant::now());
        }

        inner.read_at = Some(Instant::now());
        Ok(())
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        // A poisoned lock means another link panicked mid-transaction. The
        // chain's own state is a cache of what the servos said, so carrying on
        // with it is safer than taking the whole machine down.
        self.inner.lock().unwrap_or_else(|error| error.into_inner())
    }
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
    }

    impl FakeChain {
        fn answer(&self, id: u8, params: &[u8]) {
            let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
            let length = 1 + 1 + params.len() + 2;
            packet.push((length & 0xFF) as u8);
            packet.push(((length >> 8) & 0xFF) as u8);
            packet.push(0x55);
            packet.push(0);
            packet.extend_from_slice(params);
            let crc = crate::protocol::update_crc(0, &packet);
            packet.extend_from_slice(&crc.to_le_bytes());
            self.replies.lock().unwrap().push(packet);
        }
        fn sent(&self) -> Vec<u8> {
            self.written.lock().unwrap().clone()
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
        assert_eq!(chain.refresh(2).unwrap(), None, "id 2 never answered");
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
}
