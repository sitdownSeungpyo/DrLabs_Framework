//! What the session keeps: a bounded log, and a bounded telemetry history.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use drlabs_core::telemetry::Telemetry;

/// How an event should be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Info,
    Ok,
    Warn,
    Error,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Info => "info",
            Level::Ok => "ok",
            Level::Warn => "warn",
            Level::Error => "error",
        }
    }
}

/// One entry in the session log.
///
/// `message` says what happened; `detail` says why and what to do about it. The
/// split exists so a UI can show a readable line and keep the explanation for
/// whoever needs it — most operator confusion with motor controllers comes from
/// a message that is true and unactionable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    pub seq: u64,
    pub at: Instant,
    pub level: Level,
    pub message: String,
    pub detail: String,
}

/// One cycle of recorded telemetry, kept for plotting.
///
/// A flat struct rather than a whole [`Telemetry`], because these accumulate at
/// the control rate: at 500 Hz a two-minute history is 60,000 of them, and the
/// difference between eight floats and forty is the difference between a few
/// megabytes and a few tens.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sample {
    pub seq: u64,
    pub elapsed: Duration,
    pub position: f64,
    pub velocity: f64,
    pub current: f64,
    pub torque: f64,
    pub voltage: f64,
    pub temperature: f64,
}

/// A bounded log.
#[derive(Debug)]
pub struct EventLog {
    items: VecDeque<Event>,
    limit: usize,
    next_id: u64,
}

impl EventLog {
    pub fn new(limit: usize) -> Self {
        Self {
            items: VecDeque::with_capacity(limit.min(1024)),
            limit: limit.max(1),
            next_id: 0,
        }
    }

    pub fn append(&mut self, level: Level, message: &str, detail: &str) -> Event {
        self.next_id += 1;
        let event = Event {
            seq: self.next_id,
            at: Instant::now(),
            level,
            message: message.to_string(),
            detail: detail.to_string(),
        };
        if self.items.len() >= self.limit {
            self.items.pop_front();
        }
        self.items.push_back(event.clone());
        event
    }

    /// The events newer than `seq`, oldest first.
    ///
    /// A client polls with the last sequence number it saw, so it gets each
    /// event once however often it asks.
    pub fn since(&self, seq: u64) -> Vec<Event> {
        self.items
            .iter()
            .filter(|event| event.seq > seq)
            .cloned()
            .collect()
    }

    pub fn latest_seq(&self) -> u64 {
        self.next_id
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

/// A bounded telemetry history.
#[derive(Debug)]
pub struct SampleRing {
    items: VecDeque<Sample>,
    limit: usize,
    next_id: u64,
}

impl SampleRing {
    pub fn new(limit: usize) -> Self {
        Self {
            items: VecDeque::with_capacity(limit.min(4096)),
            limit: limit.max(1),
            next_id: 0,
        }
    }

    pub fn append(&mut self, elapsed: Duration, telemetry: &Telemetry) {
        self.next_id += 1;
        let sample = Sample {
            seq: self.next_id,
            elapsed,
            position: telemetry.position.map(|v| v.value()).unwrap_or(f64::NAN),
            velocity: telemetry.velocity.map(|v| v.value()).unwrap_or(f64::NAN),
            current: telemetry.q_current.map(|v| v.value()).unwrap_or(f64::NAN),
            torque: telemetry.torque.map(|v| v.value()).unwrap_or(f64::NAN),
            voltage: telemetry.voltage.map(|v| v.value()).unwrap_or(f64::NAN),
            temperature: telemetry
                .board_temperature
                .map(|v| v.value())
                .unwrap_or(f64::NAN),
        };
        if self.items.len() >= self.limit {
            self.items.pop_front();
        }
        self.items.push_back(sample);
    }

    /// The samples newer than `seq`, oldest first, at most `limit` of them.
    ///
    /// The cap is what stops a client that has been away for a minute pulling a
    /// megabyte in one request and stalling the loop while it is serialised.
    pub fn since(&self, seq: u64, limit: usize) -> Vec<Sample> {
        let mut out: Vec<Sample> = self
            .items
            .iter()
            .filter(|sample| sample.seq > seq)
            .copied()
            .collect();
        if out.len() > limit {
            out.drain(..out.len() - limit);
        }
        out
    }

    pub fn latest(&self) -> Option<Sample> {
        self.items.back().copied()
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::units::{Rev, Volts};

    #[test]
    fn a_log_keeps_the_newest_entries() {
        let mut log = EventLog::new(3);
        for i in 0..5 {
            log.append(Level::Info, &format!("event {i}"), "");
        }
        assert_eq!(log.len(), 3);
        assert_eq!(log.since(0)[0].message, "event 2");
        assert_eq!(log.latest_seq(), 5);
    }

    /// A client polls with the last sequence it saw, so it gets each event
    /// once however often it asks.
    #[test]
    fn a_client_sees_each_event_exactly_once() {
        let mut log = EventLog::new(100);
        log.append(Level::Info, "first", "");
        log.append(Level::Warn, "second", "");

        let seen = log.since(0);
        assert_eq!(seen.len(), 2);
        let last = seen.last().unwrap().seq;
        assert!(log.since(last).is_empty());

        log.append(Level::Error, "third", "");
        let fresh = log.since(last);
        assert_eq!(fresh.len(), 1);
        assert_eq!(fresh[0].message, "third");
    }

    /// Sequence numbers keep counting past the ring's capacity, so a client
    /// that was away cannot be handed an event it has already seen.
    #[test]
    fn sequence_numbers_survive_the_ring_wrapping() {
        let mut log = EventLog::new(2);
        for i in 0..10 {
            log.append(Level::Info, &format!("{i}"), "");
        }
        let events = log.since(0);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].seq, 9);
        assert_eq!(events[1].seq, 10);
    }

    #[test]
    fn samples_keep_the_newest_history() {
        let mut ring = SampleRing::new(3);
        for i in 0..5 {
            ring.append(
                Duration::from_millis(i * 10),
                &Telemetry {
                    position: Some(Rev(i as f64)),
                    ..Default::default()
                },
            );
        }
        assert_eq!(ring.len(), 3);
        assert_eq!(ring.latest().unwrap().position, 4.0);
    }

    /// A field the controller did not report is NaN in a sample, not zero. A
    /// plot with a gap in it is honest; a plot that dips to zero is a lie about
    /// the machine.
    #[test]
    fn an_unreported_field_is_not_recorded_as_zero() {
        let mut ring = SampleRing::new(4);
        ring.append(
            Duration::ZERO,
            &Telemetry {
                position: Some(Rev(1.0)),
                voltage: Some(Volts(24.0)),
                // No torque reported.
                ..Default::default()
            },
        );
        let sample = ring.latest().unwrap();
        assert_eq!(sample.position, 1.0);
        assert!(
            sample.torque.is_nan(),
            "an absent torque must not read as 0"
        );
    }

    /// The cap is what stops a client that has been away pulling a megabyte in
    /// one request and stalling the loop while it is serialised.
    #[test]
    fn a_long_absence_does_not_return_the_whole_history() {
        let mut ring = SampleRing::new(1000);
        for i in 0..1000 {
            ring.append(Duration::from_millis(i), &Telemetry::default());
        }
        let batch = ring.since(0, 50);
        assert_eq!(batch.len(), 50);
        // And it is the *newest* 50, which is what a plot wants to catch up on.
        assert_eq!(batch.last().unwrap().seq, 1000);
    }
}
