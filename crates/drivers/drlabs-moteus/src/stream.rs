//! The tunnelled diagnostic console.
//!
//! Alongside the register interface, moteus carries a text console over the
//! same CAN frames. It is what `tview` and `moteus_tool` talk to, and it is the
//! only way to read the controller's stored configuration — most importantly
//! `servopos.position_min`/`max`, the travel envelope the firmware enforces.
//!
//! That envelope matters more than its obscurity suggests. Starting position
//! control while the axis sits outside it raises fault 39 immediately, and
//! commanding past it makes the firmware clamp the target and report fault 103.
//! Both look from the outside exactly like "the motor ignores me". Reading the
//! bounds at connect time turns that into a sentence an operator can act on.
//!
//! Subframes (see [`crate::multiplex`] for the framing):
//!
//! ```text
//! 0x40  client -> server data:  channel, length, bytes
//! 0x41  server -> client data:  channel, length, bytes
//! 0x42  client polls server:    channel, maximum length
//! ```

use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};
use drlabs_transport::can::SharedBus;

use crate::controller::Controller;
use crate::multiplex::{Reader, Writer};

const STREAM_CLIENT_DATA: u8 = 0x40;
const STREAM_SERVER_DATA: u8 = 0x41;
const STREAM_CLIENT_POLL: u8 = 0x42;

/// The console channel. Channel 0 is the firmware's own boot log.
const DIAGNOSTIC_CHANNEL: u32 = 1;

/// Keeps a write subframe inside one CAN-FD frame after the command byte and
/// the channel and length varuints are accounted for.
const MAX_STREAM_CHUNK: usize = 48;

/// The largest reply that still fits: 64 bytes less the command byte and the
/// channel and length varuints.
const MAX_POLL_CHUNK: u32 = 61;

/// A text console session with one controller.
///
/// It should not be interleaved with control cycles: the console is slow enough
/// that a `conf get` inside a running loop will blow the period budget. Use it
/// at connect time, or while the axis is stopped.
pub struct Stream<'a> {
    controller: &'a Controller,
    bus: &'a SharedBus,

    /// The pause between polls when the controller has nothing to say yet.
    pub poll_interval: Duration,

    /// Bounds one complete command/response exchange.
    pub timeout: Duration,

    /// How long the console must say nothing before a command is written into
    /// it, and how long to wait for that. See [`Stream::settle`].
    pub settle_quiet: Duration,
    pub settle_budget: Duration,

    /// How long [`Stream::drain`] needs the console to say nothing before it
    /// calls it quiet. Longer than `settle_quiet` because draining is used
    /// where the console may be mid-record rather than merely between lines.
    pub drain_quiet: Duration,

    /// Bounds [`Stream::drain`]. A controller asked for more output than
    /// anyone read — `conf enumerate` is thousands of lines — keeps emitting
    /// for far longer than a connect should wait, so draining gives up and says
    /// so rather than blocking.
    pub drain_budget: Duration,

    buf: Vec<u8>,
}

/// The console was still emitting when a command was about to be written, so
/// nothing read afterwards could be attributed to it.
///
/// This is deliberately an error rather than a best guess. `conf get` answers
/// with a bare value and echoes no key, so a reply cannot be matched to its
/// request: the only thing separating an answer from the tail of somebody
/// else's output is that the console was quiet beforehand. Without that, a
/// stale line is indistinguishable from a correct one — and a wrong travel
/// bound or torque limit reported confidently is worse than no answer at all.
pub fn console_busy() -> Error {
    Error::Refused("the console is still emitting; its next line cannot be trusted".to_string())
}

/// The controller's own description of itself.
///
/// It is the only place the board family and hardware revision are available:
/// there is no register for them. The record is normally emitted as a binary
/// telemetry frame, which would need the schema parser this driver deliberately
/// does without — but the console can be asked to emit it as text instead, and
/// then it is six lines of key and value.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FirmwareRecord {
    pub version: Option<i32>,
    pub model: Option<i32>,
    pub family: Option<i32>,
    pub hardware_revision: Option<i32>,
}

impl<'a> Stream<'a> {
    /// Opens a console session on the diagnostic channel.
    pub fn new(controller: &'a Controller, bus: &'a SharedBus) -> Self {
        Self {
            controller,
            bus,
            poll_interval: Duration::from_millis(2),
            timeout: Duration::from_secs(3),
            settle_quiet: Duration::from_millis(30),
            settle_budget: Duration::from_secs(1),
            drain_quiet: Duration::from_millis(150),
            drain_budget: Duration::from_secs(5),
            buf: Vec::new(),
        }
    }

    /// Sends one console command and returns the lines it produced, not
    /// counting the trailing `OK`.
    ///
    /// An `ERR` response comes back as an error carrying the controller's own
    /// message, which is almost always more specific than anything this layer
    /// could infer.
    pub fn command(&mut self, command: &str) -> Result<Vec<String>> {
        if !self.settle() {
            return Err(console_busy());
        }
        self.write(format!("{command}\n").as_bytes())?;

        let deadline = Instant::now() + self.timeout;
        let mut lines = Vec::new();
        loop {
            let line = self.read_line(deadline)?;
            // The terminator is matched as a prefix, not as a whole line. A
            // board that was streaming telemetry when we connected delivers
            // binary bytes on the same line, and an equality test would never
            // see the `OK` that follows them.
            if line.starts_with("OK") {
                return Ok(lines);
            }
            if line.starts_with("ERR") {
                return Err(Error::Refused(line));
            }
            lines.push(line);
        }
    }

    /// Sends a command and returns the single line it answers with.
    ///
    /// Some console commands — `conf get` among them — reply with the value and
    /// no `OK` terminator, so waiting for one just burns the timeout and then
    /// reports a failure for a query that actually succeeded. The reference
    /// tooling reads these with its "allow any response" mode; this is the same
    /// thing.
    pub fn query(&mut self, command: &str) -> Result<String> {
        if !self.settle() {
            return Err(console_busy());
        }
        self.write(format!("{command}\n").as_bytes())?;

        let line = self.read_line(Instant::now() + self.timeout)?;
        if line.starts_with("ERR") {
            return Err(Error::Refused(line));
        }
        Ok(line)
    }

    /// Reads one configuration value.
    pub fn conf_get(&mut self, key: &str) -> Result<String> {
        let line = self.query(&format!("conf get {key}"))?;
        Ok(line.trim().to_string())
    }

    /// [`Stream::conf_get`] parsed as a number.
    ///
    /// A configured `nan` means the bound is not set, which comes back as
    /// `Ok(None)` rather than an error — an unbounded axis is a perfectly
    /// ordinary configuration, and the Go version's NaN-in-a-float64 made every
    /// caller remember to check.
    pub fn conf_get_float(&mut self, key: &str) -> Result<Option<f64>> {
        let text = self.conf_get(key)?;
        if text.is_empty() || text.eq_ignore_ascii_case("nan") {
            return Ok(None);
        }
        text.parse::<f64>().map(Some).map_err(|_| {
            Error::Malformed(format!("{key} returned {text:?}, which is not a number"))
        })
    }

    /// Asks the controller to describe itself.
    ///
    /// Switching the channel to text format is a runtime setting on an emission
    /// that is already stopped, so it disturbs nothing; what it buys is the
    /// difference between a board this framework can check against a known
    /// envelope and one it can only be told about.
    pub fn read_firmware(&mut self) -> Result<FirmwareRecord> {
        self.command("tel fmt firmware 1")?;

        // The record may or may not be terminated by an OK depending on
        // firmware, so whatever arrived is parsed either way — a partial answer
        // that carries the family is worth more than an error.
        let (lines, error) = match self.command("tel get firmware") {
            Ok(lines) => (lines, None),
            Err(error) => (Vec::new(), Some(error)),
        };
        match parse_firmware_record(&lines) {
            Some(record) => Ok(record),
            None => Err(error.unwrap_or_else(|| {
                Error::Malformed("the firmware record carried no family or revision".into())
            })),
        }
    }

    /// Waits for the console to go quiet, so the next line read really is the
    /// answer to the command about to be written.
    ///
    /// Every console read is a read of "whatever the controller says next". If
    /// anything is still queued — the tail of a previous command, telemetry,
    /// the remains of a read that timed out half way — that is what comes back,
    /// and it is attributed to the wrong question. A quiet console beforehand
    /// is the only thing that makes the attribution sound, because `conf get`
    /// echoes no key to check the reply against.
    ///
    /// Reports false when the console would not go quiet inside the budget.
    pub fn settle(&mut self) -> bool {
        self.quieten(self.settle_quiet, self.settle_budget)
    }

    /// Discards whatever the console has buffered and waits for it to stop
    /// producing.
    ///
    /// Not optional housekeeping. A board that was emitting telemetry when the
    /// host connected keeps emitting into the same channel that carries command
    /// responses, so the first command's reply arrives interleaved with binary
    /// records. Draining until quiet is what separates the two, and it is why
    /// identification drains, stops telemetry, and drains again.
    ///
    /// Reports whether the console actually went quiet. A false means the
    /// controller is still emitting more than the budget's worth, and the
    /// caller should either call again, raise the budget, or treat the console
    /// as unavailable rather than believe whatever it says next.
    pub fn drain(&mut self) -> bool {
        self.quieten(self.drain_quiet, self.drain_budget)
    }

    /// Polls until nothing has arrived for `quiet`, or the budget runs out.
    fn quieten(&mut self, quiet: Duration, budget: Duration) -> bool {
        self.buf.clear();

        let deadline = Instant::now() + budget;
        let mut last_data = Instant::now();

        while Instant::now() < deadline {
            let chunk = match self.poll() {
                Ok(chunk) => chunk,
                Err(_) => return false,
            };
            if !chunk.is_empty() {
                last_data = Instant::now();
                continue;
            }
            if last_data.elapsed() > quiet {
                return true;
            }
            std::thread::sleep(self.poll_interval);
        }
        false
    }

    /// The next non-empty console line.
    ///
    /// Lines are terminated by either carriage return or newline: the console
    /// uses both, and splitting on only one of them merges responses into a
    /// single line that then matches nothing.
    fn read_line(&mut self, deadline: Instant) -> Result<String> {
        loop {
            if let Some(index) = self
                .buf
                .iter()
                .position(|&byte| byte == b'\r' || byte == b'\n')
            {
                let raw: Vec<u8> = self.buf.drain(..=index).collect();
                let line = String::from_utf8_lossy(&raw[..index])
                    .trim_end_matches(['\r', '\n', ' ', '\t'])
                    .to_string();
                if line.is_empty() {
                    continue;
                }
                return Ok(line);
            }

            if Instant::now() > deadline {
                return Err(Error::Timeout(self.timeout));
            }

            let chunk = self.poll()?;
            if chunk.is_empty() {
                std::thread::sleep(self.poll_interval);
                continue;
            }
            self.buf.extend_from_slice(&chunk);
        }
    }

    /// Pushes bytes into the controller's console, in frame-sized pieces.
    fn write(&mut self, data: &[u8]) -> Result<()> {
        for chunk in data.chunks(MAX_STREAM_CHUNK) {
            let mut writer = Writer::new();
            writer.push_u8(STREAM_CLIENT_DATA);
            writer.push_varuint(DIAGNOSTIC_CHANNEL);
            writer.push_varuint(chunk.len() as u32);
            writer.extend(chunk);

            self.controller
                .exchange(self.bus, writer.as_bytes(), false, |_| true)?;
        }
        Ok(())
    }

    /// Asks the controller for whatever console output it has buffered.
    fn poll(&self) -> Result<Vec<u8>> {
        let mut writer = Writer::new();
        writer.push_u8(STREAM_CLIENT_POLL);
        writer.push_varuint(DIAGNOSTIC_CHANNEL);
        writer.push_varuint(MAX_POLL_CHUNK);

        let reply = self
            .controller
            .exchange(self.bus, writer.as_bytes(), true, is_stream_reply)?;
        match reply {
            Some(reply) => Ok(parse_stream_data(&reply, DIAGNOSTIC_CHANNEL)),
            None => Ok(Vec::new()),
        }
    }
}

/// Accepts server-to-client console frames.
fn is_stream_reply(data: &[u8]) -> bool {
    data[0] == STREAM_SERVER_DATA
}

/// Extracts the console bytes from a 0x41 subframe addressed to `channel`.
fn parse_stream_data(data: &[u8], channel: u32) -> Vec<u8> {
    let mut reader = Reader::new(data);

    match reader.read_u8() {
        Ok(command) if command == STREAM_SERVER_DATA => {}
        _ => return Vec::new(),
    }
    match reader.read_varuint() {
        Ok(frame_channel) if frame_channel == channel => {}
        _ => return Vec::new(),
    }
    let length = match reader.read_varuint() {
        Ok(length) => length as usize,
        Err(_) => return Vec::new(),
    };
    if length > reader.remaining() {
        return Vec::new();
    }
    let start = reader.offset();
    data[start..start + length].to_vec()
}

/// Reads the `firmware.<field> <value>` lines.
fn parse_firmware_record(lines: &[String]) -> Option<FirmwareRecord> {
    let mut record = FirmwareRecord::default();
    let mut seen = false;

    for line in lines {
        let trimmed = line.trim();
        let (key, value) = match trimmed.split_once(' ') {
            Some(parts) => parts,
            None => continue,
        };
        let number: i32 = match value.trim().parse() {
            Ok(number) => number,
            Err(_) => continue,
        };
        match key {
            "firmware.version" => record.version = Some(number),
            "firmware.model" => record.model = Some(number),
            "firmware.family" => record.family = Some(number),
            "firmware.hwrev" => record.hardware_revision = Some(number),
            _ => continue,
        }
        seen = true;
    }

    if !seen || (record.family.is_none() && record.hardware_revision.is_none()) {
        return None;
    }
    Some(record)
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_transport::can::{self, Bus, Frame};
    use std::sync::{Arc, Mutex};

    /// A controller whose console can be made to babble.
    ///
    /// `noisy_polls` is how many polls answer with output before the console
    /// goes quiet — modelling "still emitting" as a *duration of noise* rather
    /// than as a fixed backlog of bytes, which is the distinction that made the
    /// first version of this fake useless: a backlog small enough to write into
    /// the test was consumed inside the settle budget, so the test passed
    /// whether or not the code under test settled at all.
    #[derive(Default)]
    struct ConsoleState {
        noisy_polls: usize,
        poll_cost: Duration,
        queued: Vec<Vec<u8>>,
        writes: Vec<Vec<u8>>,
    }

    #[derive(Clone)]
    struct ConsoleBus {
        state: Arc<Mutex<ConsoleState>>,
        pending: Arc<Mutex<Option<Vec<u8>>>>,
    }

    impl ConsoleBus {
        fn new() -> Self {
            Self {
                state: Arc::new(Mutex::new(ConsoleState::default())),
                pending: Arc::new(Mutex::new(None)),
            }
        }

        /// Queues a reply. It is only handed out *after* a command has been
        /// written, because that is when a real console produces one — a fake
        /// that answers before it is asked lets settle() eat the answer as
        /// noise, and then the test is measuring the fake.
        fn say(&self, text: &str) {
            self.state
                .lock()
                .unwrap()
                .queued
                .push(text.as_bytes().to_vec());
        }

        fn babble_for(&self, polls: usize, cost: Duration) {
            let mut state = self.state.lock().unwrap();
            state.noisy_polls = polls;
            state.poll_cost = cost;
        }

        fn written(&self) -> String {
            let state = self.state.lock().unwrap();
            state
                .writes
                .iter()
                .map(|chunk| String::from_utf8_lossy(chunk).into_owned())
                .collect()
        }

        fn frame(payload: Vec<u8>) -> Frame {
            // Source id 1, which is the controller answering.
            Frame {
                id: 0x0100,
                data: payload,
                ..Default::default()
            }
        }

        fn console_frame(text: &[u8]) -> Frame {
            let mut writer = Writer::new();
            writer.push_u8(STREAM_SERVER_DATA);
            writer.push_varuint(DIAGNOSTIC_CHANNEL);
            writer.push_varuint(text.len() as u32);
            writer.extend(text);
            Self::frame(writer.into_bytes())
        }
    }

    impl Bus for ConsoleBus {
        fn send(&mut self, frame: &Frame) -> can::Result<()> {
            let mut reader = Reader::new(&frame.data);
            let command = reader.read_u8().unwrap_or(0);
            if command == STREAM_CLIENT_DATA {
                let _channel = reader.read_varuint().unwrap_or(0);
                let length = reader.read_varuint().unwrap_or(0) as usize;
                let start = reader.offset();
                self.state
                    .lock()
                    .unwrap()
                    .writes
                    .push(frame.data[start..start + length].to_vec());
                return Ok(());
            }

            // A poll. Answer with noise while there is any, then with whatever
            // the test queued, then with nothing.
            let mut state = self.state.lock().unwrap();
            let reply = if state.noisy_polls > 0 {
                state.noisy_polls -= 1;
                let cost = state.poll_cost;
                drop(state);
                std::thread::sleep(cost);
                Self::console_frame(b"telemetry noise\r\n")
            } else if !state.queued.is_empty() && !state.writes.is_empty() {
                let text = state.queued.remove(0);
                drop(state);
                Self::console_frame(&text)
            } else {
                drop(state);
                Self::console_frame(b"")
            };
            *self.pending.lock().unwrap() = Some(reply.data);
            Ok(())
        }

        fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
            match self.pending.lock().unwrap().take() {
                Some(data) => Ok(Self::frame(data)),
                None => Err(can::Error::Timeout),
            }
        }

        fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
            Ok(())
        }
    }

    fn fixture(bus: &ConsoleBus) -> (Controller, SharedBus) {
        (Controller::new(1), SharedBus::new(Box::new(bus.clone())))
    }

    fn quick(stream: &mut Stream<'_>) {
        stream.poll_interval = Duration::from_micros(200);
        stream.settle_quiet = Duration::from_millis(5);
        stream.settle_budget = Duration::from_millis(60);
        stream.drain_quiet = Duration::from_millis(5);
        stream.drain_budget = Duration::from_millis(60);
        stream.timeout = Duration::from_millis(200);
    }

    #[test]
    fn a_quiet_console_answers_the_question_it_was_asked() {
        let bus = ConsoleBus::new();
        bus.say("0.5\r\n");
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        assert_eq!(stream.conf_get("servopos.position_max").unwrap(), "0.5");
        assert!(bus.written().contains("conf get servopos.position_max"));
    }

    /// The defect this whole settle mechanism exists for.
    ///
    /// A console that is still emitting hands back somebody else's output as
    /// the answer, with no error, and a travel bound invented that way is worse
    /// than an absent one — the preflight check would trust it.
    #[test]
    fn a_babbling_console_is_refused_rather_than_misread() {
        let bus = ConsoleBus::new();
        bus.babble_for(10_000, Duration::from_millis(1));
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        let error = stream.conf_get("servopos.position_min").unwrap_err();
        assert!(
            format!("{error}").contains("still emitting"),
            "expected a refusal, got {error}"
        );
        assert_eq!(
            bus.written(),
            "",
            "nothing should have been written into a busy console"
        );
    }

    /// Draining reports whether it succeeded, so a caller can tell the
    /// difference between a console that went quiet and one that ran out of
    /// budget still talking.
    #[test]
    fn draining_reports_whether_the_console_actually_went_quiet() {
        let bus = ConsoleBus::new();
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);
        assert!(stream.drain(), "a quiet console drains");

        bus.babble_for(10_000, Duration::from_millis(1));
        assert!(!stream.drain(), "a babbling one does not");
    }

    /// A `conf enumerate` backlog outlasts any reasonable connect wait, and the
    /// answer is to say so rather than to block.
    #[test]
    fn a_console_that_outlasts_the_budget_does_not_block_forever() {
        let bus = ConsoleBus::new();
        bus.babble_for(10_000, Duration::from_millis(1));
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        let started = Instant::now();
        assert!(!stream.drain());
        assert!(
            started.elapsed() < Duration::from_millis(500),
            "drain took {:?}",
            started.elapsed()
        );
    }

    /// Once it goes quiet, the same stream works normally. The refusal is about
    /// this moment, not about the session.
    #[test]
    fn a_console_that_settles_is_usable_again() {
        let bus = ConsoleBus::new();
        bus.babble_for(3, Duration::from_millis(1));
        bus.say("2.5\r\n");
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        assert_eq!(stream.conf_get("servopos.position_max").unwrap(), "2.5");
    }

    #[test]
    fn an_unset_bound_reads_as_absent_rather_than_as_a_number() {
        let bus = ConsoleBus::new();
        bus.say("nan\r\n");
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        assert_eq!(
            stream.conf_get_float("servopos.position_min").unwrap(),
            None
        );
    }

    #[test]
    fn the_firmware_record_carries_the_family_and_revision() {
        let record = parse_firmware_record(&[
            "firmware.version 271".into(),
            "firmware.family 0".into(),
            "firmware.hwrev 8".into(),
        ])
        .expect("a record");
        assert_eq!(record.family, Some(0));
        assert_eq!(record.hardware_revision, Some(8));
        assert_eq!(record.model, None);
    }

    /// A record with neither family nor revision identifies nothing, so it is
    /// no record at all rather than an empty one.
    #[test]
    fn a_record_without_identification_is_not_a_record() {
        assert_eq!(
            parse_firmware_record(&["firmware.version 271".into()]),
            None
        );
        assert_eq!(parse_firmware_record(&[]), None);
        assert_eq!(parse_firmware_record(&["nonsense".into()]), None);
    }

    /// Both terminators appear on this console, and splitting on only one of
    /// them merges responses into a line that matches nothing.
    #[test]
    fn either_terminator_ends_a_line() {
        let bus = ConsoleBus::new();
        bus.say("first\rsecond\n");
        let (controller, shared) = fixture(&bus);
        let mut stream = Stream::new(&controller, &shared);
        quick(&mut stream);

        assert_eq!(stream.query("anything").unwrap(), "first");
    }
}
