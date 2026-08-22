//! The mjbots fdcanusb: a USB to CAN-FD adapter speaking line-oriented ASCII
//! over a CDC-ACM serial port.
//!
//! # Wire protocol
//!
//! ```text
//! host   -> device   can send <id_hex> <data_hex> [flags]\n
//! device -> host     OK\n                       command accepted
//! device -> host     rcv <id_hex> <data_hex> [flags]\n
//! device -> host     ERR <text>\n
//! ```
//!
//! Flags are a concatenation of single letters: `B` bit-rate switch enabled,
//! `b` disabled, `F` CAN-FD frame, `E` extended (29-bit) identifier.
//!
//! The adapter may ask for line checksums. When it answers a command with an
//! error mentioning "checksum", every subsequent line gains a trailing ` *XX`
//! CRC-8 and the mode stays on for the rest of the session.
//!
//! This implementation is deliberately synchronous: one send, then one receive,
//! driven by the caller's control loop. A moteus transaction is a single
//! request/reply pair, so a background reader would add a hand-off without
//! removing any latency.

mod checksum;

use std::time::{Duration, Instant};

use crate::can::{self, round_up_dlc, Bus, Error, Frame, Result, MAX_FD_PAYLOAD};
use crate::serial::{self, SerialPort};
use checksum::{append_checksum, strip_checksum, Checksum};

/// How long to wait for the `OK` that acknowledges a transmitted frame.
pub const DEFAULT_COMMAND_TIMEOUT: Duration = Duration::from_millis(200);

/// The pause between reads when the port has no data. The serial layer does not
/// block for long, so without it a wait would spin a core flat.
pub const DEFAULT_POLL_INTERVAL: Duration = Duration::from_micros(100);

/// How long a single read may block waiting for the adapter.
///
/// Short enough not to dominate a fast control period, long enough that an
/// answer arriving normally is caught by the blocking read rather than by the
/// next poll.
pub const READ_TIMEOUT: Duration = Duration::from_millis(1);

/// Bounds a single response line, so a device emitting garbage cannot grow the
/// buffer without limit.
const MAX_LINE_LENGTH: usize = 512;

/// Fills a CAN-FD frame out to the next valid DLC.
///
/// 0x50 is the multiplex protocol's NOP, so moteus ignores the padding rather
/// than misreading it as a register operation.
const PADDING_BYTE: u8 = 0x50;

/// A connection to one fdcanusb adapter.
pub struct Device {
    port: Box<dyn SerialPort>,
    path: String,

    /// Bytes read from the port that do not yet form a whole line.
    buf: Vec<u8>,

    /// Frames that arrived while waiting for something else, so a reply which
    /// overtakes its own acknowledgement is not lost.
    pending: Vec<Frame>,

    pub command_timeout: Duration,
    pub poll_interval: Duration,

    /// Suppresses the bit-rate switch, for buses whose wiring cannot carry the
    /// 5 Mbit/s data phase reliably.
    pub disable_brs: bool,

    checksum_active: bool,
    closed: bool,
}

/// Opens the adapter at `path`.
///
/// The baud rate is irrelevant for a CDC-ACM device — the USB endpoint runs at
/// USB speed and the line settings are ignored — but the host serial API still
/// demands one, so a conventional value goes through.
pub fn open(path: &str) -> Result<Device> {
    let port = serial::open(path, 115_200)?;
    let mut device = Device::new(Box::new(port), path);

    // A control loop's whole round trip is a few milliseconds, so a read that
    // blocks for the platform default — ten milliseconds on Windows, chosen for
    // a different protocol entirely — becomes the cycle's dominant cost. A port
    // that cannot be tuned keeps its default; the loop still works, it just
    // cannot run as fast.
    let _ = device.port.set_read_timeout(READ_TIMEOUT);
    Ok(device)
}

impl Device {
    /// Wraps an already-open port. Exists so tests can drive the protocol over
    /// an in-memory port.
    pub fn new(port: Box<dyn SerialPort>, path: &str) -> Self {
        Self {
            port,
            path: path.to_string(),
            buf: Vec::new(),
            pending: Vec::new(),
            command_timeout: DEFAULT_COMMAND_TIMEOUT,
            poll_interval: DEFAULT_POLL_INTERVAL,
            disable_brs: false,
            checksum_active: false,
            closed: false,
        }
    }

    /// The serial ports that look like a CAN adapter, best guess first.
    ///
    /// An empty result is not an error: the operator can always name the port.
    pub fn detect() -> Result<Vec<serial::PortInfo>> {
        Ok(serial::list()?)
    }

    /// Renders a frame as an fdcanusb command line.
    fn encode(&self, frame: &Frame) -> Vec<u8> {
        let on_wire = round_up_dlc(frame.data.len());

        let mut body = format!("can send {:04x} ", frame.id);
        for byte in &frame.data {
            body.push_str(&format!("{byte:02X}"));
        }
        for _ in frame.data.len()..on_wire {
            body.push_str(&format!("{PADDING_BYTE:02X}"));
        }

        body.push(' ');
        body.push(if frame.brs && !self.disable_brs {
            'B'
        } else {
            'b'
        });
        if frame.fd {
            body.push('F');
        }

        let mut line = body.into_bytes();
        if self.checksum_active {
            line = append_checksum(&line);
        }
        line.push(b'\n');
        line
    }

    /// The next complete line, with the checksum verified and stripped.
    ///
    /// A line that fails verification is skipped rather than reported: it
    /// carries no usable information, and reading again is the only sensible
    /// response.
    fn read_line(&mut self, deadline: Instant) -> Result<Vec<u8>> {
        let mut scratch = [0u8; 256];
        loop {
            if let Some(index) = self.buf.iter().position(|&byte| byte == b'\n') {
                let raw: Vec<u8> = self.buf.drain(..=index).collect();
                let trimmed = trim_end(&raw[..index]);
                let (line, verdict) = strip_checksum(trimmed);

                let usable = match verdict {
                    Checksum::Invalid => false,
                    // In checksum mode a line without one is not trustworthy
                    // either: it did not come from where we think it did, or it
                    // was mangled on the way.
                    Checksum::Absent => !self.checksum_active,
                    Checksum::Valid => true,
                };
                if !usable || line.is_empty() {
                    continue;
                }
                return Ok(line.to_vec());
            }

            if self.buf.len() > MAX_LINE_LENGTH {
                // No newline in a full line's worth of data: the stream is out
                // of sync. Drop it rather than growing without bound.
                self.buf.clear();
            }

            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }

            let n = self.port.read(&mut scratch)?;
            if n == 0 {
                std::thread::sleep(self.poll_interval);
                continue;
            }
            self.buf.extend_from_slice(&scratch[..n]);
        }
    }
}

impl Bus for Device {
    /// Transmits one frame and waits for the adapter to acknowledge it.
    ///
    /// The acknowledgement matters. Without it a caller can queue frames faster
    /// than the adapter drains them, and the backlog surfaces much later as
    /// telemetry belonging to a command several cycles old — which reads as a
    /// controller that is responding sluggishly rather than as a host that is
    /// running ahead of its link.
    fn send(&mut self, frame: &Frame) -> Result<()> {
        if self.closed {
            return Err(Error::Closed);
        }
        if frame.data.len() > MAX_FD_PAYLOAD {
            return Err(Error::Oversized(frame.data.len()));
        }

        // At most twice: once as sent, and once more if the adapter turns out
        // to be in checksum mode. A third attempt would mean it refused a line
        // that already had a checksum, which is a real error.
        for attempt in 0..2 {
            let line = self.encode(frame);
            self.port.write(&line)?;

            let deadline = Instant::now() + self.command_timeout;
            loop {
                let text = self.read_line(deadline)?;

                if text.starts_with(b"OK") {
                    return Ok(());
                } else if text.starts_with(b"rcv") {
                    // A reply that overtook its own acknowledgement. Keep it.
                    if let Ok(received) = parse_frame(&text) {
                        self.pending.push(received);
                    }
                } else if text.starts_with(b"ERR") {
                    let message = String::from_utf8_lossy(&text).trim().to_string();
                    let wants_checksum =
                        !self.checksum_active && message.to_ascii_lowercase().contains("checksum");
                    if wants_checksum && attempt == 0 {
                        // The adapter is configured to require checksums. Turn
                        // them on and resend; the session keeps using them from
                        // here.
                        self.checksum_active = true;
                        break;
                    }
                    return Err(Error::Adapter(message));
                }
                // Banner or unsolicited chatter: ignore and keep waiting.
            }
        }
        Err(Error::Adapter(
            "adapter refused the command twice, with and without a checksum".into(),
        ))
    }

    fn recv(&mut self, timeout: Duration) -> Result<Frame> {
        if !self.pending.is_empty() {
            return Ok(self.pending.remove(0));
        }
        if self.closed {
            return Err(Error::Closed);
        }

        let deadline = Instant::now() + timeout;
        loop {
            let text = self.read_line(deadline)?;
            if text.starts_with(b"rcv") {
                return parse_frame(&text);
            }
            if text.starts_with(b"ERR") {
                return Err(Error::Adapter(
                    String::from_utf8_lossy(&text).trim().to_string(),
                ));
            }
            // An `OK` from an earlier command, or a banner: not what we want.
        }
    }

    /// Discards whatever is queued and returns once the port has been quiet for
    /// `quiet`, so a stale reply cannot be mistaken for the answer to a new
    /// question.
    fn drain(&mut self, quiet: Duration) -> Result<()> {
        self.pending.clear();
        self.buf.clear();
        if self.closed {
            return Ok(());
        }

        let mut scratch = [0u8; 512];
        let mut deadline = Instant::now() + quiet;
        while Instant::now() < deadline {
            match self.port.read(&mut scratch) {
                // Still talking: the quiet period starts again.
                Ok(n) if n > 0 => deadline = Instant::now() + quiet,
                Ok(_) => std::thread::sleep(self.poll_interval),
                // A read error here becomes the caller's problem on the next
                // transaction, where it can be reported against something.
                Err(_) => return Ok(()),
            }
        }
        Ok(())
    }

    fn close(&mut self) -> Result<()> {
        self.closed = true;
        Ok(())
    }

    fn describe(&self) -> Vec<(String, String)> {
        vec![
            ("transport".into(), "fdcanusb".into()),
            ("port".into(), self.path.clone()),
        ]
    }
}

fn trim_end(bytes: &[u8]) -> &[u8] {
    let mut end = bytes.len();
    while end > 0 && (bytes[end - 1] == b'\r' || bytes[end - 1] == b' ') {
        end -= 1;
    }
    &bytes[..end]
}

/// Decodes an `rcv <id> <data> [flags]` line.
fn parse_frame(line: &[u8]) -> Result<Frame> {
    let text = String::from_utf8_lossy(line);
    let fields: Vec<&str> = text.split_whitespace().collect();
    if fields.len() < 3 {
        return Err(Error::Malformed(text.to_string()));
    }

    let id = u32::from_str_radix(fields[1], 16)
        .map_err(|_| Error::Malformed(format!("bad identifier in {text:?}")))?;
    let data = decode_hex(fields[2])
        .ok_or_else(|| Error::Malformed(format!("bad payload in {text:?}")))?;

    let mut frame = Frame {
        id,
        data,
        ..Default::default()
    };
    for flag in &fields[3..] {
        for character in flag.chars() {
            match character {
                'E' => frame.extended = true,
                'B' => frame.brs = true,
                'F' => frame.fd = true,
                _ => {}
            }
        }
    }
    Ok(frame)
}

fn decode_hex(text: &str) -> Option<Vec<u8>> {
    if text.len() % 2 != 0 {
        return None;
    }
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).ok())
        .collect()
}

/// So a caller holding a [`Device`] can hand it to anything wanting a bus.
impl From<Device> for can::SharedBus {
    fn from(device: Device) -> Self {
        can::SharedBus::new(Box::new(device))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A serial port made of a script.
    ///
    /// `written` records what the driver sent; `to_read` is what the adapter
    /// says back, one chunk per read, and an exhausted script reads as an empty
    /// port rather than as an error — which is what a real quiet port does.
    #[derive(Clone, Default)]
    struct FakePort {
        written: Arc<Mutex<Vec<u8>>>,
        to_read: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl FakePort {
        fn with_replies(lines: &[&str]) -> Self {
            Self {
                written: Arc::new(Mutex::new(Vec::new())),
                to_read: Arc::new(Mutex::new(
                    lines.iter().map(|line| line.as_bytes().to_vec()).collect(),
                )),
            }
        }

        fn sent(&self) -> String {
            String::from_utf8_lossy(&self.written.lock().unwrap()).into_owned()
        }
    }

    impl SerialPort for FakePort {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let mut queue = self.to_read.lock().unwrap();
            if queue.is_empty() {
                return Ok(0);
            }
            let chunk = queue.remove(0);
            let n = chunk.len().min(buf.len());
            buf[..n].copy_from_slice(&chunk[..n]);
            Ok(n)
        }

        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.written.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
    }

    fn device(port: &FakePort) -> Device {
        let mut device = Device::new(Box::new(port.clone()), "fake");
        device.poll_interval = Duration::from_micros(1);
        device
    }

    /// A reply as an adapter in checksum mode really sends it.
    ///
    /// The mode applies in both directions, so a fake that answers a
    /// checksummed command with a bare `OK` is not modelling the adapter — it
    /// is modelling a link that has half broken.
    fn checksummed(line: &str) -> String {
        let mut text = String::from_utf8(append_checksum(line.as_bytes())).unwrap();
        text.push('\n');
        text
    }

    #[test]
    fn a_frame_is_sent_as_an_ascii_command_line() {
        let port = FakePort::with_replies(&["OK\n"]);
        let mut bus = device(&port);
        bus.send(&Frame::fd(0x01, vec![0x0a, 0x0b, 0x0c, 0x0d]))
            .unwrap();
        assert_eq!(port.sent(), "can send 0001 0A0B0C0D BF\n");
    }

    /// Nine bytes do not exist on the wire, so the frame goes out as twelve.
    #[test]
    fn a_payload_is_padded_up_to_a_valid_length() {
        let port = FakePort::with_replies(&["OK\n"]);
        let mut bus = device(&port);
        bus.send(&Frame::fd(0x01, vec![0x11; 9])).unwrap();

        let sent = port.sent();
        let payload = sent.split_whitespace().nth(3).unwrap();
        assert_eq!(payload.len(), 24, "twelve bytes of hex: {sent}");
        // The padding is the multiplex NOP, so the controller ignores it rather
        // than reading it as a register operation.
        assert!(payload.ends_with("505050"), "{sent}");
    }

    #[test]
    fn the_bit_rate_switch_can_be_suppressed() {
        let port = FakePort::with_replies(&["OK\n"]);
        let mut bus = device(&port);
        bus.disable_brs = true;
        bus.send(&Frame::fd(0x01, vec![0x00])).unwrap();
        assert!(port.sent().ends_with(" bF\n"), "{}", port.sent());
    }

    #[test]
    fn a_received_line_becomes_a_frame() {
        let port = FakePort::with_replies(&["rcv 0001 0A0B0C EBF\n"]);
        let mut bus = device(&port);
        let frame = bus.recv(Duration::from_millis(50)).unwrap();

        assert_eq!(frame.id, 1);
        assert_eq!(frame.data, vec![0x0a, 0x0b, 0x0c]);
        assert!(frame.extended);
        assert!(frame.brs);
        assert!(frame.fd);
    }

    /// The reply can arrive before the acknowledgement it was caused by.
    /// Dropping it would leave the next receive reading the cycle after's.
    #[test]
    fn a_reply_that_overtakes_its_acknowledgement_is_kept() {
        let port = FakePort::with_replies(&["rcv 0001 0A0B0C\n", "OK\n"]);
        let mut bus = device(&port);
        bus.send(&Frame::fd(0x01, vec![0x00])).unwrap();

        let frame = bus.recv(Duration::from_millis(50)).unwrap();
        assert_eq!(frame.data, vec![0x0a, 0x0b, 0x0c]);
    }

    #[test]
    fn an_adapter_error_is_reported_rather_than_waited_out() {
        let port = FakePort::with_replies(&["ERR bad command\n"]);
        let mut bus = device(&port);
        let error = bus.send(&Frame::fd(0x01, vec![0x00])).unwrap_err();
        assert!(!error.is_timeout());
        assert!(format!("{error}").contains("bad command"), "{error}");
    }

    /// The adapter can be configured to demand checksums. Discovering that
    /// costs one refused command, and then the session carries them.
    #[test]
    fn checksum_mode_is_turned_on_when_the_adapter_asks_for_it() {
        let ok = checksummed("OK");
        let port = FakePort::with_replies(&["ERR checksum required\n", &ok]);
        let mut bus = device(&port);
        bus.send(&Frame::fd(0x01, vec![0x00])).unwrap();

        let sent = port.sent();
        let lines: Vec<&str> = sent.trim_end().split('\n').collect();
        assert_eq!(lines.len(), 2, "the frame is resent once: {sent}");
        assert!(!lines[0].contains('*'), "the first try has no checksum");
        assert!(
            lines[1].contains(" *"),
            "the retry carries one: {}",
            lines[1]
        );

        // And it stays on.
        port.to_read.lock().unwrap().push(ok.into_bytes());
        bus.send(&Frame::fd(0x02, vec![0x00])).unwrap();
        assert!(port
            .sent()
            .trim_end()
            .ends_with(|c: char| c.is_ascii_hexdigit()));
    }

    /// Once the adapter has demanded checksums, a line arriving without one did
    /// not survive the trip intact, whatever it appears to say.
    #[test]
    fn checksum_mode_ignores_lines_that_arrive_without_one() {
        let port = FakePort::with_replies(&["ERR checksum required\n", &checksummed("OK")]);
        let mut bus = device(&port);
        bus.send(&Frame::fd(0x01, vec![0x00])).unwrap();

        port.to_read
            .lock()
            .unwrap()
            .push(b"rcv 0001 0A0B0C\n".to_vec());
        let error = bus.recv(Duration::from_millis(20)).unwrap_err();
        assert!(
            error.is_timeout(),
            "expected the unchecked line to be ignored: {error}"
        );
    }

    #[test]
    fn a_silent_adapter_times_out_rather_than_blocking() {
        let port = FakePort::default();
        let mut bus = device(&port);
        let error = bus.recv(Duration::from_millis(5)).unwrap_err();
        assert!(error.is_timeout(), "{error}");
    }

    #[test]
    fn a_malformed_line_is_reported_as_malformed() {
        let port = FakePort::with_replies(&["rcv 0001\n"]);
        let mut bus = device(&port);
        let error = bus.recv(Duration::from_millis(50)).unwrap_err();
        assert!(matches!(error, Error::Malformed(_)), "{error}");
    }

    #[test]
    fn a_payload_larger_than_a_frame_is_refused_before_it_is_sent() {
        let port = FakePort::default();
        let mut bus = device(&port);
        let error = bus.send(&Frame::fd(0x01, vec![0x00; 65])).unwrap_err();
        assert!(matches!(error, Error::Oversized(65)), "{error}");
        assert_eq!(port.sent(), "", "nothing should reach the port");
    }

    /// A line split across two reads is still one line.
    #[test]
    fn a_line_split_across_reads_is_reassembled() {
        let port = FakePort::with_replies(&["rcv 0001 0A", "0B0C\n"]);
        let mut bus = device(&port);
        let frame = bus.recv(Duration::from_millis(50)).unwrap();
        assert_eq!(frame.data, vec![0x0a, 0x0b, 0x0c]);
    }

    #[test]
    fn draining_discards_what_is_already_queued() {
        let port = FakePort::with_replies(&["rcv 0001 0A0B0C\n", "rcv 0002 0D\n"]);
        let mut bus = device(&port);
        bus.drain(Duration::from_millis(2)).unwrap();

        let error = bus.recv(Duration::from_millis(5)).unwrap_err();
        assert!(error.is_timeout(), "stale frames should be gone: {error}");
    }
}
