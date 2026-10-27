//! Encoding and decoding the frames. See the crate documentation for the
//! layout.
//!
//! Every field is written and read by hand rather than by transmuting the
//! `repr(C)` structs: the structs' layout is pinned by tests and happens to
//! match, but a frame is a contract with other languages and the byte order
//! belongs in the code that produces it, not in the compiler's choice.

use drlabs_core::lowlevel::{LowCmd, LowState, MotorCmd, MotorState, MAX_AXES, PROTOCOL_VERSION};

use crate::crc::crc32;

pub const MAGIC: [u8; 4] = *b"DLRT";

pub const KIND_CMD: u16 = 1;
pub const KIND_STATE: u16 = 2;
pub const KIND_DESCRIBE: u16 = 3;
pub const KIND_DESCRIBE_REPLY: u16 = 4;

/// Frame layout sizes, for a client working out buffer sizes.
pub const HEAD: usize = 8;
pub const TAIL: usize = 4;
pub const CMD_HEADER: usize = 24;
pub const STATE_HEADER: usize = 40;
pub const MOTOR_CMD: usize = 24;
pub const MOTOR_STATE: usize = 28;

/// The largest frame the runtime will send or accept.
pub const MAX_FRAME: usize = HEAD + STATE_HEADER + MAX_AXES * MOTOR_STATE + TAIL;

/// Why a frame was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameError {
    TooShort,
    BadMagic,
    /// The sender speaks another version of the records.
    Version(u16),
    BadCrc,
    UnknownKind(u16),
    /// The payload does not hold the `n` entries the header claims.
    Truncated,
    /// More entries than a runtime carries.
    TooMany(u16),
    NotUtf8,
}

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameError::TooShort => write!(f, "frame too short"),
            FrameError::BadMagic => write!(f, "not a DrLabs frame"),
            FrameError::Version(v) => write!(
                f,
                "protocol version {v}, this runtime speaks {PROTOCOL_VERSION}"
            ),
            FrameError::BadCrc => write!(f, "CRC mismatch"),
            FrameError::UnknownKind(k) => write!(f, "unknown frame kind {k}"),
            FrameError::Truncated => write!(f, "payload shorter than its count says"),
            FrameError::TooMany(n) => write!(f, "{n} entries, at most {MAX_AXES}"),
            FrameError::NotUtf8 => write!(f, "description is not UTF-8"),
        }
    }
}

impl std::error::Error for FrameError {}

/// A decoded frame.
#[derive(Debug, Clone, PartialEq)]
pub enum Frame<'a> {
    Cmd(LowCmd),
    State(LowState),
    Describe,
    DescribeReply(&'a str),
}

struct Writer<'a>(&'a mut Vec<u8>);

impl Writer<'_> {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn zeros(&mut self, n: usize) {
        self.0.resize(self.0.len() + n, 0);
    }
}

struct Reader<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], FrameError> {
        if self.at + n > self.data.len() {
            return Err(FrameError::Truncated);
        }
        let slice = &self.data[self.at..self.at + n];
        self.at += n;
        Ok(slice)
    }
    fn u8(&mut self) -> Result<u8, FrameError> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, FrameError> {
        let b = self.take(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }
    fn u32(&mut self) -> Result<u32, FrameError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    fn u64(&mut self) -> Result<u64, FrameError> {
        let b = self.take(8)?;
        Ok(u64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }
    fn f32(&mut self) -> Result<f32, FrameError> {
        let b = self.take(4)?;
        Ok(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
}

fn open(out: &mut Vec<u8>, kind: u16) {
    out.clear();
    let mut w = Writer(out);
    w.0.extend_from_slice(&MAGIC);
    w.u16(PROTOCOL_VERSION);
    w.u16(kind);
}

fn close(out: &mut Vec<u8>) {
    let crc = crc32(out);
    out.extend_from_slice(&crc.to_le_bytes());
}

/// Encodes a command into `out`, which is cleared first. Only the `n`
/// entries in use travel.
pub fn encode_cmd(cmd: &LowCmd, out: &mut Vec<u8>) {
    let n = (cmd.n as usize).min(MAX_AXES);
    open(out, KIND_CMD);
    let mut w = Writer(out);
    w.u32(cmd.seq);
    w.u32(cmd.flags);
    w.u64(cmd.t_ns);
    w.u16(n as u16);
    w.zeros(6);
    for motor in &cmd.cmd[..n] {
        w.u8(motor.mode);
        w.zeros(3);
        w.f32(motor.q);
        w.f32(motor.dq);
        w.f32(motor.kp);
        w.f32(motor.kd);
        w.f32(motor.tau);
    }
    close(out);
}

/// Encodes a state into `out`, which is cleared first.
pub fn encode_state(state: &LowState, out: &mut Vec<u8>) {
    let n = (state.n as usize).min(MAX_AXES);
    open(out, KIND_STATE);
    let mut w = Writer(out);
    w.u32(state.seq);
    w.u32(state.cmd_seq);
    w.u64(state.tick);
    w.u64(state.t_ns);
    w.u32(state.period_ns);
    w.u32(state.deadline_miss);
    w.u16(state.estop_code);
    w.u8(state.estop);
    w.u8(0);
    w.u16(n as u16);
    w.zeros(2);
    for motor in &state.state[..n] {
        w.f32(motor.q);
        w.f32(motor.dq);
        w.f32(motor.tau);
        w.f32(motor.current);
        w.f32(motor.voltage);
        w.f32(motor.temp);
        w.u16(motor.fault);
        w.u8(motor.status);
        w.u8(motor.blocked);
    }
    close(out);
}

pub fn encode_describe(out: &mut Vec<u8>) {
    open(out, KIND_DESCRIBE);
    close(out);
}

pub fn encode_describe_reply(json: &str, out: &mut Vec<u8>) {
    open(out, KIND_DESCRIBE_REPLY);
    out.extend_from_slice(json.as_bytes());
    close(out);
}

/// Decodes any frame.
pub fn decode(data: &[u8]) -> Result<Frame<'_>, FrameError> {
    if data.len() < HEAD + TAIL {
        return Err(FrameError::TooShort);
    }
    if data[..4] != MAGIC {
        return Err(FrameError::BadMagic);
    }
    let body = &data[..data.len() - TAIL];
    let received = u32::from_le_bytes([
        data[data.len() - 4],
        data[data.len() - 3],
        data[data.len() - 2],
        data[data.len() - 1],
    ]);
    if crc32(body) != received {
        return Err(FrameError::BadCrc);
    }
    let version = u16::from_le_bytes([data[4], data[5]]);
    if version != PROTOCOL_VERSION {
        return Err(FrameError::Version(version));
    }
    let kind = u16::from_le_bytes([data[6], data[7]]);
    let mut r = Reader {
        data: body,
        at: HEAD,
    };
    match kind {
        KIND_CMD => {
            let mut cmd = LowCmd::EMPTY;
            cmd.seq = r.u32()?;
            cmd.flags = r.u32()?;
            cmd.t_ns = r.u64()?;
            let n = r.u16()?;
            r.take(6)?;
            if n as usize > MAX_AXES {
                return Err(FrameError::TooMany(n));
            }
            cmd.n = n;
            for motor in cmd.cmd.iter_mut().take(n as usize) {
                *motor = MotorCmd {
                    mode: r.u8()?,
                    reserved: {
                        r.take(3)?;
                        [0; 3]
                    },
                    q: r.f32()?,
                    dq: r.f32()?,
                    kp: r.f32()?,
                    kd: r.f32()?,
                    tau: r.f32()?,
                };
            }
            Ok(Frame::Cmd(cmd))
        }
        KIND_STATE => {
            let mut state = LowState::EMPTY;
            state.seq = r.u32()?;
            state.cmd_seq = r.u32()?;
            state.tick = r.u64()?;
            state.t_ns = r.u64()?;
            state.period_ns = r.u32()?;
            state.deadline_miss = r.u32()?;
            state.estop_code = r.u16()?;
            state.estop = r.u8()?;
            r.u8()?;
            let n = r.u16()?;
            r.take(2)?;
            if n as usize > MAX_AXES {
                return Err(FrameError::TooMany(n));
            }
            state.n = n;
            for motor in state.state.iter_mut().take(n as usize) {
                *motor = MotorState {
                    q: r.f32()?,
                    dq: r.f32()?,
                    tau: r.f32()?,
                    current: r.f32()?,
                    voltage: r.f32()?,
                    temp: r.f32()?,
                    fault: r.u16()?,
                    status: r.u8()?,
                    blocked: r.u8()?,
                };
            }
            Ok(Frame::State(state))
        }
        KIND_DESCRIBE => Ok(Frame::Describe),
        KIND_DESCRIBE_REPLY => std::str::from_utf8(&body[HEAD..])
            .map(Frame::DescribeReply)
            .map_err(|_| FrameError::NotUtf8),
        other => Err(FrameError::UnknownKind(other)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::lowlevel::mode;

    fn sample_cmd() -> LowCmd {
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = 7;
        cmd.flags = 2;
        cmd.t_ns = 0x0102_0304_0506_0708;
        cmd.n = 2;
        cmd.cmd[0] = MotorCmd {
            mode: mode::DRIVE,
            reserved: [0; 3],
            q: 0.5,
            dq: -1.0,
            kp: 10.0,
            kd: 0.5,
            tau: 0.25,
        };
        cmd
    }

    fn sample_state() -> LowState {
        let mut state = LowState::EMPTY;
        state.seq = 9;
        state.cmd_seq = 7;
        state.tick = 123_456;
        state.t_ns = 0x1122_3344_5566_7788;
        state.period_ns = 2_000_000;
        state.deadline_miss = 3;
        state.estop_code = 4;
        state.estop = 1;
        state.n = 2;
        state.state[0] = MotorState {
            q: 0.5,
            dq: 0.0,
            tau: f32::NAN,
            current: 0.1,
            voltage: 12.0,
            temp: 31.0,
            fault: 0x20,
            status: 2,
            blocked: 0,
        };
        state.state[1].status = 4;
        state
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// The byte-exact vectors the Python client is tested against. If this
    /// changes, PROTOCOL_VERSION changes with it.
    #[test]
    fn the_command_vector_is_pinned() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        assert_eq!(out.len(), HEAD + CMD_HEADER + 2 * MOTOR_CMD + TAIL);
        assert_eq!(
            hex(&out),
            "444c525401000100\
             070000000200000008070605040302010200000000000000\
             010000000000003f000080bf000020410000003f0000803e\
             000000000000000000000000000000000000000000000000\
             1b4b94a0"
        );
    }

    #[test]
    fn the_state_vector_is_pinned() {
        let mut out = Vec::new();
        encode_state(&sample_state(), &mut out);
        assert_eq!(out.len(), HEAD + STATE_HEADER + 2 * MOTOR_STATE + TAIL);
        assert_eq!(
            hex(&out),
            "444c525401000200\
             090000000700000040e2010000000000887766554433221180841e00030000000400010002000000\
             0000003f000000000000c07fcdcccc3d000040410000f84120000200\
             0000c07f0000c07f0000c07f0000c07f0000c07f0000c07f00000400\
             fd45c257"
        );
    }

    #[test]
    fn a_frame_round_trips() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        assert_eq!(decode(&out), Ok(Frame::Cmd(sample_cmd())));

        encode_state(&sample_state(), &mut out);
        match decode(&out) {
            Ok(Frame::State(state)) => {
                assert_eq!(state.tick, 123_456);
                assert!(state.state[0].tau.is_nan());
                assert_eq!(state.state[1].status, 4);
                assert_eq!(state.n, 2);
            }
            other => panic!("{other:?}"),
        }

        encode_describe(&mut out);
        assert_eq!(decode(&out), Ok(Frame::Describe));
        encode_describe_reply("{\"robot\":\"x\"}", &mut out);
        assert_eq!(decode(&out), Ok(Frame::DescribeReply("{\"robot\":\"x\"}")));
    }

    #[test]
    fn a_damaged_frame_is_refused_with_the_reason() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        let last = out.len() - 1;
        out[last] ^= 1;
        assert_eq!(decode(&out), Err(FrameError::BadCrc));

        encode_cmd(&sample_cmd(), &mut out);
        out[0] = b'X';
        assert_eq!(decode(&out), Err(FrameError::BadMagic));

        // A version bump is a refusal, not a misread.
        encode_cmd(&sample_cmd(), &mut out);
        out[4] = 9;
        let body = out.len() - 4;
        let crc = crc32(&out[..body]).to_le_bytes();
        out[body..].copy_from_slice(&crc);
        assert_eq!(decode(&out), Err(FrameError::Version(9)));

        assert_eq!(decode(b"DLRT"), Err(FrameError::TooShort));
    }

    /// A count that the payload does not back is refused rather than read
    /// past the end.
    #[test]
    fn a_count_larger_than_the_payload_is_refused() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        out[HEAD + 16] = 5; // n
        let body = out.len() - 4;
        let crc = crc32(&out[..body]).to_le_bytes();
        out[body..].copy_from_slice(&crc);
        assert_eq!(decode(&out), Err(FrameError::Truncated));
    }
}
