//! The records as ROS 2 puts them on a DDS wire: XCDR1, little-endian.
//!
//! A `drlabs_msgs/LowCmd` message serialised by `rmw_cyclonedds` is a 4-byte
//! encapsulation header (`00 01 00 00`: CDR, little-endian, no options)
//! followed by the fields in declaration order, each primitive aligned to
//! its own size *counted from the start of the body*, fixed arrays laid out
//! element after element with no length, and a `string` as a u32 length
//! that includes the terminating NUL. There is no padding anywhere a `.msg`
//! file does not put a field, which is why the in-memory records' `reserved`
//! bytes do not travel.
//!
//! Written by hand rather than through an IDL compiler because the records
//! are five fixed shapes, and because the encoder is then something the
//! tests can pin byte for byte against the Eclipse `cyclonedds` Python
//! package — the same discipline the moteus frames get against the official
//! `moteus` package.

use drlabs_core::lowlevel::{LowCmd, LowState, MotorCmd, MotorState, MAX_AXES};

/// CDR little-endian, no options: what `rmw_cyclonedds` writes.
pub const ENCAPSULATION: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// The ROS 2 names the DDS wire carries. A ROS 2 node with
/// `rmw_cyclonedds` on the same domain matches these without a bridge.
pub mod names {
    pub const TYPE_LOWCMD: &str = "drlabs_msgs::msg::dds_::LowCmd_";
    pub const TYPE_LOWSTATE: &str = "drlabs_msgs::msg::dds_::LowState_";
    pub const TYPE_DESCRIBE: &str = "drlabs_msgs::msg::dds_::Describe_";

    /// `rt/` is the DDS prefix ROS 2 puts on every topic; the rest is the
    /// topic a node sees as `/<prefix>/lowcmd`.
    pub fn topic(prefix: &str, leaf: &str) -> String {
        format!("rt/{prefix}/{leaf}")
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CdrError {
    /// The 4-byte header is not CDR little-endian.
    Encapsulation([u8; 4]),
    /// The body ended before the field at this offset.
    Short { needed: usize, have: usize },
    /// More entries than a record holds.
    TooMany(u16),
    /// A string that is not UTF-8 or not NUL-terminated.
    BadString,
}

impl std::fmt::Display for CdrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CdrError::Encapsulation(head) => write!(f, "not CDR_LE: header {head:02x?}"),
            CdrError::Short { needed, have } => {
                write!(f, "truncated: needed {needed} bytes, have {have}")
            }
            CdrError::TooMany(n) => write!(f, "{n} entries, but a record holds {MAX_AXES}"),
            CdrError::BadString => write!(f, "a string that is not UTF-8 or not NUL-terminated"),
        }
    }
}

impl std::error::Error for CdrError {}

/// Appends to a body, aligning each primitive to its size from the body's
/// first byte. The encapsulation header is not part of the body: ROS 2
/// aligns after it, not across it.
struct Writer<'a> {
    out: &'a mut Vec<u8>,
    base: usize,
}

impl<'a> Writer<'a> {
    fn begin(out: &'a mut Vec<u8>) -> Self {
        out.clear();
        out.extend_from_slice(&ENCAPSULATION);
        let base = out.len();
        Self { out, base }
    }

    fn align(&mut self, to: usize) {
        let offset = self.out.len() - self.base;
        let pad = (to - offset % to) % to;
        let len = self.out.len() + pad;
        self.out.resize(len, 0);
    }

    fn u8(&mut self, v: u8) {
        self.out.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.align(2);
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.align(4);
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.align(8);
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.align(4);
        self.out.extend_from_slice(&v.to_le_bytes());
    }
    fn string(&mut self, s: &str) {
        self.u32(s.len() as u32 + 1);
        self.out.extend_from_slice(s.as_bytes());
        self.out.push(0);
    }
}

struct Reader<'a> {
    body: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn begin(data: &'a [u8]) -> Result<Self, CdrError> {
        if data.len() < 4 {
            return Err(CdrError::Short {
                needed: 4,
                have: data.len(),
            });
        }
        let mut head = [0u8; 4];
        head.copy_from_slice(&data[..4]);
        if head != ENCAPSULATION {
            return Err(CdrError::Encapsulation(head));
        }
        Ok(Self {
            body: &data[4..],
            at: 0,
        })
    }

    fn take(&mut self, size: usize) -> Result<&'a [u8], CdrError> {
        self.at += (size - self.at % size) % size;
        let end = self.at + size;
        if end > self.body.len() {
            return Err(CdrError::Short {
                needed: end,
                have: self.body.len(),
            });
        }
        let slice = &self.body[self.at..end];
        self.at = end;
        Ok(slice)
    }

    fn u8(&mut self) -> Result<u8, CdrError> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, CdrError> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, CdrError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, CdrError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f32(&mut self) -> Result<f32, CdrError> {
        Ok(f32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn string(&mut self) -> Result<&'a str, CdrError> {
        let len = self.u32()? as usize;
        if len == 0 {
            return Err(CdrError::BadString);
        }
        // Strings are byte-aligned after their length.
        let end = self.at + len;
        if end > self.body.len() {
            return Err(CdrError::Short {
                needed: end,
                have: self.body.len(),
            });
        }
        let bytes = &self.body[self.at..end];
        self.at = end;
        if bytes[len - 1] != 0 {
            return Err(CdrError::BadString);
        }
        std::str::from_utf8(&bytes[..len - 1]).map_err(|_| CdrError::BadString)
    }
}

fn write_motor_cmd(w: &mut Writer<'_>, m: &MotorCmd) {
    w.u8(m.mode);
    w.f32(m.q);
    w.f32(m.dq);
    w.f32(m.kp);
    w.f32(m.kd);
    w.f32(m.tau);
}

fn read_motor_cmd(r: &mut Reader<'_>) -> Result<MotorCmd, CdrError> {
    Ok(MotorCmd {
        mode: r.u8()?,
        reserved: [0; 3],
        q: r.f32()?,
        dq: r.f32()?,
        kp: r.f32()?,
        kd: r.f32()?,
        tau: r.f32()?,
    })
}

fn write_motor_state(w: &mut Writer<'_>, m: &MotorState) {
    w.f32(m.q);
    w.f32(m.dq);
    w.f32(m.tau);
    w.f32(m.current);
    w.f32(m.voltage);
    w.f32(m.temp);
    w.u16(m.fault);
    w.u8(m.status);
    w.u8(m.blocked);
}

fn read_motor_state(r: &mut Reader<'_>) -> Result<MotorState, CdrError> {
    Ok(MotorState {
        q: r.f32()?,
        dq: r.f32()?,
        tau: r.f32()?,
        current: r.f32()?,
        voltage: r.f32()?,
        temp: r.f32()?,
        fault: r.u16()?,
        status: r.u8()?,
        blocked: r.u8()?,
    })
}

/// `drlabs_msgs/LowCmd`: `uint32 seq, uint32 flags, uint64 t_ns, uint16 n,
/// MotorCmd[64] cmd`. The whole array travels — the message is fixed-size —
/// so a 64-entry command is 4 + 1556 bytes on the wire.
pub fn encode_cmd(cmd: &LowCmd, out: &mut Vec<u8>) {
    let mut w = Writer::begin(out);
    w.u32(cmd.seq);
    w.u32(cmd.flags);
    w.u64(cmd.t_ns);
    w.u16(cmd.n);
    for entry in &cmd.cmd {
        write_motor_cmd(&mut w, entry);
    }
}

pub fn decode_cmd(data: &[u8]) -> Result<LowCmd, CdrError> {
    let mut r = Reader::begin(data)?;
    let mut cmd = LowCmd::EMPTY;
    cmd.seq = r.u32()?;
    cmd.flags = r.u32()?;
    cmd.t_ns = r.u64()?;
    cmd.n = r.u16()?;
    if cmd.n as usize > MAX_AXES {
        return Err(CdrError::TooMany(cmd.n));
    }
    for entry in cmd.cmd.iter_mut() {
        *entry = read_motor_cmd(&mut r)?;
    }
    Ok(cmd)
}

/// `drlabs_msgs/LowState`: `uint32 seq, uint32 cmd_seq, uint64 tick,
/// uint64 t_ns, uint32 period_ns, uint32 deadline_miss, uint16 estop_code,
/// uint8 estop, uint16 n, MotorState[64] state`.
pub fn encode_state(state: &LowState, out: &mut Vec<u8>) {
    let mut w = Writer::begin(out);
    w.u32(state.seq);
    w.u32(state.cmd_seq);
    w.u64(state.tick);
    w.u64(state.t_ns);
    w.u32(state.period_ns);
    w.u32(state.deadline_miss);
    w.u16(state.estop_code);
    w.u8(state.estop);
    w.u16(state.n);
    for entry in &state.state {
        write_motor_state(&mut w, entry);
    }
}

pub fn decode_state(data: &[u8]) -> Result<LowState, CdrError> {
    let mut r = Reader::begin(data)?;
    let mut state = LowState::EMPTY;
    state.seq = r.u32()?;
    state.cmd_seq = r.u32()?;
    state.tick = r.u64()?;
    state.t_ns = r.u64()?;
    state.period_ns = r.u32()?;
    state.deadline_miss = r.u32()?;
    state.estop_code = r.u16()?;
    state.estop = r.u8()?;
    state.n = r.u16()?;
    if state.n as usize > MAX_AXES {
        return Err(CdrError::TooMany(state.n));
    }
    for entry in state.state.iter_mut() {
        *entry = read_motor_state(&mut r)?;
    }
    Ok(state)
}

/// `drlabs_msgs/Describe`: `string json`.
pub fn encode_describe(json: &str, out: &mut Vec<u8>) {
    let mut w = Writer::begin(out);
    w.string(json);
}

pub fn decode_describe(data: &[u8]) -> Result<String, CdrError> {
    let mut r = Reader::begin(data)?;
    Ok(r.string()?.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use drlabs_core::lowlevel::mode;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    fn sample_cmd() -> LowCmd {
        let mut cmd = LowCmd::EMPTY;
        cmd.seq = 7;
        cmd.flags = 2;
        cmd.t_ns = 0x0102030405060708;
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
        state.t_ns = 0x1122334455667788;
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
        state.state[1] = MotorState {
            q: f32::NAN,
            dq: f32::NAN,
            tau: f32::NAN,
            current: f32::NAN,
            voltage: f32::NAN,
            temp: f32::NAN,
            fault: 0,
            status: 4,
            blocked: 0,
        };
        state
    }

    /// The first bytes of a command as the Eclipse `cyclonedds` Python
    /// package serialises the same `drlabs_msgs::msg::dds_::LowCmd_`
    /// (tools/drlabs_client/tests/test_cdr.py pins the whole buffer).
    #[test]
    fn a_command_opens_with_the_ros2_header_and_body_relative_alignment() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        // header · seq · flags · t_ns · n · cmd[0].mode · pad · q
        let head = "00010000" // CDR_LE
            .to_string()
            + "07000000"
            + "02000000"
            + "0807060504030201"
            + "0200"
            + "01"
            + "00" // one byte of padding: q is 4-aligned from the body
            + "0000003f";
        assert_eq!(&hex(&out)[..head.len()], head);
        // 4 header + 18 fixed + 64 entries, the first 1+1+20 and the rest
        // 24 each (mode lands on a 4-aligned byte, so 3 bytes of padding).
        assert_eq!(out.len(), 4 + 18 + 22 + 63 * 24);
    }

    #[test]
    fn a_state_opens_with_its_header_fields_packed_as_ros2_packs_them() {
        let mut out = Vec::new();
        encode_state(&sample_state(), &mut out);
        let head = "00010000".to_string()
            + "09000000"
            + "07000000"
            + "40e2010000000000"
            + "8877665544332211"
            + "80841e00"
            + "03000000"
            + "0400"
            + "01"
            + "00" // n is 2-aligned: one pad byte after estop
            + "0200"
            + "0000" // state[0].q is 4-aligned: two pad bytes before it
            + "0000003f";
        assert_eq!(&hex(&out)[..head.len()], head, "{}", hex(&out));
    }

    #[test]
    fn a_command_round_trips() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        assert_eq!(decode_cmd(&out).unwrap(), sample_cmd());
    }

    #[test]
    fn a_state_round_trips_with_its_nans() {
        let mut out = Vec::new();
        encode_state(&sample_state(), &mut out);
        let back = decode_state(&out).unwrap();
        assert_eq!(back.n, 2);
        assert!(back.state[0].tau.is_nan());
        assert_eq!(back.state[0].fault, 0x20);
        assert!(back.state[1].q.is_nan());
        assert_eq!(back.state[1].status, 4);
        assert_eq!(back.tick, 123_456);
    }

    #[test]
    fn a_description_is_a_counted_nul_terminated_string() {
        let mut out = Vec::new();
        encode_describe("{\"a\":1}", &mut out);
        assert_eq!(
            hex(&out),
            "00010000".to_string() + "08000000" + "7b2261223a317d" + "00"
        );
        assert_eq!(decode_describe(&out).unwrap(), "{\"a\":1}");
    }

    #[test]
    fn the_wrong_encapsulation_is_refused() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        out[1] = 0x00; // big-endian CDR
        assert_eq!(decode_cmd(&out), Err(CdrError::Encapsulation([0, 0, 0, 0])));
    }

    /// FNV-1a 64 over the whole buffer, so a drift anywhere in 1.5 kB shows.
    fn fnv1a64(data: &[u8]) -> u64 {
        data.iter().fold(0xCBF2_9CE4_8422_2325u64, |h, b| {
            (h ^ *b as u64).wrapping_mul(0x0000_0100_0000_01B3)
        })
    }

    /// The same two digests are asserted by
    /// tools/drlabs_client/tests/test_cdr.py over what the Eclipse
    /// `cyclonedds` Python package serialises for the same samples: the
    /// proof that a ROS 2 node reads what this crate writes.
    #[test]
    fn whole_buffer_digests_are_pinned_for_python() {
        let mut out = Vec::new();
        encode_cmd(&sample_cmd(), &mut out);
        assert_eq!(
            fnv1a64(&out),
            RUST_CMD_FNV,
            "cmd fnv1a64 = {:#018x}",
            fnv1a64(&out)
        );
        encode_state(&sample_state(), &mut out);
        assert_eq!(
            fnv1a64(&out),
            RUST_STATE_FNV,
            "state fnv1a64 = {:#018x}",
            fnv1a64(&out)
        );
    }

    const RUST_CMD_FNV: u64 = 0x452a_edbf_1163_b77c;
    const RUST_STATE_FNV: u64 = 0x6ee0_6ede_b2fa_4fd9;

    #[test]
    fn a_truncated_record_is_refused_not_zero_filled() {
        let mut out = Vec::new();
        encode_state(&sample_state(), &mut out);
        out.truncate(out.len() - 1);
        assert!(matches!(decode_state(&out), Err(CdrError::Short { .. })));
    }
}
