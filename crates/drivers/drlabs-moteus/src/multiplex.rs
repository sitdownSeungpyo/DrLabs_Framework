//! The mjlib multiplex register protocol.
//!
//! A moteus payload is a sequence of subframes, each beginning with a command
//! byte:
//!
//! | Byte | Meaning |
//! |------|---------|
//! | `0x00 \| type<<2 \| n` | write `n` registers of that type |
//! | `0x10 \| type<<2 \| n` | read `n` registers |
//! | `0x20 \| type<<2 \| n` | reply with `n` register values |
//! | `0x30` / `0x31` | write error / read error |
//! | `0x40`–`0x44` | tunnelled diagnostic stream |
//! | `0x50` | no-op, also used as frame padding |
//!
//! `type` is 0 for int8, 1 for int16, 2 for int32, 3 for float32. When `n` is
//! zero the count follows as a varuint, which is how runs longer than three
//! registers are expressed. The starting register number follows as a varuint;
//! subsequent values apply to consecutive registers.

use std::collections::BTreeMap;

/// Subframe command bytes.
pub const WRITE_BASE: u8 = 0x00;
pub const READ_BASE: u8 = 0x10;
pub const REPLY_BASE: u8 = 0x20;
pub const WRITE_ERROR: u8 = 0x30;
pub const READ_ERROR: u8 = 0x31;
pub const NOP: u8 = 0x50;

/// The width a register value is sent at.
///
/// Narrower fields leave more of the frame for the query that rides beside the
/// command, which is the whole reason the choice exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Resolution {
    Int8,
    Int16,
    Int32,
    F32,
}

impl Resolution {
    pub fn type_bits(self) -> u8 {
        match self {
            Resolution::Int8 => 0,
            Resolution::Int16 => 1,
            Resolution::Int32 => 2,
            Resolution::F32 => 3,
        }
    }

    pub fn from_type_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Resolution::Int8,
            1 => Resolution::Int16,
            2 => Resolution::Int32,
            _ => Resolution::F32,
        }
    }

    pub fn width(self) -> usize {
        match self {
            Resolution::Int8 => 1,
            Resolution::Int16 => 2,
            _ => 4,
        }
    }

    /// The largest magnitude an integer resolution can carry.
    ///
    /// One short of the negative end, because the negative-most value is the
    /// protocol's NaN sentinel — saturating onto it would turn a large negative
    /// number into "no value".
    fn max_magnitude(self) -> f64 {
        match self {
            Resolution::Int8 => 127.0,
            Resolution::Int16 => 32767.0,
            Resolution::Int32 => 2147483647.0,
            Resolution::F32 => f64::INFINITY,
        }
    }

    /// The sentinel this resolution uses for "no value".
    fn nan_sentinel(self) -> f64 {
        match self {
            Resolution::Int8 => -128.0,
            Resolution::Int16 => -32768.0,
            Resolution::Int32 => -2147483648.0,
            Resolution::F32 => f64::NAN,
        }
    }
}

/// Converts a physical value into the raw number the wire carries.
///
/// Two details matter more than they look.
///
/// **Truncation, not rounding.** `0.25 / 0.00001` is 24999.999… in binary
/// floating point, and the protocol truncates towards zero, so the wire carries
/// 24999. Rounding instead would put nearly every position command one count
/// away from what the reference implementation produces.
///
/// **The NaN sentinel.** A non-finite input becomes the negative-most value of
/// the width, which is how the protocol says "no value". Saturation therefore
/// clamps to `±max`, one short of the negative end, so a large negative number
/// cannot be mistaken for it.
pub fn saturate(value: f64, resolution: Resolution, scale: f64) -> f64 {
    if !value.is_finite() {
        return resolution.nan_sentinel();
    }

    let scaled = value / scale;
    if resolution == Resolution::F32 {
        return scaled;
    }

    let max = resolution.max_magnitude();
    if scaled < -max {
        return -max;
    }
    if scaled > max {
        return max;
    }
    scaled.trunc()
}

/// Turns the sentinel back into a NaN on the way in.
pub fn nanify(value: f64, resolution: Resolution) -> f64 {
    if resolution != Resolution::F32 && value == resolution.nan_sentinel() {
        return f64::NAN;
    }
    value
}

/// Builds a payload.
#[derive(Debug, Default, Clone)]
pub struct Writer {
    buffer: Vec<u8>,
}

impl Writer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn into_bytes(self) -> Vec<u8> {
        self.buffer
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buffer
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    pub fn push_u8(&mut self, value: u8) {
        self.buffer.push(value);
    }

    pub fn extend(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
    }

    /// Seven bits per byte, least significant first, with the top bit set on
    /// every byte but the last.
    pub fn push_varuint(&mut self, mut value: u32) {
        loop {
            let mut byte = (value & 0x7f) as u8;
            value >>= 7;
            if value != 0 {
                byte |= 0x80;
            }
            self.buffer.push(byte);
            if value == 0 {
                return;
            }
        }
    }

    /// Emits a value already converted to the raw units of `resolution`.
    pub fn push_value(&mut self, value: f64, resolution: Resolution) {
        match resolution {
            Resolution::Int8 => self.buffer.push(value as i8 as u8),
            Resolution::Int16 => self.buffer.extend_from_slice(&(value as i16).to_le_bytes()),
            Resolution::Int32 => self.buffer.extend_from_slice(&(value as i32).to_le_bytes()),
            Resolution::F32 => self.buffer.extend_from_slice(&(value as f32).to_le_bytes()),
        }
    }

    /// Emits a physical value through the per-resolution scale factors.
    pub fn push_mapped(&mut self, value: f64, scales: &Scales, resolution: Resolution) {
        let scale = scales.for_resolution(resolution);
        self.push_value(saturate(value, resolution, scale), resolution);
    }

    /// Emits a subframe header: `base | type<<2 | count`, with the count
    /// spilling into a varuint once it will not fit in two bits.
    pub fn push_header(&mut self, base: u8, resolution: Resolution, count: usize, register: u32) {
        let type_bits = resolution.type_bits() << 2;
        if count <= 3 {
            self.buffer.push(base | type_bits | count as u8);
        } else {
            self.buffer.push(base | type_bits);
            self.push_varuint(count as u32);
        }
        self.push_varuint(register);
    }
}

/// The per-resolution scale factors for one register family.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scales {
    pub int8: f64,
    pub int16: f64,
    pub int32: f64,
}

impl Scales {
    pub const fn new(int8: f64, int16: f64, int32: f64) -> Self {
        Self { int8, int16, int32 }
    }

    pub fn for_resolution(&self, resolution: Resolution) -> f64 {
        match resolution {
            Resolution::Int8 => self.int8,
            Resolution::Int16 => self.int16,
            Resolution::Int32 => self.int32,
            Resolution::F32 => 1.0,
        }
    }
}

/// Reads a payload.
#[derive(Debug, Clone)]
pub struct Reader<'a> {
    data: &'a [u8],
    offset: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ReadError {
    #[error("the frame ended mid-value")]
    Truncated,
}

impl<'a> Reader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }

    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.offset)
    }

    pub fn offset(&self) -> usize {
        self.offset
    }

    pub fn read_u8(&mut self) -> Result<u8, ReadError> {
        let byte = *self.data.get(self.offset).ok_or(ReadError::Truncated)?;
        self.offset += 1;
        Ok(byte)
    }

    pub fn read_varuint(&mut self) -> Result<u32, ReadError> {
        let mut value: u32 = 0;
        let mut shift = 0;
        loop {
            let byte = self.read_u8()?;
            value |= u32::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
            shift += 7;
            if shift >= 32 {
                return Err(ReadError::Truncated);
            }
        }
    }

    pub fn read_value(&mut self, resolution: Resolution) -> Result<f64, ReadError> {
        let width = resolution.width();
        if self.remaining() < width {
            return Err(ReadError::Truncated);
        }
        let bytes = &self.data[self.offset..self.offset + width];
        self.offset += width;
        Ok(match resolution {
            Resolution::Int8 => f64::from(bytes[0] as i8),
            Resolution::Int16 => f64::from(i16::from_le_bytes([bytes[0], bytes[1]])),
            Resolution::Int32 => {
                f64::from(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            Resolution::F32 => {
                f64::from(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
        })
    }
}

/// One register's value as it came off the wire, with the width it arrived at.
///
/// The width is kept because the scale depends on it: the same register means
/// different things at int8 and int32.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawValue {
    pub resolution: Resolution,
    pub raw: f64,
}

/// What a controller said back.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Reply {
    pub values: BTreeMap<u32, RawValue>,
    /// A register the controller refused, and the error it gave.
    pub errors: Vec<(u32, u32)>,
    /// Whether the frame ran out before it was fully decoded.
    pub truncated: bool,
}

/// Decodes the reply subframes of a received frame.
///
/// Read and write subframes are skipped: a reply frame should not contain them,
/// but a bus shared with other traffic can deliver anything. A truncated frame
/// yields whatever was decoded before the break, flagged — half a reply is
/// still worth more than none, and pretending it was whole is what turns a
/// dropped byte into a wrong reading.
pub fn parse_reply(data: &[u8]) -> Reply {
    let mut reply = Reply::default();
    let mut reader = Reader::new(data);

    while reader.remaining() > 0 {
        let command = match reader.read_u8() {
            Ok(byte) => byte,
            Err(_) => break,
        };
        if command == NOP {
            continue;
        }

        let kind = command & 0xf0;
        if kind == WRITE_ERROR
            || kind == READ_ERROR
            || command == WRITE_ERROR
            || command == READ_ERROR
        {
            match (reader.read_varuint(), reader.read_varuint()) {
                (Ok(register), Ok(code)) => reply.errors.push((register, code)),
                _ => {
                    reply.truncated = true;
                    break;
                }
            }
            continue;
        }

        let resolution = Resolution::from_type_bits(command >> 2);
        let mut count = usize::from(command & 0x03);
        if count == 0 {
            match reader.read_varuint() {
                Ok(value) => count = value as usize,
                Err(_) => {
                    reply.truncated = true;
                    break;
                }
            }
        }
        let register = match reader.read_varuint() {
            Ok(value) => value,
            Err(_) => {
                reply.truncated = true;
                break;
            }
        };

        if kind != REPLY_BASE {
            // A read or write subframe: skip its payload and carry on.
            let skip = match kind {
                READ_BASE => 0,
                _ => count * resolution.width(),
            };
            if reader.remaining() < skip {
                reply.truncated = true;
                break;
            }
            for _ in 0..skip {
                let _ = reader.read_u8();
            }
            continue;
        }

        for index in 0..count {
            match reader.read_value(resolution) {
                Ok(raw) => {
                    reply
                        .values
                        .insert(register + index as u32, RawValue { resolution, raw });
                }
                Err(_) => {
                    reply.truncated = true;
                    return reply;
                }
            }
        }
    }

    reply
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02X}")).collect()
    }

    #[test]
    fn varuint_matches_the_seven_bit_encoding() {
        let cases: &[(u32, &str)] = &[
            (0, "00"),
            (1, "01"),
            (127, "7F"),
            (128, "8001"),
            (300, "AC02"),
            (16384, "808001"),
        ];
        for (value, want) in cases {
            let mut writer = Writer::new();
            writer.push_varuint(*value);
            assert_eq!(hex(writer.as_bytes()), *want, "varuint({value})");
        }
    }

    #[test]
    fn varuint_round_trips() {
        for value in [0u32, 1, 5, 127, 128, 255, 300, 4096, 16383, 16384, 1 << 28] {
            let mut writer = Writer::new();
            writer.push_varuint(value);
            let bytes = writer.into_bytes();
            let mut reader = Reader::new(&bytes);
            assert_eq!(reader.read_varuint(), Ok(value));
            assert_eq!(reader.remaining(), 0);
        }
    }

    /// The detail that decides whether commands match the reference.
    #[test]
    fn saturate_truncates_towards_zero() {
        // 0.25 / 0.00001 is 24999.999... in binary floating point.
        assert_eq!(saturate(0.25, Resolution::Int32, 0.00001), 24999.0);
        assert_eq!(saturate(-0.25, Resolution::Int32, 0.00001), -24999.0);
    }

    #[test]
    fn saturate_clamps_rather_than_wrapping() {
        assert_eq!(saturate(1e9, Resolution::Int8, 1.0), 127.0);
        assert_eq!(saturate(-1e9, Resolution::Int8, 1.0), -127.0);
        assert_eq!(saturate(1e9, Resolution::Int16, 1.0), 32767.0);
        assert_eq!(saturate(-1e9, Resolution::Int16, 1.0), -32767.0);
    }

    /// Saturation stops one short of the negative end, because that value is
    /// the sentinel for "no value".
    #[test]
    fn saturation_never_lands_on_the_nan_sentinel() {
        for resolution in [Resolution::Int8, Resolution::Int16, Resolution::Int32] {
            let saturated = saturate(-1e12, resolution, 1.0);
            assert_ne!(
                saturated,
                resolution.nan_sentinel(),
                "{resolution:?} saturated onto its own sentinel"
            );
            assert!(!nanify(saturated, resolution).is_nan());
        }
    }

    #[test]
    fn a_non_finite_value_becomes_the_sentinel() {
        assert_eq!(saturate(f64::NAN, Resolution::Int32, 0.001), -2147483648.0);
        assert_eq!(saturate(f64::INFINITY, Resolution::Int16, 0.1), -32768.0);
        assert_eq!(saturate(f64::NEG_INFINITY, Resolution::Int8, 1.0), -128.0);
    }

    #[test]
    fn nanify_recognises_the_sentinel() {
        assert!(nanify(-128.0, Resolution::Int8).is_nan());
        assert!(nanify(-32768.0, Resolution::Int16).is_nan());
        assert!(nanify(-2147483648.0, Resolution::Int32).is_nan());

        // And leaves everything else alone, including the value one above it.
        assert_eq!(nanify(-127.0, Resolution::Int8), -127.0);
        assert_eq!(nanify(0.0, Resolution::Int32), 0.0);
    }

    #[test]
    fn short_runs_use_the_two_bit_count_and_long_runs_spill() {
        let mut short = Writer::new();
        short.push_header(WRITE_BASE, Resolution::Int32, 2, 0x020);
        assert_eq!(hex(short.as_bytes()), "0A20");

        let mut long = Writer::new();
        long.push_header(WRITE_BASE, Resolution::Int32, 5, 0x020);
        assert_eq!(hex(long.as_bytes()), "080520");
    }

    #[test]
    fn a_reply_decodes_to_registers() {
        // Reply, int16, 2 registers from 0x000: mode 10, then 0x1234.
        let frame = [0x26u8, 0x00, 0x0A, 0x00, 0x34, 0x12];
        let reply = parse_reply(&frame);
        assert!(!reply.truncated);
        assert_eq!(reply.values[&0x000].raw, 10.0);
        assert_eq!(reply.values[&0x001].raw, 0x1234 as f64);
    }

    #[test]
    fn padding_is_skipped() {
        let frame = [NOP, NOP, 0x25u8, 0x00, 0x0A, 0x00, NOP];
        let reply = parse_reply(&frame);
        assert!(!reply.truncated);
        assert_eq!(reply.values.len(), 1);
        assert_eq!(reply.values[&0x000].raw, 10.0);
    }

    /// A truncated frame gives back what it had, and says it was cut off.
    #[test]
    fn truncation_returns_partial_data_and_admits_it() {
        // Two int16 registers promised, only one delivered.
        let frame = [0x26u8, 0x00, 0x0A, 0x00];
        let reply = parse_reply(&frame);
        assert!(reply.truncated);
        assert_eq!(reply.values.len(), 1);
        assert_eq!(reply.values[&0x000].raw, 10.0);
    }
}
