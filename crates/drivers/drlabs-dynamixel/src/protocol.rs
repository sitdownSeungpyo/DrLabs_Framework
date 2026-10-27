//! Dynamixel Protocol 2.0 packets.
//!
//! A packet is a fixed header, an id, a length, an instruction, its parameters
//! and a CRC. Two details in here are easy to get subtly wrong and produce a
//! servo that answers most of the time: the byte stuffing, and where the CRC
//! is computed over.

/// The three-byte header every packet starts with, plus its reserved byte.
pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_REG_WRITE: u8 = 0x04;
pub const INST_ACTION: u8 = 0x05;
pub const INST_FACTORY_RESET: u8 = 0x06;
pub const INST_REBOOT: u8 = 0x08;
pub const INST_STATUS: u8 = 0x55;
pub const INST_SYNC_READ: u8 = 0x82;
pub const INST_SYNC_WRITE: u8 = 0x83;
pub const INST_BULK_READ: u8 = 0x92;
pub const INST_BULK_WRITE: u8 = 0x93;
/// Sync read answered as *one* packet for the whole chain. See
/// [`parse_fast_sync_read`].
pub const INST_FAST_SYNC_READ: u8 = 0x8A;
pub const INST_FAST_BULK_READ: u8 = 0x9A;

/// The broadcast id. A fast sync read's combined reply carries it in the id
/// field, since no single servo sent the packet.
pub const BROADCAST_ID: u8 = 0xFE;

/// Header(4) + id(1) + length(2).
pub const MIN_HEADER_SIZE: usize = 7;
pub const MAX_PACKET_SIZE: usize = 1024;
pub const READ_BUFFER_SIZE: usize = 1024;

/// Ids 253-255 are reserved.
pub const MAX_VALID_MOTOR_ID: u8 = 252;

/// CRC-16-IBM, polynomial 0x8005, computed a byte at a time.
pub fn update_crc(start: u16, data: &[u8]) -> u16 {
    let mut crc = start;
    for &byte in data {
        let index = (((crc >> 8) ^ byte as u16) & 0xFF) as usize;
        crc = (crc << 8) ^ CRC_TABLE[index];
    }
    crc
}

/// The nybble of the protocol that catches people out.
///
/// `FF FF FD` is the packet header, so those three bytes cannot be allowed to
/// appear inside a payload — a servo scanning for a header would find one in
/// the middle of somebody's position value. The sender inserts an extra `FD`,
/// and the receiver takes it back out.
pub fn stuff_params(params: &[u8]) -> Vec<u8> {
    let mut stuffed = Vec::with_capacity(params.len() + 2);
    let mut ff_count = 0;
    for &byte in params {
        stuffed.push(byte);
        if byte == 0xFF {
            ff_count += 1;
        } else {
            if ff_count >= 2 && byte == 0xFD {
                stuffed.push(0xFD);
            }
            ff_count = 0;
        }
    }
    stuffed
}

/// Removes the stuffing: `FF FF FD FD` becomes `FF FF FD`.
pub fn destuff_params(data: &[u8]) -> Vec<u8> {
    if data.len() < 4 {
        return data.to_vec();
    }
    let mut result = Vec::with_capacity(data.len());
    let mut i = 0;
    while i < data.len() {
        if i + 3 < data.len() && data[i..i + 4] == [0xFF, 0xFF, 0xFD, 0xFD] {
            result.extend_from_slice(&[0xFF, 0xFF, 0xFD]);
            i += 4;
        } else {
            result.push(data[i]);
            i += 1;
        }
    }
    result
}

/// Assembles one instruction packet.
pub fn build_packet(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut packet = Vec::with_capacity(params.len() + 12);
    write_packet(&mut packet, id, instruction, params);
    packet
}

/// Assembles one instruction packet into `out`, which is cleared first.
///
/// The form a real-time caller uses: with `out` kept from one cycle to the
/// next nothing here allocates once it has grown to the largest packet the
/// bus sends. The stuffing is done in place rather than through
/// [`stuff_params`], which would allocate a copy of the parameters.
pub fn write_packet(out: &mut Vec<u8>, id: u8, instruction: u8, params: &[u8]) {
    out.clear();
    out.extend_from_slice(&HEADER);
    out.push(id);
    // Length is patched once the stuffed parameters are in.
    out.push(0);
    out.push(0);
    out.push(instruction);

    let mut ff_count = 0;
    for &byte in params {
        out.push(byte);
        if byte == 0xFF {
            ff_count += 1;
        } else {
            if ff_count >= 2 && byte == 0xFD {
                out.push(0xFD);
            }
            ff_count = 0;
        }
    }

    // instruction + stuffed params + CRC.
    let length = out.len() - MIN_HEADER_SIZE + 2;
    out[5] = (length & 0xFF) as u8;
    out[6] = ((length >> 8) & 0xFF) as u8;

    // The CRC covers everything so far, header included.
    let crc = update_crc(0, out);
    out.push((crc & 0xFF) as u8);
    out.push(((crc >> 8) & 0xFF) as u8);
}

/// [`destuff_params`] into a buffer the caller keeps, which is cleared first.
pub fn destuff_into(out: &mut Vec<u8>, data: &[u8]) {
    out.clear();
    let mut i = 0;
    while i < data.len() {
        if i + 3 < data.len() && data[i..i + 4] == [0xFF, 0xFF, 0xFD, 0xFD] {
            out.extend_from_slice(&[0xFF, 0xFF, 0xFD]);
            i += 4;
        } else {
            out.push(data[i]);
            i += 1;
        }
    }
}

/// Validates a status packet and hands back its id, error byte and *raw*
/// (still stuffed) parameters, borrowing rather than copying. Pair with
/// [`destuff_into`] for the parameters as the servo meant them.
pub fn parse_packet_borrowed(packet: &[u8]) -> Result<(u8, u8, &[u8]), ParseError> {
    if packet.len() < 11 {
        return Err(ParseError::TooShort);
    }
    if packet[0..3] != HEADER[0..3] {
        return Err(ParseError::BadHeader);
    }
    let length = u16::from_le_bytes([packet[5], packet[6]]) as usize;
    if packet.len() != length + MIN_HEADER_SIZE {
        return Err(ParseError::LengthMismatch {
            expected: length + MIN_HEADER_SIZE,
            actual: packet.len(),
        });
    }
    let received = u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);
    let computed = update_crc(0, &packet[..packet.len() - 2]);
    if received != computed {
        return Err(ParseError::BadCrc { computed, received });
    }
    Ok((packet[4], packet[8], &packet[9..packet.len() - 2]))
}

/// What a status packet said.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusPacket {
    pub id: u8,
    /// The servo's own error code. Zero means it did as it was told.
    pub error: u8,
    pub params: Vec<u8>,
}

/// Why a packet could not be read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ParseError {
    #[error("packet too short")]
    TooShort,
    #[error("invalid header")]
    BadHeader,
    #[error("length mismatch: the header says {expected} bytes, the packet is {actual}")]
    LengthMismatch { expected: usize, actual: usize },
    #[error("CRC error: computed {computed:04X}, received {received:04X}")]
    BadCrc { computed: u16, received: u16 },
}

/// Validates and decodes a status packet.
pub fn parse_packet(packet: &[u8]) -> Result<StatusPacket, ParseError> {
    if packet.len() < 11 {
        return Err(ParseError::TooShort);
    }
    if packet[0..3] != HEADER[0..3] {
        return Err(ParseError::BadHeader);
    }

    let length = u16::from_le_bytes([packet[5], packet[6]]) as usize;
    if packet.len() != length + MIN_HEADER_SIZE {
        return Err(ParseError::LengthMismatch {
            expected: length + MIN_HEADER_SIZE,
            actual: packet.len(),
        });
    }

    let received = u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);
    let computed = update_crc(0, &packet[..packet.len() - 2]);
    if received != computed {
        return Err(ParseError::BadCrc { computed, received });
    }

    let params = if packet.len() > 11 {
        destuff_params(&packet[9..packet.len() - 2])
    } else {
        Vec::new()
    };

    Ok(StatusPacket {
        id: packet[4],
        error: packet[8],
        params,
    })
}

/// One servo's share of a fast sync read reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FastSegment {
    pub id: u8,
    pub error: u8,
    pub data: Vec<u8>,
}

/// Decodes the combined reply to a fast sync read (0x8A).
///
/// A plain sync read is answered once per servo, each reply a whole packet
/// with its own header, length and CRC — 11 bytes of overhead per servo and a
/// bus turnaround between every two. A fast sync read is answered as one
/// packet: the first servo starts it, and every servo after appends its
/// error byte, its id, its data and a CRC to the same stream. The id field
/// of that packet is the broadcast id, the length field counts the whole
/// stream, and the segment stride is `data_length + 4`.
///
/// Only the CRC that closes the packet is checked, over everything before
/// it. That is what the official SDK does (`updateCRC(0, rxpacket,
/// wait_length - 2)`), and what the protocol document means by "check CRC
/// only at the end". The intermediate CRCs are stepped over. Nothing in the
/// stream is byte-stuffed — the document says so, and the SDK receives these
/// replies with stuffing removal switched off.
pub fn parse_fast_sync_read(
    packet: &[u8],
    data_length: usize,
) -> Result<Vec<FastSegment>, ParseError> {
    let mut segments = Vec::new();
    for_each_fast_segment(packet, data_length, |id, error, data| {
        segments.push(FastSegment {
            id,
            error,
            data: data.to_vec(),
        })
    })?;
    Ok(segments)
}

/// [`parse_fast_sync_read`] without the allocation: validates the packet and
/// calls `sink` with each segment's id, error byte and data, borrowed.
pub fn for_each_fast_segment(
    packet: &[u8],
    data_length: usize,
    mut sink: impl FnMut(u8, u8, &[u8]),
) -> Result<(), ParseError> {
    if packet.len() < 11 {
        return Err(ParseError::TooShort);
    }
    if packet[0..3] != HEADER[0..3] {
        return Err(ParseError::BadHeader);
    }
    let length = u16::from_le_bytes([packet[5], packet[6]]) as usize;
    if packet.len() != length + MIN_HEADER_SIZE {
        return Err(ParseError::LengthMismatch {
            expected: length + MIN_HEADER_SIZE,
            actual: packet.len(),
        });
    }
    let received = u16::from_le_bytes([packet[packet.len() - 2], packet[packet.len() - 1]]);
    let computed = update_crc(0, &packet[..packet.len() - 2]);
    if received != computed {
        return Err(ParseError::BadCrc { computed, received });
    }

    // Instruction byte, then segments of error + id + data + CRC.
    let stride = data_length + 4;
    let body = length - 1;
    if body % stride != 0 {
        return Err(ParseError::LengthMismatch {
            expected: MIN_HEADER_SIZE + 1 + (body / stride + 1) * stride,
            actual: packet.len(),
        });
    }
    let mut index = 8;
    while index + stride <= packet.len() {
        sink(
            packet[index + 1],
            packet[index],
            &packet[index + 2..index + 2 + data_length],
        );
        index += stride;
    }
    Ok(())
}

/// Finds the start of a header, or `None`.
pub fn find_packet_start(data: &[u8]) -> Option<usize> {
    data.windows(3)
        .position(|window| window == [0xFF, 0xFF, 0xFD])
}

/// CRC-16-IBM table, polynomial 0x8005.
const CRC_TABLE: [u16; 256] = build_crc_table();

const fn build_crc_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A ping to id 1, byte for byte from the ROBOTIS protocol documentation.
    #[test]
    fn a_ping_matches_the_documented_bytes() {
        assert_eq!(
            build_packet(1, INST_PING, &[]),
            vec![0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E]
        );
    }

    /// Writing 1 to the torque enable register at address 64.
    #[test]
    fn a_register_write_carries_the_address_little_endian() {
        let packet = build_packet(1, INST_WRITE, &[64, 0, 1]);
        assert_eq!(&packet[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0x01]);
        assert_eq!(packet[7], INST_WRITE);
        assert_eq!(&packet[8..11], &[64, 0, 1]);
    }

    /// The stuffing rule, and the reason for it: a payload must never contain
    /// the sequence a receiver scans for.
    #[test]
    fn a_header_sequence_inside_a_payload_is_stuffed() {
        assert_eq!(
            stuff_params(&[0xFF, 0xFF, 0xFD, 0x01]),
            vec![0xFF, 0xFF, 0xFD, 0xFD, 0x01]
        );
        // Two FFs are needed, not one.
        assert_eq!(stuff_params(&[0xFF, 0xFD]), vec![0xFF, 0xFD]);
        // And an ordinary payload is untouched.
        assert_eq!(stuff_params(&[0x01, 0x02, 0x03]), vec![0x01, 0x02, 0x03]);
    }

    /// The in-place builder and the allocating one produce the same bytes,
    /// stuffing included.
    #[test]
    fn the_in_place_builder_matches_the_allocating_one() {
        let mut out = Vec::new();
        for (id, inst, params) in [
            (1u8, INST_PING, vec![]),
            (1, INST_WRITE, vec![64, 0, 1]),
            (
                0xFE,
                INST_SYNC_WRITE,
                vec![116, 0, 4, 0, 1, 0xFF, 0xFF, 0xFD, 0x00],
            ),
            (7, INST_READ, vec![0xFF, 0xFF, 0xFD, 0xFD]),
        ] {
            write_packet(&mut out, id, inst, &params);
            assert_eq!(
                out,
                build_packet(id, inst, &params),
                "{inst:02X} {params:02X?}"
            );
        }
    }

    #[test]
    fn a_borrowed_parse_agrees_with_the_owning_one() {
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x00, 0x00, 0x55, 0x80];
        let params = [0xFF, 0xFF, 0xFD, 0xFD, 0x07];
        packet.extend_from_slice(&params);
        let length = (packet.len() - MIN_HEADER_SIZE + 2) as u16;
        packet[5..7].copy_from_slice(&length.to_le_bytes());
        let crc = update_crc(0, &packet);
        packet.extend_from_slice(&crc.to_le_bytes());

        let owned = parse_packet(&packet).unwrap();
        let (id, error, raw) = parse_packet_borrowed(&packet).unwrap();
        let mut data = Vec::new();
        destuff_into(&mut data, raw);
        assert_eq!((id, error), (owned.id, owned.error));
        assert_eq!(data, owned.params);
        assert_eq!(data, vec![0xFF, 0xFF, 0xFD, 0x07]);
    }

    #[test]
    fn stuffing_round_trips() {
        for payload in [
            vec![0xFF, 0xFF, 0xFD, 0x00],
            vec![0x00, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD],
            vec![0x01, 0x02],
            vec![],
        ] {
            assert_eq!(destuff_params(&stuff_params(&payload)), payload);
        }
    }

    #[test]
    fn a_status_packet_decodes_to_its_error_and_parameters() {
        // A ping reply from id 1: model 1200, firmware 46.
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00];
        packet.extend_from_slice(&1200u16.to_le_bytes());
        packet.push(46);
        let crc = update_crc(0, &packet);
        packet.extend_from_slice(&crc.to_le_bytes());

        let status = parse_packet(&packet).expect("a valid packet");
        assert_eq!(status.id, 1);
        assert_eq!(status.error, 0);
        assert_eq!(
            u16::from_le_bytes([status.params[0], status.params[1]]),
            1200
        );
    }

    /// A corrupted packet is refused rather than half-believed. On a bus at
    /// 1 Mbaud this is not hypothetical.
    #[test]
    fn a_bad_crc_is_refused() {
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00];
        let crc = update_crc(0, &packet);
        packet.extend_from_slice(&crc.to_le_bytes());
        assert!(parse_packet(&packet).is_ok());

        let last = packet.len() - 1;
        packet[last] ^= 0xFF;
        assert!(matches!(
            parse_packet(&packet),
            Err(ParseError::BadCrc { .. })
        ));
    }

    #[test]
    fn a_truncated_packet_is_refused_rather_than_padded() {
        let packet = vec![0xFF, 0xFF, 0xFD, 0x00, 0x01];
        assert_eq!(parse_packet(&packet), Err(ParseError::TooShort));
    }

    /// Builds a fast sync read reply the way a chain does: one header, then
    /// each servo's error, id, data and a CRC over the stream so far, with the
    /// last CRC closing the packet.
    fn fast_reply(data_length: usize, servos: &[(u8, u8, Vec<u8>)]) -> Vec<u8> {
        let stride = data_length + 4;
        let length = 1 + servos.len() * stride;
        let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID];
        packet.extend_from_slice(&(length as u16).to_le_bytes());
        packet.push(INST_STATUS);
        for (id, error, data) in servos {
            packet.push(*error);
            packet.push(*id);
            packet.extend_from_slice(data);
            let crc = update_crc(0, &packet);
            packet.extend_from_slice(&crc.to_le_bytes());
        }
        packet
    }

    /// The layout from the protocol document's worked example: three servos,
    /// four bytes each, 32 bytes in all with a length field of 25.
    #[test]
    fn a_fast_sync_read_reply_decodes_every_segment() {
        let packet = fast_reply(
            4,
            &[
                (1, 0, vec![1, 0, 0, 0]),
                (2, 0, vec![2, 0, 0, 0]),
                (3, ALERT, vec![3, 0, 0, 0]),
            ],
        );
        assert_eq!(packet.len(), 32);
        assert_eq!(packet[5], 25);

        let segments = parse_fast_sync_read(&packet, 4).expect("a valid combined reply");
        assert_eq!(segments.len(), 3);
        assert_eq!(segments[0].id, 1);
        assert_eq!(segments[1].data, vec![2, 0, 0, 0]);
        assert_eq!(segments[2].error, ALERT);
    }

    const ALERT: u8 = 0x80;

    /// A length that is not a whole number of segments is a reply from a
    /// different read than the one asked, and is refused rather than sliced.
    #[test]
    fn a_fast_sync_read_reply_of_the_wrong_width_is_refused() {
        let packet = fast_reply(4, &[(1, 0, vec![1, 0, 0, 0])]);
        assert!(matches!(
            parse_fast_sync_read(&packet, 10),
            Err(ParseError::LengthMismatch { .. })
        ));
    }

    /// Only the closing CRC is checked, and it covers the whole stream —
    /// including the intermediate CRCs the servos wrote before it.
    #[test]
    fn a_fast_sync_read_reply_is_closed_by_its_last_crc() {
        let mut packet = fast_reply(2, &[(1, 0, vec![1, 0]), (2, 0, vec![2, 0])]);
        assert!(parse_fast_sync_read(&packet, 2).is_ok());

        // Corrupt the first servo's CRC: the stream is still accepted,
        // because the servo after it closed the packet over the real bytes.
        let first_crc = 8 + 2 + 2;
        packet[first_crc] ^= 0xFF;
        let mut reclosed = packet[..packet.len() - 2].to_vec();
        reclosed.extend_from_slice(&update_crc(0, &reclosed).to_le_bytes());
        assert!(parse_fast_sync_read(&reclosed, 2).is_ok());

        // Corrupt the last one and the whole reply is refused.
        let last = packet.len() - 1;
        packet[last] ^= 0xFF;
        assert!(matches!(
            parse_fast_sync_read(&packet, 2),
            Err(ParseError::BadCrc { .. })
        ));
    }

    #[test]
    fn a_header_is_found_wherever_it_starts() {
        assert_eq!(find_packet_start(&[0xFF, 0xFF, 0xFD, 0x00]), Some(0));
        assert_eq!(find_packet_start(&[0x12, 0x34, 0xFF, 0xFF, 0xFD]), Some(2));
        assert_eq!(find_packet_start(&[0x12, 0x34]), None);
        assert_eq!(find_packet_start(&[]), None);
    }
}
