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
    packet.extend_from_slice(&HEADER);
    packet.push(id);

    let stuffed = stuff_params(params);
    // instruction + params + CRC.
    let length = 1 + stuffed.len() + 2;
    packet.push((length & 0xFF) as u8);
    packet.push(((length >> 8) & 0xFF) as u8);
    packet.push(instruction);
    packet.extend_from_slice(&stuffed);

    // The CRC covers everything so far, header included.
    let crc = update_crc(0, &packet);
    packet.push((crc & 0xFF) as u8);
    packet.push(((crc >> 8) & 0xFF) as u8);
    packet
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

    #[test]
    fn a_header_is_found_wherever_it_starts() {
        assert_eq!(find_packet_start(&[0xFF, 0xFF, 0xFD, 0x00]), Some(0));
        assert_eq!(find_packet_start(&[0x12, 0x34, 0xFF, 0xFF, 0xFD]), Some(2));
        assert_eq!(find_packet_start(&[0x12, 0x34]), None);
        assert_eq!(find_packet_start(&[]), None);
    }
}
