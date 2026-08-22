//! The optional line checksum the fdcanusb firmware can demand.

/// The nybble-indexed table for CRC-8 with polynomial 0x97, the variant the
/// firmware uses.
const CRC8_TABLE: [u8; 16] = [
    0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb, 0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
];

/// Computes the checksum a nybble at a time.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc = CRC8_TABLE[(((crc >> 4) ^ (byte >> 4)) & 0x0f) as usize] ^ (crc << 4);
        crc = CRC8_TABLE[(((crc >> 4) ^ (byte & 0x0f)) & 0x0f) as usize] ^ (crc << 4);
    }
    crc
}

/// Returns `line` with ` *XX` appended.
///
/// The firmware computes the CRC over everything before the `*`, which includes
/// the separating space — so the space has to be part of the input, not only of
/// the output. Getting that wrong produces a checksum that is consistently one
/// space out and a link that never accepts a single command.
pub fn append_checksum(line: &[u8]) -> Vec<u8> {
    let mut with_space = line.to_vec();
    with_space.push(b' ');
    let crc = crc8(&with_space);
    with_space.extend_from_slice(format!("*{crc:02X}").as_bytes());
    with_space
}

/// What a line's trailing checksum turned out to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Checksum {
    /// No ` *XX` at the end.
    ///
    /// Whether that is acceptable depends on whether the adapter has demanded
    /// checksum mode, which is the caller's business rather than this
    /// function's.
    Absent,
    /// Present and correct.
    Valid,
    /// Present and wrong, so the line carries no usable information.
    Invalid,
}

/// Removes a trailing ` *XX` and verifies it.
pub fn strip_checksum(line: &[u8]) -> (&[u8], Checksum) {
    let star = match line.iter().rposition(|&byte| byte == b'*') {
        Some(index) if index + 3 == line.len() => index,
        _ => return (line, Checksum::Absent),
    };

    let claimed = match std::str::from_utf8(&line[star + 1..star + 3])
        .ok()
        .and_then(|text| u8::from_str_radix(text, 16).ok())
    {
        Some(value) => value,
        None => return (line, Checksum::Absent),
    };

    let content = &line[..star];
    let computed = crc8(content);
    let trimmed = trim_end_spaces(content);
    if claimed == computed {
        (trimmed, Checksum::Valid)
    } else {
        (trimmed, Checksum::Invalid)
    }
}

fn trim_end_spaces(bytes: &[u8]) -> &[u8] {
    let mut end = bytes.len();
    while end > 0 && bytes[end - 1] == b' ' {
        end -= 1;
    }
    &bytes[..end]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_checksum_survives_the_round_trip() {
        let line = b"can send 0001 0A0B0C";
        let with = append_checksum(line);
        let (stripped, checksum) = strip_checksum(&with);
        assert_eq!(checksum, Checksum::Valid);
        assert_eq!(stripped, line);
    }

    #[test]
    fn a_corrupted_checksum_is_rejected() {
        let mut with = append_checksum(b"rcv 0001 0A0B0C");
        // Flip the last hex digit of the checksum.
        let last = with.len() - 1;
        with[last] = if with[last] == b'0' { b'1' } else { b'0' };
        let (_, checksum) = strip_checksum(&with);
        assert_eq!(checksum, Checksum::Invalid);
    }

    #[test]
    fn a_line_without_a_checksum_is_reported_as_absent() {
        let (stripped, checksum) = strip_checksum(b"rcv 0001 0A0B0C");
        assert_eq!(checksum, Checksum::Absent);
        assert_eq!(stripped, b"rcv 0001 0A0B0C");
    }

    /// A `*` that is not in the checksum position is part of the line.
    #[test]
    fn a_star_elsewhere_is_not_mistaken_for_a_checksum() {
        let (stripped, checksum) = strip_checksum(b"ERR *unexpected");
        assert_eq!(checksum, Checksum::Absent);
        assert_eq!(stripped, b"ERR *unexpected");
    }

    /// The separating space is inside the CRC, which is the detail worth
    /// pinning: computing it over the line alone gives a different answer, and
    /// every command would be refused.
    #[test]
    fn the_separating_space_is_part_of_what_is_summed() {
        let line = b"can send 0001 00";
        let over_line_only = crc8(line);
        let over_line_and_space = crc8(b"can send 0001 00 ");
        assert_ne!(over_line_only, over_line_and_space);

        let appended = append_checksum(line);
        let text = String::from_utf8(appended).unwrap();
        assert!(
            text.ends_with(&format!("*{over_line_and_space:02X}")),
            "{text}"
        );
    }
}
