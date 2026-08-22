//! Grouping consecutive registers into as few subframes as possible.
//!
//! A command touches a run of registers from 0x020 upward, and any of them may
//! be omitted or sent at a different width. The combiner walks that run and
//! says where a new subframe header belongs — which is what turns eleven
//! separate writes into two or three subframes, and is the difference between a
//! frame that fits in the 64-byte CAN-FD payload and one the adapter refuses.
//!
//! It deliberately does not hold the writer. Values interleave with headers on
//! the wire — header, its values, next header, its values — so the caller has
//! to be the one emitting, in that order. An earlier version borrowed the
//! writer and wrote every header first, which produced a frame with all the
//! framing at the front and all the data behind it. The reference byte tests
//! caught it immediately, which is what they are for.

use crate::multiplex::{Resolution, Writer};

/// The width a field is sent at, or [`Slot::Skip`] to leave the register alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    Skip,
    At(Resolution),
}

impl Slot {
    /// The width when transmitting, or `Skip` when not.
    ///
    /// This is where "send" and "the value" stay separate. A field carrying NaN
    /// may still be transmitted — that is how velocity mode says "no position
    /// target" — so presence is tracked here rather than inferred from the
    /// number.
    pub fn when(send: bool, resolution: Resolution) -> Self {
        if send {
            Slot::At(resolution)
        } else {
            Slot::Skip
        }
    }
}

/// One register's worth of instruction to the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Step {
    /// Emit this subframe header before the value, when present.
    pub header: Option<Header>,
    /// Whether this register carries a value at all.
    pub emit_value: bool,
    /// The width to emit it at.
    pub resolution: Option<Resolution>,
}

/// A subframe header waiting to be written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub resolution: Resolution,
    pub count: usize,
    pub register: u32,
}

/// Walks a run of registers, saying where the framing goes.
#[derive(Debug, Clone)]
pub struct Combiner {
    base_command: u8,
    start_register: u32,
    slots: Vec<Slot>,
    offset: usize,
    current: Option<Slot>,
    reply_size: usize,
}

impl Combiner {
    pub fn new(base_command: u8, start_register: u32, slots: Vec<Slot>) -> Self {
        Self {
            base_command,
            start_register,
            slots,
            offset: 0,
            current: None,
            reply_size: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.slots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// How many bytes the subframes walked so far occupy, headers included.
    pub fn reply_size(&self) -> usize {
        self.reply_size
    }

    pub fn base_command(&self) -> u8 {
        self.base_command
    }

    /// Advances one register, or `None` once the run is finished.
    pub fn advance(&mut self) -> Option<Step> {
        if self.offset >= self.slots.len() {
            return None;
        }
        let index = self.offset;
        self.offset += 1;
        let slot = self.slots[index];

        if self.current == Some(slot) {
            // Still inside the current run: no framing needed, and a value goes
            // out only if this run is not a run of skips.
            return Some(Step {
                header: None,
                emit_value: slot != Slot::Skip,
                resolution: match slot {
                    Slot::At(resolution) => Some(resolution),
                    Slot::Skip => None,
                },
            });
        }
        self.current = Some(slot);

        let resolution = match slot {
            Slot::Skip => {
                return Some(Step {
                    header: None,
                    emit_value: false,
                    resolution: None,
                })
            }
            Slot::At(resolution) => resolution,
        };

        // Look ahead to see how far the new width extends.
        let mut count = 1;
        let mut lookahead = index + 1;
        while lookahead < self.slots.len() && self.slots[lookahead] == slot {
            count += 1;
            lookahead += 1;
        }

        let header = Header {
            resolution,
            count,
            register: self.start_register + index as u32,
        };
        self.reply_size += header_len(self.base_command, &header) + count * resolution.width();

        Some(Step {
            header: Some(header),
            emit_value: true,
            resolution: Some(resolution),
        })
    }

    /// Writes every header and no values, which is what a read subframe is.
    pub fn write_reads(&mut self, writer: &mut Writer) {
        while let Some(step) = self.advance() {
            if let Some(header) = step.header {
                writer.push_header(
                    self.base_command,
                    header.resolution,
                    header.count,
                    header.register,
                );
            }
        }
    }
}

/// How many bytes a header occupies, without writing it.
fn header_len(base_command: u8, header: &Header) -> usize {
    let mut scratch = Writer::new();
    scratch.push_header(
        base_command,
        header.resolution,
        header.count,
        header.register,
    );
    scratch.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multiplex::WRITE_BASE;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02X}")).collect()
    }

    /// Emits headers only, as a read subframe does.
    fn headers(mut combiner: Combiner) -> String {
        let mut writer = Writer::new();
        combiner.write_reads(&mut writer);
        hex(writer.as_bytes())
    }

    #[test]
    fn consecutive_registers_of_one_width_share_a_header() {
        let combiner = Combiner::new(WRITE_BASE, 0x020, vec![Slot::At(Resolution::Int32); 3]);
        // One header for three registers, not three.
        assert_eq!(headers(combiner), "0B20");
    }

    #[test]
    fn a_change_of_width_starts_a_new_subframe() {
        let combiner = Combiner::new(
            WRITE_BASE,
            0x020,
            vec![Slot::At(Resolution::Int32), Slot::At(Resolution::Int16)],
        );
        assert_eq!(headers(combiner), concat!("0920", "0521"));
    }

    #[test]
    fn skipped_registers_produce_nothing_and_split_the_run() {
        let combiner = Combiner::new(
            WRITE_BASE,
            0x020,
            vec![
                Slot::At(Resolution::Int32),
                Slot::Skip,
                Slot::At(Resolution::Int32),
            ],
        );
        // Two single-register subframes, at 0x020 and 0x022.
        assert_eq!(headers(combiner), concat!("0920", "0922"));
    }

    #[test]
    fn runs_longer_than_three_spill_the_count() {
        let combiner = Combiner::new(WRITE_BASE, 0x020, vec![Slot::At(Resolution::Int8); 5]);
        assert_eq!(headers(combiner), "000520");
    }

    /// The ordering the wire needs, and the bug the reference tests found.
    #[test]
    fn a_header_is_reported_before_the_value_it_frames() {
        let mut combiner = Combiner::new(
            WRITE_BASE,
            0x020,
            vec![Slot::At(Resolution::Int32), Slot::At(Resolution::Int16)],
        );

        let first = combiner.advance().expect("a first register");
        assert!(first.header.is_some(), "the run starts with a header");
        assert!(first.emit_value);

        let second = combiner.advance().expect("a second register");
        assert!(
            second.header.is_some(),
            "the width changed, so a new header"
        );
        assert!(second.emit_value);

        assert!(combiner.advance().is_none());
    }

    #[test]
    fn a_continuing_run_reports_no_header() {
        let mut combiner = Combiner::new(WRITE_BASE, 0x020, vec![Slot::At(Resolution::Int32); 2]);
        assert!(combiner.advance().unwrap().header.is_some());
        let second = combiner.advance().unwrap();
        assert!(second.header.is_none(), "still inside the run");
        assert!(second.emit_value);
    }

    #[test]
    fn reply_size_counts_headers_and_payload() {
        let mut combiner = Combiner::new(WRITE_BASE, 0x020, vec![Slot::At(Resolution::Int32); 2]);
        while combiner.advance().is_some() {}
        // Two bytes of header plus two four-byte values.
        assert_eq!(combiner.reply_size(), 2 + 8);
    }
}
