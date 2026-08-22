//! Grouping consecutive registers into as few subframes as possible.
//!
//! A command touches a run of registers from 0x020 upward, and any of them may
//! be omitted or sent at a different width. The combiner walks that run,
//! emitting a subframe header whenever the width changes and nothing at all
//! while it does not — which is what turns eleven separate writes into two or
//! three subframes, and is the difference between a frame that fits in the
//! 64-byte CAN-FD payload and one the adapter refuses.

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

/// Walks a run of registers, writing framing as the width changes.
pub struct Combiner<'a> {
    writer: &'a mut Writer,
    base_command: u8,
    start_register: u32,
    slots: Vec<Slot>,
    offset: usize,
    current: Option<Slot>,
    reply_size: usize,
}

impl<'a> Combiner<'a> {
    pub fn new(
        writer: &'a mut Writer,
        base_command: u8,
        start_register: u32,
        slots: Vec<Slot>,
    ) -> Self {
        Self {
            writer,
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

    /// How many bytes the subframes written so far occupy, headers included.
    pub fn reply_size(&self) -> usize {
        self.reply_size
    }

    /// Advances one register and reports whether the caller should now emit its
    /// value. Framing bytes are written here whenever the width changes.
    pub fn advance(&mut self) -> bool {
        if self.offset >= self.slots.len() {
            return false;
        }
        let index = self.offset;
        self.offset += 1;
        let slot = self.slots[index];

        if self.current == Some(slot) {
            // Still inside the current run: no framing needed, and the value
            // goes out only if this run is not a run of skips.
            return slot != Slot::Skip;
        }
        self.current = Some(slot);

        let resolution = match slot {
            Slot::Skip => return false,
            Slot::At(resolution) => resolution,
        };

        // Look ahead to see how far the new width extends.
        let mut count = 1;
        let mut lookahead = index + 1;
        while lookahead < self.slots.len() && self.slots[lookahead] == slot {
            count += 1;
            lookahead += 1;
        }

        let before = self.writer.len();
        self.writer.push_header(
            self.base_command,
            resolution,
            count,
            self.start_register + index as u32,
        );
        self.reply_size += self.writer.len() - before;
        self.reply_size += count * resolution.width();
        true
    }

    /// Runs to completion without emitting values, for sizing a read subframe.
    pub fn write_all(&mut self) {
        while self.advance() {}
        while self.offset < self.slots.len() {
            self.advance();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02X}")).collect()
    }

    #[test]
    fn consecutive_registers_of_one_width_share_a_header() {
        let mut writer = Writer::new();
        {
            let mut combiner = Combiner::new(
                &mut writer,
                crate::multiplex::WRITE_BASE,
                0x020,
                vec![
                    Slot::At(Resolution::Int32),
                    Slot::At(Resolution::Int32),
                    Slot::At(Resolution::Int32),
                ],
            );
            for _ in 0..3 {
                assert!(combiner.advance());
            }
        }
        // One header for three registers, not three.
        assert_eq!(hex(writer.as_bytes()), "0B20");
    }

    #[test]
    fn a_change_of_width_starts_a_new_subframe() {
        let mut writer = Writer::new();
        {
            let mut combiner = Combiner::new(
                &mut writer,
                crate::multiplex::WRITE_BASE,
                0x020,
                vec![Slot::At(Resolution::Int32), Slot::At(Resolution::Int16)],
            );
            assert!(combiner.advance());
            assert!(combiner.advance());
        }
        assert_eq!(hex(writer.as_bytes()), concat!("0920", "0521"));
    }

    #[test]
    fn skipped_registers_produce_nothing_and_split_the_run() {
        let mut writer = Writer::new();
        {
            let mut combiner = Combiner::new(
                &mut writer,
                crate::multiplex::WRITE_BASE,
                0x020,
                vec![
                    Slot::At(Resolution::Int32),
                    Slot::Skip,
                    Slot::At(Resolution::Int32),
                ],
            );
            assert!(combiner.advance(), "first register is written");
            assert!(!combiner.advance(), "the skipped one emits nothing");
            assert!(combiner.advance(), "and the run restarts after it");
        }
        // Two single-register subframes, at 0x020 and 0x022.
        assert_eq!(hex(writer.as_bytes()), concat!("0920", "0922"));
    }

    #[test]
    fn runs_longer_than_three_spill_the_count() {
        let mut writer = Writer::new();
        {
            let mut combiner = Combiner::new(
                &mut writer,
                crate::multiplex::WRITE_BASE,
                0x020,
                vec![Slot::At(Resolution::Int8); 5],
            );
            for _ in 0..5 {
                assert!(combiner.advance());
            }
        }
        assert_eq!(hex(writer.as_bytes()), "000520");
    }
}
