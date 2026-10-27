//! Lock-free hand-off of `Copy` records between threads.
//!
//! A triple buffer: three slots, one owned by the writer, one by the reader,
//! one in the middle. The writer fills its slot and swaps it into the
//! middle; the reader swaps the middle into its own slot when it wants the
//! latest. Each swap is one atomic exchange, so neither side ever waits for
//! the other, and the reader always gets a whole record — never half of the
//! previous one and half of the next, which is what a plain shared struct
//! would hand it.
//!
//! One writer, one reader per buffer. A record that several threads read
//! goes through a [`Broadcast`], which is one buffer per reader and a copy
//! per publish: a 1.5 kB record copied five times at 500 Hz is nothing, and
//! the alternative — a seqlock the readers spin on — reads the record while
//! it is being written, which Rust rightly refuses to call sound.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::Instant;

/// Set on the index of the middle slot when the writer has published since
/// the reader last looked.
const DIRTY: u8 = 0b100;
const INDEX: u8 = 0b011;

struct Shared<T> {
    slots: [UnsafeCell<T>; 3],
    /// The middle slot's index, plus [`DIRTY`].
    back: AtomicU8,
}

// The writer only ever touches the slot it holds, the reader only the slot it
// holds, and the middle one changes hands by atomic exchange. No slot is ever
// reachable from two threads at once.
unsafe impl<T: Copy + Send> Sync for Shared<T> {}
unsafe impl<T: Copy + Send> Send for Shared<T> {}

/// The producing end.
pub struct Writer<T: Copy> {
    shared: Arc<Shared<T>>,
    index: u8,
}

/// The consuming end.
pub struct Reader<T: Copy> {
    shared: Arc<Shared<T>>,
    index: u8,
}

/// A buffer holding `initial` until something is published.
pub fn triple<T: Copy>(initial: T) -> (Writer<T>, Reader<T>) {
    let shared = Arc::new(Shared {
        slots: [
            UnsafeCell::new(initial),
            UnsafeCell::new(initial),
            UnsafeCell::new(initial),
        ],
        back: AtomicU8::new(1),
    });
    (
        Writer {
            shared: Arc::clone(&shared),
            index: 0,
        },
        Reader { shared, index: 2 },
    )
}

impl<T: Copy> Writer<T> {
    /// Makes `value` the latest. Never blocks; a value the reader did not get
    /// to in time is simply replaced.
    pub fn publish(&mut self, value: T) {
        // The writer's slot is its own until the exchange below.
        unsafe {
            *self.shared.slots[self.index as usize].get() = value;
        }
        let previous = self.shared.back.swap(self.index | DIRTY, Ordering::AcqRel);
        self.index = previous & INDEX;
    }
}

impl<T: Copy> Reader<T> {
    /// The latest value, and whether it is newer than the last call returned.
    pub fn latest(&mut self) -> (T, bool) {
        let mut fresh = false;
        if self.shared.back.load(Ordering::Acquire) & DIRTY != 0 {
            let previous = self.shared.back.swap(self.index, Ordering::AcqRel);
            self.index = previous & INDEX;
            fresh = previous & DIRTY != 0;
        }
        // The reader's slot is its own until the next exchange.
        (
            unsafe { *self.shared.slots[self.index as usize].get() },
            fresh,
        )
    }
}

/// One writer, several readers. Readers are created before the threads
/// start; publishing copies the value into every reader's buffer.
pub struct Broadcast<T: Copy> {
    writers: Vec<Writer<T>>,
}

impl<T: Copy> Broadcast<T> {
    pub fn new() -> Self {
        Self {
            writers: Vec::new(),
        }
    }

    /// A reader that sees `initial` until the first publish.
    pub fn subscribe(&mut self, initial: T) -> Reader<T> {
        let (writer, reader) = triple(initial);
        self.writers.push(writer);
        reader
    }

    pub fn publish(&mut self, value: T) {
        for writer in &mut self.writers {
            writer.publish(value);
        }
    }

    pub fn readers(&self) -> usize {
        self.writers.len()
    }
}

impl<T: Copy> Default for Broadcast<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// A record with the moment it arrived, so a reader can tell a command that
/// is still being sent from one whose sender went away.
#[derive(Debug, Clone, Copy)]
pub struct Stamped<T: Copy> {
    pub value: T,
    pub at: Instant,
}

impl<T: Copy> Stamped<T> {
    pub fn now(value: T) -> Self {
        Self {
            value,
            at: Instant::now(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reader_sees_the_initial_value_and_then_the_latest() {
        let (mut writer, mut reader) = triple(0u32);
        assert_eq!(reader.latest(), (0, false));

        writer.publish(1);
        writer.publish(2);
        assert_eq!(reader.latest(), (2, true), "the newest, not the first");
        assert_eq!(reader.latest(), (2, false), "and it is no longer fresh");
    }

    /// A record is read whole or not at all. Hammer it from two threads and
    /// every value that comes out must be one that went in.
    #[test]
    fn a_reader_never_sees_a_torn_record() {
        #[derive(Clone, Copy)]
        struct Wide([u64; 64]);

        let (mut writer, mut reader) = triple(Wide([0; 64]));
        let producer = std::thread::spawn(move || {
            for i in 1..=20_000u64 {
                writer.publish(Wide([i; 64]));
            }
        });
        let mut torn = 0;
        let mut seen = 0u64;
        for _ in 0..200_000 {
            let (value, _) = reader.latest();
            if value.0.iter().any(|&v| v != value.0[0]) {
                torn += 1;
            }
            seen = seen.max(value.0[0]);
        }
        producer.join().unwrap();
        assert_eq!(torn, 0);
        assert!(seen > 0, "the reader saw something the writer published");
    }

    #[test]
    fn a_broadcast_reaches_every_reader() {
        let mut broadcast = Broadcast::new();
        let mut a = broadcast.subscribe(0u8);
        let mut b = broadcast.subscribe(0u8);
        broadcast.publish(7);
        assert_eq!(a.latest().0, 7);
        assert_eq!(b.latest().0, 7);
        assert_eq!(broadcast.readers(), 2);
    }
}
