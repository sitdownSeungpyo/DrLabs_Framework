//! CAN and CAN-FD frames, and the bus that carries them.
//!
//! Two frame widths matter. Classic CAN 2.0 carries at most 8 data bytes and is
//! what Robstride and ODrive use. CAN-FD carries up to 64, which is what moteus
//! needs: a command and the telemetry query riding with it share one frame, and
//! the useful query sets do not fit in 8 bytes.

use std::sync::{Mutex, MutexGuard};
use std::time::Duration;

/// The largest CAN-FD data field.
pub const MAX_FD_PAYLOAD: usize = 64;

/// The largest classic CAN 2.0 data field.
pub const MAX_CLASSIC_PAYLOAD: usize = 8;

/// What can go wrong on a bus.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// No frame arrived in time.
    ///
    /// An ordinary condition on a polled bus, not a transport failure — which
    /// is why it is a variant a caller can match on rather than an io error.
    #[error("receive timed out")]
    Timeout,

    /// The adapter answered with an error line.
    #[error("adapter error: {0}")]
    Adapter(String),

    /// The bus was used after being closed.
    #[error("bus is closed")]
    Closed,

    /// A payload longer than a frame can carry.
    #[error("payload of {0} bytes exceeds the CAN-FD limit of 64")]
    Oversized(usize),

    /// A line that did not parse.
    #[error("malformed response: {0}")]
    Malformed(String),

    #[error("io: {0}")]
    Io(#[from] std::io::Error),
}

impl Error {
    /// Whether this is a timeout, which a polling caller treats as "nothing
    /// yet" rather than as a fault.
    pub fn is_timeout(&self) -> bool {
        matches!(self, Error::Timeout)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

/// One CAN or CAN-FD frame.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Frame {
    /// The arbitration identifier: 11 bits normally, 29 when `extended`.
    pub id: u32,
    /// The payload, at most [`MAX_FD_PAYLOAD`] bytes.
    pub data: Vec<u8>,
    /// Selects the 29-bit identifier format.
    pub extended: bool,
    /// Marks a CAN-FD frame, which permits payloads above 8 bytes.
    pub fd: bool,
    /// Requests the CAN-FD bit rate switch, so the data phase runs at the
    /// higher bitrate. Without it a 64-byte frame occupies the bus long enough
    /// to matter at kHz control rates.
    pub brs: bool,
}

impl Frame {
    /// A CAN-FD frame with the bit rate switch on, which is what the moteus
    /// driver sends.
    pub fn fd(id: u32, data: Vec<u8>) -> Self {
        Self {
            id,
            data,
            extended: false,
            fd: true,
            brs: true,
        }
    }

    /// A classic frame, which is what Robstride and ODrive use.
    pub fn classic(id: u32, data: Vec<u8>) -> Self {
        Self {
            id,
            data,
            extended: false,
            fd: false,
            brs: false,
        }
    }
}

impl std::fmt::Display for Frame {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Frame{{id={:03x} len={} fd={} brs={}}}",
            self.id,
            self.data.len(),
            self.fd,
            self.brs
        )
    }
}

/// The payload sizes a CAN-FD frame can actually carry. Anything between them
/// is padded up by the sender.
pub const VALID_DLCS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// The on-wire payload size for a frame of `size` bytes.
///
/// CAN-FD has no byte-granular length above 8, so a 9-byte payload is
/// transmitted as 12 with the remainder padded.
pub fn round_up_dlc(size: usize) -> usize {
    VALID_DLCS
        .iter()
        .copied()
        .find(|&dlc| size <= dlc)
        .unwrap_or(MAX_FD_PAYLOAD)
}

/// A CAN interface that can send and receive frames.
///
/// `recv` blocks for at most the given duration and then returns
/// [`Error::Timeout`]. Implementations need not be safe for concurrent use —
/// that is what [`SharedBus`] is for.
pub trait Bus: Send {
    /// Transmits one frame.
    fn send(&mut self, frame: &Frame) -> Result<()>;

    /// Waits up to `timeout` for the next frame.
    fn recv(&mut self, timeout: Duration) -> Result<Frame>;

    /// Discards frames already queued, so a fresh transaction is not answered
    /// by a stale reply. Returns once the bus has been quiet for `quiet`.
    fn drain(&mut self, quiet: Duration) -> Result<()>;

    /// Releases the interface. Calling it twice is harmless.
    fn close(&mut self) -> Result<()> {
        Ok(())
    }

    /// What this bus is attached to, for status output.
    fn describe(&self) -> Vec<(String, String)> {
        Vec::new()
    }
}

/// One bus, used by several controllers.
///
/// The lock covers a whole transaction, and the only way to reach the bus is
/// through the guard:
///
/// ```ignore
/// let mut bus = shared.lock();
/// bus.send(&request)?;
/// let reply = bus.recv(timeout)?;
/// ```
///
/// Serialising transactions does not make the bus faster. The achievable rate
/// per controller is the bus rate divided by the number of controllers on it,
/// and that arithmetic is the real constraint on a multi-joint machine.
pub struct SharedBus {
    bus: Mutex<Box<dyn Bus>>,
}

impl SharedBus {
    pub fn new(bus: Box<dyn Bus>) -> Self {
        Self {
            bus: Mutex::new(bus),
        }
    }

    /// Claims the bus for one transaction.
    ///
    /// Panics only if another thread panicked while holding the bus, which
    /// means the transport is in an unknown state; carrying on would be sending
    /// commands to hardware on the strength of a guess.
    pub fn lock(&self) -> MutexGuard<'_, Box<dyn Bus>> {
        self.bus.lock().expect("bus poisoned by a panicking thread")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_payload_rounds_up_to_the_next_valid_length() {
        assert_eq!(round_up_dlc(0), 0);
        assert_eq!(round_up_dlc(8), 8);
        // There is no 9, 10 or 11 on the wire.
        assert_eq!(round_up_dlc(9), 12);
        assert_eq!(round_up_dlc(13), 16);
        assert_eq!(round_up_dlc(33), 48);
        assert_eq!(round_up_dlc(64), 64);
    }

    #[test]
    fn an_oversized_payload_saturates_rather_than_wrapping() {
        assert_eq!(round_up_dlc(65), MAX_FD_PAYLOAD);
        assert_eq!(round_up_dlc(usize::MAX), MAX_FD_PAYLOAD);
    }

    #[test]
    fn a_timeout_is_distinguishable_from_a_failure() {
        assert!(Error::Timeout.is_timeout());
        assert!(!Error::Closed.is_timeout());
    }

    #[test]
    fn a_classic_frame_does_not_ask_for_the_bit_rate_switch() {
        let frame = Frame::classic(0x01, vec![0x50; MAX_CLASSIC_PAYLOAD]);
        assert!(!frame.fd);
        assert!(!frame.brs);
    }
}
