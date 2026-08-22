//! One moteus on a CAN bus.
//!
//! The controller owns the framing, not the timing: a caller drives it from its
//! own control loop, and every method here performs one bus transaction and
//! returns.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};
use drlabs_transport::can::{self, Frame, SharedBus};

use crate::multiplex::{parse_reply, RawValue, Resolution, READ_ERROR, REPLY_BASE};
use crate::query::QueryResolution;
use crate::registers::{self as reg, PositionResolution, Register};

/// The transaction timeout a controller starts with.
///
/// Deliberately generous. Cutting a transaction short because the control
/// period elapsed leaves the adapter's pending-request state inconsistent,
/// after which every following transaction returns empty — a 9 ms cap on a
/// 100 Hz loop turns a working link into a total loss. The transport has its
/// own retries; this only catches a wedged adapter, and when it fires the link
/// should be torn down rather than reused.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(2);

/// What one transaction read back, by register.
pub type Values = BTreeMap<Register, RawValue>;

/// One moteus controller.
pub struct Controller {
    /// The controller's CAN identifier, 1..126.
    pub id: u8,

    /// The identifier this host answers to. Replies are addressed back to it,
    /// so leaving it at 0 is right unless several hosts share a bus.
    pub source_id: u8,

    /// An optional 13-bit prefix that segments one physical bus into several
    /// logical ones.
    pub can_prefix: u16,

    /// How wide each command field goes out.
    pub position_resolution: PositionResolution,

    /// Bounds a single transaction.
    pub timeout: Duration,
}

impl Controller {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            source_id: 0,
            can_prefix: 0,
            position_resolution: PositionResolution::default(),
            timeout: DEFAULT_TIMEOUT,
        }
    }

    /// The CAN identifier for a frame to this controller.
    ///
    /// ```text
    /// bits 0-7     destination id
    /// bits 8-14    source id
    /// bit  15      set when a reply is required
    /// bits 16-28   optional bus prefix
    /// ```
    pub fn arbitration_id(&self, reply_required: bool) -> u32 {
        let mut id = self.id as u32 | (self.source_id as u32) << 8 | (self.can_prefix as u32) << 16;
        if reply_required {
            id |= 0x8000;
        }
        id
    }

    /// Sends one payload and decodes the register values that come back.
    pub fn transact(
        &self,
        bus: &SharedBus,
        payload: &[u8],
        expect_reply: bool,
    ) -> Result<Option<Values>> {
        let reply = self.exchange(bus, payload, expect_reply, is_register_reply)?;
        let reply = match reply {
            Some(reply) => reply,
            None => return Ok(None),
        };

        let parsed = parse_reply(&reply);
        if parsed.truncated {
            return Err(Error::Malformed(format!(
                "reply from CAN id {} ended mid-value",
                self.id
            )));
        }
        if !parsed.errors.is_empty() {
            // The controller answered a read with "no such register". Worth
            // reporting rather than silently returning nothing, because it is
            // almost always a driver or firmware version mismatch.
            let registers: Vec<String> = parsed
                .errors
                .iter()
                .map(|(register, code)| format!("{register:#05x} (error {code})"))
                .collect();
            return Err(Error::Refused(format!(
                "CAN id {} refused a register read: {}",
                self.id,
                registers.join(", ")
            )));
        }

        Ok(Some(parsed.values))
    }

    /// Sends one payload and, when a reply is expected, waits for a frame this
    /// controller sent that `accept` recognises.
    ///
    /// Frames from other controllers on a shared bus are discarded rather than
    /// misread: the reply's source field has to match the controller we
    /// addressed. `Ok(None)` means nothing answered in time, which is a missed
    /// cycle rather than a transport failure — one dropped reply on a noisy bus
    /// is normal and the control loop should keep running.
    pub(crate) fn exchange(
        &self,
        bus: &SharedBus,
        payload: &[u8],
        expect_reply: bool,
        accept: fn(&[u8]) -> bool,
    ) -> Result<Option<Vec<u8>>> {
        let id = self.arbitration_id(expect_reply);
        let frame = Frame {
            id,
            data: payload.to_vec(),
            extended: id > 0x7ff,
            fd: true,
            brs: true,
        };

        // A send and its reply are one transaction. On a bus shared with other
        // controllers, letting them interleave means each reads the other's
        // answer, discards it as not its own, and both time out. The guard
        // holds the pair together for as long as this scope lasts.
        let mut bus = bus.lock();

        bus.send(&frame).map_err(|error| Error::Transport {
            operation: "send",
            source: Box::new(error),
        })?;
        if !expect_reply {
            return Ok(None);
        }

        let deadline = Instant::now() + self.timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }

            let reply = match bus.recv(remaining) {
                Ok(reply) => reply,
                Err(can::Error::Timeout) => return Ok(None),
                Err(error) => {
                    return Err(Error::Transport {
                        operation: "receive",
                        source: Box::new(error),
                    })
                }
            };

            // The controller answers with its own id in the source field.
            if ((reply.id >> 8) as u8) & 0x7f != self.id {
                continue;
            }
            if reply.data.is_empty() || !accept(&reply.data) {
                continue;
            }
            return Ok(Some(reply.data));
        }
    }

    /// Reads an arbitrary set of registers, scaled into physical units.
    ///
    /// The escape hatch for registers the typed API does not cover.
    pub fn read_registers(
        &self,
        bus: &SharedBus,
        want: &BTreeMap<Register, Resolution>,
    ) -> Result<BTreeMap<Register, f64>> {
        if want.is_empty() {
            return Ok(BTreeMap::new());
        }
        let query = QueryResolution {
            extra: want.clone(),
            ..Default::default()
        };
        let (payload, _) = query.encode();

        let values = match self.transact(bus, &payload, true)? {
            Some(values) => values,
            None => return Ok(BTreeMap::new()),
        };
        Ok(values
            .into_iter()
            .map(|(register, value)| {
                (
                    register,
                    reg::scale_value(register, value.raw, value.resolution),
                )
            })
            .collect())
    }
}

/// Accepts the frames that answer a register request: a reply subframe, or the
/// read-error subframe the controller sends when it refuses.
fn is_register_reply(data: &[u8]) -> bool {
    data[0] & 0xf0 == REPLY_BASE || data[0] == READ_ERROR
}

impl std::fmt::Display for Controller {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "moteus[id={}]", self.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_arbitration_id_carries_destination_source_and_reply() {
        let mut controller = Controller::new(3);
        assert_eq!(controller.arbitration_id(false), 0x0003);
        // Bit 15 asks for a reply.
        assert_eq!(controller.arbitration_id(true), 0x8003);

        controller.source_id = 1;
        assert_eq!(controller.arbitration_id(true), 0x8103);
    }

    /// A prefix pushes the identifier past 11 bits, which is what makes the
    /// frame extended — get that wrong and the controller never sees it.
    #[test]
    fn a_bus_prefix_lands_above_the_eleven_bit_range() {
        let controller = Controller {
            can_prefix: 0x0002,
            ..Controller::new(1)
        };
        let id = controller.arbitration_id(true);
        assert_eq!(id, 0x2_8001);
        assert!(id > 0x7ff, "must be sent as an extended frame");
    }

    #[test]
    fn only_a_register_reply_or_a_read_error_is_accepted() {
        assert!(is_register_reply(&[REPLY_BASE | 0x01]));
        assert!(is_register_reply(&[READ_ERROR]));
        // A write subframe echoed back is not an answer to anything.
        assert!(!is_register_reply(&[0x01]));
    }
}
