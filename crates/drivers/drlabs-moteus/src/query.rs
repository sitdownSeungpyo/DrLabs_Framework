//! Which telemetry a cycle asks for, and how wide.
//!
//! Bandwidth is the binding constraint. A CAN-FD frame carries at most 64
//! bytes, and a command shares that frame with the query it piggybacks on, so
//! asking for everything at full precision every period is what stops a loop
//! reaching its rate.
//!
//! Three profiles rotate:
//!
//! | Profile | Contents | Reply | Sent |
//! |---------|----------|-------|------|
//! | fast | mode, position, velocity, torque, q current, voltage, board temperature, fault | ~28 B | every cycle |
//! | full | fast + d current, absolute position, power, motor temperature, trajectory complete, home state | ~40 B | every 10th |
//! | verify | fast + the controller's *active* command registers | ~40 B | every 97th |
//!
//! The verify divisor is prime so that verify and full cycles do not repeatedly
//! coincide — two wide queries landing on the same period is exactly the frame
//! that overruns.

use std::collections::BTreeMap;

use crate::combiner::{Combiner, Slot};
use crate::multiplex::{Resolution, Writer, READ_BASE};
use crate::registers::{self as reg, Register};

/// Which registers a query asks for, and at what width.
///
/// A field of `None` is not requested at all.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct QueryResolution {
    pub mode: Option<Resolution>,
    pub position: Option<Resolution>,
    pub velocity: Option<Resolution>,
    pub torque: Option<Resolution>,
    pub q_current: Option<Resolution>,
    pub d_current: Option<Resolution>,
    pub abs_position: Option<Resolution>,
    pub power: Option<Resolution>,

    pub motor_temperature: Option<Resolution>,
    pub trajectory_complete: Option<Resolution>,
    pub home_state: Option<Resolution>,
    pub voltage: Option<Resolution>,
    pub temperature: Option<Resolution>,
    pub fault: Option<Resolution>,

    /// Anything else, by register number.
    pub extra: BTreeMap<Register, Resolution>,
}

fn slot(resolution: Option<Resolution>) -> Slot {
    match resolution {
        Some(resolution) => Slot::At(resolution),
        None => Slot::Skip,
    }
}

impl QueryResolution {
    /// The minimum needed to control and to trip safely.
    pub fn fast() -> Self {
        Self {
            mode: Some(Resolution::Int8),
            position: Some(Resolution::Int32),  // 0.00001 rev
            velocity: Some(Resolution::Int32),  // 0.00001 rev/s
            torque: Some(Resolution::Int16),    // 0.01 Nm
            q_current: Some(Resolution::Int16), // 0.1 A
            voltage: Some(Resolution::Int16),
            temperature: Some(Resolution::Int16),
            fault: Some(Resolution::Int8),
            ..Default::default()
        }
    }

    /// Adds the diagnostic registers.
    pub fn full() -> Self {
        Self {
            d_current: Some(Resolution::Int16),
            abs_position: Some(Resolution::Int32),
            power: Some(Resolution::Int16),
            motor_temperature: Some(Resolution::Int16),
            trajectory_complete: Some(Resolution::Int8),
            home_state: Some(Resolution::Int8),
            ..Self::fast()
        }
    }

    /// Adds the controller's live command registers.
    ///
    /// Command registers survive things the host does not see: a brownout that
    /// resets the controller, a dropped frame, another program on the same bus
    /// writing them. When they disagree with what the host believes it set, the
    /// parameter frame is re-sent immediately. That turns "the machine behaved
    /// unexpectedly" into "the controller's torque limit is 0.05 Nm, not the
    /// 0.4 we set" — a sentence somebody can act on.
    pub fn verify() -> Self {
        let mut query = Self::fast();
        query
            .extra
            .insert(reg::COMMAND_MAX_TORQUE, Resolution::Int16);
        query.extra.insert(reg::COMMAND_TIMEOUT, Resolution::Int16);
        query
            .extra
            .insert(reg::COMMAND_VELOCITY_LIMIT, Resolution::Int32);
        query
            .extra
            .insert(reg::COMMAND_ACCEL_LIMIT, Resolution::Int32);
        query
    }

    /// Renders the read subframes, and reports how many bytes the reply will
    /// occupy.
    ///
    /// The registers are requested in runs that mirror how they are laid out,
    /// so consecutive ones share a subframe header.
    pub fn encode(&self) -> (Vec<u8>, usize) {
        let mut writer = Writer::new();
        let mut reply_size = 0;

        {
            let mut first = Combiner::new(
                READ_BASE,
                reg::MODE,
                vec![
                    slot(self.mode),
                    slot(self.position),
                    slot(self.velocity),
                    slot(self.torque),
                    slot(self.q_current),
                    slot(self.d_current),
                    slot(self.abs_position),
                    slot(self.power),
                ],
            );
            first.write_reads(&mut writer);
            reply_size += first.reply_size();
        }

        {
            let mut second = Combiner::new(
                READ_BASE,
                reg::MOTOR_TEMPERATURE,
                vec![
                    slot(self.motor_temperature),
                    slot(self.trajectory_complete),
                    slot(self.home_state),
                    slot(self.voltage),
                    slot(self.temperature),
                    slot(self.fault),
                ],
            );
            second.write_reads(&mut writer);
            reply_size += second.reply_size();
        }

        if !self.extra.is_empty() {
            let low = *self.extra.keys().next().expect("not empty");
            let high = *self.extra.keys().next_back().expect("not empty");
            let slots: Vec<Slot> = (low..=high)
                .map(|register| slot(self.extra.get(&register).copied()))
                .collect();

            let mut extra = Combiner::new(READ_BASE, low, slots);
            extra.write_reads(&mut writer);
            reply_size += extra.reply_size();
        }

        (writer.into_bytes(), reply_size)
    }

    /// The on-wire length of the read subframes alone.
    pub fn encoded_len(&self) -> usize {
        self.encode().0.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02X}")).collect()
    }

    /// The suffix every fast-query frame carries, pinned against the reference.
    const FAST_QUERY_SUFFIX: &str = "11001A011603160D110F";

    #[test]
    fn the_fast_query_matches_the_reference_encoding() {
        let (data, _) = QueryResolution::fast().encode();
        assert_eq!(hex(&data), FAST_QUERY_SUFFIX);
    }

    #[test]
    fn the_verify_query_asks_for_the_command_registers() {
        let (data, _) = QueryResolution::verify().encode();
        let rendered = hex(&data);
        assert!(
            rendered.starts_with(FAST_QUERY_SUFFIX),
            "verify should extend fast: {rendered}"
        );
        // int16 at 0x025, int16 at 0x027, two int32 from 0x028.
        assert_eq!(&rendered[FAST_QUERY_SUFFIX.len()..], "152515271A28");
    }

    #[test]
    fn the_full_query_is_wider_than_the_fast_one() {
        let fast = QueryResolution::fast().encode();
        let full = QueryResolution::full().encode();
        assert!(full.1 > fast.1, "full reply {} vs fast {}", full.1, fast.1);
        assert!(full.0.len() > fast.0.len());
    }

    /// The frame budget depends on this number being right.
    #[test]
    fn reply_sizes_stay_inside_what_a_can_fd_frame_holds() {
        for (name, query) in [
            ("fast", QueryResolution::fast()),
            ("full", QueryResolution::full()),
            ("verify", QueryResolution::verify()),
        ] {
            let (data, reply) = query.encode();
            assert!(reply <= 64, "{name} reply is {reply} bytes");
            assert!(data.len() <= 64, "{name} request is {} bytes", data.len());
        }
    }
}
