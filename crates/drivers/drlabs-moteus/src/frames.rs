//! Frame builders.
//!
//! Each returns the multiplex payload for one command, optionally with a
//! telemetry query appended so that the reply carries state without costing a
//! second bus transaction.

use crate::combiner::{Combiner, Slot};
use crate::multiplex::{Resolution, Scales, Writer, WRITE_BASE};
use crate::query::QueryResolution;
use crate::registers::{self as reg, ControllerMode, PositionResolution};

/// A command field that may or may not be transmitted.
///
/// Presence is tracked separately from the value, because NaN is *meaningful*
/// on this protocol rather than a stand-in for "absent". Sending NaN for the
/// position register means "no position target" — which is how velocity mode is
/// expressed — and sending it for the maximum torque register means "use the
/// controller's own configured limit". Letting NaN double as "omit" would
/// silently turn a velocity command into a stale position hold, because a
/// command register that is not written keeps whatever was last put in it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Send {
    /// Omit the register entirely.
    #[default]
    No,
    /// Transmit this value, NaN included.
    Value(f64),
}

impl Send {
    /// Transmits only when the value is a real number. For the parameters
    /// where NaN genuinely does mean "leave it alone".
    pub fn present(value: f64) -> Self {
        if value.is_nan() {
            Send::No
        } else {
            Send::Value(value)
        }
    }

    fn slot(self, resolution: Resolution) -> Slot {
        match self {
            Send::No => Slot::Skip,
            Send::Value(_) => Slot::At(resolution),
        }
    }

    fn value(self) -> f64 {
        match self {
            Send::No => f64::NAN,
            Send::Value(value) => value,
        }
    }
}

/// Emits the two-byte "write mode register" prologue every command begins with.
fn set_mode(writer: &mut Writer, mode: ControllerMode) {
    writer.push_u8(WRITE_BASE | (Resolution::Int8.type_bits() << 2) | 0x01);
    writer.push_u8(reg::MODE as u8);
    writer.push_value(mode.0 as f64, Resolution::Int8);
}

fn append_query(writer: &mut Writer, query: Option<&QueryResolution>) {
    if let Some(query) = query {
        let (data, _) = query.encode();
        writer.extend(&data);
    }
}

/// Mode 0: the bridge is released and the shaft turns free.
pub fn stop(query: Option<&QueryResolution>) -> Vec<u8> {
    let mut writer = Writer::new();
    set_mode(&mut writer, ControllerMode::STOPPED);
    append_query(&mut writer, query);
    writer.into_bytes()
}

/// Mode 15: the phases are shorted, which brakes the axis without the host
/// having to keep commanding it.
pub fn brake(query: Option<&QueryResolution>) -> Vec<u8> {
    let mut writer = Writer::new();
    set_mode(&mut writer, ControllerMode::BRAKE);
    append_query(&mut writer, query);
    writer.into_bytes()
}

/// Reads telemetry without changing the controller's mode, which is what a
/// disarmed session sends: it wants to see the axis, not drive it.
pub fn query_only(query: Option<&QueryResolution>) -> Vec<u8> {
    let mut writer = Writer::new();
    append_query(&mut writer, query);
    writer.into_bytes()
}

/// Mode 12: hold zero velocity without holding a position, so the axis resists
/// motion but does not fight back to a target.
pub fn zero_velocity(
    kd_scale: Send,
    resolutions: &PositionResolution,
    query: Option<&QueryResolution>,
) -> Vec<u8> {
    let mut writer = Writer::new();
    set_mode(&mut writer, ControllerMode::ZERO_VELOCITY);

    write_run(
        &mut writer,
        reg::COMMAND_KD_SCALE,
        &[kd_scale.slot(resolutions.kd_scale)],
        &[(kd_scale, resolutions.kd_scale, reg::PWM_SCALES)],
        &[],
    );

    append_query(&mut writer, query);
    writer.into_bytes()
}

/// The full set of position-mode parameters.
///
/// A field left unset is omitted from the frame, which leaves the controller's
/// existing value in force — that is what makes it affordable to send only the
/// handful of registers that change every period.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PositionCommand {
    pub position: Send,
    pub velocity: Send,
    pub feedforward_torque: Send,
    pub kp_scale: Send,
    pub kd_scale: Send,
    pub max_torque: Send,
    pub stop_position: Send,
    pub watchdog: Send,
    pub velocity_limit: Send,
    pub accel_limit: Send,
    pub ilimit_scale: Send,
    pub ignore_position_bounds: Send,
}

/// Mode 10.
pub fn position(
    command: &PositionCommand,
    resolutions: &PositionResolution,
    query: Option<&QueryResolution>,
) -> Vec<u8> {
    let mut writer = Writer::new();
    set_mode(&mut writer, ControllerMode::POSITION);

    // Registers 0x020..0x02d, in order. The two the controller has but this
    // driver does not use — fixed voltage and fixed current override — are
    // skipped, which splits the run exactly where the reference does.
    let fields: [(Send, Resolution, Scales); 12] = [
        (command.position, resolutions.position, reg::POSITION_SCALES),
        (command.velocity, resolutions.velocity, reg::VELOCITY_SCALES),
        (
            command.feedforward_torque,
            resolutions.feedforward_torque,
            reg::TORQUE_SCALES,
        ),
        (command.kp_scale, resolutions.kp_scale, reg::PWM_SCALES),
        (command.kd_scale, resolutions.kd_scale, reg::PWM_SCALES),
        (
            command.max_torque,
            resolutions.max_torque,
            reg::TORQUE_SCALES,
        ),
        (
            command.stop_position,
            resolutions.stop_position,
            reg::POSITION_SCALES,
        ),
        (command.watchdog, resolutions.watchdog, reg::TIME_SCALES),
        (
            command.velocity_limit,
            resolutions.velocity_limit,
            reg::VELOCITY_SCALES,
        ),
        (
            command.accel_limit,
            resolutions.accel_limit,
            reg::ACCEL_SCALES,
        ),
        (
            command.ilimit_scale,
            resolutions.ilimit_scale,
            reg::PWM_SCALES,
        ),
        (
            command.ignore_position_bounds,
            resolutions.ignore_position_bounds,
            reg::PWM_SCALES,
        ),
    ];

    // The two override registers sit at 0x02a and 0x02c, between the fields
    // above, so the slot list has to carry them as skips.
    let mut slots = Vec::with_capacity(14);
    for (index, (send, resolution, _)) in fields.iter().enumerate() {
        slots.push(send.slot(*resolution));
        if index == 9 {
            slots.push(Slot::Skip); // 0x02a fixed voltage override
        }
        if index == 10 {
            slots.push(Slot::Skip); // 0x02c fixed current override
        }
    }

    write_run(
        &mut writer,
        reg::COMMAND_POSITION,
        &slots,
        &fields,
        &[10, 12],
    );

    append_query(&mut writer, query);
    writer.into_bytes()
}

/// The parameters of a stay-within command.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StayWithinCommand {
    pub lower: Send,
    pub upper: Send,
    pub feedforward_torque: Send,
    pub kp_scale: Send,
    pub kd_scale: Send,
    pub max_torque: Send,
    pub watchdog: Send,
    pub ilimit_scale: Send,
    pub ignore_position_bounds: Send,
}

/// Mode 13: torque only outside the bounds.
pub fn stay_within(
    command: &StayWithinCommand,
    resolutions: &PositionResolution,
    query: Option<&QueryResolution>,
) -> Vec<u8> {
    let mut writer = Writer::new();
    set_mode(&mut writer, ControllerMode::STAY_WITHIN);

    let fields: [(Send, Resolution, Scales); 9] = [
        (command.lower, resolutions.position, reg::POSITION_SCALES),
        (command.upper, resolutions.position, reg::POSITION_SCALES),
        (
            command.feedforward_torque,
            resolutions.feedforward_torque,
            reg::TORQUE_SCALES,
        ),
        (command.kp_scale, resolutions.kp_scale, reg::PWM_SCALES),
        (command.kd_scale, resolutions.kd_scale, reg::PWM_SCALES),
        (
            command.max_torque,
            resolutions.max_torque,
            reg::TORQUE_SCALES,
        ),
        (command.watchdog, resolutions.watchdog, reg::TIME_SCALES),
        (
            command.ilimit_scale,
            resolutions.ilimit_scale,
            reg::PWM_SCALES,
        ),
        (
            command.ignore_position_bounds,
            resolutions.ignore_position_bounds,
            reg::PWM_SCALES,
        ),
    ];

    let slots: Vec<Slot> = fields
        .iter()
        .map(|(send, resolution, _)| send.slot(*resolution))
        .collect();

    write_run(&mut writer, reg::WITHIN_LOWER_BOUND, &slots, &fields, &[]);

    append_query(&mut writer, query);
    writer.into_bytes()
}

/// Re-references the output position. Runtime only: it does not touch the
/// controller's stored configuration.
pub fn set_output(
    register: reg::Register,
    position: f64,
    query: Option<&QueryResolution>,
) -> Vec<u8> {
    let mut writer = Writer::new();
    writer.push_header(WRITE_BASE, Resolution::F32, 1, register);
    writer.push_value(position, Resolution::F32);
    append_query(&mut writer, query);
    writer.into_bytes()
}

/// Walks the slot list, emitting each header and then the value it frames.
///
/// The interleaving is the whole point: a subframe is a header followed by its
/// own values, and writing all the headers first produces a frame with the
/// framing at the front and the data behind it. That is a real mistake, made
/// once here and caught by the reference bytes.
///
/// `holes` are indices in the slot list that carry no field — registers this
/// driver never writes but which sit inside the run, so the combiner has to
/// know they break it.
fn write_run(
    writer: &mut Writer,
    start: reg::Register,
    slots: &[Slot],
    fields: &[(Send, Resolution, Scales)],
    holes: &[usize],
) {
    let mut combiner = Combiner::new(WRITE_BASE, start, slots.to_vec());
    let mut field_index = 0;

    for slot_index in 0..slots.len() {
        let step = match combiner.advance() {
            Some(step) => step,
            None => break,
        };
        if let Some(header) = step.header {
            writer.push_header(WRITE_BASE, header.resolution, header.count, header.register);
        }
        if holes.contains(&slot_index) {
            continue;
        }
        let (send, resolution, scales) = fields[field_index];
        field_index += 1;
        if step.emit_value {
            writer.push_mapped(send.value(), &scales, resolution);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The expected byte strings below were produced by the reference
    /// implementation — the official `moteus` Python package — and are pinned
    /// here verbatim. They are the only check that this encoder is compatible
    /// with real firmware rather than merely self-consistent, so they should be
    /// regenerated against the reference rather than adjusted to match a change
    /// here.
    const FAST_QUERY_SUFFIX: &str = "11001A011603160D110F";

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02X}")).collect()
    }

    fn fast() -> QueryResolution {
        QueryResolution::fast()
    }

    fn val(value: f64) -> Send {
        Send::Value(value)
    }

    #[test]
    fn stop_without_query() {
        assert_eq!(hex(&stop(None)), "010000");
    }

    #[test]
    fn stop_with_fast_query() {
        assert_eq!(
            hex(&stop(Some(&fast()))),
            format!("010000{FAST_QUERY_SUFFIX}")
        );
    }

    #[test]
    fn query_only_frame() {
        assert_eq!(hex(&query_only(Some(&fast()))), FAST_QUERY_SUFFIX);
    }

    #[test]
    fn brake_with_fast_query() {
        assert_eq!(
            hex(&brake(Some(&fast()))),
            format!("01000F{FAST_QUERY_SUFFIX}")
        );
    }

    /// Only the registers a steady-state position cycle needs.
    ///
    /// Note the position value: 0.25 / 0.00001 is 24999.999… in binary floating
    /// point, and the protocol truncates rather than rounds, so the wire carries
    /// 24999 (0x61A7). Rounding here would put us one count away from the
    /// reference on almost every command.
    #[test]
    fn position_with_the_minimal_parameter_set() {
        let command = PositionCommand {
            position: val(0.25),
            velocity: val(0.0),
            max_torque: val(0.5),
            watchdog: val(0.04),
            ..Default::default()
        };
        let want = format!(
            "{}{}{}{}{}{}{}{}",
            "01000A",
            "0A20",
            "A761000000000000", // position, velocity
            "0925",
            "F4010000", // maximum torque 0.5 Nm
            "0527",
            "2800", // watchdog 0.04 s
            FAST_QUERY_SUFFIX
        );
        assert_eq!(
            hex(&position(
                &command,
                &PositionResolution::default(),
                Some(&fast())
            )),
            want
        );
    }

    #[test]
    fn position_with_the_full_parameter_set() {
        let command = PositionCommand {
            position: val(-0.125),
            velocity: val(0.0),
            feedforward_torque: val(0.0),
            kp_scale: val(1.0),
            kd_scale: val(1.0),
            max_torque: val(0.3),
            watchdog: val(0.04),
            velocity_limit: val(1.5),
            accel_limit: val(5.0),
            ilimit_scale: val(1.0),
            ignore_position_bounds: val(0.0),
            ..Default::default()
        };
        let want = format!(
            "{}{}{}{}{}{}{}{}{}{}{}{}",
            "01000A",
            "0A20",
            "2DCFFFFF00000000",
            "0722",
            "0000FF7FFF7F", // feedforward, kp, kd
            "0925",
            "2C010000", // maximum torque 0.3 Nm
            "0527",
            "2800",
            "0A28F04902001FA10700", // velocity limit, accel limit
            "052BFF7F",             // ilimit scale
            format!("012D00{FAST_QUERY_SUFFIX}"), // ignore position bounds
        );
        assert_eq!(
            hex(&position(
                &command,
                &PositionResolution::default(),
                Some(&fast())
            )),
            want
        );
    }

    /// NaN in the maximum torque register hands the limit back to the
    /// controller's own configuration. It must be transmitted as the
    /// negative-most int32, not omitted.
    #[test]
    fn position_deferring_the_torque_limit() {
        let command = PositionCommand {
            position: val(0.1),
            velocity: val(0.0),
            max_torque: val(f64::NAN),
            watchdog: val(0.04),
            ..Default::default()
        };
        let want = format!(
            "01000A0A20{}0925{}0527{}{}",
            "1027000000000000", "00000080", "2800", FAST_QUERY_SUFFIX
        );
        assert_eq!(
            hex(&position(
                &command,
                &PositionResolution::default(),
                Some(&fast())
            )),
            want
        );
    }

    /// Velocity mode: a NaN position is what says "no position target". If it
    /// were omitted instead, the controller would keep servoing to whatever
    /// position was last commanded.
    #[test]
    fn velocity_sends_the_position_as_nan() {
        let command = PositionCommand {
            position: val(f64::NAN),
            velocity: val(0.75),
            max_torque: val(0.5),
            watchdog: val(0.04),
            ..Default::default()
        };
        let want = format!(
            "01000A0A20{}0925{}0527{}{}",
            "00000080F8240100", "F4010000", "2800", FAST_QUERY_SUFFIX
        );
        assert_eq!(
            hex(&position(
                &command,
                &PositionResolution::default(),
                Some(&fast())
            )),
            want
        );
    }

    #[test]
    fn set_output_exact_frame() {
        assert_eq!(
            hex(&set_output(reg::SET_OUTPUT_EXACT, 0.0, Some(&fast()))),
            format!("0DB10200000000{FAST_QUERY_SUFFIX}")
        );
    }

    #[test]
    fn stay_within_frame() {
        let command = StayWithinCommand {
            lower: val(-0.3),
            upper: val(0.3),
            feedforward_torque: val(0.0),
            kp_scale: val(1.0),
            kd_scale: val(1.0),
            max_torque: val(0.5),
            watchdog: val(0.04),
            ilimit_scale: val(1.0),
            ignore_position_bounds: val(0.0),
        };
        let want = format!(
            "{}{}{}{}{}{}{}{}{}",
            "01000D",
            "0A40",
            "D18AFFFF2F750000", // lower -0.3, upper 0.3
            "0742",
            "0000FF7FFF7F",
            "0945",
            "F4010000",
            "06462800FF7F", // watchdog, ilimit
            format!("014800{FAST_QUERY_SUFFIX}"),
        );
        assert_eq!(
            hex(&stay_within(
                &command,
                &PositionResolution::default(),
                Some(&fast())
            )),
            want
        );
    }

    /// A steady cycle carries only the registers that change, and fits beside
    /// any query.
    #[test]
    fn a_steady_cycle_fits_one_can_fd_payload() {
        let steady = PositionCommand {
            position: val(0.25),
            velocity: val(0.5),
            max_torque: val(0.5),
            watchdog: val(0.25),
            ..Default::default()
        };
        for (name, query) in [
            ("fast", QueryResolution::fast()),
            ("full", QueryResolution::full()),
            ("verify", QueryResolution::verify()),
        ] {
            let frame = position(&steady, &PositionResolution::default(), Some(&query));
            assert!(
                frame.len() <= 64,
                "a steady cycle with the {name} query is {} bytes",
                frame.len()
            );
        }
    }

    /// A full parameter refresh beside a wide query does *not* fit, and that is
    /// the fact the driver has to handle rather than hope about.
    ///
    /// This is not hypothetical. The Go driver sent every populated field every
    /// cycle, and on the one cycle in ten where a wide query landed on top the
    /// frame came to 66 bytes; the adapter answers an over-long frame by not
    /// receiving it, so the symptom was a dead link rather than a truncated
    /// reply. Two things follow, and both are the layer above this one: the
    /// parameter set is only re-sent when it changes, and the query is narrowed
    /// to whatever fits beside the command.
    #[test]
    fn a_full_refresh_beside_a_wide_query_overruns_the_frame() {
        let full = PositionCommand {
            position: val(0.25),
            velocity: val(0.5),
            feedforward_torque: val(0.0),
            kp_scale: val(1.0),
            kd_scale: val(1.0),
            max_torque: val(0.5),
            stop_position: val(0.3),
            watchdog: val(0.25),
            velocity_limit: val(0.4),
            accel_limit: val(1.5),
            ilimit_scale: val(1.0),
            ignore_position_bounds: val(0.0),
        };
        let resolutions = PositionResolution::default();

        let alone = position(&full, &resolutions, None);
        assert!(
            alone.len() <= 64,
            "the parameter set alone is {} bytes",
            alone.len()
        );

        let with_wide = position(&full, &resolutions, Some(&QueryResolution::full()));
        assert!(
            with_wide.len() > 64,
            "expected the full refresh plus a wide query to overrun; it was {} bytes",
            with_wide.len()
        );
    }
}
