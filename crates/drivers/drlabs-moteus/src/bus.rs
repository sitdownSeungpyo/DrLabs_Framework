//! Several moteus on one CAN-FD adapter, driven as a [`Bus`].
//!
//! One cycle is N sends and then up to N receives, not N transactions: every
//! command frame goes out before any reply is waited for, so the controllers
//! answer in parallel and the adapter's latency is paid once a cycle rather
//! than once a joint. The replies are sorted back to their axes by the
//! source id the controller puts in the arbitration field.
//!
//! What this bus does *not* do is read the diagnostic console. Family,
//! hardware revision and the stored travel bounds come from `drlabs probe`
//! and the one-axis [`MoteusLink`](crate::link::MoteusLink); here the open
//! reads only the model, firmware and serial registers, so a 26-axis machine
//! opens in a fraction of a second rather than a console drain per axis.
//!
//! Allocation: the adapter's line protocol allocates for every frame it
//! renders and parses, and a decoded reply is a map. Neither is in this
//! file's control, and both are small; the cycle time this costs is what
//! the bench measures, and it has not been measured yet (2026-11-04).

use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::bus::{AxisCmd, AxisReply, AxisStatus, Bus};
use drlabs_core::error::{Error, Result};
use drlabs_core::link::{Capabilities, Envelope, Identity};
use drlabs_transport::can::{self, Frame, SharedBus};

use crate::controller::Controller;
use crate::frames;
use crate::hardware;
use crate::link::{self, Codec};
use crate::multiplex::{parse_reply, READ_ERROR, REPLY_BASE};

/// How long a cycle waits for the last of its replies before giving the
/// silent axes up for this tick.
///
/// A moteus answers within a few hundred microseconds of the frame it
/// answers; the adapter adds its USB turnaround. Five milliseconds is far
/// above both and below any control period this bus is meant for, so a
/// reply that is not here by then is not coming this tick.
pub const DEFAULT_REPLY_TIMEOUT: Duration = Duration::from_millis(5);

struct Axis {
    controller: Controller,
    codec: Codec,
    /// Whether this tick's frame asked for a reply.
    expect_reply: bool,
    reply: AxisReply,
}

/// Every moteus on one adapter.
pub struct MoteusBus {
    bus: Arc<SharedBus>,
    axes: Vec<Axis>,
    open: bool,
    reply_timeout: Duration,
}

impl MoteusBus {
    /// Controllers `ids` on a bus the caller opened. The bus is this
    /// struct's to drive; nothing else should transact on it while a cycle
    /// is running.
    pub fn new(bus: Arc<SharedBus>, ids: &[u8]) -> Self {
        Self {
            bus,
            axes: ids
                .iter()
                .map(|&id| Axis {
                    controller: Controller::new(id),
                    codec: Codec::new(),
                    expect_reply: false,
                    reply: AxisReply::SILENT,
                })
                .collect(),
            open: false,
            reply_timeout: DEFAULT_REPLY_TIMEOUT,
        }
    }

    pub fn with_reply_timeout(mut self, timeout: Duration) -> Self {
        self.reply_timeout = timeout;
        self
    }

    /// The controller for one axis, for callers that need registers.
    pub fn controller(&self, axis: usize) -> &Controller {
        &self.axes[axis].controller
    }

    pub fn ids(&self) -> Vec<u8> {
        self.axes.iter().map(|axis| axis.controller.id).collect()
    }

    /// Stops one controller and reads its identity registers.
    fn open_axis(&mut self, index: usize) -> Result<Option<Identity>> {
        let axis = &mut self.axes[index];
        axis.codec.reset();

        // The opening stop is not a formality: a controller left driving by
        // a previous process keeps driving until it is told otherwise. Its
        // reply is the proof the controller is there.
        let payload = frames::stop(Some(&axis.codec.fast));
        // A missing controller should be reported in well under the
        // transaction deadline a wedged adapter gets.
        let controller = Controller {
            timeout: axis.controller.timeout.min(Duration::from_millis(200)),
            ..axis.controller
        };
        match controller.transact(&self.bus, &payload, true)? {
            Some(_) => {}
            None => {
                return Err(Error::Refused(format!(
                    "no reply from CAN id {} — check the id, the bus wiring and termination, and \
                     that the controller is powered",
                    axis.controller.id
                )))
            }
        }

        let mut identity = link::identify_registers(&axis.controller, &self.bus);
        // The family and revision are not read here (see the module notes),
        // so the board is named by what the registers give.
        identity.board = match identity.model_number {
            Some(model) => format!("moteus (model {model:#06x})"),
            None => "moteus".into(),
        };
        identity.status = hardware::board_status(None, None);
        Ok(Some(identity))
    }
}

impl Bus for MoteusBus {
    fn kind(&self) -> &'static str {
        "moteus"
    }

    fn axes(&self) -> usize {
        self.axes.len()
    }

    fn open(&mut self) -> Result<Vec<Option<Identity>>> {
        self.bus
            .lock()
            .drain(Duration::from_millis(20))
            .map_err(|error| Error::Transport {
                operation: "drain",
                source: Box::new(error),
            })?;
        let mut identities = Vec::with_capacity(self.axes.len());
        for index in 0..self.axes.len() {
            identities.push(self.open_axis(index)?);
        }
        self.open = true;
        Ok(identities)
    }

    /// Stops every controller. The adapter belongs to whoever opened it.
    fn close(&mut self) -> Result<()> {
        if !self.open {
            return Ok(());
        }
        for axis in &self.axes {
            // The safest state the controller has; a failure here must not
            // stop the caller tearing down.
            let _ = axis
                .controller
                .transact(&self.bus, &frames::stop(None), false);
        }
        self.open = false;
        Ok(())
    }

    fn cycle(&mut self, cmds: &[AxisCmd], out: &mut [AxisReply]) -> Result<()> {
        if !self.open {
            return Err(Error::Closed);
        }
        assert_eq!(cmds.len(), self.axes.len(), "one command per axis");
        assert_eq!(out.len(), self.axes.len(), "one reply slot per axis");

        // The whole cycle is one transaction on the adapter: the sends and
        // the receives belong together, and nothing else may interleave.
        let mut bus = self.bus.lock();

        // Anything still queued from last tick is a reply to last tick's
        // question. Dropped, so it cannot be taken for this tick's.
        bus.drain(Duration::ZERO).map_err(transport("drain"))?;

        // --- sends ----------------------------------------------------------
        let mut expected = 0usize;
        for (axis, cmd) in self.axes.iter_mut().zip(cmds) {
            let (payload, expect_reply) = axis.codec.frame_payload(
                &cmd.command,
                cmd.profile,
                &axis.controller.position_resolution,
            );
            axis.expect_reply = expect_reply;
            axis.reply = AxisReply::SILENT;
            if payload.is_empty() && !expect_reply {
                continue;
            }
            let id = axis.controller.arbitration_id(expect_reply);
            let frame = Frame {
                id,
                data: payload,
                extended: id > 0x7ff,
                fd: true,
                brs: true,
            };
            bus.send(&frame).map_err(transport("send"))?;
            if expect_reply {
                expected += 1;
            }
        }

        // --- receives -------------------------------------------------------
        let deadline = Instant::now() + self.reply_timeout;
        let mut answered = 0usize;
        while answered < expected {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            let frame = match bus.recv(remaining) {
                Ok(frame) => frame,
                Err(can::Error::Timeout) => break,
                Err(error) => {
                    return Err(Error::Transport {
                        operation: "receive",
                        source: Box::new(error),
                    })
                }
            };
            // The controller answers with its own id in the source field.
            let source = ((frame.id >> 8) as u8) & 0x7f;
            let Some(axis) = self
                .axes
                .iter_mut()
                .find(|axis| axis.controller.id == source && axis.expect_reply)
            else {
                continue;
            };
            if frame.data.is_empty() || !is_register_reply(&frame.data) {
                continue;
            }
            if axis.reply.status.answered() {
                // A second answer this tick: a late one from last tick that
                // the drain did not catch. The first is kept.
                continue;
            }
            let parsed = parse_reply(&frame.data);
            axis.reply = if parsed.truncated || !parsed.errors.is_empty() {
                // The controller answered a read with "no such register", or
                // the frame ended mid-value: an answer, and a useless one.
                AxisReply {
                    status: AxisStatus::Refused,
                    ..AxisReply::SILENT
                }
            } else {
                link::decode_axis(&parsed.values)
            };
            answered += 1;
        }
        drop(bus);

        for (axis, slot) in self.axes.iter().zip(out.iter_mut()) {
            *slot = axis.reply;
        }
        Ok(())
    }

    fn capabilities(&self, _axis: usize) -> Capabilities {
        link::capabilities()
    }

    fn envelope(&self, _axis: usize) -> Option<Envelope> {
        Some(link::envelope())
    }
}

fn transport(operation: &'static str) -> impl Fn(can::Error) -> Error {
    move |error| Error::Transport {
        operation,
        source: Box::new(error),
    }
}

/// Accepts the frames that answer a register request: a reply subframe, or
/// the read-error subframe the controller sends when it refuses.
fn is_register_reply(data: &[u8]) -> bool {
    data[0] & 0xf0 == REPLY_BASE || data[0] == READ_ERROR
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::Mutex;

    use drlabs_core::command::{Command, Field, Mode, QueryProfile};
    use drlabs_core::link::Link;
    use drlabs_core::units::{Rev, RevPerSec};
    use drlabs_transport::can::Bus as CanBus;

    use crate::link::MoteusLink;
    use crate::multiplex::{Resolution, Writer};
    use crate::registers as reg;

    /// An adapter made of a script: records every frame sent, and answers a
    /// frame that asks for a reply with whatever the test scripted for that
    /// destination, delivered on the next receive.
    #[derive(Clone, Default)]
    struct FakeAdapter {
        state: Arc<Mutex<FakeState>>,
    }

    #[derive(Default)]
    struct FakeState {
        sent: Vec<Frame>,
        /// Reply payload per controller id; an id with none stays silent.
        answers: Vec<(u8, Vec<u8>)>,
        inbox: VecDeque<Frame>,
        drains: usize,
    }

    impl FakeAdapter {
        fn answer(&self, id: u8, payload: Vec<u8>) {
            self.state.lock().unwrap().answers.push((id, payload));
        }

        fn sent(&self) -> Vec<Frame> {
            self.state.lock().unwrap().sent.clone()
        }

        fn clear_sent(&self) {
            self.state.lock().unwrap().sent.clear();
        }
    }

    impl CanBus for FakeAdapter {
        fn send(&mut self, frame: &Frame) -> can::Result<()> {
            let mut state = self.state.lock().unwrap();
            state.sent.push(frame.clone());
            if frame.id & 0x8000 != 0 {
                let destination = (frame.id & 0x7f) as u8;
                if let Some((_, payload)) = state
                    .answers
                    .iter()
                    .find(|(id, _)| *id == destination)
                    .cloned()
                {
                    // Replies come back in the order the questions went
                    // out, as they do from a real bus.
                    state.inbox.push_back(Frame {
                        id: (destination as u32) << 8,
                        data: payload,
                        ..Default::default()
                    });
                }
            }
            Ok(())
        }

        fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
            self.state
                .lock()
                .unwrap()
                .inbox
                .pop_front()
                .ok_or(can::Error::Timeout)
        }

        fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
            let mut state = self.state.lock().unwrap();
            state.drains += 1;
            state.inbox.clear();
            Ok(())
        }
    }

    /// A reply carrying mode, position and a fault code, as a controller
    /// would answer the fast query.
    fn reply(mode: i32, position_rev: f64, fault: i32) -> Vec<u8> {
        let mut writer = Writer::new();
        writer.push_header(REPLY_BASE, Resolution::Int8, 1, reg::MODE);
        writer.push_value(mode as f64, Resolution::Int8);
        writer.push_header(REPLY_BASE, Resolution::Int32, 1, reg::POSITION);
        // Int32 position counts: 1e-5 rev each.
        writer.push_value((position_rev * 100_000.0).round(), Resolution::Int32);
        writer.push_header(REPLY_BASE, Resolution::Int8, 1, reg::FAULT);
        writer.push_value(fault as f64, Resolution::Int8);
        writer.into_bytes()
    }

    fn drive(position: f64) -> AxisCmd {
        AxisCmd {
            command: Command {
                position: Field::Set(Rev(position)),
                velocity: Field::Set(RevPerSec(0.0)),
                velocity_limit: Field::Set(RevPerSec(2.0)),
                ..Command::new(Mode::Position)
            },
            profile: QueryProfile::Fast,
        }
    }

    fn opened(adapter: &FakeAdapter, ids: &[u8]) -> MoteusBus {
        for &id in ids {
            adapter.answer(id, reply(0, 0.0, 0));
        }
        let mut bus = MoteusBus::new(Arc::new(SharedBus::new(Box::new(adapter.clone()))), ids);
        let identities = bus.open().expect("every scripted controller answers");
        assert_eq!(identities.len(), ids.len());
        adapter.clear_sent();
        bus
    }

    /// The frame a joint receives is the same whichever path built it. The
    /// link's frames are pinned against the reference implementation, so
    /// this inherits that pin.
    #[test]
    fn the_bus_sends_the_frames_the_link_would() {
        let adapter = FakeAdapter::default();
        let mut bus = opened(&adapter, &[1, 2]);
        let cmds = [drive(0.25), drive(-0.5)];
        let mut out = [AxisReply::SILENT; 2];
        bus.cycle(&cmds, &mut out).unwrap();
        let from_bus = adapter.sent();
        assert_eq!(from_bus.len(), 2);

        for (index, cmd) in cmds.iter().enumerate() {
            let link_adapter = FakeAdapter::default();
            let shared = Arc::new(SharedBus::new(Box::new(link_adapter.clone())));
            let mut link = MoteusLink::new(shared, (index + 1) as u8);
            link.open().unwrap_or(()); // no reply scripted: the link still sends its stop
            link_adapter.clear_sent();
            let _ = link.cycle(&cmd.command, cmd.profile);
            let from_link = link_adapter.sent();
            assert_eq!(from_link.len(), 1);
            assert_eq!(from_bus[index].id, from_link[0].id, "axis {index}");
            assert_eq!(from_bus[index].data, from_link[0].data, "axis {index}");
            assert!(from_bus[index].fd && from_bus[index].brs);
        }
    }

    /// Replies are sorted to their axes by source id, whatever order they
    /// arrive in, and an axis that does not answer is silent rather than an
    /// error.
    #[test]
    fn replies_are_routed_by_source_id_and_silence_is_not_an_error() {
        let adapter = FakeAdapter::default();
        let mut bus = opened(&adapter, &[3, 1, 7]);
        // Re-script the answers: 1 and 7 answer, 3 goes quiet.
        {
            let mut state = adapter.state.lock().unwrap();
            state.answers.clear();
            state.answers.push((1, reply(10, 0.25, 0)));
            state.answers.push((7, reply(10, -0.5, 102)));
        }
        let cmds = [drive(0.0), drive(0.25), drive(-0.5)];
        let mut out = [AxisReply::SILENT; 3];
        bus.cycle(&cmds, &mut out).unwrap();

        assert_eq!(out[0].status, AxisStatus::Silent, "{:?}", out[0]);
        assert_eq!(out[0].position, None);
        assert_eq!(out[1].status, AxisStatus::Ok);
        assert!((out[1].position.unwrap().value() - 0.25).abs() < 1e-9);
        assert_eq!(out[1].mode, Some(10));
        assert_eq!(out[2].status, AxisStatus::Ok);
        assert!((out[2].position.unwrap().value() + 0.5).abs() < 1e-9);
        let fault = out[2].fault.unwrap();
        assert_eq!(fault.code, 102);
        assert!(!fault.hard, "limiting is information, not a fault");
    }

    /// One cycle is N sends, one drain before them, and then the receives:
    /// every axis answers, and the stale-frame drain happens once a cycle.
    #[test]
    fn a_cycle_drains_once_sends_every_frame_and_collects_every_reply() {
        let adapter = FakeAdapter::default();
        let mut bus = opened(&adapter, &[1, 2, 3]);
        let cmds = [drive(0.0); 3];
        let mut out = [AxisReply::SILENT; 3];
        bus.cycle(&cmds, &mut out).unwrap();
        assert_eq!(adapter.sent().len(), 3);
        assert!(out.iter().all(|r| r.status == AxisStatus::Ok));
        assert_eq!(
            adapter.state.lock().unwrap().drains,
            2,
            "one at open, one per cycle"
        );
        bus.cycle(&cmds, &mut out).unwrap();
        assert_eq!(adapter.state.lock().unwrap().drains, 3);
    }

    /// An id that does not answer the opening stop refuses the open by
    /// name, and a closed bus does not cycle.
    #[test]
    fn open_refuses_an_axis_that_does_not_answer() {
        let adapter = FakeAdapter::default();
        adapter.answer(1, reply(0, 0.0, 0));
        let mut bus = MoteusBus::new(
            Arc::new(SharedBus::new(Box::new(adapter.clone()))),
            &[1, 9],
        );
        let error = bus.open().unwrap_err();
        assert!(format!("{error}").contains("CAN id 9"), "{error}");
        let cmds = [drive(0.0); 2];
        let mut out = [AxisReply::SILENT; 2];
        assert!(matches!(bus.cycle(&cmds, &mut out), Err(Error::Closed)));
    }

    /// Opening stops every controller first, and closing stops them again.
    #[test]
    fn open_and_close_both_stop_every_controller() {
        let adapter = FakeAdapter::default();
        adapter.answer(1, reply(0, 0.0, 0));
        adapter.answer(2, reply(0, 0.0, 0));
        let mut bus = MoteusBus::new(
            Arc::new(SharedBus::new(Box::new(adapter.clone()))),
            &[1, 2],
        );
        bus.open().unwrap();
        let stops = adapter
            .sent()
            .iter()
            .filter(|f| f.data.starts_with(&frames::stop(None)))
            .count();
        assert!(stops >= 2, "{} stops at open", stops);

        adapter.clear_sent();
        bus.close().unwrap();
        let sent = adapter.sent();
        assert_eq!(sent.len(), 2);
        assert!(sent.iter().all(|f| f.data == frames::stop(None)));
        assert!(sent.iter().all(|f| f.id & 0x8000 == 0), "no reply asked");
    }

    /// A query-only command still asks, and a mode that writes nothing and
    /// asks nothing sends no frame at all.
    #[test]
    fn a_query_is_a_frame_with_no_command_in_it() {
        let adapter = FakeAdapter::default();
        let mut bus = opened(&adapter, &[1]);
        let cmds = [AxisCmd {
            command: Command::new(Mode::Query),
            profile: QueryProfile::Fast,
        }];
        let mut out = [AxisReply::SILENT; 1];
        bus.cycle(&cmds, &mut out).unwrap();
        let sent = adapter.sent();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].data, crate::query::QueryResolution::fast().encode().0);
        assert_eq!(out[0].status, AxisStatus::Ok);
    }

    /// A transport failure is the bus's error, not an axis's silence.
    #[test]
    fn a_transport_failure_is_an_error() {
        struct Dead;
        impl CanBus for Dead {
            fn send(&mut self, _frame: &Frame) -> can::Result<()> {
                Err(can::Error::Closed)
            }
            fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
                Err(can::Error::Closed)
            }
            fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
                Ok(())
            }
        }
        let mut bus = MoteusBus::new(Arc::new(SharedBus::new(Box::new(Dead))), &[1]);
        bus.open = true; // bypass the open, which would also fail
        let cmds = [drive(0.0)];
        let mut out = [AxisReply::SILENT; 1];
        assert!(matches!(
            bus.cycle(&cmds, &mut out),
            Err(Error::Transport { .. })
        ));
    }
}
