//! One RobStride actuator on a CAN bus.
//!
//! The controller owns framing and addressing, not timing: a caller drives it
//! from its own control loop, and every method performs one bus transaction and
//! returns.

use std::time::{Duration, Instant};

use drlabs_core::error::{Error, Result};
use drlabs_transport::can::{self, Frame, SharedBus};

use crate::models::Model;
use crate::parameters::{
    decode_param_reply, encode_param_read, encode_param_write, Parameter, RunMode,
};
use crate::protocol::{
    arbitration_id, decode_fault_report, decode_feedback, encode_control, parse_arbitration_id,
    CommType, ControlCommand, FaultReport, Feedback, BROADCAST_REPLY_ADDRESS, DEFAULT_HOST_ID,
};

/// The transaction timeout a controller starts with.
///
/// RobStride replies to a command within a millisecond or so on a healthy
/// 1 Mbit/s bus; this is generous enough to survive an adapter hiccup and short
/// enough that a wedged bus is noticed within one control period at modest
/// rates.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// A rule for recognising the reply to a request.
///
/// A closure rather than a fixed match, because "which frame is mine" differs
/// per command: a feedback frame comes back addressed to this host, and a
/// device-id reply comes back to a fixed broadcast address instead.
type Accept<'a> = &'a dyn Fn(u32, &[u8]) -> bool;

/// One RobStride actuator.
pub struct Controller {
    /// The motor's CAN address, 1..127.
    pub id: u8,
    /// The address this host answers to. Replies are addressed back to it, so
    /// two programs sharing a bus need different values.
    pub host_id: u8,
    /// The per-variant scaling. Not optional: the wrong model mis-scales every
    /// torque command and every torque reading.
    pub model: Model,
    /// Bounds a single transaction.
    pub timeout: Duration,
}

impl Controller {
    pub fn new(id: u8, model: Model) -> Self {
        Self {
            id,
            host_id: DEFAULT_HOST_ID,
            model,
            timeout: DEFAULT_TIMEOUT,
        }
    }

    /// Transmits one frame and waits for a reply that `accept` recognises.
    ///
    /// `Ok(None)` means nothing answered in time — a dropped cycle rather than
    /// a transport failure, which is what a noisy bus produces occasionally and
    /// what a control loop should ride out.
    fn send(
        &self,
        bus: &SharedBus,
        id: u32,
        data: &[u8],
        accept: Option<Accept<'_>>,
    ) -> Result<Option<(u32, Vec<u8>)>> {
        // Every frame is eight bytes, whatever the payload said.
        let mut payload = vec![0u8; 8];
        let n = data.len().min(8);
        payload[..n].copy_from_slice(&data[..n]);

        let frame = Frame {
            id,
            data: payload,
            extended: true,
            // RobStride speaks classic CAN 2.0B, not CAN-FD. Setting the FD
            // flag would make an adapter transmit a frame the motor cannot
            // receive — and the symptom is a motor that never answers, not an
            // error anybody reports.
            fd: false,
            brs: false,
        };

        // A send and its reply are one transaction: on a shared bus, letting
        // two interleave means each reads the other's answer and both time out.
        let mut bus = bus.lock();
        bus.send(&frame).map_err(|error| Error::Transport {
            operation: "send",
            source: Box::new(error),
        })?;

        let accept = match accept {
            Some(accept) => accept,
            None => return Ok(None),
        };

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
            if accept(reply.id, &reply.data) {
                return Ok(Some((reply.id, reply.data)));
            }
        }
    }

    /// Recognises a feedback frame from *this* motor addressed to *this* host.
    fn expect_feedback(&self) -> impl Fn(u32, &[u8]) -> bool + '_ {
        self.expect_type(CommType::Feedback)
    }

    fn expect_type(&self, want: CommType) -> impl Fn(u32, &[u8]) -> bool + '_ {
        move |id, data| {
            let (comm_type, data_area2, address) = parse_arbitration_id(id);
            comm_type == Some(want)
                && (data_area2 & 0xff) as u8 == self.id
                && address == self.host_id
                && data.len() >= 8
        }
    }

    fn feedback_from(&self, reply: Option<(u32, Vec<u8>)>) -> Result<Option<Feedback>> {
        let (id, data) = match reply {
            Some(reply) => reply,
            None => return Ok(None),
        };
        decode_feedback(id, &data, &self.model)
            .ok_or_else(|| {
                Error::Malformed(format!(
                    "feedback from motor {} carried {} bytes, want 8",
                    self.id,
                    data.len()
                ))
            })
            .map(Some)
    }

    /// Puts the motor into run mode.
    pub fn enable(&self, bus: &SharedBus) -> Result<Option<Feedback>> {
        let accept = self.expect_feedback();
        let reply = self.send(
            bus,
            arbitration_id(CommType::Enable, self.host_id as u16, self.id),
            &[],
            Some(&accept),
        )?;
        self.feedback_from(reply)
    }

    /// Takes it out of run mode, optionally clearing latched faults.
    pub fn stop(&self, bus: &SharedBus, clear_faults: bool) -> Result<Option<Feedback>> {
        let mut data = vec![0u8; 8];
        if clear_faults {
            data[0] = 1;
        }
        let accept = self.expect_feedback();
        let reply = self.send(
            bus,
            arbitration_id(CommType::Stop, self.host_id as u16, self.id),
            &data,
            Some(&accept),
        )?;
        self.feedback_from(reply)
    }

    /// One operation-control frame.
    pub fn control(&self, bus: &SharedBus, command: &ControlCommand) -> Result<Option<Feedback>> {
        let (id, data) = encode_control(command, &self.model, self.id);
        let accept = self.expect_feedback();
        let reply = self.send(bus, id, &data, Some(&accept))?;
        self.feedback_from(reply)
    }

    /// Declares the current shaft position to be mechanical zero.
    pub fn set_mechanical_zero(&self, bus: &SharedBus) -> Result<Option<Feedback>> {
        let mut data = vec![0u8; 8];
        data[0] = 1;
        let accept = self.expect_feedback();
        let reply = self.send(
            bus,
            arbitration_id(CommType::SetZero, self.host_id as u16, self.id),
            &data,
            Some(&accept),
        )?;
        self.feedback_from(reply)
    }

    /// Writes one parameter. The write is volatile until [`Controller::save_parameters`].
    pub fn write_parameter(
        &self,
        bus: &SharedBus,
        index: Parameter,
        value: f64,
    ) -> Result<Option<Feedback>> {
        let data =
            encode_param_write(index, value).map_err(|error| Error::Config(error.to_string()))?;
        let accept = self.expect_feedback();
        let reply = self.send(
            bus,
            arbitration_id(CommType::WriteParam, self.host_id as u16, self.id),
            &data,
            Some(&accept),
        )?;
        self.feedback_from(reply)
    }

    /// Reads one parameter.
    ///
    /// A reply carrying a different index means the bus delivered someone
    /// else's answer, and using it would put another parameter's value into
    /// this one's variable — so it is an error rather than a value.
    pub fn read_parameter(&self, bus: &SharedBus, index: Parameter) -> Result<Option<f64>> {
        let accept = self.expect_type(CommType::ReadParam);
        let reply = self.send(
            bus,
            arbitration_id(CommType::ReadParam, self.host_id as u16, self.id),
            &encode_param_read(index),
            Some(&accept),
        )?;
        let (_, data) = match reply {
            Some(reply) => reply,
            None => return Ok(None),
        };

        let (got, value) = decode_param_reply(&data)
            .ok_or_else(|| Error::Malformed("parameter reply was short".into()))?;
        if got != index {
            return Err(Error::Malformed(format!(
                "asked for parameter {index}, motor answered with {got}"
            )));
        }
        Ok(Some(value))
    }

    pub fn set_run_mode(&self, bus: &SharedBus, mode: RunMode) -> Result<Option<Feedback>> {
        self.write_parameter(bus, Parameter::RUN_MODE, mode.code() as f64)
    }

    /// Commits the volatile parameters so they survive a power cycle.
    pub fn save_parameters(&self, bus: &SharedBus) -> Result<Option<Feedback>> {
        let accept = self.expect_feedback();
        let reply = self.send(
            bus,
            arbitration_id(CommType::SaveParams, self.host_id as u16, self.id),
            &[1, 2, 3, 4, 5, 6, 7, 8],
            Some(&accept),
        )?;
        self.feedback_from(reply)
    }

    /// The 64-bit MCU unique identifier, as hex.
    pub fn device_id(&self, bus: &SharedBus) -> Result<Option<String>> {
        // A device-id reply goes to a fixed broadcast address rather than to
        // this host, so it needs its own acceptance rule.
        let accept = |id: u32, data: &[u8]| {
            let (comm_type, data_area2, address) = parse_arbitration_id(id);
            comm_type == Some(CommType::GetDeviceId)
                && (data_area2 & 0xff) as u8 == self.id
                && address == BROADCAST_REPLY_ADDRESS
                && data.len() >= 8
        };
        let reply = self.send(
            bus,
            arbitration_id(CommType::GetDeviceId, self.host_id as u16, self.id),
            &[],
            Some(&accept),
        )?;
        Ok(reply.map(|(_, data)| data[..8].iter().map(|byte| format!("{byte:02x}")).collect()))
    }

    /// Changes the motor's own address. It takes effect immediately, so the
    /// controller's own id moves with it.
    pub fn set_can_id(&mut self, bus: &SharedBus, new_id: u8) -> Result<()> {
        if !(1..=127).contains(&new_id) {
            return Err(Error::Config(format!(
                "CAN id {new_id} is out of range (1..127)"
            )));
        }
        let data_area2 = self.host_id as u16 | (new_id as u16) << 8;
        let accept = |id: u32, _: &[u8]| {
            let (comm_type, _, address) = parse_arbitration_id(id);
            comm_type == Some(CommType::GetDeviceId) && address == BROADCAST_REPLY_ADDRESS
        };
        self.send(
            bus,
            arbitration_id(CommType::SetCanId, data_area2, self.id),
            &[],
            Some(&accept),
        )?;
        self.id = new_id;
        Ok(())
    }

    /// The full fault and warning words.
    pub fn fault_report(&self, bus: &SharedBus) -> Result<Option<FaultReport>> {
        let accept = self.expect_type(CommType::FaultFeedback);
        let reply = self.send(
            bus,
            arbitration_id(CommType::FaultFeedback, self.host_id as u16, self.id),
            &[],
            Some(&accept),
        )?;
        let (_, data) = match reply {
            Some(reply) => reply,
            None => return Ok(None),
        };
        decode_fault_report(&data)
            .ok_or_else(|| Error::Malformed("fault frame was short".into()))
            .map(Some)
    }

    /// Turns the motor's unsolicited reporting on or off.
    ///
    /// A motor that reports on its own fills the same bus the control loop
    /// needs, and its frames arrive where a reply is expected.
    pub fn set_active_reporting(&self, bus: &SharedBus, enabled: bool) -> Result<()> {
        let mut data = vec![1, 2, 3, 4, 5, 6, 0, 0];
        if enabled {
            data[6] = 1;
        }
        self.send(
            bus,
            arbitration_id(CommType::SetReporting, self.host_id as u16, self.id),
            &data,
            None,
        )?;
        Ok(())
    }
}

impl std::fmt::Display for Controller {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "robstride[{} id={}]", self.model.name, self.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::RS01;
    use crate::protocol::POSITION_RANGE;
    use std::sync::{Arc, Mutex};

    /// A motor made of a script.
    #[derive(Clone, Default)]
    struct FakeMotor {
        sent: Arc<Mutex<Vec<Frame>>>,
        replies: Arc<Mutex<Vec<Frame>>>,
    }

    impl FakeMotor {
        fn will_reply(&self, comm_type: CommType, motor_id: u8, data: Vec<u8>) {
            self.replies.lock().unwrap().push(Frame {
                id: arbitration_id(comm_type, motor_id as u16, DEFAULT_HOST_ID),
                data,
                extended: true,
                fd: false,
                brs: false,
            });
        }

        fn last_sent(&self) -> Frame {
            self.sent.lock().unwrap().last().cloned().expect("a frame")
        }
    }

    impl can::Bus for FakeMotor {
        fn send(&mut self, frame: &Frame) -> can::Result<()> {
            self.sent.lock().unwrap().push(frame.clone());
            Ok(())
        }
        fn recv(&mut self, _timeout: Duration) -> can::Result<Frame> {
            let mut replies = self.replies.lock().unwrap();
            if replies.is_empty() {
                return Err(can::Error::Timeout);
            }
            Ok(replies.remove(0))
        }
        fn drain(&mut self, _quiet: Duration) -> can::Result<()> {
            Ok(())
        }
    }

    fn fixture() -> (Controller, SharedBus, FakeMotor) {
        let motor = FakeMotor::default();
        let bus = SharedBus::new(Box::new(motor.clone()));
        let mut controller = Controller::new(1, RS01);
        controller.timeout = Duration::from_millis(20);
        (controller, bus, motor)
    }

    fn feedback_payload() -> Vec<u8> {
        let mut data = vec![0u8; 8];
        data[0..2]
            .copy_from_slice(&crate::protocol::encode_uint16(0.0, POSITION_RANGE).to_be_bytes());
        data[6..8].copy_from_slice(&250i16.to_be_bytes());
        data
    }

    /// RobStride is classic CAN. An FD frame is one the motor cannot receive,
    /// and the symptom is silence rather than an error.
    #[test]
    fn every_frame_is_classic_can_with_an_extended_identifier() {
        let (controller, bus, motor) = fixture();
        motor.will_reply(CommType::Feedback, 1, feedback_payload());
        controller.enable(&bus).unwrap();

        let frame = motor.last_sent();
        assert!(frame.extended);
        assert!(!frame.fd, "RobStride does not speak CAN-FD");
        assert!(!frame.brs);
        assert_eq!(frame.data.len(), 8, "always a full eight bytes");
    }

    #[test]
    fn enable_asks_for_run_mode_and_reads_the_feedback() {
        let (controller, bus, motor) = fixture();
        motor.will_reply(CommType::Feedback, 1, feedback_payload());

        let feedback = controller.enable(&bus).unwrap().expect("a reply");
        assert_eq!(motor.last_sent().id >> 24, CommType::Enable as u32);
        assert!((feedback.temperature - 25.0).abs() < 1e-9);
    }

    #[test]
    fn stop_can_clear_latched_faults() {
        let (controller, bus, motor) = fixture();
        motor.will_reply(CommType::Feedback, 1, feedback_payload());
        controller.stop(&bus, true).unwrap();
        assert_eq!(motor.last_sent().data[0], 1);

        motor.will_reply(CommType::Feedback, 1, feedback_payload());
        controller.stop(&bus, false).unwrap();
        assert_eq!(motor.last_sent().data[0], 0);
    }

    /// A frame addressed to somebody else is not this motor's answer. On a
    /// shared bus that is the difference between reading your own telemetry and
    /// reading the axis next door's.
    #[test]
    fn a_reply_from_another_motor_is_ignored() {
        let (controller, bus, motor) = fixture();
        motor.will_reply(CommType::Feedback, 2, feedback_payload()); // not us
        motor.will_reply(CommType::Feedback, 1, feedback_payload());

        let feedback = controller.enable(&bus).unwrap().expect("our own reply");
        assert_eq!(feedback.motor_id, 1);
    }

    /// A motor that says nothing is a dropped cycle, not a failure.
    #[test]
    fn silence_is_a_missed_cycle_rather_than_an_error() {
        let (controller, bus, _motor) = fixture();
        assert_eq!(controller.enable(&bus).unwrap(), None);
    }

    /// The check that stops one parameter's value landing in another's
    /// variable.
    #[test]
    fn a_parameter_reply_for_the_wrong_index_is_refused() {
        let (controller, bus, motor) = fixture();
        // Asked for loc_ref, answered about limit_torque.
        let mut data = vec![0u8; 8];
        data[0..2].copy_from_slice(&Parameter::LIMIT_TORQUE.0.to_le_bytes());
        data[4..8].copy_from_slice(&17.0f32.to_le_bytes());
        motor.will_reply(CommType::ReadParam, 1, data);

        let error = controller
            .read_parameter(&bus, Parameter::LOC_REF)
            .unwrap_err();
        let text = format!("{error}");
        assert!(text.contains("loc_ref"), "{text}");
        assert!(text.contains("limit_torque"), "{text}");
    }

    #[test]
    fn a_parameter_read_returns_the_value_it_asked_for() {
        let (controller, bus, motor) = fixture();
        let mut data = vec![0u8; 8];
        data[0..2].copy_from_slice(&Parameter::VBUS.0.to_le_bytes());
        data[4..8].copy_from_slice(&48.2f32.to_le_bytes());
        motor.will_reply(CommType::ReadParam, 1, data);

        let value = controller
            .read_parameter(&bus, Parameter::VBUS)
            .unwrap()
            .expect("a value");
        assert!((value - 48.2).abs() < 1e-5);
    }

    /// Changing the address takes effect immediately, so the controller has to
    /// move with it or every subsequent frame goes to an id nothing answers.
    #[test]
    fn setting_the_can_id_moves_this_controller_too() {
        let (mut controller, bus, motor) = fixture();
        motor.replies.lock().unwrap().push(Frame {
            id: arbitration_id(CommType::GetDeviceId, 1, BROADCAST_REPLY_ADDRESS),
            data: vec![0u8; 8],
            extended: true,
            ..Default::default()
        });

        controller.set_can_id(&bus, 5).unwrap();
        assert_eq!(controller.id, 5);
    }

    #[test]
    fn an_out_of_range_can_id_never_reaches_the_wire() {
        let (mut controller, bus, motor) = fixture();
        assert!(controller.set_can_id(&bus, 0).is_err());
        assert!(controller.set_can_id(&bus, 200).is_err());
        assert_eq!(controller.id, 1);
        assert!(motor.sent.lock().unwrap().is_empty());
    }

    /// Turning off active reporting is fire-and-forget: there is nothing to
    /// wait for, and waiting would cost a timeout on every connect.
    #[test]
    fn active_reporting_is_set_without_expecting_an_answer() {
        let (controller, bus, motor) = fixture();
        let started = Instant::now();
        controller.set_active_reporting(&bus, false).unwrap();
        assert!(started.elapsed() < Duration::from_millis(10));
        assert_eq!(motor.last_sent().data[6], 0);
    }
}
