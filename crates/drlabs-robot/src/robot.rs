//! A machine: several joints, several buses, one description.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use drlabs_core::command::ControlMode;
use drlabs_core::error::{Error, Result};
use drlabs_core::link::Link;
use drlabs_core::units::{Nm, Rad, RadPerSec, RadPerSec2};
use drlabs_dynamixel::DynamixelLink;
use drlabs_moteus::link::MoteusLink;
use drlabs_odrive::OdriveLink;
use drlabs_robstride::RobstrideLink;
use drlabs_safety::Config as SafetyConfig;
use drlabs_session::{Options, Session};
use drlabs_transport::can::SharedBus;
use drlabs_transport::serial::SharedPort;

use crate::config::{BusConfig, Config, JointConfig};
use crate::joint::Mapping;
use crate::units::JointUnits;

/// A bus that has been opened, whatever kind it is.
enum Bus {
    Can(Arc<SharedBus>),
    Serial(SharedPort),
}

/// One named degree of freedom.
pub struct Joint {
    pub name: String,
    pub config: JointConfig,
    pub mapping: Mapping,

    /// This joint's control loop.
    ///
    /// Exposed because everything the framework can do to one axis is already
    /// on it, and wrapping all of it again would only hide capability.
    pub session: Arc<Session>,
}

/// What one joint is doing, in joint space.
#[derive(Debug, Clone, PartialEq)]
pub struct JointState {
    pub name: String,

    pub armed: bool,
    pub mode: ControlMode,
    pub estop: bool,
    /// Empty unless the stop is latched.
    pub estop_reason: String,

    pub position_rad: Option<f64>,
    pub velocity_rad_s: Option<f64>,
    pub torque_nm: Option<f64>,
    pub current_a: Option<f64>,
    pub voltage_v: Option<f64>,
    pub temperature_c: Option<f64>,

    pub target_rad: f64,

    pub fault: Option<i32>,
    pub fault_name: String,

    pub moving_to_rad: Option<f64>,
    pub progress: f64,

    pub rate_hz: f64,

    /// The preflight reasons, while a joint is not armed.
    pub blocked: Vec<String>,
}

/// A machine built from a description.
pub struct Robot {
    pub config: Config,
    joints: Vec<Joint>,
    /// Opened buses, in the order they were opened, so they can be closed in
    /// reverse once every joint has let go.
    buses: Vec<(String, Bus)>,
}

impl Robot {
    /// Builds a robot from a description, without touching any hardware.
    pub fn new(mut config: Config) -> Result<Self> {
        config.validate().map_err(|error| Error::Config(error.0))?;
        Ok(Self {
            config,
            joints: Vec::new(),
            buses: Vec::new(),
        })
    }

    /// Opens every bus, connects every joint, and energises the machine.
    ///
    /// All or nothing. A humanoid with one dead shoulder is more dangerous than
    /// one that never came up: the rest of the machine will happily hold poses
    /// that assume the missing joint is holding too. So the first joint that
    /// cannot be brought up takes the whole robot down with it, torque off, and
    /// says which one and why.
    pub fn bring_up(&mut self) -> Result<()> {
        if !self.joints.is_empty() {
            return Err(Error::Config("the robot is already brought up".into()));
        }
        match self.bring_up_inner() {
            Ok(()) => Ok(()),
            Err(error) => {
                // Unwind whatever came up before the failure.
                self.shutdown();
                Err(error)
            }
        }
    }

    fn bring_up_inner(&mut self) -> Result<()> {
        let base = SafetyConfig::default();
        let joints = self.config.joints.clone();

        for config in joints {
            let mapping = Mapping::of(&config);
            let safety = mapping.safety_config(&config.limits, base);
            let rate = self.config.rate_for_bus(&config.bus);

            let link = self
                .link_for(&config)
                .map_err(|error| Error::Config(format!("joint {:?}: {error}", config.name)))?;

            let session = Session::new(Options {
                safety,
                units: Some(Arc::new(JointUnits::new(mapping))),
                rate_hz: rate,
                // A robot's joints are commanded by a program: there is nobody
                // to press Arm, and nobody to acknowledge an advisory.
                auto_arm: true,
                require_acknowledgement: false,
                initial_mode: ControlMode::Position,
                ..Default::default()
            });

            session
                .connect(link, true)
                .map_err(|error| Error::Config(format!("joint {:?}: {error}", config.name)))?;

            self.joints.push(Joint {
                name: config.name.clone(),
                config,
                mapping,
                session,
            });
        }

        self.wait_until_armed(Duration::from_secs(5))
    }

    /// Waits for every joint to energise, and says which ones did not.
    fn wait_until_armed(&self, budget: Duration) -> Result<()> {
        let deadline = Instant::now() + budget;
        loop {
            let blocked: Vec<String> = self
                .joints
                .iter()
                .filter(|joint| !joint.session.status().armed)
                .map(|joint| {
                    let preflight = joint.session.preflight();
                    format!("{} ({})", joint.name, preflight.blocking.join(" "))
                })
                .collect();

            if blocked.is_empty() {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(Error::Refused(format!(
                    "joints did not energise: {}",
                    blocked.join(", ")
                )));
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Builds the link for one joint, opening its bus if this is the first
    /// joint on it.
    fn link_for(&mut self, config: &JointConfig) -> std::result::Result<Box<dyn Link>, String> {
        let bus = self
            .config
            .bus(&config.bus)
            .cloned()
            .unwrap_or_else(|| BusConfig {
                name: config.bus.clone(),
                transport: default_transport(&config.driver).into(),
                port: String::new(),
                baud: 0,
                rate_hz: 0,
            });

        match config.driver.as_str() {
            "moteus" => {
                let shared = self.can_bus(&bus)?;
                Ok(Box::new(MoteusLink::new(shared, config.id as u8)))
            }
            "robstride" => {
                let name = config.options.get("model").ok_or(
                    "robstride needs a model option; the scaling differs by up to a \
                            factor of twenty across the family",
                )?;
                let model = drlabs_robstride::lookup_model(name)
                    .ok_or_else(|| format!("unknown robstride model {name:?}"))?;
                let shared = self.can_bus(&bus)?;
                Ok(Box::new(RobstrideLink::new(shared, config.id as u8, model)))
            }
            "odrive" => {
                let shared = self.can_bus(&bus)?;
                Ok(Box::new(OdriveLink::new(shared, config.id as u8)))
            }
            "dynamixel" => {
                let port = self.serial_port(&bus)?;
                let profile = match config.options.get("profile") {
                    Some(name) => Some(
                        drlabs_dynamixel::profile::profile_named(name)
                            .ok_or_else(|| format!("unknown dynamixel profile {name:?}"))?,
                    ),
                    // Undeclared: the servo's own model number chooses.
                    None => None,
                };
                let baud = if bus.baud == 0 { 1_000_000 } else { bus.baud };
                Ok(Box::new(
                    DynamixelLink::new(port, config.id as u8, profile)
                        .with_port_name(&bus.port, baud),
                ))
            }
            // A simulated joint needs no bus at all, which is what lets a
            // whole machine be brought up and exercised on a laptop.
            "sim" | "moteus-sim" | "robstride-sim" => {
                Ok(Box::new(drlabs_sim::SimLink::new(config.id as u8)))
            }
            other => Err(format!("unknown driver {other:?}")),
        }
    }

    fn can_bus(&mut self, config: &BusConfig) -> std::result::Result<Arc<SharedBus>, String> {
        if let Some((_, bus)) = self.buses.iter().find(|(name, _)| name == &config.name) {
            return match bus {
                Bus::Can(shared) => Ok(shared.clone()),
                Bus::Serial(_) => Err(format!(
                    "bus {:?} is already open as a serial port",
                    config.name
                )),
            };
        }
        if config.transport != "fdcanusb" {
            return Err(format!(
                "bus {:?} has transport {:?}; a CAN driver needs fdcanusb",
                config.name, config.transport
            ));
        }

        let port = if config.port.is_empty() {
            let detected =
                drlabs_transport::fdcanusb::Device::detect().map_err(|error| error.to_string())?;
            match detected.first() {
                Some(port) if port.is_can_adapter => port.path.clone(),
                _ => {
                    return Err(format!(
                        "bus {:?} has no port and no CAN adapter was recognised",
                        config.name
                    ))
                }
            }
        } else {
            config.port.clone()
        };

        let device = drlabs_transport::fdcanusb::open(&port).map_err(|error| error.to_string())?;
        let shared = Arc::new(SharedBus::new(Box::new(device)));
        self.buses
            .push((config.name.clone(), Bus::Can(shared.clone())));
        Ok(shared)
    }

    fn serial_port(&mut self, config: &BusConfig) -> std::result::Result<SharedPort, String> {
        if let Some((_, bus)) = self.buses.iter().find(|(name, _)| name == &config.name) {
            return match bus {
                Bus::Serial(shared) => Ok(shared.clone()),
                Bus::Can(_) => Err(format!(
                    "bus {:?} is already open as a CAN bus",
                    config.name
                )),
            };
        }
        if config.transport != "serial" {
            return Err(format!(
                "bus {:?} has transport {:?}; a Dynamixel chain needs serial",
                config.name, config.transport
            ));
        }
        if config.port.is_empty() {
            return Err(format!("bus {:?} has no port", config.name));
        }

        let baud = if config.baud == 0 {
            1_000_000
        } else {
            config.baud
        };
        let port = drlabs_transport::serial::open(&config.port, baud)
            .map_err(|error| error.to_string())?;
        let shared = SharedPort::new(Box::new(port));
        self.buses
            .push((config.name.clone(), Bus::Serial(shared.clone())));
        Ok(shared)
    }

    /// Stops every joint and releases every bus.
    ///
    /// Joints first, buses second, and buses in reverse order of opening: a
    /// port closed while a session is still writing to it turns a clean stop
    /// into a transport error on the way out.
    pub fn shutdown(&mut self) {
        for joint in &self.joints {
            joint.session.shutdown();
        }
        self.joints.clear();
        while self.buses.pop().is_some() {}
    }

    pub fn joint(&self, name: &str) -> Result<&Joint> {
        self.joints
            .iter()
            .find(|joint| joint.name == name)
            .ok_or_else(|| Error::Config(format!("no joint named {name:?}")))
    }

    pub fn joint_names(&self) -> Vec<&str> {
        self.joints
            .iter()
            .map(|joint| joint.name.as_str())
            .collect()
    }

    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    /// Commands one joint, in joint space.
    pub fn set_position(&self, name: &str, radians: Rad) -> Result<Rad> {
        let joint = self.joint(name)?;
        joint.session.set_mode(ControlMode::Position)?;
        let commanded = joint
            .session
            .set_position(joint.mapping.motor_position(radians));
        Ok(joint.mapping.joint_position(commanded))
    }

    /// Commands several joints at once.
    ///
    /// The joints are commanded as fast as their loops pick them up rather than
    /// simultaneously — each runs its own cycle — so this is coordinated to
    /// within a control period, not to the microsecond. For a pose that is
    /// right; anything needing true synchrony would have to share a loop.
    ///
    /// Everything is checked before anything is commanded, and the joints are
    /// visited in the order they were declared.
    ///
    /// Both halves matter. Iterating a map applies a pose in whatever order the
    /// hash felt like, so a joint that refuses leaves a different subset of the
    /// machine moved every time — the same command, the same state, a different
    /// posture. And a posture assembled out of the joints that happened to be
    /// reached first is not the pose anybody asked for.
    pub fn set_positions(&self, targets: &BTreeMap<String, Rad>) -> Result<()> {
        // Every name has to exist…
        for name in targets.keys() {
            self.joint(name)?;
        }
        // …and every joint has to be able to take the command, before any of
        // them does.
        let ordered: Vec<&Joint> = self
            .joints
            .iter()
            .filter(|joint| targets.contains_key(&joint.name))
            .collect();

        for joint in &ordered {
            joint
                .session
                .can_command(ControlMode::Position)
                .map_err(|error| {
                    Error::Refused(format!(
                        "joint {:?}: {error} (the pose was not applied)",
                        joint.name
                    ))
                })?;
        }

        for joint in ordered {
            let target = targets[&joint.name];
            joint.session.set_mode(ControlMode::Position)?;
            joint
                .session
                .set_position(joint.mapping.motor_position(target));
        }
        Ok(())
    }

    pub fn set_velocity(&self, name: &str, radians_per_second: RadPerSec) -> Result<()> {
        let joint = self.joint(name)?;
        joint.session.set_mode(ControlMode::Velocity)?;
        joint
            .session
            .set_velocity(joint.mapping.motor_velocity(radians_per_second));
        Ok(())
    }

    /// A host-profiled move on one joint.
    pub fn move_to(
        &self,
        name: &str,
        radians: Rad,
        max_velocity: RadPerSec,
        accel: RadPerSec2,
    ) -> Result<()> {
        let joint = self.joint(name)?;
        joint.session.start_trajectory(
            joint.mapping.motor_position(radians),
            joint.mapping.motor_velocity(max_velocity).abs(),
            joint.mapping.motor_accel(accel).abs(),
        )
    }

    pub fn hold(&self, name: &str) -> Result<()> {
        self.joint(name)?.session.set_mode(ControlMode::Hold)
    }

    pub fn stop(&self, name: &str) -> Result<()> {
        let joint = self.joint(name)?;
        joint.session.disarm("stopped");
        Ok(())
    }

    /// Stops the whole machine.
    ///
    /// Every joint, whatever any of them says about it: a machine half stopped
    /// is not stopped.
    pub fn estop(&self, reason: &str) {
        for joint in &self.joints {
            joint.session.estop("robot_estop", "Emergency stop", reason);
        }
    }

    pub fn reset_estop(&self) -> Result<()> {
        let mut failures = Vec::new();
        for joint in &self.joints {
            if let Err(error) = joint.session.reset_estop() {
                failures.push(format!("{}: {error}", joint.name));
            }
        }
        if failures.is_empty() {
            Ok(())
        } else {
            Err(Error::Refused(failures.join(", ")))
        }
    }

    /// Tells every joint's deadman the supervisor is still there.
    pub fn heartbeat(&self) {
        for joint in &self.joints {
            joint.session.heartbeat();
        }
    }

    /// What the whole machine is doing, in joint space.
    pub fn state(&self) -> Vec<JointState> {
        self.joints
            .iter()
            .map(|joint| {
                let status = joint.session.status();
                let mapping = joint.mapping;
                let telemetry = &status.telemetry;

                let moving_to = status
                    .trajectory
                    .running
                    .then(|| mapping.joint_position(status.trajectory.target).value());

                JointState {
                    name: joint.name.clone(),
                    armed: status.armed,
                    mode: status.mode,
                    estop: status.estop.is_some(),
                    estop_reason: status
                        .estop
                        .as_ref()
                        .map(|estop| estop.message.clone())
                        .unwrap_or_default(),

                    position_rad: telemetry
                        .position
                        .map(|value| mapping.joint_position(value).value()),
                    velocity_rad_s: telemetry
                        .velocity
                        .map(|value| mapping.joint_velocity(value).value()),
                    torque_nm: telemetry
                        .torque
                        .map(|value| mapping.joint_torque(value).value()),
                    current_a: telemetry.q_current.map(|value| value.value()),
                    voltage_v: telemetry.voltage.map(|value| value.value()),
                    temperature_c: telemetry.board_temperature.map(|value| value.value()),

                    target_rad: mapping.joint_position(status.setpoint.position).value(),

                    fault: telemetry.fault.as_ref().map(|fault| fault.code),
                    fault_name: telemetry
                        .fault
                        .as_ref()
                        .map(|fault| fault.name.clone())
                        .unwrap_or_default(),

                    moving_to_rad: moving_to,
                    progress: status.trajectory.progress,
                    rate_hz: status.achieved_rate_hz,
                    blocked: if status.armed {
                        Vec::new()
                    } else {
                        joint.session.preflight().blocking
                    },
                }
            })
            .collect()
    }

    /// The torque ceiling each joint is running with, at the mechanism.
    pub fn torque_limits(&self) -> Vec<(String, Nm)> {
        self.joints
            .iter()
            .map(|joint| {
                let safety = joint.session.safety();
                (
                    joint.name.clone(),
                    joint.mapping.joint_torque(safety.max_torque),
                )
            })
            .collect()
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        // A dropped robot must not leave a machine energised.
        self.shutdown();
    }
}

/// What transport a driver needs when the description did not say.
fn default_transport(driver: &str) -> &'static str {
    match driver {
        "dynamixel" => "serial",
        _ => "fdcanusb",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ONE_JOINT: &str = r#"{
      "joints": [{
        "name": "axis", "driver": "moteus", "id": 1,
        "gear_ratio": 19.0, "direction": 1.0,
        "limits": {"min_rad": -0.5, "max_rad": 0.5, "max_torque_nm": 19.0,
                   "max_velocity_rad_s": 3.0}
      }]
    }"#;

    /// Building a robot touches no hardware, which is what makes `bringup
    /// -check` possible.
    #[test]
    fn building_a_robot_opens_nothing() {
        let config = Config::parse(ONE_JOINT).unwrap();
        let robot = Robot::new(config).expect("a valid description builds");
        assert!(robot.joints.is_empty(), "nothing is open until bring-up");
        assert_eq!(robot.config.joints[0].name, "axis");
    }

    #[test]
    fn an_invalid_description_is_refused_before_anything_opens() {
        let config = Config {
            robot: String::new(),
            rate_hz: 0,
            buses: Vec::new(),
            joints: Vec::new(),
        };
        assert!(Robot::new(config).is_err());
    }

    /// The driver decides which kind of bus an undeclared one is.
    #[test]
    fn a_driver_implies_its_transport() {
        assert_eq!(default_transport("dynamixel"), "serial");
        assert_eq!(default_transport("moteus"), "fdcanusb");
        assert_eq!(default_transport("robstride"), "fdcanusb");
    }

    /// The joint's declared envelope reaches the session as a motor-space one.
    #[test]
    fn a_joints_limits_become_its_sessions_envelope() {
        let config = Config::parse(ONE_JOINT).unwrap();
        let joint = &config.joints[0];
        let mapping = Mapping::of(joint);
        let safety = mapping.safety_config(&joint.limits, SafetyConfig::default());

        // ±0.5 rad through 19:1.
        assert!((safety.position_max.value() - 1.512).abs() < 0.001);
        // 19 Nm at the mechanism is 1 Nm at the rotor.
        assert!((safety.max_torque.value() - 1.0).abs() < 1e-9);
    }
}
