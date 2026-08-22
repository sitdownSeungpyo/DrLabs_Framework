//! A machine, as a JSON file.
//!
//! This is the file the whole binary-distribution idea rests on: a robot is a
//! description and an executable, and nothing else has to be installed on the
//! target. So the description has to be checked hard enough that "it loaded"
//! means "it will come up", and every refusal has to say which joint and why.

use serde::{Deserialize, Serialize};

/// The whole machine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// A name for logs and status output.
    #[serde(default)]
    pub robot: String,

    /// The control rate every bus runs at unless it names its own.
    #[serde(default)]
    pub rate_hz: u32,

    #[serde(default)]
    pub buses: Vec<BusConfig>,

    pub joints: Vec<JointConfig>,
}

/// One physical link, shared by the joints on it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {
    pub name: String,
    /// `fdcanusb` or `serial`.
    pub transport: String,
    #[serde(default)]
    pub port: String,
    #[serde(default)]
    pub baud: u32,

    /// This bus's own control rate.
    ///
    /// A CAN-FD adapter and a 1 Mbaud serial chain do not have the same budget,
    /// and running both at the faster one's rate is how the slower one falls
    /// behind until its watchdog latches. Zero means "use the robot's rate".
    #[serde(default)]
    pub rate_hz: u32,
}

/// One named degree of freedom.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointConfig {
    pub name: String,
    #[serde(default)]
    pub bus: String,
    pub driver: String,
    pub id: i32,

    #[serde(default)]
    pub options: std::collections::BTreeMap<String, String>,

    /// Motor revolutions per output revolution. 1.0 is a direct drive.
    pub gear_ratio: f64,
    /// +1 or -1.
    pub direction: f64,
    /// Where the joint's zero sits, in output radians.
    #[serde(default)]
    pub offset_rad: f64,

    pub limits: JointLimits,
}

/// What one joint is allowed to do, in joint space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimits {
    pub min_rad: f64,
    pub max_rad: f64,
    pub max_torque_nm: f64,
    pub max_velocity_rad_s: f64,
    #[serde(default)]
    pub max_accel_rad_s2: f64,
}

/// Why a description was refused.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct ConfigError(pub String);

pub const DEFAULT_RATE_HZ: u32 = 200;
pub const MIN_RATE_HZ: u32 = 10;
pub const MAX_RATE_HZ: u32 = 500;

impl Config {
    /// Reads and validates a description.
    pub fn load(path: &std::path::Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path)
            .map_err(|error| ConfigError(format!("cannot read {}: {error}", path.display())))?;
        Self::parse(&text).map_err(|error| ConfigError(format!("{}: {}", path.display(), error.0)))
    }

    /// Parses and validates a description.
    ///
    /// Unknown fields are refused rather than ignored: a typo in a limit is a
    /// machine running with a limit nobody set, and silence is the worst
    /// possible answer to that.
    pub fn parse(text: &str) -> Result<Self, ConfigError> {
        let mut config: Config =
            serde_json::from_str(text).map_err(|error| ConfigError(error.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    /// Checks the description, filling in the defaults it is allowed to have.
    pub fn validate(&mut self) -> Result<(), ConfigError> {
        if self.joints.is_empty() {
            return Err(ConfigError("no joints defined".into()));
        }
        if self.rate_hz == 0 {
            self.rate_hz = DEFAULT_RATE_HZ;
        }
        if !(MIN_RATE_HZ..=MAX_RATE_HZ).contains(&self.rate_hz) {
            return Err(ConfigError(format!(
                "rate_hz {} is outside {MIN_RATE_HZ}..{MAX_RATE_HZ}",
                self.rate_hz
            )));
        }

        let mut bus_names = Vec::new();
        for (index, bus) in self.buses.iter().enumerate() {
            if bus.name.is_empty() {
                return Err(ConfigError(format!("bus {index} has no name")));
            }
            if bus_names.contains(&bus.name.as_str()) {
                return Err(ConfigError(format!(
                    "two buses are both named {:?}",
                    bus.name
                )));
            }
            bus_names.push(&bus.name);
            if !matches!(bus.transport.as_str(), "fdcanusb" | "serial") {
                return Err(ConfigError(format!(
                    "bus {:?}: unknown transport {:?} (fdcanusb or serial)",
                    bus.name, bus.transport
                )));
            }
            if bus.rate_hz != 0 && !(MIN_RATE_HZ..=MAX_RATE_HZ).contains(&bus.rate_hz) {
                return Err(ConfigError(format!(
                    "bus {:?}: rate_hz {} is outside {MIN_RATE_HZ}..{MAX_RATE_HZ}",
                    bus.name, bus.rate_hz
                )));
            }
        }

        let mut seen: Vec<&str> = Vec::new();
        let mut addresses: Vec<(String, &str)> = Vec::new();

        for (index, joint) in self.joints.iter().enumerate() {
            let where_ = if joint.name.is_empty() {
                format!("joint {index}")
            } else {
                format!("joint {:?}", joint.name)
            };

            if joint.name.is_empty() {
                return Err(ConfigError(format!("{where_} has no name")));
            }
            if seen.contains(&joint.name.as_str()) {
                return Err(ConfigError(format!(
                    "two joints are both named {:?}",
                    joint.name
                )));
            }
            seen.push(&joint.name);

            if !self.buses.is_empty() && !bus_names.contains(&joint.bus.as_str()) {
                return Err(ConfigError(format!(
                    "{where_} names bus {:?}, which is not defined",
                    joint.bus
                )));
            }
            if joint.driver.is_empty() {
                return Err(ConfigError(format!("{where_} has no driver")));
            }

            let key = format!("{}/{}", joint.bus, joint.id);
            if let Some((_, other)) = addresses.iter().find(|(existing, _)| existing == &key) {
                return Err(ConfigError(format!(
                    "{where_} and {other:?} are both at address {} on bus {:?}",
                    joint.id, joint.bus
                )));
            }
            addresses.push((key, &joint.name));

            if joint.gear_ratio == 0.0 {
                return Err(ConfigError(format!(
                    "{where_} has no gear_ratio (use 1.0 for a direct drive)"
                )));
            }
            if joint.gear_ratio < 0.0 {
                return Err(ConfigError(format!(
                    "{where_} has a negative gear_ratio; use direction: -1 to reverse it"
                )));
            }
            if joint.direction != 1.0 && joint.direction != -1.0 {
                return Err(ConfigError(format!(
                    "{where_} has direction {}; it must be 1 or -1",
                    joint.direction
                )));
            }

            let limits = &joint.limits;
            if !limits.min_rad.is_finite() || !limits.max_rad.is_finite() {
                return Err(ConfigError(format!(
                    "{where_} has a non-numeric travel limit"
                )));
            }
            if limits.max_rad <= limits.min_rad {
                return Err(ConfigError(format!(
                    "{where_} has an empty travel range [{}, {}]",
                    limits.min_rad, limits.max_rad
                )));
            }
            if limits.max_torque_nm <= 0.0 {
                return Err(ConfigError(format!(
                    "{where_} has no max_torque_nm; a joint with no torque ceiling has no envelope"
                )));
            }
            if limits.max_velocity_rad_s <= 0.0 {
                return Err(ConfigError(format!("{where_} has no max_velocity_rad_s")));
            }
        }

        Ok(())
    }

    pub fn bus(&self, name: &str) -> Option<&BusConfig> {
        self.buses.iter().find(|bus| bus.name == name)
    }

    pub fn joint(&self, name: &str) -> Option<&JointConfig> {
        self.joints.iter().find(|joint| joint.name == name)
    }

    pub fn joint_names(&self) -> Vec<&str> {
        self.joints
            .iter()
            .map(|joint| joint.name.as_str())
            .collect()
    }

    /// The rate a bus actually runs at.
    pub fn rate_for_bus(&self, name: &str) -> u32 {
        match self.bus(name) {
            Some(bus) if bus.rate_hz > 0 => bus.rate_hz,
            _ => self.rate_hz,
        }
    }

    /// How many transactions a second each bus is being asked for.
    ///
    /// This is the number that decides whether a machine works: a serial chain
    /// at 1 Mbaud manages a few hundred, and two joints on it at 200 Hz each is
    /// already 400.
    pub fn bus_load(&self) -> Vec<(String, u32)> {
        let mut load: Vec<(String, u32)> = Vec::new();
        for joint in &self.joints {
            let rate = self.rate_for_bus(&joint.bus);
            match load.iter_mut().find(|(name, _)| name == &joint.bus) {
                Some((_, total)) => *total += rate,
                None => load.push((joint.bus.clone(), rate)),
            }
        }
        load.sort_by(|a, b| a.0.cmp(&b.0));
        load
    }

    /// The buses that carry a joint, in a stable order.
    pub fn bus_names(&self) -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        for joint in &self.joints {
            if !names.contains(&joint.bus) {
                names.push(joint.bus.clone());
            }
        }
        names.sort();
        names
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHOULDER_PAIR: &str = r#"{
      "robot": "shoulder_pair",
      "rate_hz": 200,
      "buses": [
        {"name": "can0", "transport": "fdcanusb", "port": "COM5", "rate_hz": 200},
        {"name": "dxl0", "transport": "serial", "port": "COM6", "baud": 1000000, "rate_hz": 100}
      ],
      "joints": [
        {
          "name": "r_sh_p", "bus": "can0", "driver": "moteus", "id": 1,
          "gear_ratio": 1.0, "direction": 1.0,
          "limits": {"min_rad": -1.4, "max_rad": 1.4, "max_torque_nm": 2.0,
                     "max_velocity_rad_s": 3.0}
        },
        {
          "name": "l_sh_p", "bus": "dxl0", "driver": "dynamixel", "id": 1,
          "options": {"profile": "xl330"},
          "gear_ratio": 1.0, "direction": 1.0, "offset_rad": 4.5974,
          "limits": {"min_rad": -1.2, "max_rad": 1.2, "max_torque_nm": 0.3,
                     "max_velocity_rad_s": 3.0}
        }
      ]
    }"#;

    #[test]
    fn a_real_description_loads() {
        let config = Config::parse(SHOULDER_PAIR).expect("the bench pair is valid");
        assert_eq!(config.robot, "shoulder_pair");
        assert_eq!(config.joints.len(), 2);
        assert_eq!(config.joint("l_sh_p").unwrap().options["profile"], "xl330");
    }

    /// The per-bus rate, which exists because a CAN-FD adapter and a 1 Mbaud
    /// serial chain do not have the same budget.
    #[test]
    fn each_bus_may_run_at_its_own_rate() {
        let config = Config::parse(SHOULDER_PAIR).unwrap();
        assert_eq!(config.rate_for_bus("can0"), 200);
        assert_eq!(config.rate_for_bus("dxl0"), 100);
        // A bus with nothing declared falls back to the robot's rate.
        assert_eq!(config.rate_for_bus("nonexistent"), 200);
    }

    /// The number that decides whether a machine works.
    #[test]
    fn the_bus_load_is_the_sum_of_what_is_on_it() {
        let mut config = Config::parse(SHOULDER_PAIR).unwrap();
        assert_eq!(
            config.bus_load(),
            vec![("can0".into(), 200), ("dxl0".into(), 100)]
        );

        // A second joint on the serial bus doubles what it is asked for.
        let mut second = config.joints[1].clone();
        second.name = "l_sh_r".into();
        second.id = 2;
        config.joints.push(second);
        assert_eq!(config.bus_load()[1], ("dxl0".into(), 200));
    }

    /// A typo in a limit is a machine running with a limit nobody set. Silence
    /// is the worst possible answer.
    #[test]
    fn an_unknown_field_is_refused_rather_than_ignored() {
        let text = SHOULDER_PAIR.replace("\"max_torque_nm\"", "\"max_torqu_nm\"");
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("max_torqu_nm"), "{error}");
    }

    #[test]
    fn two_joints_at_the_same_address_are_refused() {
        let text = SHOULDER_PAIR.replace(
            r#""name": "l_sh_p", "bus": "dxl0""#,
            r#""name": "l_sh_p", "bus": "can0""#,
        );
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("both at address 1"), "{error}");
    }

    #[test]
    fn two_joints_with_the_same_name_are_refused() {
        let text = SHOULDER_PAIR.replace(r#""name": "l_sh_p""#, r#""name": "r_sh_p""#);
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("both named"), "{error}");
    }

    /// A joint that names a bus nobody declared would be opened on a port the
    /// description never mentioned.
    #[test]
    fn a_joint_on_an_undeclared_bus_is_refused() {
        let text =
            SHOULDER_PAIR.replace(r#""bus": "dxl0", "driver""#, r#""bus": "dxl9", "driver""#);
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("dxl9"), "{error}");
    }

    /// A negative gear ratio is somebody trying to reverse a joint the wrong
    /// way, and the message says which way is the right one.
    #[test]
    fn a_negative_gear_ratio_points_at_direction_instead() {
        let text = SHOULDER_PAIR.replace(
            r#""gear_ratio": 1.0, "direction": 1.0,
          "limits""#,
            r#""gear_ratio": -1.0, "direction": 1.0,
          "limits""#,
        );
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("direction: -1"), "{error}");
    }

    #[test]
    fn a_joint_with_no_travel_is_refused() {
        let text = SHOULDER_PAIR.replace(
            r#""min_rad": -1.4, "max_rad": 1.4"#,
            r#""min_rad": 1.4, "max_rad": 1.4"#,
        );
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("empty travel range"), "{error}");
    }

    /// A joint with no torque ceiling has no envelope, so there is nothing for
    /// the safety layer to enforce.
    #[test]
    fn a_joint_with_no_torque_ceiling_is_refused() {
        let text = SHOULDER_PAIR.replace(r#""max_torque_nm": 2.0"#, r#""max_torque_nm": 0"#);
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("no envelope"), "{error}");
    }

    #[test]
    fn a_rate_outside_what_a_transport_can_carry_is_refused() {
        let text = SHOULDER_PAIR.replace(r#""rate_hz": 200,"#, r#""rate_hz": 5000,"#);
        assert!(Config::parse(&text).is_err());
    }

    /// A description with no joints describes no machine.
    #[test]
    fn a_description_with_no_joints_is_refused() {
        let error = Config::parse(r#"{"joints": []}"#).unwrap_err();
        assert!(format!("{error}").contains("no joints"), "{error}");
    }

    /// Every error names the joint it is about, because a description with
    /// twenty joints and a message that says "invalid" is not actionable.
    #[test]
    fn every_refusal_names_the_joint_it_is_about() {
        // The second joint's velocity limit, which leaves the first one valid.
        let text = SHOULDER_PAIR.replace(
            r#""max_torque_nm": 0.3,
                     "max_velocity_rad_s": 3.0"#,
            r#""max_torque_nm": 0.3,
                     "max_velocity_rad_s": 0"#,
        );
        let error = Config::parse(&text).unwrap_err();
        assert!(format!("{error}").contains("l_sh_p"), "{error}");
        assert!(
            !format!("{error}").contains("r_sh_p"),
            "it should name the joint that is wrong, not the other one: {error}"
        );
    }

    /// A single-bus description does not have to declare the bus at all, which
    /// is what makes a one-motor test file short.
    #[test]
    fn a_description_may_leave_the_buses_implicit() {
        let config = Config::parse(
            r#"{
              "joints": [{
                "name": "axis", "driver": "moteus", "id": 1,
                "gear_ratio": 1.0, "direction": 1.0,
                "limits": {"min_rad": -1, "max_rad": 1, "max_torque_nm": 1,
                           "max_velocity_rad_s": 1}
              }]
            }"#,
        )
        .expect("a bare description is valid");
        assert_eq!(config.rate_hz, DEFAULT_RATE_HZ, "the default rate fills in");
        assert_eq!(config.rate_for_bus(""), DEFAULT_RATE_HZ);
    }
}
