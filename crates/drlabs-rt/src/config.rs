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

    /// Named poses, in joint space: `{ "base": { "l_sh_p": 0.26, … } }`.
    ///
    /// A pose belongs in the description rather than in a panel, for the same
    /// reason the envelope does: it is a fact about this machine, and the
    /// binary that drives it should not have to be rebuilt to change where
    /// "base" is. A pose may name a subset of the joints; the ones it leaves
    /// out are not commanded.
    ///
    /// The runtime does not act on poses. It hands them to clients in its
    /// description, and a client moves the machine to one.
    #[serde(default)]
    pub poses: std::collections::BTreeMap<String, std::collections::BTreeMap<String, f64>>,

    #[serde(default)]
    pub rt: RtConfig,

    #[serde(default)]
    pub api: ApiConfig,
}

/// How the bus threads are scheduled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct RtConfig {
    /// Cores to pin the bus threads to, in bus order (cycled if fewer).
    /// Empty leaves them to the scheduler. Linux and Windows; macOS cannot
    /// pin and reports the request refused.
    #[serde(default)]
    pub cpus: Vec<usize>,
    /// Real-time priority: Linux SCHED_FIFO 1..99; on macOS any non-zero
    /// value asks for the mach time-constraint policy and the number is
    /// otherwise ignored; Windows TIME_CRITICAL. Zero asks for none.
    #[serde(default)]
    pub priority: u32,
    /// Consecutive ticks that start a whole period late before the robot is
    /// stopped. Zero fills in the default of 5; set it deliberately high on
    /// a machine with no real-time scheduling.
    #[serde(default)]
    pub deadline_miss_trip: u32,
}

/// How the runtime talks to clients.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApiConfig {
    /// Where the UDP endpoint listens.
    #[serde(default = "default_udp")]
    pub udp: String,
    /// Publish a state record every this many ticks.
    #[serde(default = "default_state_div")]
    pub state_div: u32,
    /// A command older than this is stale.
    #[serde(default = "default_cmd_timeout_ms")]
    pub cmd_timeout_ms: u32,
    /// `hold` keeps the last position; `release` turns every axis off.
    #[serde(default = "default_on_cmd_timeout")]
    pub on_cmd_timeout: String,
}

fn default_udp() -> String {
    "127.0.0.1:7700".into()
}
fn default_state_div() -> u32 {
    1
}
fn default_cmd_timeout_ms() -> u32 {
    100
}
fn default_on_cmd_timeout() -> String {
    "hold".into()
}

impl Default for ApiConfig {
    fn default() -> Self {
        Self {
            udp: default_udp(),
            state_div: default_state_div(),
            cmd_timeout_ms: default_cmd_timeout_ms(),
            on_cmd_timeout: default_on_cmd_timeout(),
        }
    }
}

/// One physical link, shared by the joints on it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {
    pub name: String,
    /// `fdcanusb`, `serial` or `sim`.
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

    /// The supply band this joint should be judged against, in volts.
    ///
    /// Leave these out and the motor's own documented range is used — the
    /// driver knows what it is connected to, and its datasheet is the authority
    /// on what counts as a sagging supply. Set them to hold a joint to a
    /// tighter band than its datasheet allows: a 12 V servo on a bench supply
    /// that must never be allowed below 11.5 V, say. Both have to be given
    /// together, because half a band is not one.
    #[serde(default)]
    pub bus_voltage_min: Option<f64>,
    #[serde(default)]
    pub bus_voltage_max: Option<f64>,
}

/// Why a description was refused.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct ConfigError(pub String);

/// One joint's limits and sense, as an operator edited them.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LimitEdit {
    pub min_rad: Option<f64>,
    pub max_rad: Option<f64>,
    /// +1 or -1. Changing it flips the joint's sense, so the travel measured
    /// under the old one no longer describes the same places.
    pub direction: Option<f64>,
}

impl Config {
    /// Applies edited limits and writes the description back.
    ///
    /// The file is the machine's definition, so this is deliberately strict:
    /// the edits are applied to a copy, the copy is validated exactly as a
    /// freshly loaded description would be, and only then does anything reach
    /// the disk. A narrowed limit that no longer contains a named pose is
    /// refused here rather than discovered on the next bring-up.
    ///
    /// The previous file is kept alongside as `<name>.bak`. It is usually
    /// hand-written, and the first thing anybody does with a learned range is
    /// decide they preferred the old one.
    pub fn write_limits(
        path: &std::path::Path,
        edits: &std::collections::BTreeMap<String, LimitEdit>,
    ) -> Result<Self, ConfigError> {
        let mut config = Self::load(path)?;

        for (name, edit) in edits {
            let joint = config
                .joints
                .iter_mut()
                .find(|joint| &joint.name == name)
                .ok_or_else(|| ConfigError(format!("no joint named {name:?}")))?;

            if let Some(value) = edit.min_rad {
                joint.limits.min_rad = value;
            }
            if let Some(value) = edit.max_rad {
                joint.limits.max_rad = value;
            }
            if let Some(value) = edit.direction {
                if value != 1.0 && value != -1.0 {
                    return Err(ConfigError(format!(
                        "joint {name:?}: direction {value} must be 1 or -1"
                    )));
                }
                joint.direction = value;
            }
        }

        config.validate()?;

        let text = serde_json::to_string_pretty(&config)
            .map_err(|error| ConfigError(format!("cannot render the description: {error}")))?;

        // Prove the rendered file loads before replacing anything with it.
        Self::parse(&text).map_err(|error| {
            ConfigError(format!(
                "the edited description would not load: {}",
                error.0
            ))
        })?;

        if path.exists() {
            let backup = path.with_extension("json.bak");
            std::fs::copy(path, &backup).map_err(|error| {
                ConfigError(format!("cannot back up to {}: {error}", backup.display()))
            })?;
        }
        std::fs::write(path, text.as_bytes())
            .map_err(|error| ConfigError(format!("cannot write {}: {error}", path.display())))?;

        Ok(config)
    }
}

/// Checks the driver-specific options a joint carries.
///
/// These name things inside a driver — a Dynamixel control-table profile, a
/// Robstride model — and until this ran, a typo in one of them was found by
/// powering the robot: the description validated, the plan printed, and the
/// bring-up failed on the first joint it tried to open. A name that does not
/// resolve is a description that will not come up, which is precisely what
/// `-check` exists to say.
fn check_driver_options(where_: &str, joint: &JointConfig) -> Result<(), ConfigError> {
    match joint.driver.as_str() {
        "dynamixel" => {
            if let Some(name) = joint.options.get("profile") {
                if drlabs_dynamixel::profile::profile_named(name).is_none() {
                    return Err(ConfigError(format!(
                        "{where_}: unknown dynamixel profile {name:?} (known: {})",
                        drlabs_dynamixel::profile::PROFILE_NAMES.join(", ")
                    )));
                }
            }
        }
        "robstride" => {
            // The driver refuses an undeclared model outright, because the
            // scaling differs by up to a factor of twenty across the family.
            match joint.options.get("model") {
                None => {
                    return Err(ConfigError(format!(
                        "{where_}: robstride needs a model option; the scaling differs by up                          to a factor of twenty across the family"
                    )))
                }
                Some(name) if drlabs_robstride::lookup_model(name).is_none() => {
                    return Err(ConfigError(format!(
                        "{where_}: unknown robstride model {name:?}"
                    )))
                }
                Some(_) => {}
            }
        }
        _ => {}
    }
    Ok(())
}

pub const DEFAULT_RATE_HZ: u32 = 200;
pub const MIN_RATE_HZ: u32 = 10;
pub const MAX_RATE_HZ: u32 = 1000;
pub const DEFAULT_DEADLINE_MISS_TRIP: u32 = 5;

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
        if self.joints.len() > drlabs_core::MAX_AXES {
            return Err(ConfigError(format!(
                "{} joints, but a runtime carries at most {}",
                self.joints.len(),
                drlabs_core::MAX_AXES
            )));
        }
        if self.rt.deadline_miss_trip == 0 {
            self.rt.deadline_miss_trip = DEFAULT_DEADLINE_MISS_TRIP;
        }
        if self.rt.priority > 99 {
            return Err(ConfigError(format!(
                "rt.priority {} is outside 0..99",
                self.rt.priority
            )));
        }
        if !matches!(self.api.on_cmd_timeout.as_str(), "hold" | "release") {
            return Err(ConfigError(format!(
                "api.on_cmd_timeout {:?} must be \"hold\" or \"release\"",
                self.api.on_cmd_timeout
            )));
        }
        if self.api.state_div == 0 {
            self.api.state_div = 1;
        }
        if self.api.cmd_timeout_ms == 0 {
            return Err(ConfigError(
                "api.cmd_timeout_ms must be positive: a command that is never stale is a \
                 client that can die unnoticed"
                    .into(),
            ));
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
            if !matches!(bus.transport.as_str(), "fdcanusb" | "serial" | "sim") {
                return Err(ConfigError(format!(
                    "bus {:?}: unknown transport {:?} (fdcanusb, serial or sim)",
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

            match (limits.bus_voltage_min, limits.bus_voltage_max) {
                (None, None) => {}
                (Some(min), Some(max)) => {
                    if !min.is_finite() || !max.is_finite() || min <= 0.0 {
                        return Err(ConfigError(format!(
                            "{where_} has a non-numeric bus voltage band"
                        )));
                    }
                    if max <= min {
                        return Err(ConfigError(format!(
                            "{where_} has an empty bus voltage band [{min}, {max}]"
                        )));
                    }
                }
                _ => {
                    return Err(ConfigError(format!(
                        "{where_} gives only half a bus voltage band; set both                          bus_voltage_min and bus_voltage_max, or neither"
                    )))
                }
            }

            check_driver_options(&where_, joint)?;
        }

        self.check_poses()?;
        Ok(())
    }

    /// Checks that every pose names joints that exist and angles they can
    /// reach.
    ///
    /// A pose that asks for a joint outside its envelope is not a pose: the
    /// target is clamped on the way through, so the machine would go to a
    /// different shape from the one written down and nothing would say so.
    fn check_poses(&self) -> Result<(), ConfigError> {
        for (pose, targets) in &self.poses {
            if targets.is_empty() {
                return Err(ConfigError(format!("pose {pose:?} commands no joints")));
            }
            for (name, &radians) in targets {
                let joint = self
                    .joints
                    .iter()
                    .find(|joint| &joint.name == name)
                    .ok_or_else(|| {
                        ConfigError(format!(
                            "pose {pose:?} names joint {name:?}, which does not exist"
                        ))
                    })?;
                if !radians.is_finite() {
                    return Err(ConfigError(format!(
                        "pose {pose:?} gives joint {name:?} a non-numeric angle"
                    )));
                }
                if radians < joint.limits.min_rad || radians > joint.limits.max_rad {
                    return Err(ConfigError(format!(
                        "pose {pose:?} puts joint {name:?} at {radians} rad, outside its travel [{}, {}]; it would be clamped on the way through and the machine would not take the shape the pose describes",
                        joint.limits.min_rad, joint.limits.max_rad
                    )));
                }
            }
        }
        Ok(())
    }

    /// A named pose, in joint-space radians.
    pub fn pose(&self, name: &str) -> Option<&std::collections::BTreeMap<String, f64>> {
        self.poses.get(name)
    }

    /// The poses this description carries, in a stable order.
    pub fn pose_names(&self) -> Vec<&str> {
        self.poses.keys().map(String::as_str).collect()
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
        let mut chained: Vec<&str> = Vec::new();

        for joint in &self.joints {
            let rate = self.rate_for_bus(&joint.bus);

            // Dynamixels on one port are cycled together: a sync write for
            // every goal and a sync read for every reading, two transactions a
            // period whatever the joint count. Counting them one apiece was
            // right when each link went to the wire on its own, and reporting
            // it that way now overstates a sixteen-joint chain by eightfold.
            let cost = if joint.driver == "dynamixel" {
                if chained.contains(&joint.bus.as_str()) {
                    0
                } else {
                    chained.push(&joint.bus);
                    2 * rate
                }
            } else {
                rate
            };

            match load.iter_mut().find(|(name, _)| name == &joint.bus) {
                Some((_, total)) => *total += cost,
                None => load.push((joint.bus.clone(), cost)),
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
    /// A profile name that does not resolve is a description that will not
    /// come up, and `-check` is the place to say so — not the first joint of
    /// a bring-up with the machine powered.
    #[test]
    fn a_profile_name_that_does_not_resolve_is_refused() {
        let text = r#"{
            "robot": "t", "rate_hz": 100,
            "buses": [{ "name": "dxl0", "transport": "serial", "port": "COM3" }],
            "joints": [{
                "name": "j1", "bus": "dxl0", "driver": "dynamixel", "id": 1,
                "options": { "profile": "mx-64" },
                "gear_ratio": 1.0, "direction": 1,
                "limits": { "min_rad": -1, "max_rad": 1,
                            "max_torque_nm": 1, "max_velocity_rad_s": 1 }
            }]
        }"#;
        let error = Config::parse(text).expect_err("the profile does not exist");
        assert!(error.0.contains("unknown dynamixel profile"), "{}", error.0);
        assert!(error.0.contains("mx64"), "the message lists the real names");

        // And the corrected name loads.
        let ok = text.replace("mx-64", "mx64");
        assert!(Config::parse(&ok).is_ok());
    }

    /// A Robstride with no model is refused for the same reason the driver
    /// refuses it: the scaling differs by up to a factor of twenty.
    #[test]
    fn a_robstride_without_a_model_is_refused_before_anything_opens() {
        let text = r#"{
            "robot": "t", "rate_hz": 100,
            "buses": [{ "name": "can0", "transport": "fdcanusb", "port": "COM5" }],
            "joints": [{
                "name": "j1", "bus": "can0", "driver": "robstride", "id": 1,
                "gear_ratio": 1.0, "direction": 1,
                "limits": { "min_rad": -1, "max_rad": 1,
                            "max_torque_nm": 1, "max_velocity_rad_s": 1 }
            }]
        }"#;
        let error = Config::parse(text).expect_err("no model was named");
        assert!(error.0.contains("model"), "{}", error.0);
    }

    /// A CAN bus carries a transaction per joint per cycle. A Dynamixel chain
    /// carries two a period — the sync write and the sync read — however many
    /// servos are on it, which is the whole reason a sixteen-joint chain can
    /// keep time at all.
    #[test]
    fn a_can_bus_is_charged_per_joint_and_a_chain_is_charged_once() {
        let mut config = Config::parse(SHOULDER_PAIR).unwrap();
        assert_eq!(
            config.bus_load(),
            vec![("can0".into(), 200), ("dxl0".into(), 200)],
            "one moteus at 200 Hz, and one chain at 100 Hz costing a write and a read"
        );

        // A second servo on the same chain rides the transactions already
        // being sent, and costs nothing more.
        let mut second = config.joints[1].clone();
        second.name = "l_sh_r".into();
        second.id = 2;
        config.joints.push(second);
        assert_eq!(config.bus_load()[1], ("dxl0".into(), 200));

        // A second joint on the CAN bus does cost another transaction.
        let mut third = config.joints[0].clone();
        third.name = "r_sh_r".into();
        third.id = 2;
        config.joints.push(third);
        assert_eq!(config.bus_load()[0], ("can0".into(), 400));
    }

    /// A pose is part of the description, and a description that validates has
    /// to mean the pose will be taken — not clamped into something else.
    #[test]
    fn a_pose_outside_a_joints_travel_is_refused() {
        let with_pose = |value: &str| {
            SHOULDER_PAIR.replace(
                "\"joints\":",
                &format!("\"poses\": {{ \"base\": {{ \"r_sh_p\": {value} }} }}, \"joints\":"),
            )
        };

        let ok = Config::parse(&with_pose("0.5")).expect("0.5 rad is inside the travel");
        assert_eq!(ok.pose("base").unwrap()["r_sh_p"], 0.5);
        assert_eq!(ok.pose_names(), vec!["base"]);

        let error = Config::parse(&with_pose("2.5")).expect_err("2.5 rad is outside it");
        assert!(error.0.contains("outside its travel"), "{}", error.0);
        assert!(error.0.contains("r_sh_p"), "{}", error.0);
    }

    /// A pose naming a joint that is not on the machine is a rename nobody
    /// finished, and it would silently move fewer joints than it says.
    #[test]
    fn a_pose_naming_an_unknown_joint_is_refused() {
        let text = SHOULDER_PAIR.replace(
            "\"joints\":",
            "\"poses\": { \"base\": { \"l_elbow\": 0.0 } }, \"joints\":",
        );
        let error = Config::parse(&text).expect_err("there is no l_elbow");
        assert!(error.0.contains("l_elbow"), "{}", error.0);
        assert!(error.0.contains("does not exist"), "{}", error.0);
    }

    /// A description with no poses is the ordinary case and must stay legal.
    #[test]
    fn a_description_without_poses_is_fine() {
        let config = Config::parse(SHOULDER_PAIR).unwrap();
        assert!(config.pose_names().is_empty());
        assert!(config.pose("base").is_none());
    }

    /// The whole point of writing back: the edit lands, the rest of the file
    /// survives, and what is on disk still loads.
    #[test]
    fn edited_limits_are_written_and_the_description_still_loads() {
        let dir = std::env::temp_dir().join("drlabs-write-limits");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.json");
        std::fs::write(&path, SHOULDER_PAIR).unwrap();

        let mut edits = std::collections::BTreeMap::new();
        edits.insert(
            "r_sh_p".to_string(),
            LimitEdit {
                min_rad: Some(-0.8),
                max_rad: Some(1.1),
                direction: Some(-1.0),
            },
        );

        let written = Config::write_limits(&path, &edits).expect("the edit is valid");
        assert_eq!(written.joints[0].limits.min_rad, -0.8);
        assert_eq!(written.joints[0].limits.max_rad, 1.1);
        assert_eq!(written.joints[0].direction, -1.0);

        // What is on disk is what comes back.
        let reloaded = Config::load(&path).expect("the written file loads");
        assert_eq!(reloaded, written);

        // The joint nobody edited is untouched.
        assert_eq!(reloaded.joints[1].name, written.joints[1].name);
        assert_eq!(
            reloaded.joints[1].limits.max_rad,
            Config::parse(SHOULDER_PAIR).unwrap().joints[1]
                .limits
                .max_rad
        );

        // And the file it replaced is still there.
        assert!(path.with_extension("json.bak").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// An edit that would not load must not reach the disk. The description is
    /// the machine's definition, and a half-written one is a robot that will
    /// not come up.
    #[test]
    fn an_edit_that_would_not_validate_leaves_the_file_alone() {
        let dir = std::env::temp_dir().join("drlabs-write-limits-bad");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.json");
        std::fs::write(&path, SHOULDER_PAIR).unwrap();

        let mut edits = std::collections::BTreeMap::new();
        // An empty travel range: max is not above min.
        edits.insert(
            "r_sh_p".to_string(),
            LimitEdit {
                min_rad: Some(0.5),
                max_rad: Some(0.5),
                direction: None,
            },
        );
        let error = Config::write_limits(&path, &edits).expect_err("an empty range is not a range");
        assert!(error.0.contains("empty travel range"), "{}", error.0);

        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            SHOULDER_PAIR,
            "the file must be exactly as it was"
        );

        // A direction that is neither sense is refused too.
        let mut edits = std::collections::BTreeMap::new();
        edits.insert(
            "r_sh_p".to_string(),
            LimitEdit {
                min_rad: None,
                max_rad: None,
                direction: Some(0.0),
            },
        );
        assert!(Config::write_limits(&path, &edits).is_err());
        assert_eq!(std::fs::read_to_string(&path).unwrap(), SHOULDER_PAIR);

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A limit narrowed past a named pose is caught before it is written, not
    /// on the next bring-up.
    #[test]
    fn a_limit_that_strands_a_pose_is_refused() {
        let dir = std::env::temp_dir().join("drlabs-write-limits-pose");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.json");
        let with_pose = SHOULDER_PAIR.replace(
            "\"joints\":",
            "\"poses\": { \"base\": { \"r_sh_p\": 1.0 } }, \"joints\":",
        );
        std::fs::write(&path, &with_pose).unwrap();

        let mut edits = std::collections::BTreeMap::new();
        edits.insert(
            "r_sh_p".to_string(),
            LimitEdit {
                min_rad: Some(-0.2),
                max_rad: Some(0.2),
                direction: None,
            },
        );
        let error = Config::write_limits(&path, &edits).expect_err("base sits at 1.0 rad");
        assert!(error.0.contains("outside its travel"), "{}", error.0);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), with_pose);
        let _ = std::fs::remove_dir_all(&dir);
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

    /// The runtime sections are optional and fill in with defaults that a
    /// bench can live with.
    #[test]
    fn the_runtime_sections_default_sensibly() {
        let config = Config::parse(SHOULDER_PAIR).unwrap();
        assert_eq!(config.rt.deadline_miss_trip, DEFAULT_DEADLINE_MISS_TRIP);
        assert_eq!(config.rt.priority, 0);
        assert_eq!(config.api.udp, "127.0.0.1:7700");
        assert_eq!(config.api.on_cmd_timeout, "hold");
        assert_eq!(config.api.cmd_timeout_ms, 100);
    }

    #[test]
    fn a_timeout_policy_that_is_neither_is_refused() {
        let text = SHOULDER_PAIR.replace(
            r#""rate_hz": 200,"#,
            r#""rate_hz": 200, "api": {"on_cmd_timeout": "limp"},"#,
        );
        let error = Config::parse(&text).unwrap_err();
        assert!(error.0.contains("on_cmd_timeout"), "{}", error.0);
    }

    /// A simulated bus is a transport like any other, so a description can
    /// run the whole runtime with nothing plugged in.
    #[test]
    fn a_sim_transport_is_a_valid_bus() {
        let text = r#"{
            "robot": "t", "rate_hz": 500,
            "buses": [{ "name": "sim0", "transport": "sim" }],
            "joints": [{
                "name": "j1", "bus": "sim0", "driver": "sim", "id": 1,
                "gear_ratio": 1.0, "direction": 1,
                "limits": { "min_rad": -1, "max_rad": 1,
                            "max_torque_nm": 1, "max_velocity_rad_s": 1 }
            }]
        }"#;
        assert!(Config::parse(text).is_ok());
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
