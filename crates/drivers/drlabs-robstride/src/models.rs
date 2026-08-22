//! The RobStride family, and what the protocol depends on for each variant.
//!
//! The critical fields are `torque_range`, `velocity_range`, `kp_max` and
//! `kd_max`: they are the full-scale values of the 16-bit control and feedback
//! fields, and they are *not* the same across the family. Driving an RS04 with
//! the RS01 table would scale every torque command and every torque reading by
//! a factor of seven, in the dangerous direction — so the model is a required
//! setting, never a guess.
//!
//! Every number is quoted from the model's official user manual.

/// Everything about a variant that the protocol depends on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Model {
    pub name: &'static str,

    // --- control and feedback frame full-scale values ---------------------
    /// The ± range of the torque field, in Nm. It appears in the identifier of
    /// a control command and in the payload of a feedback frame.
    pub torque_range: f64,
    /// The ± range of the velocity field, in rad/s.
    pub velocity_range: f64,
    /// The top of the one-sided gain fields.
    pub kp_max: f64,
    pub kd_max: f64,

    // --- parameter table limits (the 0x70xx registers) --------------------
    /// Bounds `iq_ref` (0x7006), in amps.
    pub current_ref_max: f64,
    /// Bounds `spd_ref` (0x700A), in rad/s.
    pub speed_ref_max: f64,
    /// Bounds `limit_torque` (0x700B), in Nm.
    pub torque_limit_max: f64,
    /// Bounds `limit_spd` (0x7017), in rad/s.
    pub speed_limit_max: f64,
    /// Bounds `limit_cur` (0x7018), in amps.
    pub current_limit_max: f64,

    // --- supply -----------------------------------------------------------
    pub voltage_min: f64,
    pub voltage_max: f64,
    pub voltage_rated: f64,

    // --- host-side defaults -----------------------------------------------
    /// What a gain scale of 1.0 maps to.
    ///
    /// These are deliberately conservative starting points, not tuned values.
    /// The right stiffness depends on the load, the gearing and the mechanism,
    /// and no table can know those — set them explicitly for a real machine
    /// once the axis is characterised.
    pub default_kp: f64,
    pub default_kd: f64,
}

/// RS00 — 14 Nm class.
pub const RS00: Model = Model {
    name: "RS00",
    torque_range: 14.0,
    velocity_range: 33.0,
    kp_max: 500.0,
    kd_max: 5.0,
    current_ref_max: 16.0,
    speed_ref_max: 33.0,
    torque_limit_max: 14.0,
    speed_limit_max: 33.0,
    current_limit_max: 16.0,
    voltage_min: 24.0,
    voltage_max: 60.0,
    voltage_rated: 48.0,
    default_kp: 20.0,
    default_kd: 1.0,
};

/// RS01 — 17 Nm class.
pub const RS01: Model = Model {
    name: "RS01",
    torque_range: 17.0,
    velocity_range: 44.0,
    current_ref_max: 23.0,
    speed_ref_max: 44.0,
    torque_limit_max: 17.0,
    speed_limit_max: 44.0,
    current_limit_max: 23.0,
    voltage_max: 50.0,
    voltage_rated: 36.0,
    ..RS00
};

/// RS02 — 17 Nm class, but with the RS00's parameter-table limits.
pub const RS02: Model = Model {
    name: "RS02",
    torque_range: 17.0,
    velocity_range: 44.0,
    ..RS00
};

/// RS03 — 60 Nm class. Note the gain ranges: ten to twenty times the smaller
/// models', which is the second reason the model has to be declared.
pub const RS03: Model = Model {
    name: "RS03",
    torque_range: 60.0,
    velocity_range: 20.0,
    kp_max: 5000.0,
    kd_max: 100.0,
    current_ref_max: 43.0,
    speed_ref_max: 20.0,
    torque_limit_max: 60.0,
    speed_limit_max: 20.0,
    current_limit_max: 43.0,
    default_kp: 200.0,
    default_kd: 10.0,
    ..RS00
};

/// RS04 — 120 Nm class.
pub const RS04: Model = Model {
    name: "RS04",
    torque_range: 120.0,
    velocity_range: 15.0,
    current_ref_max: 90.0,
    speed_ref_max: 20.0,
    torque_limit_max: 120.0,
    speed_limit_max: 20.0,
    current_limit_max: 90.0,
    ..RS03
};

/// RS05 — 5.5 Nm class.
pub const RS05: Model = Model {
    name: "RS05",
    torque_range: 5.5,
    velocity_range: 50.0,
    current_ref_max: 11.0,
    speed_ref_max: 50.0,
    torque_limit_max: 5.5,
    speed_limit_max: 50.0,
    current_limit_max: 11.0,
    voltage_min: 15.0,
    default_kp: 10.0,
    default_kd: 0.5,
    ..RS00
};

/// RS06 — 36 Nm class.
pub const RS06: Model = Model {
    name: "RS06",
    torque_range: 36.0,
    velocity_range: 50.0,
    current_ref_max: 57.0,
    speed_ref_max: 50.0,
    torque_limit_max: 36.0,
    speed_limit_max: 50.0,
    current_limit_max: 57.0,
    voltage_min: 15.0,
    ..RS03
};

/// The family, smallest to largest.
pub const MODELS: &[Model] = &[RS00, RS01, RS02, RS03, RS04, RS05, RS06];

/// Finds a model by name, forgiving about how it is written.
///
/// `RS-04`, `rs4`, `RS04` and `4` all name the same motor, and an operator
/// typing a config file should not have to guess which spelling this build
/// wants.
pub fn lookup_model(name: &str) -> Option<Model> {
    let mut key: String = name
        .trim()
        .to_ascii_lowercase()
        .chars()
        .filter(|c| !matches!(c, '-' | '_' | ' '))
        .collect();
    if !key.starts_with("rs") {
        key = format!("rs{key}");
    }
    if key.len() == 3 {
        key = format!("rs0{}", &key[2..]);
    }
    MODELS
        .iter()
        .find(|model| model.name.eq_ignore_ascii_case(&key))
        .copied()
}

/// The known model names, for an error message that says what *is* accepted.
pub fn model_names() -> Vec<&'static str> {
    MODELS.iter().map(|model| model.name).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_model_is_found_however_it_is_written() {
        for spelling in ["RS04", "rs04", "RS-04", "rs_04", " rs4 ", "4", "RS 04"] {
            assert_eq!(
                lookup_model(spelling).map(|model| model.name),
                Some("RS04"),
                "{spelling}"
            );
        }
    }

    #[test]
    fn an_unknown_model_is_not_guessed_at() {
        assert_eq!(lookup_model("RS99"), None);
        assert_eq!(lookup_model("moteus"), None);
        assert_eq!(lookup_model(""), None);
    }

    /// The split that makes a shared gain value dangerous: the same number is
    /// ten times the stiffness on one half of the family.
    #[test]
    fn the_family_splits_into_two_gain_ranges() {
        for model in [RS00, RS01, RS02, RS05] {
            assert_eq!(model.kp_max, 500.0, "{}", model.name);
            assert_eq!(model.kd_max, 5.0, "{}", model.name);
        }
        for model in [RS03, RS04, RS06] {
            assert_eq!(model.kp_max, 5000.0, "{}", model.name);
            assert_eq!(model.kd_max, 100.0, "{}", model.name);
        }
    }

    /// Written as deltas of each other, so the table has to be checked at the
    /// places the deltas were meant to change.
    #[test]
    fn each_model_carries_its_own_torque_and_supply() {
        assert_eq!(RS01.torque_range, 17.0);
        assert_eq!(RS01.voltage_rated, 36.0);
        assert_eq!(RS02.torque_range, 17.0);
        assert_eq!(RS02.voltage_rated, 48.0, "RS02 keeps the RS00 supply");
        assert_eq!(RS04.torque_range, 120.0);
        assert_eq!(RS05.torque_range, 5.5);
        assert_eq!(RS05.voltage_min, 15.0);
        assert_eq!(RS06.torque_range, 36.0);
    }

    /// Every model has to be nameable, or a config file can ask for one that
    /// exists in the table and still be refused.
    #[test]
    fn every_model_in_the_table_can_be_looked_up() {
        for model in MODELS {
            assert_eq!(lookup_model(model.name).map(|m| m.name), Some(model.name));
        }
        assert_eq!(model_names().len(), MODELS.len());
    }
}
