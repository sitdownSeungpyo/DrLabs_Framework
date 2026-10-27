//! What a client is told about the machine.
//!
//! The record indexes are the joint order in the description; a client
//! looks a joint up by name here once and addresses it by index after.

use drlabs_rt::Config;

/// The description as JSON.
///
/// ```json
/// {"protocol":1,"robot":"UBP","rate_hz":500,"period_ns":2000000,
///  "joints":[{"index":0,"name":"l_sh_p","bus":"dxl0","driver":"dynamixel","id":1,
///             "min_rad":-1.2,"max_rad":1.2,"max_torque_nm":0.3,
///             "max_velocity_rad_s":3.0,"max_accel_rad_s2":6.0}, …],
///  "poses":{"base":{"l_sh_p":0.26, …}}}
/// ```
pub fn describe_json(config: &Config) -> String {
    let joints: Vec<serde_json::Value> = config
        .joints
        .iter()
        .enumerate()
        .map(|(index, joint)| {
            serde_json::json!({
                "index": index,
                "name": joint.name,
                "bus": joint.bus,
                "driver": joint.driver,
                "id": joint.id,
                "min_rad": joint.limits.min_rad,
                "max_rad": joint.limits.max_rad,
                "max_torque_nm": joint.limits.max_torque_nm,
                "max_velocity_rad_s": joint.limits.max_velocity_rad_s,
                "max_accel_rad_s2": joint.limits.max_accel_rad_s2,
            })
        })
        .collect();
    serde_json::json!({
        "protocol": drlabs_core::PROTOCOL_VERSION,
        "robot": config.robot,
        "rate_hz": config.rate_hz,
        "period_ns": (1_000_000_000.0 / config.rate_hz as f64).round() as u64,
        "cmd_timeout_ms": config.api.cmd_timeout_ms,
        "on_cmd_timeout": config.api.on_cmd_timeout,
        "joints": joints,
        "poses": config.poses,
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_description_names_every_joint_in_record_order() {
        let config = Config::parse(
            r#"{"robot":"t","rate_hz":500,"buses":[{"name":"s","transport":"sim"}],
                "poses":{"base":{"b":0.1}},
                "joints":[
                 {"name":"a","bus":"s","driver":"sim","id":1,"gear_ratio":1,"direction":1,
                  "limits":{"min_rad":-1,"max_rad":1,"max_torque_nm":1,"max_velocity_rad_s":1}},
                 {"name":"b","bus":"s","driver":"sim","id":2,"gear_ratio":1,"direction":1,
                  "limits":{"min_rad":-1,"max_rad":1,"max_torque_nm":1,"max_velocity_rad_s":1}}]}"#,
        )
        .unwrap();
        let json: serde_json::Value = serde_json::from_str(&describe_json(&config)).unwrap();
        assert_eq!(json["protocol"], 1);
        assert_eq!(json["period_ns"], 2_000_000);
        assert_eq!(json["joints"][1]["name"], "b");
        assert_eq!(json["joints"][1]["index"], 1);
        assert_eq!(json["poses"]["base"]["b"], 0.1);
    }
}
