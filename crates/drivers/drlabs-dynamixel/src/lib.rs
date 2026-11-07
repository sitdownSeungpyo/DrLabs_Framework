//! ROBOTIS Dynamixel servos, over Protocol 2.0 on a serial bus.
//!
//! One driver, not two. The Go tree carried three layers that had accumulated
//! rather than been designed — `legacy/dxl`, `drivers/dynamixel/driver.go` and
//! the `Link` that replaced them — and every capability of the older two is
//! here: register reads and writes, ping, reboot, factory reset, and the sync
//! read and sync write that let a daisy chain move on one period instead of one
//! per servo.
//!
//! What is not here is the goroutine-and-channel calling convention the oldest
//! layer wrapped around them. That was a way of *calling* the driver, not a
//! thing the driver could do, and keeping a second copy of it would mean two
//! places to fix the next time the wire turns out to work differently than
//! anyone thought.

pub mod bus;
pub mod chain;
pub mod driver;
pub mod link;
pub mod profile;
pub mod protocol;

pub use bus::{DynamixelBus, Limiter, TorqueOverride};

/// Reads a joint's native torque ceiling from its options:
/// `current_limit_ma` (Goal Current, mA) and `pwm_limit` (Goal PWM counts).
/// A value that is not a whole number in range is an error naming the key.
pub fn torque_override(
    options: &std::collections::BTreeMap<String, String>,
) -> Result<TorqueOverride, String> {
    let read = |key: &str, max: u16| -> Result<Option<u16>, String> {
        match options.get(key) {
            None => Ok(None),
            Some(text) => match text.trim().parse::<u16>() {
                Ok(value) if (1..=max).contains(&value) => Ok(Some(value)),
                _ => Err(format!("{key} {text:?} must be a whole number in 1..{max}")),
            },
        }
    };
    let value = TorqueOverride {
        current_ma: read("current_limit_ma", i16::MAX as u16)?,
        pwm: read("pwm_limit", profile::PWM_FULL_SCALE)?,
    };
    if value.current_ma.is_some() && value.pwm.is_some() {
        return Err("give current_limit_ma or pwm_limit, not both".into());
    }
    Ok(value)
}
pub use driver::Driver;
pub use link::DynamixelLink;
pub use profile::{Profile, XL330, X_SERIES};
