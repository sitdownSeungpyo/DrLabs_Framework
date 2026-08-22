//! What can go wrong, and the difference between a failure and an answer.
//!
//! A refusal is not an error in the transport sense. "Arm the axis first" is
//! the machine telling the operator something true, and the Go framework had
//! to say so in a comment on every layer that carried one. Here refusals are
//! their own variant, so a caller can tell them apart without reading strings.

use std::time::Duration;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The link is not open, or was closed underneath the caller.
    #[error("the link is closed")]
    Closed,

    /// The device did not answer inside the transaction budget.
    ///
    /// A single one of these is a dropped cycle, not a failure: the loop counts
    /// it and carries on, and only a run of them is fatal. Cancelling a
    /// transaction because the control period elapsed is a different thing and
    /// is never done — it leaves the adapter's pending-request state
    /// inconsistent, after which every following transaction returns empty.
    #[error("no reply within {0:?}")]
    Timeout(Duration),

    /// The transport itself failed: a port that went away, a cable pulled.
    #[error("{operation}: {source}")]
    Transport {
        operation: &'static str,
        #[source]
        source: Box<dyn std::error::Error + Send + Sync>,
    },

    /// The device answered, and said no. This is information.
    #[error("{0}")]
    Refused(String),

    /// The device answered with something this build cannot parse.
    #[error("malformed reply: {0}")]
    Malformed(String),

    /// The driver cannot express this mode.
    #[error("{driver} does not support {mode} mode")]
    UnsupportedMode {
        driver: &'static str,
        mode: crate::ControlMode,
    },

    /// The configuration is wrong, and was caught before anything opened.
    #[error("{0}")]
    Config(String),
}

impl Error {
    pub fn transport<E>(operation: &'static str, source: E) -> Self
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        Error::Transport {
            operation,
            source: Box::new(source),
        }
    }

    pub fn refused(message: impl Into<String>) -> Self {
        Error::Refused(message.into())
    }

    pub fn config(message: impl Into<String>) -> Self {
        Error::Config(message.into())
    }

    /// Whether this is the device declining, rather than the link failing.
    ///
    /// The difference decides what a caller does: a refusal is shown to the
    /// operator as-is, a transport failure is retried or escalated.
    pub fn is_refusal(&self) -> bool {
        matches!(self, Error::Refused(_))
    }

    /// Whether the machine should be considered to have lost its link.
    pub fn is_link_failure(&self) -> bool {
        matches!(self, Error::Closed | Error::Transport { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refusal_is_distinguishable_from_a_broken_link() {
        let refusal = Error::refused("arm the axis first");
        assert!(refusal.is_refusal());
        assert!(!refusal.is_link_failure());

        let closed = Error::Closed;
        assert!(!closed.is_refusal());
        assert!(closed.is_link_failure());
    }

    #[test]
    fn a_timeout_is_neither() {
        let timeout = Error::Timeout(Duration::from_secs(2));
        assert!(!timeout.is_refusal());
        assert!(!timeout.is_link_failure());
    }
}
