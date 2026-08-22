//! A small flag parser.
//!
//! Deliberately small. A robot binary that has to be copied onto a machine and
//! run is worth keeping free of dependencies it does not need, and what the
//! subcommands here need is `-name value`, `-name=value`, `-flag`, and
//! `-opt key=value` repeated. That is sixty lines, and it can be tested.

use std::collections::BTreeMap;

/// The flags and positional arguments one subcommand was given.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Flags {
    values: BTreeMap<String, String>,
    repeated: BTreeMap<String, Vec<String>>,
    pub positional: Vec<String>,
}

/// What a flag takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// `-name value` or `-name=value`.
    Value,
    /// `-name`, with no argument.
    Switch,
    /// `-name key=value`, more than once.
    Repeated,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FlagError {
    #[error("unknown flag {0:?}")]
    Unknown(String),
    #[error("flag {0:?} needs a value")]
    MissingValue(String),
    #[error("{flag} {value:?} must be key=value")]
    NotAPair { flag: String, value: String },
}

impl Flags {
    /// Parses `args` against a table of what each flag takes.
    ///
    /// An unrecognised flag is an error rather than a positional argument: a
    /// mistyped `-port` silently becoming a filename is how somebody ends up
    /// opening the wrong thing.
    pub fn parse(args: &[String], known: &[(&str, Kind)]) -> Result<Self, FlagError> {
        let mut out = Flags::default();
        let mut index = 0;

        while index < args.len() {
            let argument = &args[index];
            index += 1;

            let Some(rest) = argument.strip_prefix('-') else {
                out.positional.push(argument.clone());
                continue;
            };
            let rest = rest.strip_prefix('-').unwrap_or(rest);

            let (name, inline) = match rest.split_once('=') {
                Some((name, value)) => (name, Some(value.to_string())),
                None => (rest, None),
            };

            let kind = known
                .iter()
                .find(|(known, _)| *known == name)
                .map(|(_, kind)| *kind)
                .ok_or_else(|| FlagError::Unknown(format!("-{name}")))?;

            match kind {
                Kind::Switch => {
                    out.values
                        .insert(name.to_string(), inline.unwrap_or_else(|| "true".into()));
                }
                Kind::Value | Kind::Repeated => {
                    let value = match inline {
                        Some(value) => value,
                        None => {
                            let value = args
                                .get(index)
                                .ok_or_else(|| FlagError::MissingValue(format!("-{name}")))?;
                            index += 1;
                            value.clone()
                        }
                    };
                    if kind == Kind::Repeated {
                        if !value.contains('=') {
                            return Err(FlagError::NotAPair {
                                flag: format!("-{name}"),
                                value,
                            });
                        }
                        out.repeated
                            .entry(name.to_string())
                            .or_default()
                            .push(value);
                    } else {
                        out.values.insert(name.to_string(), value);
                    }
                }
            }
        }
        Ok(out)
    }

    pub fn string(&self, name: &str, default: &str) -> String {
        self.values
            .get(name)
            .cloned()
            .unwrap_or_else(|| default.to_string())
    }

    pub fn switch(&self, name: &str) -> bool {
        self.values.get(name).map(|v| v == "true").unwrap_or(false)
    }

    /// A numeric flag, or the default when it was not given.
    ///
    /// A flag that was given and is not a number is an error rather than the
    /// default: silently running at the default rate because somebody typed
    /// `-rate 10O` is exactly the kind of thing that gets diagnosed as a
    /// hardware fault.
    pub fn number<T: std::str::FromStr>(&self, name: &str, default: T) -> Result<T, String> {
        match self.values.get(name) {
            Some(text) => text
                .parse()
                .map_err(|_| format!("-{name} {text:?} is not a number")),
            None => Ok(default),
        }
    }

    /// The `key=value` pairs of a repeated flag.
    pub fn pairs(&self, name: &str) -> BTreeMap<String, String> {
        self.repeated
            .get(name)
            .map(|values| {
                values
                    .iter()
                    .filter_map(|value| value.split_once('='))
                    .map(|(key, value)| (key.to_string(), value.to_string()))
                    .collect()
            })
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(text: &str) -> Vec<String> {
        text.split_whitespace().map(String::from).collect()
    }

    const KNOWN: &[(&str, Kind)] = &[
        ("driver", Kind::Value),
        ("id", Kind::Value),
        ("check", Kind::Switch),
        ("opt", Kind::Repeated),
    ];

    #[test]
    fn a_flag_takes_its_value_either_way_round() {
        let flags = Flags::parse(&args("-driver moteus -id=3"), KNOWN).unwrap();
        assert_eq!(flags.string("driver", ""), "moteus");
        assert_eq!(flags.number("id", 1u8).unwrap(), 3);
    }

    #[test]
    fn a_switch_needs_no_value() {
        let flags = Flags::parse(&args("-check robot.json"), KNOWN).unwrap();
        assert!(flags.switch("check"));
        assert_eq!(flags.positional, vec!["robot.json"]);
        assert!(!flags.switch("missing"));
    }

    #[test]
    fn a_repeated_flag_collects_its_pairs() {
        let flags = Flags::parse(&args("-opt profile=xl330 -opt no_brs=1"), KNOWN).unwrap();
        let pairs = flags.pairs("opt");
        assert_eq!(pairs["profile"], "xl330");
        assert_eq!(pairs["no_brs"], "1");
    }

    /// A mistyped flag silently becoming a filename is how somebody ends up
    /// opening the wrong thing.
    #[test]
    fn an_unknown_flag_is_refused_rather_than_taken_as_a_filename() {
        let error = Flags::parse(&args("-prot COM5"), KNOWN).unwrap_err();
        assert_eq!(error, FlagError::Unknown("-prot".into()));
    }

    #[test]
    fn a_flag_with_no_value_says_so() {
        let error = Flags::parse(&args("-driver"), KNOWN).unwrap_err();
        assert_eq!(error, FlagError::MissingValue("-driver".into()));
    }

    /// Running at the default rate because somebody typed a letter into a
    /// number is exactly the kind of thing that gets diagnosed as a hardware
    /// fault.
    #[test]
    fn a_number_that_is_not_a_number_is_an_error_not_the_default() {
        let flags = Flags::parse(&args("-id 1O"), KNOWN).unwrap();
        assert!(flags.number::<u32>("id", 1).is_err());
        // But an absent flag is simply the default.
        let flags = Flags::parse(&args(""), KNOWN).unwrap();
        assert_eq!(flags.number("id", 7u32).unwrap(), 7);
    }

    #[test]
    fn a_repeated_flag_that_is_not_a_pair_is_refused() {
        let error = Flags::parse(&args("-opt profile"), KNOWN).unwrap_err();
        assert!(matches!(error, FlagError::NotAPair { .. }), "{error:?}");
    }

    /// Both spellings of a long flag work, because muscle memory does not care.
    #[test]
    fn one_dash_and_two_are_the_same_flag() {
        let one = Flags::parse(&args("-driver moteus"), KNOWN).unwrap();
        let two = Flags::parse(&args("--driver moteus"), KNOWN).unwrap();
        assert_eq!(one, two);
    }
}
