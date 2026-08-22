//! The mjbots moteus brushless controller family, in pure Rust.
//!
//! The wire details follow the moteus reference documentation; `docs/MOTEUS.md`
//! records what this implementation depends on and why. Everything this crate
//! exposes upward is in framework units, so callers work against
//! `drlabs_core::Link` and never see a register.

pub mod combiner;
pub mod controller;
pub mod frames;
pub mod hardware;
pub mod link;
pub mod multiplex;
pub mod query;
pub mod registers;
pub mod stream;
