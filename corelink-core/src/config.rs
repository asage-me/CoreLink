//! Config value types and pure validation (design D8).
//!
//! Serde-only types — the contract that the daemon and GUI build against.
//! `validate` is pure: config-time problems surface here as structured
//! errors, never panics, and never at tick time.
