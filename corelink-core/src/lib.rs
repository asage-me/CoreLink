//! `corelink-core` — the pure fan-control core of CoreLink Phase 1.
//!
//! The entire per-tick fan control pipeline lives here with **zero I/O**:
//! no file access, no USB, no device/hwmon concepts, no internal clock.
//! Timestamps, sensor readings, and the last-commanded PWM are injected per
//! tick by the daemon (a later change); the core emits exactly one
//! [`Outcome`](profile::Outcome) per profile per tick.
//!
//! Module map:
//! - [`config`]  — serde config value types + pure validation (design D8)
//! - [`ema`]     — time-corrected exponential moving average (design D3)
//! - [`curve`]   — piecewise-linear curve lookup (design D5)
//! - [`pid`]     — PID controller with back-calculation anti-windup (design D4)
//! - [`slew`]    — asymmetric slew-rate limiting
//! - [`tripwire`]- per-source tripwire state, shared across profiles (design D2)
//! - [`profile`] — the `ProfileHandle::tick` pipeline (design D1, D6, D7)

pub mod config;
pub mod curve;
pub mod ema;
pub mod fold;
pub mod pid;
pub mod profile;
pub mod slew;
pub mod tripwire;
