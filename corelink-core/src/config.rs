//! Config value types and pure validation (design D8).
//!
//! Serde-only types — the contract that the daemon and GUI build against.
//! `validate` is pure: config-time problems surface here as structured
//! errors, never panics, and never at tick time.
//!
//! The schema is two tables:
//! - a `sensor_sources` map of opaque [`SourceKey`] → [`SensorSourceConfig`];
//!   tripwire tuning lives *per source* (design D8 — a fact about the system,
//!   shared by every profile referencing it).
//! - `profiles: Vec<FanProfileConfig>`, each declaring which source keys
//!   feed it and in what role.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Opaque sensor-source key, chosen by the daemon (e.g. a normalized hwmon
/// path or PCIe BDF). The core stores readings keyed by this and never
/// inspects it; the daemon guarantees uniqueness across sources.
pub type SourceKey = String;

// ---------------------------------------------------------------------------
// Top-level config
// ---------------------------------------------------------------------------

/// Complete fan-control configuration: the per-source sensor table plus the
/// fan profiles. The daemon deserializes this and hands the core one
/// profile at a time (see [`profile`](crate::profile)).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FanControlConfig {
    /// Per-source sensor declarations, keyed by source key. Tripwire tuning
    /// lives here (design D8), so every profile referencing the source sees
    /// the same tuning.
    #[serde(default)]
    pub sensor_sources: BTreeMap<SourceKey, SensorSourceConfig>,
    /// Fan profiles (one per owned PWM port).
    #[serde(default)]
    pub profiles: Vec<FanProfileConfig>,
    /// Staleness bound in seconds: the *true* (unclamped) age of a source's
    /// newest sample, relative to the injected timestamp, at which the source
    /// is considered stale and its profile fail-safes. Default 10 s.
    #[serde(default = "defaults::max_staleness_s")]
    pub max_staleness_s: f64,
    /// Maximum filter-integration step in seconds: if the injected tick delta
    /// exceeds this, EMA and PID integrate with the clamped delta and the
    /// tick emits a fault. The staleness check always uses the *true* gap
    /// (design D3). Default 5 s.
    #[serde(default = "defaults::max_tick_gap_s")]
    pub max_tick_gap_s: f64,
}

impl Default for FanControlConfig {
    fn default() -> Self {
        Self {
            sensor_sources: BTreeMap::new(),
            profiles: Vec::new(),
            max_staleness_s: defaults::max_staleness_s(),
            max_tick_gap_s: defaults::max_tick_gap_s(),
        }
    }
}

/// Per-source sensor declaration. At most one role (comfort / tripwire /
/// display_only) per source (the core validates this; see design D8 — the
/// role declaration *is* the complete description of how the value may reach
/// a controller).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorSourceConfig {
    /// `display_only` sources never touch the fan path (GUI/telemetry only).
    #[serde(default)]
    pub display_only: bool,
    /// `comfort_driver` role declaration (feeds the mode controller through
    /// an EMA).
    pub comfort: Option<ComfortSourceConfig>,
    /// `tripwire` role declaration (protection only; never enters the mode
    /// output).
    pub tripwire: Option<TripwireConfig>,
}

/// Comfort-driver role settings. `required` defaults to `true`: a required
/// source that is absent/stale fail-safes its profile; an optional one only
/// soft-degrades the fold (spec: *Required-vs-Optional Sensor Sources*).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComfortSourceConfig {
    /// EMA time constant in seconds (τ ≈ 1–2 s suggested).
    #[serde(default = "defaults::comfort_ema_tau_s")]
    pub ema_tau_s: f64,
    /// Whether the profile fail-safes when this source is missing/stale.
    /// Default `true`.
    #[serde(default = "defaults::comfort_required")]
    pub required: bool,
}

/// Tripwire role settings (per source — shared by all referencing profiles,
/// design D8: per-profile tuning would silently diverge latches).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TripwireConfig {
    /// Absolute trip threshold in °C (smoothed value ≥ threshold latches).
    pub threshold_c: f64,
    /// dT/dt spike rate in °C/s; the smoothed rate of change ≥ this latches.
    /// Disabled until two samples exist. Must be > 0.
    #[serde(default = "defaults::spike_rate_c_per_s")]
    pub spike_rate_c_per_s: f64,
    /// Tripwire EMA time constant; suggested shorter than the comfort τ
    /// (hotspot dynamics are fast).
    #[serde(default = "defaults::tripwire_ema_tau_s")]
    pub ema_tau_s: f64,
    /// Hysteresis in °C: the latch clears at `threshold_c − hysteresis_c`
    /// (≤, on that tick). Must be > 0.
    #[serde(default = "defaults::hysteresis_c")]
    pub hysteresis_c: f64,
    /// Protection floor target in % PWM while latched (0–100).
    #[serde(default = "defaults::protection_target")]
    pub protection_target: f64,
}

// ---------------------------------------------------------------------------
// Fan profiles
// ---------------------------------------------------------------------------

/// One fan profile (one owned PWM port — the daemon maps profiles to
/// channels; one-profile-one-port is enforced there, design D2).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FanProfileConfig {
    /// Stable profile id (names the port binding in the daemon config); also
    /// used in validation error paths.
    pub id: String,
    /// Control mode. Exactly one per profile.
    pub mode: Mode,
    /// `static_percent` mode: fixed duty 0–100. Required iff
    /// [`Mode::StaticPercent`]; must be absent otherwise.
    pub static_percent: Option<f64>,
    /// `curve` mode: piecewise-linear points (temp °C, pwm %). Required iff
    /// [`Mode::Curve`], with ≥ 2 distinct-temperature points; must be absent
    /// otherwise.
    pub curve_points: Option<Vec<CurvePoint>>,
    /// `target_temp` mode: setpoint in °C. Required iff [`Mode::TargetTemp`];
    /// must be absent otherwise.
    pub setpoint_c: Option<f64>,
    /// `target_temp` mode: PID gains; defaults to Kp=2, Ki=0.2, Kd=0 when
    /// absent. Ignored (inert) in other modes.
    pub gains: Option<PidGains>,
    /// Minimum duty floor in % for `curve`/`target_temp` outputs (default 0
    /// — off is allowed until the user says otherwise). Explicit
    /// `static_percent` values and the failsafe are exempt.
    #[serde(default)]
    pub min_duty: f64,
    /// Fold rule reducing comfort drivers to one scalar. Default `max`.
    #[serde(default)]
    pub fold: FoldRule,
    /// Asymmetric slew rates (default 20 %/s up, 7 %/s down).
    #[serde(default)]
    pub slew: SlewRates,
    /// Comfort-driver source keys (with optional per-profile overrides).
    #[serde(default)]
    pub comfort: Vec<ComfortSourceRef>,
    /// Tripwire source keys (tuning from the source table).
    #[serde(default)]
    pub tripwires: Vec<TripwireRef>,
    /// Display-only source keys (GUI/telemetry; never enter the fan path).
    #[serde(default)]
    pub display_only: Vec<SourceKey>,
}

/// Per-profile reference to a comfort-driver source. The key resolves against
/// the global `sensor_sources` table; `ema_tau_s` / `required` are optional
/// per-profile overrides of the source-table values.
///
/// Deserializes from either the bare key string (`"gpu0-edge"`) or the full
/// object (`{ "key": "gpu0-edge", "required": false }`); always serializes as
/// the full object.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct ComfortSourceRef {
    pub key: SourceKey,
    /// Per-profile EMA τ override (s); falls back to the source table when
    /// `None`.
    pub ema_tau_s: Option<f64>,
    /// Per-profile required override; falls back to the source table when
    /// `None`.
    pub required: Option<bool>,
}

impl<'de> Deserialize<'de> for ComfortSourceRef {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged, deny_unknown_fields)]
        enum V {
            Key(String),
            Full {
                key: SourceKey,
                #[serde(default)]
                ema_tau_s: Option<f64>,
                #[serde(default)]
                required: Option<bool>,
            },
        }
        Ok(match V::deserialize(d)? {
            V::Key(key) => Self {
                key,
                ema_tau_s: None,
                required: None,
            },
            V::Full {
                key,
                ema_tau_s,
                required,
            } => Self {
                key,
                ema_tau_s,
                required,
            },
        })
    }
}

/// Per-profile reference to a tripwire source (no per-profile tuning —
/// design D8). Accepts the bare key string or `{ "key": "..." }`.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct TripwireRef {
    pub key: SourceKey,
}

impl<'de> Deserialize<'de> for TripwireRef {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged, deny_unknown_fields)]
        enum V {
            Key(String),
            Full {
                key: SourceKey,
            },
        }
        Ok(match V::deserialize(d)? {
            V::Key(key) => Self { key },
            V::Full { key } => Self { key },
        })
    }
}

/// Control mode. Exactly one per profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// Fixed duty; no comfort drivers; MAY declare tripwires (its only
    /// possible protection).
    StaticPercent,
    /// Piecewise-linear curve lookup (gain-1 follower, no PID).
    Curve,
    /// Full PID driving the folded temperature to a setpoint.
    TargetTemp,
    /// CoreLink does not own the port; never emits a command.
    DeviceMemory,
}

/// (temperature °C, pwm %) curve point.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CurvePoint {
    pub temp_c: f64,
    pub pwm: f64,
}

/// PID gains. Defaults: Kp = 2 %/°C, Ki = 0.2 %/(°C·s), Kd = 0 — documented
/// starting values for a ~1 Hz loop, deliberately biased low (start low,
/// raise Kp if it sags, add Ki if it creeps) and to be tuned in the daemon
/// change (Phase 5 health monitoring).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PidGains {
    /// Proportional gain, % per °C of error (error = folded °C − setpoint °C,
    /// positive when hot).
    #[serde(default = "defaults::kp")]
    pub kp: f64,
    /// Integral gain, % per °C·s of accumulated error.
    #[serde(default = "defaults::ki")]
    pub ki: f64,
    /// Derivative gain, %·s per °C/s of error (derivative on error, design
    /// D4).
    #[serde(default = "defaults::kd")]
    pub kd: f64,
}

/// Fold rule: reduces the comfort drivers' smoothed values to one scalar.
/// Declared rules always produce a value within the observed range of their
/// inputs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FoldRule {
    /// Hottest driver wins (conservative; the default).
    #[default]
    Max,
    /// Equal-weight arithmetic mean.
    Avg,
}

/// Asymmetric slew rates. Defaults: up 20 %/s, down 7 %/s (suggested band:
/// up 15–25 %/s, down 5–10 %/s).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SlewRates {
    /// Upward rate in %/s (> 0).
    #[serde(default = "defaults::slew_up")]
    pub up: f64,
    /// Downward rate in %/s (> 0).
    #[serde(default = "defaults::slew_down")]
    pub down: f64,
}

impl Default for SlewRates {
    fn default() -> Self {
        Self {
            up: defaults::slew_up(),
            down: defaults::slew_down(),
        }
    }
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

/// A single validation error: a config path plus a human-readable message.
#[derive(Debug, Clone, PartialEq)]
pub struct ValidationError {
    /// Path to the offending field, e.g. `profiles[aio_fan].curve_points`.
    pub path: String,
    pub message: String,
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.path, self.message)
    }
}

/// All validation errors for a config (empty ⇒ valid).
#[derive(Debug, Default)]
pub struct ValidationErrors {
    errors: Vec<ValidationError>,
}

impl ValidationErrors {
    /// Push one error.
    pub fn push(&mut self, path: impl Into<String>, message: impl Into<String>) {
        self.errors.push(ValidationError {
            path: path.into(),
            message: message.into(),
        });
    }

    /// True when validation found no problems.
    pub fn is_empty(&self) -> bool {
        self.errors.is_empty()
    }

    /// Number of errors.
    pub fn len(&self) -> usize {
        self.errors.len()
    }

    /// The errors.
    pub fn into_errors(self) -> impl Iterator<Item = ValidationError> {
        self.errors.into_iter()
    }
}

/// Validate the whole config: top-level bounds, source-table coherence, and
/// every profile (via [`validate_profile`]). Returns *all* errors found
/// (empty ⇒ valid); never panics.
pub fn validate(config: &FanControlConfig) -> ValidationErrors {
    let mut e = ValidationErrors::default();

    if !(config.max_staleness_s.is_finite() && config.max_staleness_s > 0.0) {
        e.push("max_staleness_s", format!("{} must be finite and > 0", config.max_staleness_s));
    }
    if !(config.max_tick_gap_s.is_finite() && config.max_tick_gap_s > 0.0) {
        e.push("max_tick_gap_s", format!("{} must be finite and > 0", config.max_tick_gap_s));
    }

    for (key, src) in &config.sensor_sources {
        let base = format!("sensor_sources[{key}]");
        let role_count = u8::from(src.comfort.is_some())
            + u8::from(src.tripwire.is_some())
            + u8::from(src.display_only);
        if role_count > 1 {
            e.push(&base, "a source may declare at most one role (comfort / tripwire / display_only)");
        }
        if let Some(t) = &src.tripwire {
            validate_tripwire(&base, t, &mut e);
        }
        if let Some(c) = &src.comfort {
            if !(c.ema_tau_s.is_finite() && c.ema_tau_s > 0.0) {
                e.push(format!("{base}.comfort.ema_tau_s"), format!("tau {} must be finite and > 0", c.ema_tau_s));
            }
        }
    }

    let mut ids = std::collections::BTreeSet::new();
    for (i, profile) in config.profiles.iter().enumerate() {
        if !ids.insert(profile.id.as_str()) {
            e.push(format!("profiles[{i}].id"), format!("duplicate profile id '{}'", profile.id));
        }
        let mut pe = validate_profile(profile, &config.sensor_sources);
        e.errors.append(&mut pe.errors);
    }

    e
}

/// Validate one profile against the global source table.
///
/// Covers:
/// - **mode-field coherence** (spec *Mode is exclusive*): `curve` requires ≥ 2
///   distinct-temp points; `target_temp` requires setpoint (+ gains default);
///   `static_percent` requires a value and must not declare comfort sources
///   (tripwire/display allowed — its only possible protection);
///   `device_memory` needs no declarations;
/// - **sensor role checks** (spec *Missing comfort_driver rejected*):
///   temperature-driven modes require ≥ 1 comfort driver; every declared key
///   must exist in the table with the matching role;
/// - **bounds**: tau/hysteresis/spike-rate > 0, slew rates > 0,
///   0 ≤ min-duty ≤ 100, finite setpoint, finite non-negative gains,
///   in-range curve points; duplicate keys within a profile.
///
/// Never panics.
pub fn validate_profile(
    profile: &FanProfileConfig,
    sources: &BTreeMap<SourceKey, SensorSourceConfig>,
) -> ValidationErrors {
    let mut e = ValidationErrors::default();
    let p = |field: &str| format!("profiles[{}].{}", profile.id, field);

    // ---- scalar bounds -----------------------------------------------------
    if let Some(duty) = profile.static_percent {
        if !in_range0100(&duty) {
            e.push(p("static_percent"), format!("value {duty} out of range 0..=100"));
        }
    }
    if !in_range0100(&profile.min_duty) {
        e.push(p("min_duty"), format!("value {} out of range 0..=100", profile.min_duty));
    }
    if let Some(sp) = profile.setpoint_c {
        if !sp.is_finite() {
            e.push(p("setpoint_c"), "setpoint must be finite");
        }
    }
    if let Some(g) = &profile.gains {
        for (name, v) in [("gains.kp", g.kp), ("gains.ki", g.ki), ("gains.kd", g.kd)] {
            if !v.is_finite() || v < 0.0 {
                e.push(p(name), format!("{v} must be a finite value >= 0"));
            }
        }
    }
    for (name, v) in [("slew.up", profile.slew.up), ("slew.down", profile.slew.down)] {
        if !v.is_finite() || v <= 0.0 {
            e.push(p(name), format!("{v} must be a finite value > 0"));
        }
    }

    // ---- curve points (spec *Curve Mode*, design D5) ------------------------
    if let Some(points) = &profile.curve_points {
        let path = p("curve_points");
        if points.len() < 2 {
            e.push(
                path.clone(),
                format!(
                    "needs at least 2 points (got {}); a single constant point degenerates to static_percent",
                    points.len()
                ),
            );
        }
        if points.iter().any(|c| !c.temp_c.is_finite()) {
            e.push(path.clone(), "all point temperatures must be finite");
        }
        if points.iter().any(|c| !in_range0100(&c.pwm)) {
            e.push(path.clone(), "all point pwm values must be in 0..=100");
        }
        let mut temps: Vec<f64> = points.iter().map(|c| c.temp_c).filter(|t| t.is_finite()).collect();
        temps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Less));
        for w in temps.windows(2) {
            let (a, b) = (w[0], w[1]);
            if b <= a {
                e.push(path, format!("duplicate or unmonotone temperature {b} (must be strictly increasing)"));
                break;
            }
        }
    }

    // ---- mode coherence ------------------------------------------------------
    match profile.mode {
        Mode::StaticPercent => {
            if profile.static_percent.is_none() {
                e.push(p("static_percent"), "required for static_percent mode");
            }
            if !profile.comfort.is_empty() {
                e.push(
                    p("comfort"),
                    "static_percent mode must not declare comfort drivers (the mode requires no sensor input; tripwires are the only protection it may declare)",
                );
            }
            check_absent(&mut e, &p, "curve_points", profile.curve_points.is_some(), "static_percent mode");
            check_absent(&mut e, &p, "setpoint_c", profile.setpoint_c.is_some(), "static_percent mode");
        }
        Mode::Curve => {
            if profile.curve_points.is_none() {
                e.push(p("curve_points"), "required for curve mode");
            }
            check_drivers(&mut e, &p, &profile.comfort);
            check_absent(&mut e, &p, "static_percent", profile.static_percent.is_some(), "curve mode");
            check_absent(&mut e, &p, "setpoint_c", profile.setpoint_c.is_some(), "curve mode");
        }
        Mode::TargetTemp => {
            if profile.setpoint_c.is_none() {
                e.push(p("setpoint_c"), "required for target_temp mode");
            }
            check_drivers(&mut e, &p, &profile.comfort);
            check_absent(&mut e, &p, "static_percent", profile.static_percent.is_some(), "target_temp mode");
            check_absent(&mut e, &p, "curve_points", profile.curve_points.is_some(), "target_temp mode");
        }
        Mode::DeviceMemory => {
            if !profile.comfort.is_empty() {
                e.push(p("comfort"), "device_memory mode declares no sensor input (CoreLink never owns the port for control)");
            }
            if !profile.tripwires.is_empty() {
                e.push(p("tripwires"), "device_memory mode cannot declare tripwires (no port owned by CoreLink for a protection floor to act on)");
            }
            if !profile.display_only.is_empty() {
                e.push(p("display_only"), "device_memory mode cannot declare display_only sources (CoreLink owns no port for them)");
            }
            check_absent(&mut e, &p, "static_percent", profile.static_percent.is_some(), "device_memory mode");
            check_absent(&mut e, &p, "curve_points", profile.curve_points.is_some(), "device_memory mode");
            check_absent(&mut e, &p, "setpoint_c", profile.setpoint_c.is_some(), "device_memory mode");
        }
    }

    // ---- tripwire refs ---------------------------------------------------------
    for (i, tr) in profile.tripwires.iter().enumerate() {
        let path = format!("profiles[{}].tripwires[{}]", profile.id, i);
        let Some(src) = sources.get(&tr.key) else {
            e.push(format!("{path}.key"), format!("source key '{}' not found in sensor_sources", tr.key));
            continue;
        };
        let Some(t) = &src.tripwire else {
            e.push(format!("{path}.key"), format!("source '{}' has no tripwire role declared", tr.key));
            continue;
        };
        validate_tripwire(&path, t, &mut e);
    }

    // ---- comfort refs ------------------------------------------------------------
    for (i, cr) in profile.comfort.iter().enumerate() {
        let path = format!("profiles[{}].comfort[{}]", profile.id, i);
        if let Some(tau) = cr.ema_tau_s {
            if !(tau.is_finite() && tau > 0.0) {
                e.push(format!("{path}.ema_tau_s"), format!("tau {tau} must be finite and > 0"));
            }
        }
        let Some(src) = sources.get(&cr.key) else {
            e.push(format!("{path}.key"), format!("source key '{}' not found in sensor_sources", cr.key));
            continue;
        };
        if src.comfort.is_none() {
            e.push(format!("{path}.key"), format!("source '{}' has no comfort_driver role declared", cr.key));
        }
    }

    // ---- display-only refs -------------------------------------------------------
    for (i, key) in profile.display_only.iter().enumerate() {
        let path = format!("profiles[{}].display_only[{}]", profile.id, i);
        let Some(src) = sources.get(key) else {
            e.push(path, format!("source key '{key}' not found in sensor_sources"));
            continue;
        };
        if !src.display_only {
            e.push(path, format!("source '{key}' is not declared display_only"));
        }
    }

    // ---- duplicates within the profile ---------------------------------------------
    check_duplicates(&mut e, &p("comfort"), profile.comfort.iter().map(|r| r.key.as_str()));
    check_duplicates(&mut e, &p("tripwires"), profile.tripwires.iter().map(|r| r.key.as_str()));
    check_duplicates(&mut e, &p("display_only"), profile.display_only.iter().map(|k| k.as_str()));

    e
}

/// Comfort sources are required in any temperature-driven mode (spec:
/// *Missing comfort_driver rejected* — the error names the profile).
fn check_drivers(e: &mut ValidationErrors, p: &impl Fn(&str) -> String, comfort: &[ComfortSourceRef]) {
    if comfort.is_empty() {
        e.push(
            p("comfort"),
            "temperature-driven mode (curve/target_temp) requires at least one comfort_driver source",
        );
    }
}

fn check_absent(
    e: &mut ValidationErrors,
    p: &impl Fn(&str) -> String,
    field: &str,
    is_set: bool,
    mode_name: &str,
) {
    if is_set {
        e.push(p(field), format!("must not be set in {mode_name}"));
    }
}

fn check_duplicates<'a>(
    e: &mut ValidationErrors,
    path: &str,
    keys: impl Iterator<Item = &'a str>,
) {
    let mut seen = std::collections::BTreeSet::new();
    for (i, key) in keys.enumerate() {
        if !seen.insert(key) {
            e.push(format!("{path}[{i}]"), "duplicate source key");
        }
    }
}

fn validate_tripwire(path: &str, t: &TripwireConfig, e: &mut ValidationErrors) {
    if !t.threshold_c.is_finite() {
        e.push(format!("{path}.threshold_c"), "threshold must be finite");
    }
    if !(t.spike_rate_c_per_s.is_finite() && t.spike_rate_c_per_s > 0.0) {
        e.push(
            format!("{path}.spike_rate_c_per_s"),
            format!("spike rate {} must be finite and > 0", t.spike_rate_c_per_s),
        );
    }
    if !(t.ema_tau_s.is_finite() && t.ema_tau_s > 0.0) {
        e.push(format!("{path}.ema_tau_s"), format!("tau {} must be finite and > 0", t.ema_tau_s));
    }
    if !(t.hysteresis_c.is_finite() && t.hysteresis_c > 0.0) {
        e.push(format!("{path}.hysteresis_c"), format!("hysteresis {} must be finite and > 0", t.hysteresis_c));
    }
    if !in_range0100(&t.protection_target) {
        e.push(
            format!("{path}.protection_target"),
            format!("protection target {} out of range 0..=100", t.protection_target),
        );
    }
}

fn in_range0100(v: &f64) -> bool {
    v.is_finite() && *v >= 0.0 && *v <= 100.0
}

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------

pub mod defaults {
    /// Comfort EMA τ default: 1.5 s (suggested band 1–2 s).
    pub fn comfort_ema_tau_s() -> f64 {
        1.5
    }
    /// Comfort source `required` default.
    pub fn comfort_required() -> bool {
        true
    }
    /// Tripwire EMA τ default: 0.3 s (shorter than comfort — hotspot is fast).
    pub fn tripwire_ema_tau_s() -> f64 {
        0.3
    }
    /// dT/dt spike rate default: 20 °C/s.
    pub fn spike_rate_c_per_s() -> f64 {
        20.0
    }
    /// Hysteresis default: 5 °C.
    pub fn hysteresis_c() -> f64 {
        5.0
    }
    /// Protection target default: 100 %.
    pub fn protection_target() -> f64 {
        100.0
    }
    /// Kp default: 2 %/°C.
    pub fn kp() -> f64 {
        2.0
    }
    /// Ki default: 0.2 %/(°C·s).
    pub fn ki() -> f64 {
        0.2
    }
    /// Kd default: 0.
    pub fn kd() -> f64 {
        0.0
    }
    /// Slew-up default: 20 %/s.
    pub fn slew_up() -> f64 {
        20.0
    }
    /// Slew-down default: 7 %/s.
    pub fn slew_down() -> f64 {
        7.0
    }
    /// Max staleness default: 10 s.
    pub fn max_staleness_s() -> f64 {
        10.0
    }
    /// Max tick gap default: 5 s (design D3).
    pub fn max_tick_gap_s() -> f64 {
        5.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn comfort() -> ComfortSourceConfig {
        ComfortSourceConfig {
            ema_tau_s: 1.5,
            required: true,
        }
    }

    fn tripwire() -> TripwireConfig {
        TripwireConfig {
            threshold_c: 80.0,
            spike_rate_c_per_s: 20.0,
            ema_tau_s: 0.3,
            hysteresis_c: 5.0,
            protection_target: 90.0,
        }
    }

    fn sources() -> BTreeMap<SourceKey, SensorSourceConfig> {
        BTreeMap::from([
            (
                "gpu0-edge".to_string(),
                SensorSourceConfig { display_only: false, comfort: Some(comfort()), tripwire: None },
            ),
            (
                "gpu0-hotspot".to_string(),
                SensorSourceConfig { display_only: false, comfort: None, tripwire: Some(tripwire()) },
            ),
            (
                "gpu0-mem".to_string(),
                SensorSourceConfig { display_only: true, comfort: None, tripwire: None },
            ),
        ])
    }

    fn ref_key(key: &str) -> ComfortSourceRef {
        ComfortSourceRef {
            key: key.to_string(),
            ema_tau_s: None,
            required: None,
        }
    }

    fn curve_profile() -> FanProfileConfig {
        FanProfileConfig {
            id: "aio_fan".into(),
            mode: Mode::Curve,
            static_percent: None,
            curve_points: Some(vec![
                CurvePoint { temp_c: 20.0, pwm: 20.0 },
                CurvePoint { temp_c: 90.0, pwm: 100.0 },
            ]),
            setpoint_c: None,
            gains: None,
            min_duty: 0.0,
            fold: FoldRule::default(),
            slew: SlewRates::default(),
            comfort: vec![ref_key("gpu0-edge")],
            tripwires: vec![TripwireRef { key: "gpu0-hotspot".into() }],
            display_only: vec!["gpu0-mem".into()],
        }
    }

    fn static_profile(duty: Option<f64>) -> FanProfileConfig {
        FanProfileConfig {
            id: "s".into(),
            mode: Mode::StaticPercent,
            static_percent: duty,
            curve_points: None,
            setpoint_c: None,
            gains: None,
            min_duty: 0.0,
            fold: FoldRule::default(),
            slew: SlewRates::default(),
            comfort: vec![],
            tripwires: vec![],
            display_only: vec![],
        }
    }

    fn dump(errs: ValidationErrors) -> Vec<String> {
        errs.into_errors().map(|e| e.to_string()).collect()
    }

    #[test]
    fn happy_path_curve_profile_is_valid() {
        let errs = dump(validate_profile(&curve_profile(), &sources()));
        assert!(errs.is_empty(), "{errs:?}");
    }

    #[test]
    fn full_config_valid() {
        let c = FanControlConfig {
            sensor_sources: sources(),
            profiles: vec![curve_profile()],
            ..FanControlConfig::default()
        };
        let errs = dump(validate(&c));
        assert!(errs.is_empty(), "{errs:?}");
    }

    #[test]
    fn curve_with_single_point_rejected() {
        let mut p = curve_profile();
        p.curve_points = Some(vec![CurvePoint { temp_c: 50.0, pwm: 40.0 }]);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("at least 2")));
    }

    #[test]
    fn curve_with_zero_points_rejected() {
        let mut p = curve_profile();
        p.curve_points = Some(vec![]);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("at least 2")));
    }

    #[test]
    fn curve_with_duplicate_temps_rejected() {
        let mut p = curve_profile();
        p.curve_points = Some(vec![
            CurvePoint { temp_c: 40.0, pwm: 20.0 },
            CurvePoint { temp_c: 40.0, pwm: 30.0 },
        ]);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("strictly increasing")));
    }

    #[test]
    fn curve_pwm_out_of_range_rejected() {
        let mut p = curve_profile();
        p.curve_points = Some(vec![
            CurvePoint { temp_c: 20.0, pwm: 20.0 },
            CurvePoint { temp_c: 90.0, pwm: 150.0 },
        ]);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("pwm values must be in 0..=100")));
    }

    #[test]
    fn static_with_comfort_rejected() {
        let mut p = static_profile(Some(45.0));
        p.comfort = vec![ref_key("gpu0-edge")];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("must not declare comfort")));
    }

    #[test]
    fn static_without_value_rejected() {
        assert!(dump(validate_profile(&static_profile(None), &sources()))
            .iter()
            .any(|m| m.contains("required for static_percent mode")));
    }

    #[test]
    fn static_out_of_range_rejected() {
        assert!(dump(validate_profile(&static_profile(Some(101.0)), &sources()))
            .iter()
            .any(|m| m.contains("0..=100")));
    }

    #[test]
    fn static_with_tripwire_is_allowed() {
        let mut p = static_profile(Some(45.0));
        p.tripwires = vec![TripwireRef { key: "gpu0-hotspot".into() }];
        assert!(dump(validate_profile(&p, &sources())).is_empty());
    }

    #[test]
    fn target_temp_without_setpoint_rejected() {
        let mut p = curve_profile();
        p.mode = Mode::TargetTemp;
        p.curve_points = None;
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("required for target_temp mode")));
    }

    #[test]
    fn target_temp_with_setpoint_is_valid() {
        let mut p = curve_profile();
        p.mode = Mode::TargetTemp;
        p.curve_points = None;
        p.setpoint_c = Some(55.0);
        assert!(dump(validate_profile(&p, &sources())).is_empty());
    }

    #[test]
    fn missing_comfort_driver_rejected_names_profile() {
        let mut p = curve_profile();
        p.comfort = vec![];
        let msgs = dump(validate_profile(&p, &sources()));
        assert!(msgs.iter().any(|m| m.starts_with("profiles[aio_fan].comfort")));
        assert!(msgs.iter().any(|m| m.contains("at least one comfort_driver")));
    }

    #[test]
    fn unknown_source_key_rejected() {
        let mut p = curve_profile();
        p.comfort = vec![ref_key("ghost")];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("not found in sensor_sources")));
    }

    #[test]
    fn comfort_ref_on_tripwire_source_rejected() {
        let mut p = curve_profile();
        p.comfort = vec![ref_key("gpu0-hotspot")];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("no comfort_driver role")));
    }

    #[test]
    fn tripwire_ref_on_comfort_source_rejected() {
        let mut p = curve_profile();
        p.tripwires = vec![TripwireRef { key: "gpu0-edge".into() }];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("no tripwire role")));
    }

    #[test]
    fn bad_comfort_tau_rejected() {
        let mut s = sources();
        s.insert(
            "bad".to_string(),
            SensorSourceConfig {
                display_only: false,
                comfort: Some(ComfortSourceConfig { ema_tau_s: 0.0, required: true }),
                tripwire: None,
            },
        );
        let errs = validate(&FanControlConfig { sensor_sources: s, ..FanControlConfig::default() });
        assert!(dump(errs).iter().any(|m| m.contains("comfort.ema_tau_s")));
    }

    #[test]
    fn bad_hysteresis_rejected() {
        let mut t = tripwire();
        t.hysteresis_c = 0.0;
        let mut s = sources();
        s.insert("hot".to_string(), SensorSourceConfig { display_only: false, comfort: None, tripwire: Some(t) });
        let errs = validate(&FanControlConfig { sensor_sources: s, ..FanControlConfig::default() });
        assert!(dump(errs).iter().any(|m| m.contains("hysteresis")));
    }

    #[test]
    fn bad_spike_rate_rejected() {
        let mut t = tripwire();
        t.spike_rate_c_per_s = -1.0;
        let mut s = sources();
        s.insert("hot".to_string(), SensorSourceConfig { display_only: false, comfort: None, tripwire: Some(t) });
        let errs = validate(&FanControlConfig { sensor_sources: s, ..FanControlConfig::default() });
        assert!(dump(errs).iter().any(|m| m.contains("spike rate")));
    }

    #[test]
    fn bad_protection_target_rejected() {
        let mut t = tripwire();
        t.protection_target = 101.0;
        let mut s = sources();
        s.insert("hot".to_string(), SensorSourceConfig { display_only: false, comfort: None, tripwire: Some(t) });
        let errs = validate(&FanControlConfig { sensor_sources: s, ..FanControlConfig::default() });
        assert!(dump(errs).iter().any(|m| m.contains("protection target")));
    }

    #[test]
    fn min_duty_out_of_range_rejected() {
        let mut p = curve_profile();
        p.min_duty = 101.0;
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("min_duty") && m.contains("0..=100")));
    }

    #[test]
    fn nonpositive_slew_rate_rejected() {
        let mut p = curve_profile();
        p.slew.down = 0.0;
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("slew.down")));
    }

    #[test]
    fn nonfinite_setpoint_rejected() {
        let mut p = curve_profile();
        p.mode = Mode::TargetTemp;
        p.curve_points = None;
        p.setpoint_c = Some(f64::NAN);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("setpoint must be finite")));
    }

    #[test]
    fn device_memory_needs_no_declarations() {
        let mut p = curve_profile();
        p.mode = Mode::DeviceMemory;
        p.curve_points = None;
        p.tripwires = vec![];
        p.display_only = vec![];
        p.comfort = vec![];
        let errs = dump(validate_profile(&p, &sources()));
        assert!(errs.is_empty(), "{errs:?}");
    }

    #[test]
    fn device_memory_rejects_comfort() {
        let mut p = curve_profile();
        p.mode = Mode::DeviceMemory;
        p.curve_points = None;
        p.tripwires = vec![];
        p.display_only = vec![];
        p.comfort = vec![ref_key("gpu0-edge")];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("device_memory mode declares no sensor input")));
    }

    #[test]
    fn display_only_ref_must_be_display_source() {
        let mut p = curve_profile();
        p.display_only = vec!["gpu0-edge".into()];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("not declared display_only")));
    }

    #[test]
    fn duplicate_comfort_keys_rejected() {
        let mut p = curve_profile();
        p.comfort = vec![ref_key("gpu0-edge"), ref_key("gpu0-edge")];
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("duplicate source key")));
    }

    #[test]
    fn mode_exclusive_fields_rejected() {
        // curve mode with a setpoint set → setpoint is inert and rejected
        let mut p = curve_profile();
        p.setpoint_c = Some(50.0);
        assert!(dump(validate_profile(&p, &sources()))
            .iter()
            .any(|m| m.contains("must not be set in curve mode")));
        // static mode with curve points set → rejected
        let mut s = static_profile(Some(45.0));
        s.curve_points = curve_profile().curve_points;
        assert!(dump(validate_profile(&s, &sources()))
            .iter()
            .any(|m| m.contains("must not be set in static_percent mode")));
    }

    #[test]
    fn source_double_role_rejected() {
        let s = BTreeMap::from([(
            "duo".to_string(),
            SensorSourceConfig {
                display_only: false,
                comfort: Some(comfort()),
                tripwire: Some(tripwire()),
            },
        )]);
        let errs = validate(&FanControlConfig { sensor_sources: s, ..FanControlConfig::default() });
        assert!(dump(errs).iter().any(|m| m.contains("at most one role")));
    }

    #[test]
    fn duplicate_profile_ids_rejected() {
        let c = FanControlConfig {
            sensor_sources: sources(),
            profiles: vec![curve_profile(), curve_profile()],
            ..FanControlConfig::default()
        };
        assert!(dump(validate(&c)).iter().any(|m| m.contains("duplicate profile id")));
    }

    #[test]
    fn serde_roundtrip_with_string_shorthand_and_defaults() {
        let json = r###"{
            "sensor_sources": {
                "a": { "comfort": { "ema_tau_s": 1.0 } }
            },
            "profiles": [
                {
                    "id": "p1",
                    "mode": "curve",
                    "curve_points": [
                        { "temp_c": 20, "pwm": 20 },
                        { "temp_c": 90, "pwm": 100 }
                    ],
                    "comfort": ["a"],
                    "tripwires": ["gpu0-hotspot"]
                }
            ]
        }"###;
        let c: FanControlConfig = serde_json::from_str(json).unwrap();
        assert_eq!(c.max_staleness_s, defaults::max_staleness_s());
        assert_eq!(c.max_tick_gap_s, defaults::max_tick_gap_s());
        assert_eq!(c.profiles[0].min_duty, 0.0);
        assert_eq!(c.profiles[0].fold, FoldRule::Max);
        assert_eq!(c.profiles[0].slew.up, defaults::slew_up());
        assert_eq!(c.profiles[0].slew.down, defaults::slew_down());
        assert_eq!(c.profiles[0].comfort[0].key, "a");
        assert!(c.profiles[0].comfort[0].required.unwrap_or(true));
        let mut sensor_sources = sources();
        sensor_sources.insert("a".into(), SensorSourceConfig { display_only: false, comfort: Some(comfort()), tripwire: None });
        let full = FanControlConfig {
            sensor_sources,
            profiles: c.profiles,
            ..FanControlConfig::default()
        };
        // the tripwire ref resolves against the full table now
        assert!(dump(validate(&full)).is_empty());
    }

    #[test]
    fn serde_roundtrip_full_profile() {
        let full = FanControlConfig {
            sensor_sources: sources(),
            profiles: vec![curve_profile()],
            ..FanControlConfig::default()
        };
        let json = serde_json::to_string(&full).unwrap();
        let back: FanControlConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back.profiles[0].comfort.len(), 1);
        assert_eq!(back.profiles[0].tripwires.len(), 1);
        assert!(dump(validate(&back)).is_empty());
    }
}
