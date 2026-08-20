//! The per-profile tick pipeline: `ProfileHandle::tick(&TickInput) -> TickResult`
//! (design D1), with the D6 pipeline ordering and the D7
//! health/cold-start state machine.
//!
//! One handle = one owned PWM port. The daemon feeds back exactly one fact
//! the core cannot know — the last commanded PWM — and hands over the shared
//! [`TripwireEval`]s, evaluated once per source per tick (design D2).
//!
//! D6 ordering (fixed once in design.md):
//! ```text
//!  1. Mode gate:          device_memory     -> NoCommand (report-only faults)
//!                         static_percent    -> fixed % (no health gate, no failsafe)
//!  2. Sensor health:      required missing/stale -> failsafe 100 (bypasses 3-8)
//!                         first seed tick  -> NoCommand (withhold, one tick)
//!  3. Filter updates:     comfort EMAs (gap-clamped), soft tripwire-loss faults
//!  4. Fold:               max (default) / avg of the usable smoothed values
//!  5. Mode controller:    curve lookup / PID tick
//!  6. Protection floor:   max(out, latched tripwires' protection targets)
//!  7. Min-duty floor:     max(out, min_duty)   [static & failsafe exempt]
//!  8. Slew:               step_toward(last_commanded_pwm, out, rate * dt)
//!  9. Emit:               Command{pwm, failsafe:false}
//! ```

use std::collections::BTreeMap;
use std::time::Duration;

use crate::config::{
    defaults, CurvePoint, FanControlConfig, FanProfileConfig, Mode, PidGains, SourceKey,
};
use crate::curve::Curve;
use crate::ema::Ema;
use crate::pid::Pid;
use crate::slew;
use crate::tripwire::TripwireEval;

/// One injected reading: source key → temperature °C. `None` is a
/// daemon-reported read error on a source that did deliver; a key absent
/// from the map did not deliver at all (see [`TickInput::readings`]).
pub type Readings = BTreeMap<SourceKey, Option<f64>>;

/// Per-tick inputs (design D1). All time comes from here — the core has no
/// clock.
pub struct TickInput<'a> {
    /// Injected monotonic timestamp.
    pub now: Duration,
    /// A usable reading (`Some(°C)`) per source, keyed by source. `None` is
    /// a read error on the source this tick; staleness is judged from the
    /// last *usable* sample, so `None` does not refresh it.
    pub readings: &'a Readings,
    /// Daemon feedback: the PWM currently applied to this port. Seeds the
    /// slew limiter every tick (design D2 — the core stores no position).
    pub last_commanded_pwm: f64,
    /// The shared tripwire evaluations for this tick (design D2), from the
    /// daemon's single `TripwireEvaluator` pass. An eval is `online: false`
    /// when the source did not deliver; its `latched` field is then the
    /// last known state and still carries the protection floor.
    pub tripwires: &'a [TripwireEval],
}

/// Exactly one outcome per profile per tick (spec *Single outcome per profile
/// per tick*). `NoCommand` vs `Command` vs failsafe is unrepresentable-as-wrong:
/// the failsafe is a `Command` carrying its flag, and `NoCommand` carries no
/// PWM at all.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    /// Emit `pwm`. `failsafe = true` is the fail-safe 100 % command, which
    /// bypasses the mode controller, protection floor, min-duty, and slew
    /// limiter (spec *Fail-Safe to 100%*).
    Command { pwm: f64, failsafe: bool },
    /// Do not touch the port this tick: the one cold-start seed tick (spec
    /// *Cold-Start Withholds Command*), or `device_memory` mode.
    NoCommand,
}

/// What went wrong on a tick (design D1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultKind {
    /// A comfort source has no usable value: never arrived, a present
    /// `None` read error, or the newest sample is past the staleness bound.
    ComfortSourceMissing,
    /// A comfort source that was healthy has gone quiet past
    /// `max_staleness_s` and must now be treated as missing.
    ComfortSourceStale,
    /// A declared tripwire source did not deliver this tick (soft: the
    /// last-known latch state holds; never a fail-safe).
    TripwireSourceLost,
    /// The tick delta exceeded `max_tick_gap_s`; the filters integrated the
    /// clamped gap (design D3).
    ClampedTickGap,
}

impl FaultKind {
    /// Stable machine-readable label.
    pub const fn as_str(self) -> &'static str {
        match self {
            FaultKind::ComfortSourceMissing => "comfort-source-missing",
            FaultKind::ComfortSourceStale => "comfort-source-stale",
            FaultKind::TripwireSourceLost => "tripwire-source-lost",
            FaultKind::ClampedTickGap => "clamped-tick-gap",
        }
    }
}

/// How seriously a fault should be surfaced (design D1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// The port is in fail-safe, or a required source is unusable.
    Error,
    /// Degraded but still controlled: lost tripwire, optional source down,
    /// tick-gap clamp.
    Warning,
}

/// A structured fault event attached to a tick (design D1). Failures are
/// never silent (spec *Fail-Safe to 100%*): every fault that accompanies a
/// `Command` names its source and kind.
#[derive(Debug, Clone, PartialEq)]
pub struct Fault {
    /// The affected sensor source (`<tick-gap>` for loop faults).
    pub source: SourceKey,
    pub kind: FaultKind,
    pub severity: Severity,
    /// Human-readable diagnostic (includes the profile id).
    pub message: String,
}

/// The result of one `ProfileHandle::tick` — one outcome plus its fault
/// events (design D1, D9: `#[must_use]`, never a `Result`).
#[derive(Debug, Clone, PartialEq)]
#[must_use]
pub struct TickResult {
    pub outcome: Outcome,
    pub faults: Vec<Fault>,
}

impl TickResult {
    /// The emitted PWM, `None` for a `NoCommand` tick.
    pub const fn pwm(&self) -> Option<f64> {
        match self.outcome {
            Outcome::Command { pwm, .. } => Some(pwm),
            Outcome::NoCommand => None,
        }
    }

    /// True when the fail-safe 100 % command was emitted this tick.
    pub const fn is_failsafe(&self) -> bool {
        matches!(self.outcome, Outcome::Command { failsafe: true, .. })
    }

    /// True when no fault events ride on this tick.
    pub fn is_clean(&self) -> bool {
        self.faults.is_empty()
    }
}

/// Per comfort-driver state (design D2: private to the port).
#[derive(Debug, Clone)]
struct ComfortSourceState {
    /// Required per config (per-profile override > source table > default).
    required: bool,
    /// Effective EMA τ (per-profile override > source table > default).
    tau_s: f64,
    /// The comfort EMA; seeded by the first usable sample.
    ema: Ema,
    /// Timestamp of the last *usable* sample — its true (unclamped) gap from
    /// this feeds the staleness check (design D3).
    last_update: Option<Duration>,
}

/// Per-profile state (design D2: everything here is private to the port).
pub struct ProfileHandle {
    /// The profile config this pipeline runs on (cloned at construction, so
    /// the handle is independent of the config map).
    profile: FanProfileConfig,
    /// Staleness bound for comfort sources (true-gap comparison).
    max_staleness: Duration,
    /// Tick-gap clamp for filter/PID integration (design D3).
    max_tick_gap: Duration,

    /// The one cold-start flag (design D7): false until the first
    /// comfort-mode tick, which is the `NoCommand` withhold.
    started: bool,
    /// Previous tick timestamp; `None` before the first tick (Δt = 0).
    last_tick: Option<Duration>,

    /// Comfort-driver state, keyed by source.
    comfort: BTreeMap<SourceKey, ComfortSourceState>,
    /// Declared tripwire sources (deduplicated, declaration order).
    tripwire_keys: Vec<SourceKey>,
    /// Latched protection target per tripwire source, from the source table
    /// (design D8 — tuning is per source, shared by all referencing
    /// profiles).
    tripwire_targets: BTreeMap<SourceKey, f64>,

    /// Curve lookup (`Curve` mode only). Config validation guarantees ≥2
    /// points; a hand-built degenerate config falls back to the identity
    /// line instead of panicking mid-pipeline.
    curve: Option<Curve>,
    /// PID controller (`TargetTemp` mode only — `Curve` mode has no
    /// integrator by construction, design D4).
    pid: Option<Pid>,
}

impl ProfileHandle {
    /// Build a handle from (validated) config.
    pub fn new(profile: &FanProfileConfig, control: &FanControlConfig) -> Self {
        let sources = &control.sensor_sources;

        let mut comfort = BTreeMap::new();
        for refc in &profile.comfort {
            if comfort.contains_key(&refc.key) {
                // Duplicate declaration: keep the first (deterministic).
                continue;
            }
            let (table_tau, table_required) =
                match sources.get(&refc.key).and_then(|s| s.comfort.as_ref()) {
                    Some(c) => (c.ema_tau_s, c.required),
                    None => (
                        defaults::comfort_ema_tau_s(),
                        defaults::comfort_required(),
                    ),
                };
            comfort.insert(
                refc.key.clone(),
                ComfortSourceState {
                    required: refc.required.unwrap_or(table_required),
                    tau_s: refc.ema_tau_s.unwrap_or(table_tau),
                    ema: Ema::new(),
                    last_update: None,
                },
            );
        }

        let mut tripwire_keys = Vec::new();
        let mut tripwire_targets = BTreeMap::new();
        for refc in &profile.tripwires {
            if tripwire_keys.contains(&refc.key) {
                continue;
            }
            let target = sources
                .get(&refc.key)
                .and_then(|s| s.tripwire.as_ref())
                .map(|t| t.protection_target)
                .unwrap_or_else(defaults::protection_target);
            tripwire_targets.insert(refc.key.clone(), target);
            tripwire_keys.push(refc.key.clone());
        }

        let curve = matches!(profile.mode, Mode::Curve).then(|| {
            match &profile.curve_points {
                Some(pts) if pts.len() >= 2 => Curve::new(pts.clone()),
                _ => Curve::new(vec![
                    CurvePoint {
                        temp_c: 0.0,
                        pwm: 0.0,
                    },
                    CurvePoint {
                        temp_c: 100.0,
                        pwm: 100.0,
                    },
                ]),
            }
        });

        let pid = matches!(profile.mode, Mode::TargetTemp)
            .then(|| Pid::new(profile.gains.unwrap_or_else(default_gains)));

        Self {
            profile: profile.clone(),
            max_staleness: Duration::from_secs_f64(control.max_staleness_s),
            max_tick_gap: Duration::from_secs_f64(control.max_tick_gap_s),
            started: false,
            last_tick: None,
            comfort,
            tripwire_keys,
            tripwire_targets,
            curve,
            pid,
        }
    }

    /// The profile id (names the port binding).
    pub fn profile_id(&self) -> &str {
        &self.profile.id
    }

    /// Run one tick (design D6 ordering).
    pub fn tick(&mut self, input: &TickInput) -> TickResult {
        let now = input.now;
        let dt = self
            .last_tick
            .map(|last| now.saturating_sub(last))
            .unwrap_or(Duration::ZERO);

        match self.profile.mode {
            // D6 step 1: `device_memory` never commands; faults are
            // report-only, labeled as affecting a device-managed port
            // (spec *device_memory never commands*).
            Mode::DeviceMemory => {
                let faults = self.report_only_faults(input.tripwires);
                self.last_tick = Some(now);
                TickResult {
                    outcome: Outcome::NoCommand,
                    faults,
                }
            }

            // D6 static branch (step 5 annotation): skips the comfort health
            // gate, filter updates, and fold entirely. No fail-safe path at
            // all; a declared tripwire is the only protection a static port
            // can have (spec *Fan Profile Modes*, *Protection Floor*).
            Mode::StaticPercent => {
                let base = self.profile.static_percent.unwrap_or(0.0);
                let out = self.protection_floor(input.tripwires, base);
                let out = self.slew(out, input.last_commanded_pwm, dt);
                let faults = self.report_only_faults(input.tripwires);
                self.last_tick = Some(now);
                TickResult {
                    outcome: Outcome::Command {
                        pwm: out.clamp(0.0, 100.0),
                        failsafe: false,
                    },
                    faults,
                }
            }

            // `curve` / `target_temp`: D6 steps 2–9.
            Mode::Curve | Mode::TargetTemp => self.tick_comfort_mode(input, now, dt),
        }
    }

    /// D6 steps 2–9 for the temperature-driven modes.
    fn tick_comfort_mode(
        &mut self,
        input: &TickInput,
        now: Duration,
        dt: Duration,
    ) -> TickResult {
        let max_staleness = self.max_staleness;

        // Classify every comfort source before any other state change (D6
        // step 2 runs before any filter update). A present `Some(°C)` is
        // usable this tick; `None` (read error) and absence are not, but a
        // present source still refreshes its last-seen time (health
        // bookkeeping, not filter state).
        // Collect the usable readings into owned keys first, so the map is
        // not tied to the lifetime of `self.comfort` (which is re-borrowed
        // for filter updates before it is dropped). A declared source is in
        // `present` iff it delivered a usable `Some(°C)` this tick.
        let mut present: BTreeMap<SourceKey, f64> = BTreeMap::new();
        for (key, st) in &mut self.comfort {
            if let Some(Some(x)) = input.readings.get(key) {
                st.last_update = Some(now);
                present.insert(key.clone(), *x);
            }
        }

        let mut faults: Vec<Fault> = Vec::new();

        if !self.started {
            // D7 cold start: one `NoCommand` withhold tick per profile
            // (spec *Cold-Start Withholds Command*). Seed what is present;
            // nothing has ever been unhealthy, so a missing source is a
            // report, not a fail-safe.
            self.started = true;
            for (key, st) in &self.comfort {
                if present.contains_key(key) {
                    continue;
                }
                faults.push(Fault {
                    source: key.clone(),
                    kind: FaultKind::ComfortSourceMissing,
                    severity: if st.required {
                        Severity::Error
                    } else {
                        Severity::Warning
                    },
                    message: format!(
                        "comfort source {key} did not arrive on the cold-start tick (required: {}; profile {})",
                        st.required, self.profile.id
                    ),
                });
            }
            // First sample seeds the EMA exactly (spec *First sample seeds
            // the filter*).
            for (key, &x) in &present {
                let st = self.comfort.get_mut(key).unwrap();
                st.ema.update(now, st.tau_s, x, self.max_tick_gap);
            }
            self.last_tick = Some(now);
            return TickResult {
                outcome: Outcome::NoCommand,
                faults,
            };
        }

        // Running: classify against the staleness bound (spec
        // *Required-vs-Optional*).
        let mut failsafe = false;
        for (key, st) in &self.comfort {
            if present.contains_key(key) {
                continue;
            }
            match st.last_update {
                None => {
                    // Never arrived: a required source is missing from the
                    // first decision tick (spec *Never-arrived sole source*).
                    if st.required {
                        failsafe = true;
                    }
                    faults.push(Fault {
                        source: key.clone(),
                        kind: FaultKind::ComfortSourceMissing,
                        severity: if st.required {
                            Severity::Error
                        } else {
                            Severity::Warning
                        },
                        message: format!(
                            "comfort source {key} never arrived (required: {}; profile {})",
                            st.required, self.profile.id
                        ),
                    });
                }
                Some(last) => {
                    let age = now.saturating_sub(last);
                    if age >= max_staleness {
                        // At/past the bound: a required source fails the
                        // port; an optional one only soft-degrades the fold
                        // (design D7).
                        if st.required {
                            failsafe = true;
                        }
                        faults.push(Fault {
                            source: key.clone(),
                            kind: FaultKind::ComfortSourceStale,
                            severity: if st.required {
                                Severity::Error
                            } else {
                                Severity::Warning
                            },
                            message: format!(
                                "comfort source {key} stale: no usable sample for {:.1}s (>= max_staleness_s {}; required: {}; profile {})",
                                age.as_secs_f64(),
                                max_staleness.as_secs_f64(),
                                st.required,
                                self.profile.id
                            ),
                        });
                    } else if !st.required {
                        // Optional source absent but still fresh: soft fault,
                        // no fail-safe (design D7).
                        faults.push(Fault {
                            source: key.clone(),
                            kind: FaultKind::ComfortSourceMissing,
                            severity: Severity::Warning,
                            message: format!(
                                "optional comfort source {key} has no sample this tick (profile {})",
                                self.profile.id
                            ),
                        });
                    }
                    // Required + fresh: a normal transient gap — no fault, no
                    // fail-safe until the bound (spec *Stale sample triggers
                    // fail-safe*).
                }
            }
        }

        if failsafe {
            self.last_tick = Some(now);
            // Fail-safe: bypasses mode controller, floors, and slew.
            return TickResult {
                outcome: Outcome::Command {
                    pwm: 100.0,
                    failsafe: true,
                },
                faults,
            };
        }

        // D6 step 3: update the comfort EMAs (time-corrected, gap-clamped).
        let mut clamped = false;
        for (key, st) in &mut self.comfort {
            if let Some(&x) = present.get(key) {
                let upd = st.ema.update(now, st.tau_s, x, self.max_tick_gap);
                clamped = clamped || upd.clamped;
            }
        }

        // Soft fault for declared tripwires that went offline this tick —
        // the last-known latch state carries the floor (spec *Soft Handling
        // of a Lost Tripwire Source*).
        faults.extend(self.tripwire_loss_faults(input.tripwires));

        // D3: the tick-gap clamp is only reportable when a filter actually
        // integrated a clamped gap (or the PID ran with a clamped Δt).
        if clamped
            || (matches!(self.profile.mode, Mode::TargetTemp) && dt > self.max_tick_gap)
        {
            faults.push(gap_clamp_fault(dt, self.max_tick_gap, &self.profile.id));
        }

        // D6 step 4: fold the usable smoothed values. Present sources
        // contribute their just-updated value; fresh-but-absent sources
        // (within the staleness budget) contribute their last smoothed
        // value. Deterministic key order (max/avg are order-independent
        // anyway).
        let mut values = Vec::new();
        for (key, st) in &self.comfort {
            let usable = if present.contains_key(key) {
                true
            } else {
                st.last_update
                    .map(|lu| now.saturating_sub(lu) < max_staleness)
                    .unwrap_or(false)
            };
            if usable {
                if let Some(v) = st.ema.value() {
                    values.push(v);
                }
            }
        }

        if values.is_empty() {
            // No usable measurement at all — every comfort driver missing
            // (the required ones failed us over above; the rest were
            // reported as optional faults). Spec *Fail-Safe*: "all comfort
            // drivers missing" → 100 %.
            self.last_tick = Some(now);
            return TickResult {
                outcome: Outcome::Command {
                    pwm: 100.0,
                    failsafe: true,
                },
                faults,
            };
        }

        let folded = self.profile.fold.fold(values.into_iter());

        // D6 step 5: the mode controller.
        let dt_clamped = dt.min(self.max_tick_gap);
        let mode_out = match self.profile.mode {
            Mode::Curve => {
                self.curve.as_ref().map(|c| c.lookup(folded)).unwrap_or(0.0)
            }
            Mode::TargetTemp => {
                let setpoint = self.profile.setpoint_c.unwrap_or(0.0);
                let pid = self
                    .pid
                    .as_mut()
                    .expect("target_temp profiles always carry a PID; see ProfileHandle::new");
                pid.update(folded - setpoint, dt_clamped)
            }
            _ => unreachable!("tick_comfort_mode is only reached for curve/target_temp"),
        };

        // D6 step 6: protection floor.
        let out = self.protection_floor(input.tripwires, mode_out);
        // D6 step 7: min-duty floor (curve/target_temp only — static and
        // the fail-safe are exempt by construction).
        let out = out.max(self.profile.min_duty);
        // D6 step 8: slew, seeded from the injected last-commanded PWM
        // (design D2).
        let out = self.slew(out, input.last_commanded_pwm, dt);

        // D6 step 9: emit.
        self.last_tick = Some(now);
        TickResult {
            outcome: Outcome::Command {
                pwm: out.clamp(0.0, 100.0),
                failsafe: false,
            },
            faults,
        }
    }

    /// D6 step 6: `max(mode_out, max protection_target of latched
    /// tripwires)` (spec *Protection Floor*). Offline tripwires carry their
    /// last-known state (spec *Soft Handling of a Lost Tripwire Source*).
    fn protection_floor(&self, tripwires: &[TripwireEval], mode_out: f64) -> f64 {
        let mut out = mode_out;
        for key in &self.tripwire_keys {
            let Some(ev) = tripwires.iter().find(|e| &e.key == key) else {
                continue; // Declared but not evaluated: no data, no floor.
            };
            if !ev.latched {
                continue;
            }
            if let Some(&target) = self.tripwire_targets.get(key) {
                out = out.max(target);
            }
        }
        out
    }

    /// D6 step 8: asymmetric slew toward the effective target, seeded every
    /// tick from the injected last-commanded PWM (design D2).
    fn slew(&self, target: f64, last_commanded_pwm: f64, dt: Duration) -> f64 {
        slew::step_toward(
            last_commanded_pwm,
            target,
            self.profile.slew.up,
            self.profile.slew.down,
            dt,
        )
        .pwm
    }

    /// Soft faults for declared tripwires that did not deliver this tick
    /// (or that the daemon never evaluated).
    fn tripwire_loss_faults(&self, tripwires: &[TripwireEval]) -> Vec<Fault> {
        let mut faults = Vec::new();
        for key in &self.tripwire_keys {
            let Some(ev) = tripwires.iter().find(|e| &e.key == key) else {
                faults.push(Fault {
                    source: key.clone(),
                    kind: FaultKind::TripwireSourceLost,
                    severity: Severity::Warning,
                    message: format!(
                        "tripwire source {key} was not evaluated this tick (profile {}); no last-known state, floor not applied",
                        self.profile.id
                    ),
                });
                continue;
            };
            if ev.online {
                continue;
            }
            let latch_text = if ev.latched {
                "last-known state LATCHED — protection floor still applies"
            } else {
                "last-known state clear — no floor applied"
            };
            faults.push(Fault {
                source: key.clone(),
                kind: FaultKind::TripwireSourceLost,
                severity: Severity::Warning,
                message: format!(
                    "tripwire source {key} offline (profile {}); {latch_text}",
                    self.profile.id
                ),
            });
        }
        faults
    }

    /// Faults for the modes outside the comfort path (`static_percent`,
    /// `device_memory`): tripwire loss remains soft; there is no comfort
    /// filter path for these modes to clamp, so no gap-clamp fault.
    /// `device_memory` labels its events as affecting a device-managed port
    /// (spec *device_memory never commands*).
    fn report_only_faults(&self, tripwires: &[TripwireEval]) -> Vec<Fault> {
        let mut faults = self.tripwire_loss_faults(tripwires);
        if matches!(self.profile.mode, Mode::DeviceMemory) && !faults.is_empty() {
            for f in &mut faults {
                f.message.push_str(" (device-managed port)");
            }
        }
        faults
    }
}

fn default_gains() -> PidGains {
    PidGains {
        kp: defaults::kp(),
        ki: defaults::ki(),
        kd: defaults::kd(),
    }
}

fn gap_clamp_fault(dt: Duration, max_tick_gap: Duration, profile_id: &str) -> Fault {
    Fault {
        source: "<tick-gap>".into(),
        kind: FaultKind::ClampedTickGap,
        severity: Severity::Warning,
        message: format!(
            "tick gap {:.1}s exceeded max_tick_gap_s {:.1}s; EMA/PID integration clamped to the bound (profile {profile_id})",
            dt.as_secs_f64(),
            max_tick_gap.as_secs_f64(),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{
        ComfortSourceConfig, ComfortSourceRef, FanControlConfig, FanProfileConfig, FoldRule,
        Mode, SensorSourceConfig, SlewRates, TripwireConfig, TripwireRef,
    };

    fn dur(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// Minimal control config: comfort source `c0` (τ 1.5 s, required),
    /// tripwire source `t0` (threshold 80, protection target 90), default
    /// bounds (staleness 10 s, gap 5 s).
    fn control() -> FanControlConfig {
        let mut control = FanControlConfig::default();
        control.sensor_sources.insert(
            "c0".into(),
            SensorSourceConfig {
                display_only: false,
                comfort: Some(ComfortSourceConfig {
                    ema_tau_s: 1.5,
                    required: true,
                }),
                tripwire: None,
            },
        );
        control.sensor_sources.insert(
            "t0".into(),
            SensorSourceConfig {
                display_only: false,
                comfort: None,
                tripwire: Some(TripwireConfig {
                    threshold_c: 80.0,
                    spike_rate_c_per_s: 20.0,
                    ema_tau_s: 0.3,
                    hysteresis_c: 5.0,
                    protection_target: 90.0,
                }),
            },
        );
        control
    }

    /// Minimal `FanProfileConfig` with an `id` and static-50 defaults.
    fn default_stub(id: &str) -> FanProfileConfig {
        FanProfileConfig {
            id: id.into(),
            mode: Mode::StaticPercent,
            static_percent: Some(50.0),
            curve_points: None,
            setpoint_c: None,
            gains: None,
            min_duty: 0.0,
            fold: FoldRule::Max,
            slew: SlewRates::default(),
            comfort: Vec::new(),
            tripwires: Vec::new(),
            display_only: Vec::new(),
        }
    }

    /// A `curve` profile on `c0` (τ 1.5) with curve (20,10)–(70,90) and
    /// fast slew (100/100) so the limiter never interferes.
    fn curve_profile() -> (FanControlConfig, FanProfileConfig) {
        let control = control();
        let profile = FanProfileConfig {
            id: "p".into(),
            mode: Mode::Curve,
            static_percent: None,
            curve_points: Some(vec![
                CurvePoint {
                    temp_c: 20.0,
                    pwm: 10.0,
                },
                CurvePoint {
                    temp_c: 70.0,
                    pwm: 90.0,
                },
            ]),
            setpoint_c: None,
            gains: None,
            min_duty: 0.0,
            fold: FoldRule::Max,
            slew: SlewRates {
                up: 100.0,
                down: 100.0,
            },
            comfort: vec![ComfortSourceRef {
                key: "c0".into(),
                ema_tau_s: None,
                required: None,
            }],
            tripwires: Vec::new(),
            display_only: Vec::new(),
        };
        (control, profile)
    }

    fn mk_readings(pairs: &[(&str, Option<f64>)]) -> Readings {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect()
    }

    fn tick_now(
        handle: &mut ProfileHandle,
        at: f64,
        last_cmd: f64,
        readings: &[(&str, Option<f64>)],
        tripwires: &[TripwireEval],
    ) -> TickResult {
        let r = mk_readings(readings);
        handle.tick(&TickInput {
            now: dur(at),
            readings: &r,
            last_commanded_pwm: last_cmd,
            tripwires,
        })
    }

    fn tick(handle: &mut ProfileHandle, at: f64, last_cmd: f64) -> TickResult {
        tick_now(handle, at, last_cmd, &[], &[])
    }

    /// Drive one cold-start (seed) tick.
    fn cold_start(
        handle: &mut ProfileHandle,
        at: f64,
        readings: &[(&str, Option<f64>)],
    ) -> TickResult {
        tick_now(handle, at, 0.0, readings, &[])
    }

    // -- 9.1 / 9.2 mode gate --------------------------------------------------

    #[test]
    fn device_memory_never_commands() {
        // spec *device_memory never commands*
        let control = control();
        let profile = default_stub("dm");
        let profile = FanProfileConfig {
            id: "dm".into(),
            mode: Mode::DeviceMemory,
            ..profile
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let r1 = tick(&mut handle, 1.0, 42.0);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        assert_eq!(r1.pwm(), None);
        assert!(!r1.is_failsafe());
        // ... under any sensor condition (healthy, missing, stale).
        let r2 = tick_now(&mut handle, 12.0, 42.0, &[("c0", Some(200.0))], &[]);
        assert_eq!(r2.outcome, Outcome::NoCommand);
        let r3 = tick(&mut handle, 40.0, 42.0);
        assert_eq!(r3.outcome, Outcome::NoCommand);
    }

    #[test]
    fn device_memory_faults_are_report_only() {
        // spec *device_memory never commands*: faults ride along labeled as
        // affecting a device-managed port.
        let control = control();
        let profile = FanProfileConfig {
            tripwires: vec![TripwireRef { key: "t0".into() }],
            ..default_stub("dm2")
        };
        let profile = FanProfileConfig {
            id: "dm2".into(),
            mode: Mode::DeviceMemory,
            ..profile
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let evs = [TripwireEval {
            key: "t0".into(),
            latched: true,
            online: false,
        }];
        let r = tick_now(&mut handle, 1.0, 0.0, &[], &evs);
        assert_eq!(r.outcome, Outcome::NoCommand);
        assert_eq!(r.faults.len(), 1);
        assert_eq!(r.faults[0].kind, FaultKind::TripwireSourceLost);
        assert!(
            r.faults[0]
                .message
                .ends_with("(device-managed port)"),
            "device_memory faults must be labeled report-only: {}",
            r.faults[0].message
        );
    }

    #[test]
    fn static_percent_is_constant_and_faultless() {
        // spec *static_percent ignores sensors* + *Fan Profile Modes*: with
        // no declared tripwires the duty is constant, fault-free, and there
        // is no fail-safe path at all.
        let control = control();
        let profile = default_stub("st");
        let mut handle = ProfileHandle::new(&profile, &control);
        let r1 = tick(
            &mut handle,
            1.0,
            50.0,
        );
        assert_eq!(r1.pwm(), Some(50.0));
        assert!(!r1.is_failsafe());
        assert!(r1.is_clean());
        // Read errors, huge gaps: the mode owns no sensor input, so none of
        // it can fault or fail.
        let r2 = tick_now(&mut handle, 55.0, 50.0, &[("c0", None), ("c1", Some(200.0))], &[]);
        assert_eq!(r2.pwm(), Some(50.0));
        assert!(!r2.is_failsafe());
        assert!(r2.is_clean());
    }

    #[test]
    fn static_percent_not_floored_by_min_duty() {
        // spec *Static percent not floored*
        let control = control();
        let profile = FanProfileConfig {
            static_percent: Some(0.0),
            min_duty: 10.0,
            ..default_stub("st0")
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let r = tick(&mut handle, 1.0, 0.0);
        assert_eq!(r.pwm(), Some(0.0));
        assert!(r.is_clean());
    }

    #[test]
    fn static_percent_raised_by_latched_tripwire_floor() {
        // spec *Protection Floor* (D8 resolution): a static profile's
        // declared tripwire is the only protection it has. Default slew is
        // 20 %/s up, so the floor is still rate-limited, not instant.
        let control = control();
        let profile = FanProfileConfig {
            static_percent: Some(30.0),
            tripwires: vec![TripwireRef { key: "t0".into() }],
            ..default_stub("st30")
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let evs = [TripwireEval {
            key: "t0".into(),
            latched: true,
            online: true,
        }];
        let r1 = tick_now(&mut handle, 1.0, 30.0, &[], &evs);
        assert_eq!(r1.pwm(), Some(30.0), "first tick: dt=0, slew holds");
        let r2 = tick_now(&mut handle, 2.0, 30.0, &[], &evs);
        assert_eq!(r2.pwm(), Some(50.0), "floor 90 approached at 20 %/s up");
        assert!(!r2.is_failsafe());
    }

    // -- 9.3 cold start / required-vs-optional --------------------------------

    #[test]
    fn cold_start_withholds_one_tick() {
        // spec *Cold-Start Withholds Command* / *Second tick emits a command*
        let (control, profile) = curve_profile();
        let mut handle = ProfileHandle::new(&profile, &control);

        let r1 = cold_start(&mut handle, 1.0, &[("c0", Some(50.0))]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        assert!(r1.is_clean());

        let r2 = tick_now(&mut handle, 2.0, 10.0, &[("c0", Some(50.0))], &[]);
        assert!(matches!(
            r2.outcome,
            Outcome::Command {
                failsafe: false,
                ..
            }
        ));
        // EMA: 50 + (1-e^(-1/1.5))·(50−50) = 50 → curve(50) = 10 +
        // (50−20)/50·80 = 58.
        assert_eq!(r2.pwm(), Some(58.0));
        assert!(r2.is_clean());
    }

    #[test]
    fn cold_start_is_not_immediately_failsafe() {
        // spec *Cold start is not immediately fail-safe*: one healthy, one
        // required never-arrived → NoCommand (with fault), not 100 %.
        let (mut control, mut profile) = curve_profile();
        profile.comfort.push(ComfortSourceRef {
            key: "c1".into(),
            ema_tau_s: None,
            required: None,
        });
        control.sensor_sources.insert(
            "c1".into(),
            SensorSourceConfig {
                display_only: false,
                comfort: Some(ComfortSourceConfig {
                    ema_tau_s: 1.5,
                    required: true,
                }),
                tripwire: None,
            },
        );
        let mut handle = ProfileHandle::new(&profile, &control);

        let r1 = cold_start(&mut handle, 1.0, &[("c0", Some(50.0))]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        assert_eq!(r1.faults.len(), 1);
        assert_eq!(r1.faults[0].source, "c1");
        assert_eq!(r1.faults[0].kind, FaultKind::ComfortSourceMissing);

        // From the first decision tick onward: required missing → 100 %.
        let r2 = tick_now(&mut handle, 2.0, 10.0, &[("c0", Some(50.0))], &[]);
        assert!(r2.is_failsafe());
        assert_eq!(r2.pwm(), Some(100.0));
        assert!(r2.faults.iter().any(|f| f.source == "c1"));
    }

    #[test]
    fn never_arrived_sole_source_failsafes_after_withhold() {
        // spec *Never-arrived sole source*
        let (control, profile) = curve_profile();
        let mut handle = ProfileHandle::new(&profile, &control);

        let r1 = cold_start(&mut handle, 1.0, &[]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        assert_eq!(r1.faults.len(), 1);

        let r2 = tick(&mut handle, 2.0, 10.0);
        assert!(r2.is_failsafe());
        assert_eq!(r2.pwm(), Some(100.0));
        assert!(r2.faults.iter().any(|f| {
            f.source == "c0" && f.kind == FaultKind::ComfortSourceMissing
        }));
    }

    #[test]
    fn required_source_lost_after_health_triggers_failsafe_at_bound() {
        // spec *Required source lost after health* / *Stale sample triggers
        // fail-safe*: "at least the staleness bound old" → 100 %.
        let (control, profile) = curve_profile();
        let mut handle = ProfileHandle::new(&profile, &control);
        let r1 = cold_start(&mut handle, 0.0, &[("c0", Some(50.0))]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        let r2 = tick_now(&mut handle, 1.0, 50.0, &[("c0", Some(50.0))], &[]);
        assert!(!r2.is_failsafe());

        // Silent: age 9 s < 10 s bound → still fresh; the last smoothed
        // value carries the fold, no fail-safe, no fault.
        let r3 = tick(&mut handle, 10.0, 58.0);
        assert!(!r3.is_failsafe());
        assert!(r3.is_clean());
        // EMA still 50 → curve(50) = 58.
        assert_eq!(r3.pwm(), Some(58.0));

        // Age 10 s at the bound → fail-safe 100 %.
        let r4 = tick(&mut handle, 11.0, 100.0);
        assert!(r4.is_failsafe());
        assert_eq!(r4.pwm(), Some(100.0));
        assert!(r4.faults.iter().any(|f| {
            f.source == "c0"
                && f.kind == FaultKind::ComfortSourceStale
                && f.severity == Severity::Error
        }));
    }

    #[test]
    fn optional_source_missing_is_soft_not_failsafe() {
        // design D7: an optional source down soft-degrades the fold — no
        // fail-safe, the rest of the drivers carry the output.
        let (mut control, mut profile) = curve_profile();
        profile.comfort.push(ComfortSourceRef {
            key: "c1".into(),
            ema_tau_s: None,
            required: Some(false),
        });
        control.sensor_sources.insert(
            "c1".into(),
            SensorSourceConfig {
                display_only: false,
                comfort: Some(ComfortSourceConfig {
                    ema_tau_s: 1.5,
                    required: false,
                }),
                tripwire: None,
            },
        );
        let mut handle = ProfileHandle::new(&profile, &control);
        let r1 = cold_start(
            &mut handle,
            0.0,
            &[("c0", Some(50.0)), ("c1", Some(50.0))],
        );
        assert_eq!(r1.outcome, Outcome::NoCommand);
        assert!(r1.is_clean());

        let r2 = tick_now(&mut handle, 1.0, 58.0, &[("c0", Some(50.0))], &[]);
        assert!(!r2.is_failsafe());
        assert!(r2.faults.iter().any(|f| {
            f.source == "c1"
                && f.kind == FaultKind::ComfortSourceMissing
                && f.severity == Severity::Warning
        }));
        // c0 folds to 50 and c1 contributes its fresh last value of 50 →
        // curve(50) = 58.
        assert_eq!(r2.pwm(), Some(58.0));
    }

    // -- 9.4 protection floor --------------------------------------------------

    fn curve_handle_with_tripwire(latched: bool, online: bool) -> (ProfileHandle, Vec<TripwireEval>) {
        let (control, mut profile) = curve_profile();
        profile.tripwires.push(TripwireRef { key: "t0".into() });
        let handle = ProfileHandle::new(&profile, &control);
        let evs = vec![TripwireEval {
            key: "t0".into(),
            latched,
            online,
        }];
        (handle, evs)
    }

    #[test]
    fn tripwire_raises_calm_curve() {
        // spec *Tripwire raises a calm curve*: mode target 26 + floor 90 →
        // 90, with a soft online eval (no tripwire fault).
        let (mut handle, evs) = curve_handle_with_tripwire(true, true);
        let r1 = cold_start(&mut handle, 1.0, &[("c0", Some(30.0))]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        let r2 = tick_now(&mut handle, 2.0, 88.0, &[("c0", Some(30.0))], &evs);
        // curve(30) = 10 + (30−20)/50·80 = 26 → floor lifts to 90.
        assert_eq!(r2.pwm(), Some(90.0));
        assert!(!r2.is_failsafe());
        assert!(r2.is_clean());
    }

    #[test]
    fn protection_does_not_suppress_failsafe() {
        // spec *Protection does not suppress failsafe*: latched floor 90,
        // comfort stale on that tick → 100 %, not 90 %.
        let (mut handle, evs) = curve_handle_with_tripwire(true, true);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(50.0))]);
        let _ = tick_now(&mut handle, 1.0, 50.0, &[("c0", Some(50.0))], &evs);
        let r = tick_now(&mut handle, 11.0, 90.0, &[], &evs);
        assert!(r.is_failsafe());
        assert_eq!(r.pwm(), Some(100.0));
    }

    // -- 9.5 soft tripwire loss -------------------------------------------------

    #[test]
    fn lost_latched_tripwire_holds_floor_with_soft_fault() {
        // spec *Missing tripwire stays latched*
        let (mut handle, evs) = curve_handle_with_tripwire(true, false);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(30.0))]);
        let r = tick_now(&mut handle, 1.0, 90.0, &[("c0", Some(30.0))], &evs);
        assert!(!r.is_failsafe());
        assert_eq!(r.pwm(), Some(90.0), "last-known latched floor holds");
        assert!(r.faults.iter().any(|f| {
            f.source == "t0"
                && f.kind == FaultKind::TripwireSourceLost
                && f.severity == Severity::Warning
        }));
    }

    #[test]
    fn lost_clear_tripwire_applies_no_floor() {
        // spec *Missing tripwire that was clear*: no false floor from an
        // absence, soft fault rides along.
        let (mut handle, evs) = curve_handle_with_tripwire(false, false);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(30.0))]);
        let r = tick_now(&mut handle, 1.0, 26.0, &[("c0", Some(30.0))], &evs);
        assert!(!r.is_failsafe());
        // curve(30) = 26, unlatched → no floor.
        assert_eq!(r.pwm(), Some(26.0));
        assert!(r.faults.iter().any(|f| f.kind == FaultKind::TripwireSourceLost));
    }

    // -- 9.6 faults never silent -------------------------------------------------

    #[test]
    fn read_error_none_is_not_usable() {
        // A `None` read error does not refresh staleness; held until the
        // bound, a required source fails over as stale.
        let (control, profile) = curve_profile();
        let mut handle = ProfileHandle::new(&profile, &control);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(50.0))]);
        let _ = tick_now(&mut handle, 1.0, 50.0, &[("c0", Some(50.0))], &[]);
        let r = tick_now(&mut handle, 11.0, 50.0, &[("c0", None)], &[]);
        assert!(r.is_failsafe());
        assert!(r.faults.iter().any(|f| f.kind == FaultKind::ComfortSourceStale));
    }

    #[test]
    fn tick_gap_clamp_is_reported() {
        // design D3 / 9.6: gap > max_tick_gap → ClampedTickGap fault; the
        // EMA integrates the clamped gap (no 20 s jump).
        let (control, profile) = curve_profile();
        let mut handle = ProfileHandle::new(&profile, &control);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(50.0))]);
        let _ = tick_now(&mut handle, 1.0, 50.0, &[("c0", Some(50.0))], &[]);
        let r = tick_now(&mut handle, 21.0, 50.0, &[("c0", Some(100.0))], &[]);
        assert!(!r.is_failsafe(), "comfort is fresh on this tick");
        assert!(r.faults.iter().any(|f| f.kind == FaultKind::ClampedTickGap));
        // EMA moved only one clamped 5 s step: 50 + (1−e^(−5/1.5))·50 ≈
        // 98.2 → curve clamps at 90 → pre-slew 90; slew (100 %/s × 20 s)
        // reaches it on one tick.
        assert_eq!(r.pwm(), Some(90.0));
    }

    // -- spec cross-checks ---------------------------------------------------------

    #[test]
    fn display_only_is_inert() {
        // spec *display_only is inert*: the 200 °C display reading changes
        // nothing about the outcome or the faults.
        let (control, mut profile) = curve_profile();
        profile.display_only.push("d0".into());
        let mut handle = ProfileHandle::new(&profile, &control);
        let _ = cold_start(
            &mut handle,
            0.0,
            &[("c0", Some(50.0)), ("d0", Some(200.0))],
        );
        let r1 = tick_now(
            &mut handle,
            1.0,
            50.0,
            &[("c0", Some(50.0)), ("d0", Some(200.0))],
            &[],
        );

        let (control2, profile2) = curve_profile();
        let mut handle2 = ProfileHandle::new(&profile2, &control2);
        let _ = cold_start(&mut handle2, 0.0, &[("c0", Some(50.0))]);
        let r2 = tick_now(&mut handle2, 1.0, 50.0, &[("c0", Some(50.0))], &[]);

        assert_eq!(r1.outcome, r2.outcome);
        assert_eq!(r1.faults, r2.faults);
    }

    #[test]
    fn curve_mode_min_duty_floor() {
        // spec *Curve output floored*: lookup 10 % under a 50 % min-duty →
        // 50 % (pre-slew, already at the seeded position).
        let (control, mut profile) = curve_profile();
        profile.min_duty = 50.0;
        let mut handle = ProfileHandle::new(&profile, &control);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(20.0))]);
        let r = tick_now(&mut handle, 1.0, 50.0, &[("c0", Some(20.0))], &[]);
        assert_eq!(r.pwm(), Some(50.0), "min_duty 50 lifts curve(20)=10");
    }

    #[test]
    fn target_temp_ramps_up_proportional() {
        // spec *Ramps up when hot* (through the pipeline): error 20 °C,
        // Kp=2, Ki=0 → 40 %.
        let (control, base) = curve_profile();
        let profile = FanProfileConfig {
            id: "tt".into(),
            mode: Mode::TargetTemp,
            curve_points: None,
            setpoint_c: Some(55.0),
            gains: Some(PidGains {
                kp: 2.0,
                ki: 0.0,
                kd: 0.0,
            }),
            ..base
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let r1 = cold_start(&mut handle, 0.0, &[("c0", Some(75.0))]);
        assert_eq!(r1.outcome, Outcome::NoCommand);
        let r2 = tick_now(&mut handle, 1.0, 40.0, &[("c0", Some(75.0))], &[]);
        assert_eq!(r2.pwm(), Some(40.0), "Kp·e = 2 × 20 = 40");
        assert!(r2.is_clean());
    }

    #[test]
    fn target_temp_holds_setpoint() {
        // spec *Reaches and holds setpoint*: folded == setpoint → error 0 →
        // steady zero output (no self-induced oscillation).
        let (control, base) = curve_profile();
        let profile = FanProfileConfig {
            id: "tt2".into(),
            mode: Mode::TargetTemp,
            curve_points: None,
            setpoint_c: Some(55.0),
            ..base
        };
        let mut handle = ProfileHandle::new(&profile, &control);
        let _ = cold_start(&mut handle, 0.0, &[("c0", Some(55.0))]);
        let r1 = tick_now(&mut handle, 1.0, 0.0, &[("c0", Some(55.0))], &[]);
        assert_eq!(r1.pwm(), Some(0.0));
        let r2 = tick_now(&mut handle, 2.0, 0.0, &[("c0", Some(55.0))], &[]);
        assert_eq!(r2.pwm(), Some(0.0), "steady at setpoint");
        assert!(r2.is_clean());
    }
}
