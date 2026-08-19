//! Per-source tripwire state (design D2).
//!
//! A latched tripwire is *a fact about the system, not per-profile state*:
//! the daemon (or tests) evaluates every tripwire source once per tick via
//! [`TripwireEvaluator`] and hands the resulting [`TripwireEval`] map to
//! every profile referencing that source — one evaluation, many consumers.
//!
//! Per source:
//! - own light EMA (τ shorter than the comfort drivers' — hotspots move fast);
//! - absolute condition: smoothed value ≥ threshold → latch;
//! - rate condition: d(smoothed)/dt ≥ spike rate → latch (design: computed
//!   from consecutive EMA samples);
//! - dT/dt is **disabled until two samples exist** (spec first-tick rule);
//! - latch holds until the smoothed value falls to ≤ threshold − hysteresis
//!   (spec *Tripwire Latching*);
//! - **absence never clears a latch**: an offline source keeps its last
//!   known state, and the evaluator flags it `online: false` so profiles
//!   emit the soft fault (spec *Soft Handling of a Lost Tripwire Source*).

use std::collections::BTreeMap;
use std::time::Duration;

use crate::config::TripwireConfig;
use crate::ema::Ema;

/// The core's sensor-source key (opaque, daemon-chosen).
pub type SourceKey = crate::config::SourceKey;

/// One per-source evaluation for a tick — the read-only input profiles
/// consume (design D1/D2).
#[derive(Debug, Clone, PartialEq)]
pub struct TripwireEval {
    pub key: SourceKey,
    /// Latched (protection floor active). When `online` is false this is the
    /// *last known* state — absence never clears a latch.
    pub latched: bool,
    /// Did a reading arrive this tick? `false` → soft fault in the profiles
    /// that reference the source; latch state is carried forward.
    pub online: bool,
}

/// Per-source internal state: EMA, latch, and the trip-condition that
/// latched it (for the diagnostic message). `cfg` is `None` for sources that
/// have been touched but never declared as tripwires — they can never latch.
#[derive(Debug, Clone)]
struct SourceState {
    cfg: Option<TripwireConfig>,
    ema: Ema,
    latched: bool,
    latched_by: Option<TripKind>,
}

/// Which condition latched the tripwire (diagnostic for fault messages).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripKind {
    /// Smoothed value crossed the absolute threshold.
    Threshold,
    /// Smoothed rate of change crossed the spike rate.
    Spike,
}

impl TripKind {
    fn as_str(&self) -> &'static str {
        match self {
            TripKind::Threshold => "threshold",
            TripKind::Spike => "dT/dt spike",
        }
    }
}

/// Consume one sample for a configured tripwire source; returns the new
/// latch state. Free function (not a method) so `evaluate` can hold both
/// `&mut self.sources` (via `entry`) and the per-step computation.
fn step_source(st: &mut SourceState, now: Duration, x: f64, max_gap: Duration) -> bool {
    let cfg = st
        .cfg
        .as_ref()
        .expect("step_source called on a source without tripwire config");
    let update = st.ema.update(now, cfg.ema_tau_s, x, max_gap);

    // Remember the pre-existing latch so the clear check cannot undo a trip
    // that latched *on this tick* (spec *Tripwire Latching*: a spike latches
    // the source for that tick even though the smoothed value is still far
    // below threshold − hysteresis).
    let was_latched = st.latched;

    // 1. Absolute trip: smoothed value ≥ threshold (spec *Absolute threshold
    //    trips*).
    let threshold_hit = update.value >= cfg.threshold_c;

    // 2. dT/dt spike, from consecutive smoothed samples. Disabled until two
    //    samples exist (spec *dT/dt disabled before two samples*).
    let spike_hit = !threshold_hit
        && st.ema.samples() >= 2
        && update.prev.is_some()
        && update.dt > Duration::ZERO
        && (update.value - update.prev.unwrap()) / update.dt.as_secs_f64() >= cfg.spike_rate_c_per_s;

    if (threshold_hit || spike_hit) && !st.latched {
        st.latched = true;
        st.latched_by = Some(if threshold_hit { TripKind::Threshold } else { TripKind::Spike });
    }

    // Clear at ≤ threshold − hysteresis, but only a latch that existed *before*
    // this tick (spec *Clear below hysteresis*: "falls to exactly threshold −
    // hysteresis or below → clears on that tick").
    if was_latched && update.value <= cfg.threshold_c - cfg.hysteresis_c {
        st.latched = false;
        st.latched_by = None;
    }

    st.latched
}

/// Evaluates tripwire sources once per tick; shared read-only by profiles.
///
/// Construct with the per-source tripwire tuning (from the `sensor_sources`
/// table). The daemon evaluates this before invoking any profile tick.
#[derive(Debug, Clone, Default)]
pub struct TripwireEvaluator {
    sources: BTreeMap<SourceKey, SourceState>,
}

impl TripwireEvaluator {
    /// Empty evaluator (no tripwires yet).
    pub fn new() -> Self {
        Self {
            sources: BTreeMap::new(),
        }
    }

    /// Evaluate all declared tripwire sources for this tick.
    ///
    /// - `now`: injected timestamp.
    /// - `max_gap`: the gap clamp applied to EMA/pid integration (design D3).
    /// - `readings`: `key → raw °C` for the sources present this tick;
    ///   absent keys keep their last known latch state with `online: false`.
    ///
    /// Returns one [`TripwireEval`] per *declared* source, in deterministic
    /// (key-sorted) order.
    pub fn evaluate(
        &mut self,
        now: Duration,
        max_gap: Duration,
        readings: &BTreeMap<SourceKey, f64>,
    ) -> Vec<TripwireEval> {
        // Evaluate every source that has been touched (configured or present).
        let mut keys: Vec<SourceKey> = readings.keys().cloned().collect();
        keys.extend(self.sources.keys().cloned());
        keys.sort();
        keys.dedup();

        keys.into_iter()
            .map(|key| {
                let eval = match readings.get(&key) {
                    Some(x) => {
                        let st = self.sources.entry(key.clone()).or_insert_with(|| SourceState {
                            cfg: None,
                            ema: Ema::new(),
                            latched: false,
                            latched_by: None,
                        });
                        let latched = if st.cfg.is_some() {
                            step_source(st, now, *x, max_gap)
                        } else {
                            st.latched
                        };
                        TripwireEval {
                            key: key.clone(),
                            latched,
                            online: true,
                        }
                    }
                    None => {
                        // Offline: carry the last known state forward
                        // (absence never clears a latch).
                        let latched = self.sources.get(&key).map(|st| st.latched).unwrap_or(false);
                        TripwireEval {
                            key: key.clone(),
                            latched,
                            online: false,
                        }
                    }
                };
                eval
            })
            .collect()
    }

    /// Install/replace the tripwire tuning for one source (from the
    /// `sensor_sources` table at config load). Preserves any existing EMA and
    /// latch state (config reloads must not clear latches).
    pub fn set_config(&mut self, key: SourceKey, cfg: TripwireConfig) {
        match self.sources.get_mut(&key) {
            Some(st) => st.cfg = Some(cfg),
            None => {
                self.sources.insert(
                    key,
                    SourceState {
                        cfg: Some(cfg),
                        ema: Ema::new(),
                        latched: false,
                        latched_by: None,
                    },
                );
            }
        }
    }

    /// Remove a source (e.g. removed from config on reload).
    pub fn remove(&mut self, key: &SourceKey) {
        self.sources.remove(key);
    }

    /// Whether a source has delivered at least one reading.
    pub fn has_seen(&self, key: &SourceKey) -> bool {
        self.sources.get(key).map(|s| s.ema.samples() > 0).unwrap_or(false)
    }

    /// The last latched-by diagnostic for a source (for fault messages).
    pub fn last_trip(&self, key: &SourceKey) -> Option<TripKind> {
        self.sources.get(key).and_then(|s| s.latched_by)
    }

    /// Number of sources that have been registered/touched.
    pub fn len(&self) -> usize {
        self.sources.len()
    }

    /// True when no sources have been registered.
    pub fn is_empty(&self) -> bool {
        self.sources.is_empty()
    }
}

impl std::fmt::Display for TripKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(threshold: f64, spike: f64, tau: f64, hyst: f64) -> TripwireConfig {
        TripwireConfig {
            threshold_c: threshold,
            spike_rate_c_per_s: spike,
            ema_tau_s: tau,
            hysteresis_c: hyst,
            protection_target: 90.0,
        }
    }

    fn readings(pairs: &[(&str, f64)]) -> BTreeMap<SourceKey, f64> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect()
    }

    fn gap() -> Duration {
        Duration::from_secs(5)
    }

    #[test]
    fn absolute_threshold_trips() {
        // spec *Absolute threshold trips*
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 1.0, 5.0));
        let r = ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 85.0)]));
        assert_eq!(r.len(), 1);
        assert!(r[0].latched, "85 ≥ 80 → latched");
        assert!(r[0].online);
    }

    #[test]
    fn below_threshold_does_not_trip() {
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 1.0, 5.0));
        let r = ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 70.0)]));
        assert!(!r[0].latched);
    }

    #[test]
    fn dtdt_spike_trips() {
        // spec *dT/dt spike trips*: smoothed 40 → 80 between consecutive
        // ticks, spike rate 20 °C/s. With τ=1s and Δt=1s the smoothed value
        // rises 40 → 40 + 0.632·40 ≈ 65.3 °C — well above a 20 °C/s rate.
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(200.0, 20.0, 1.0, 5.0)); // high threshold: only a spike can latch
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 40.0)]));
        let r = ev.evaluate(Duration::from_secs(1), gap(), &readings(&[("hot", 80.0)]));
        assert!(r[0].latched, "40→80 in 1 s with spike 20 °C/s must latch");
    }

    #[test]
    fn dtdt_disabled_before_two_samples() {
        // spec *dT/dt disabled before two samples*: one sample of 80 °C with
        // threshold 90 → no false trip (no previous sample to rate from).
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(90.0, 5.0, 0.5, 5.0));
        let r = ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 80.0)]));
        assert!(!r[0].latched, "first sample cannot spike-trip (no previous sample)");
    }

    #[test]
    fn latch_survives_dip_within_hysteresis() {
        // spec *Latch survives a dip within hysteresis*: latched at 85
        // (threshold 80, hysteresis 5); dip to 82 → still latched (82 > 75).
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 0.2, 5.0));
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 85.0)]));
        let r = ev.evaluate(Duration::from_secs_f64(0.5), gap(), &readings(&[("hot", 82.0)]));
        assert!(r[0].latched, "82 > threshold−hysteresis (75) → still latched");
    }

    #[test]
    fn clears_at_or_below_threshold_minus_hysteresis() {
        // spec *Clear below hysteresis*: the latch clears on the tick the
        // smoothed value reaches threshold − hysteresis (75 °C here).
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 0.05, 5.0)); // fast τ tracks the reading
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 85.0)]));
        let mut cleared = false;
        let mut t = 0.0;
        for _ in 0..200 {
            t += 0.05;
            let r = ev.evaluate(Duration::from_secs_f64(t), gap(), &readings(&[("hot", 70.0)]));
            if !r[0].latched {
                cleared = true;
                break;
            }
        }
        assert!(cleared, "must clear once the smoothed value reaches ≤ 75 (threshold−hysteresis)");
    }

    #[test]
    fn drop_above_clear_level_does_not_clear() {
        // Latched; drops to ~80 (above 75) → still latched.
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 0.02, 5.0)); // fast τ
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 90.0)]));
        let r = ev.evaluate(Duration::from_secs_f64(0.1), gap(), &readings(&[("hot", 81.0)]));
        assert!(r[0].latched, "81 > 75 → still latched");
    }

    #[test]
    fn absent_source_keeps_last_known_state() {
        // spec *Soft Handling of a Lost Tripwire Source*: while missing the
        // latch holds (latched stays latched) and online=false.
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 0.5, 5.0));
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 85.0)]));
        // Source stops arriving.
        let r = ev.evaluate(Duration::from_secs(1), gap(), &BTreeMap::new());
        let eval = r.iter().find(|e| e.key == "hot").unwrap();
        assert!(eval.latched, "latched tripwire stays latched while its source is absent");
        assert!(!eval.online, "absence must flag online=false");
    }

    #[test]
    fn absent_unlatched_source_stays_unlatched() {
        // spec *Missing tripwire that was clear*: an unlatched source that
        // stops reading is still counted clear (no false protection floor).
        let mut ev = TripwireEvaluator::new();
        ev.set_config("cool".into(), cfg(80.0, 20.0, 0.5, 5.0));
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("cool", 30.0)]));
        let r2 = ev.evaluate(Duration::from_secs(1), gap(), &BTreeMap::new());
        let eval2 = r2.iter().find(|e| e.key == "cool").unwrap();
        assert!(!eval2.latched, "unlatched tripwire stays unlatched while absent");
        assert!(!eval2.online);
    }

    #[test]
    fn one_evaluation_many_consumers_deterministic() {
        // One eval per source per tick; deterministic across identical inputs.
        let mut a = TripwireEvaluator::new();
        let mut b = TripwireEvaluator::new();
        for x in [&mut a, &mut b] {
            x.set_config("s1".into(), cfg(80.0, 20.0, 1.0, 5.0));
            x.set_config("s2".into(), cfg(90.0, 10.0, 1.0, 3.0));
        }
        let ra = a.evaluate(Duration::ZERO, gap(), &readings(&[("s1", 85.0), ("s2", 50.0)]));
        let rb = b.evaluate(Duration::ZERO, gap(), &readings(&[("s1", 85.0), ("s2", 50.0)]));
        assert_eq!(ra, rb, "identical inputs → identical evaluations");
        assert!(ra[0].latched && !ra[1].latched);
    }

    #[test]
    fn source_without_tripwire_config_cannot_trip() {
        let mut ev = TripwireEvaluator::new();
        // A present source that was never configured as a tripwire.
        let r = ev.evaluate(Duration::ZERO, gap(), &readings(&[("x", 999.0)]));
        assert!(!r[0].latched);
        assert!(r[0].online);
    }

    #[test]
    fn set_config_preserves_existing_state() {
        // A config reload must not clear a live latch (spec: one evaluation,
        // shared state).
        let mut ev = TripwireEvaluator::new();
        ev.set_config("hot".into(), cfg(80.0, 20.0, 1.0, 5.0));
        ev.evaluate(Duration::ZERO, gap(), &readings(&[("hot", 90.0)]));
        // Reload the same (or new) config.
        ev.set_config("hot".into(), cfg(80.0, 20.0, 1.0, 6.0));
        assert!(
            ev.last_trip(&"hot".to_string()) == Some(TripKind::Threshold),
            "latch survives set_config"
        );
    }
}
