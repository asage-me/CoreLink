//! Integration tests for the full per-tick loop (tasks 10.1–10.5):
//! [`TripwireEvaluator`] + [`ProfileHandle`] driven by scripted injected
//! timestamps and readings — no hardware, no I/O, fully deterministic.

use std::collections::BTreeMap;
use std::time::Duration;

use corelink_core::config::{
    ComfortSourceConfig, ComfortSourceRef, CurvePoint, FanControlConfig, FanProfileConfig,
    FoldRule, Mode, SensorSourceConfig, SlewRates, TripwireConfig, TripwireRef,
};
use corelink_core::profile::{FaultKind, Outcome, ProfileHandle, Readings, TickResult};
use corelink_core::tripwire::TripwireEvaluator;

fn f64_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// One comfort driver source in the sensor table.
fn comfort_src(tau: f64, required: bool) -> SensorSourceConfig {
    SensorSourceConfig {
        display_only: false,
        comfort: Some(ComfortSourceConfig {
            ema_tau_s: tau,
            required,
        }),
        tripwire: None,
    }
}

/// One tripwire source in the sensor table (threshold / hysteresis / target).
fn trip_src(threshold: f64, hysteresis: f64, target: f64) -> SensorSourceConfig {
    SensorSourceConfig {
        display_only: false,
        comfort: None,
        tripwire: Some(TripwireConfig {
            threshold_c: threshold,
            spike_rate_c_per_s: 20.0,
            ema_tau_s: 0.3,
            hysteresis_c: hysteresis,
            protection_target: target,
        }),
    }
}

/// A `curve` profile with the (20,10)–(70,90) curve, fold max, optional
/// extra tripwire refs.
fn curve_profile(
    id: &str,
    comfort: Vec<ComfortSourceRef>,
    tripwires: Vec<TripwireRef>,
    slew: SlewRates,
) -> FanProfileConfig {
    FanProfileConfig {
        id: id.into(),
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
        slew,
        comfort,
        tripwires,
        display_only: Vec::new(),
    }
}

/// A scripted simulation of the daemon loop: one shared
/// [`TripwireEvaluator`] plus N [`ProfileHandle`]s, fed injected readings.
/// The daemon side is modeled faithfully enough for the spec: tripwires are
/// evaluated once per tick before profiles; each profile is fed back the PWM
/// it last commanded (0 on `NoCommand`).
struct Sim {
    control: FanControlConfig,
    handles: Vec<ProfileHandle>,
    tw: TripwireEvaluator,
    last_cmd: Vec<f64>,
}

impl Sim {
    fn new(control: FanControlConfig, profiles: Vec<FanProfileConfig>) -> Self {
        let mut tw = TripwireEvaluator::new();
        for (key, src) in &control.sensor_sources {
            if let Some(cfg) = &src.tripwire {
                tw.set_config(key.clone(), cfg.clone());
            }
        }
        let handles: Vec<ProfileHandle> = profiles
            .iter()
            .map(|p| ProfileHandle::new(p, &control))
            .collect();
        Sim {
            last_cmd: vec![0.0; handles.len()],
            control,
            handles,
            tw,
        }
    }

    /// Run one tick for every profile. `comfort` / `trip` hold the sources
    /// that delivered this tick.
    fn tick(
        &mut self,
        now: f64,
        comfort: &[(&str, f64)],
        trip: &[(&str, f64)],
    ) -> Vec<TickResult> {
        let comfort_map: Readings = comfort
            .iter()
            .map(|(k, v)| (k.to_string(), Some(*v)))
            .collect();
        let trip_map: BTreeMap<String, f64> =
            trip.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        let now_d = Duration::from_secs_f64(now);
        let max_gap = Duration::from_secs_f64(self.control.max_tick_gap_s);
        let evals = self.tw.evaluate(now_d, max_gap, &trip_map);
        self.handles
            .iter_mut()
            .zip(self.last_cmd.iter_mut())
            .map(|(h, last)| {
                let r = h.tick(&corelink_core::profile::TickInput {
                    now: now_d,
                    readings: &comfort_map,
                    last_commanded_pwm: *last,
                    tripwires: &evals,
                });
                if let Some(pwm) = r.pwm() {
                    *last = pwm;
                }
                r
            })
            .collect()
    }

    fn pwm(&self, i: usize) -> Option<f64> {
        // Convenience for tests: last commanded pwm.
        self.last_cmd.get(i).copied()
    }

    #[allow(dead_code)]
    fn control(&self) -> &FanControlConfig {
        &self.control
    }
}

/// Control config used by the end-to-end test: comfort `g` (τ 1.5 s,
/// required) + tripwire `h` (threshold 80, hysteresis 5, target 90),
/// default staleness 10 s and gap 5 s.
fn control_end_to_end() -> FanControlConfig {
    let mut control = FanControlConfig::default();
    control.sensor_sources.insert("g".into(), comfort_src(1.5, true));
    control.sensor_sources.insert("h".into(), trip_src(80.0, 5.0, 90.0));
    control
}

// -- 10.1 end-to-end scripted sequence ------------------------------------

#[test]
fn end_to_end_scripted_sequence() {
    // Cold start -> steady curve -> tripwire latch -> floor holds while
    // comfort drops -> tripwire clears -> tripwire source lost (soft) ->
    // comfort stale -> failsafe 100 %.
    let control = control_end_to_end();
    let profile = curve_profile(
        "p",
        vec![ComfortSourceRef {
            key: "g".into(),
            ema_tau_s: None,
            required: None,
        }],
        vec![TripwireRef { key: "h".into() }],
        SlewRates {
            up: 100.0,
            down: 100.0,
        },
    );
    let mut sim = Sim::new(control, vec![profile]);

    // t0: cold start — one NoCommand withhold tick, filters seeded.
    let r0 = sim.tick(0.0, &[("g", 50.0)], &[("h", 60.0)]);
    assert_eq!(r0[0].outcome, Outcome::NoCommand, "t0 must withhold: {:?}", r0[0]);
    assert!(r0[0].is_clean());

    // t1: steady curve operation. EMA 50 -> curve(50) = 58.
    let r1 = sim.tick(1.0, &[("g", 50.0)], &[("h", 60.0)]);
    assert!(f64_eq(r1[0].pwm().unwrap(), 58.0), "t1: {:?}", r1[0]);
    assert!(r1[0].is_clean());

    // t2: hotspot crosses threshold (smoothed ~84.1 >= 80) -> latch ->
    // protection floor 90 raises the calm 58.
    let r2 = sim.tick(2.0, &[("g", 50.0)], &[("h", 85.0)]);
    assert!(f64_eq(r2[0].pwm().unwrap(), 90.0), "t2: {:?}", r2[0]);
    assert!(!r2[0].is_failsafe());
    assert!(r2[0].is_clean());

    // t3: comfort drops to 30 (mode target ~42) but the latch holds -> the
    // floor keeps the port at 90.
    let r3 = sim.tick(3.0, &[("g", 30.0)], &[("h", 85.0)]);
    assert!(f64_eq(r3[0].pwm().unwrap(), 90.0), "t3: {:?}", r3[0]);

    // t4: hotspot cools below threshold - hysteresis (smoothed ~73.4 <=
    // 75) -> latch clears on this tick -> mode-derived output returns.
    let r4 = sim.tick(4.0, &[("g", 30.0)], &[("h", 73.0)]);
    let pwm4 = r4[0].pwm().unwrap();
    assert!(
        (34.0..35.0).contains(&pwm4),
        "t4: curve of the folded 35-something, got {pwm4}: {:?}",
        r4[0]
    );
    assert!(r4[0].is_clean(), "clear tick carries no fault: {:?}", r4[0]);

    // t5: hotspot source stops delivering; comfort is fresh -> soft fault,
    // last-known state is clear -> no floor, no failsafe.
    let r5 = sim.tick(5.0, &[("g", 35.0)], &[]);
    assert!(!r5[0].is_failsafe());
    assert!(
        r5[0]
            .faults
            .iter()
            .any(|f| f.source == "h" && f.kind == FaultKind::TripwireSourceLost),
        "t5: lost-tripwire soft fault expected: {:?}",
        r5[0]
    );
    assert!(r5[0].pwm().unwrap() < 90.0, "no latched floor may apply");

    // t6: still lost — soft fault rides along again.
    let r6 = sim.tick(6.0, &[("g", 35.0)], &[]);
    assert!(!r6[0].is_failsafe());
    assert!(r6[0]
        .faults
        .iter()
        .any(|f| f.kind == FaultKind::TripwireSourceLost));

    // t7..t14: comfort goes quiet but stays within the 10 s staleness
    // bound -> no failsafe; the only fault that may ride along is the soft
    // lost-tripwire one (h stopped delivering at t5).
    for i in 7..15 {
        let t = i as f64;
        let r = sim.tick(t, &[], &[]);
        assert!(!r[0].is_failsafe(), "t{t}: within staleness bound");
        assert!(
            r[0]
                .faults
                .iter()
                .all(|f| f.kind == FaultKind::TripwireSourceLost),
            "t{t}: only the lost-tripwire soft fault is expected: {:?}",
            r[0]
        );
    }

    // t16: comfort sample is now 10 s old — at least the bound ->
    // fail-safe 100 % with a staleness fault (the tripwire, still lost,
    // carries its last-known clear state; D6 skips filter-stage faults on a
    // fail-safe tick).
    let r16 = sim.tick(16.0, &[], &[]);
    assert!(r16[0].is_failsafe(), "t16: {:?}", r16[0]);
    assert!(f64_eq(r16[0].pwm().unwrap(), 100.0));
    assert!(
        r16[0]
            .faults
            .iter()
            .any(|f| f.source == "g" && f.kind == FaultKind::ComfortSourceStale),
        "t16 fault: {:?}",
        r16[0]
    );
}

// -- 10.2 per-port isolation ------------------------------------------------

#[test]
fn per_port_isolation_same_tick() {
    // spec *Per-port isolation*: profile A fail-safes, profile B is healthy
    // on the same tick and is unaffected.
    let mut control = FanControlConfig::default();
    control.sensor_sources.insert("a".into(), comfort_src(1.5, true));
    control.sensor_sources.insert("b".into(), comfort_src(1.5, true));
    control.sensor_sources.insert("h".into(), trip_src(80.0, 5.0, 90.0));
    // No tripwires declared here: the isolation test compares comfort
    // health only, without soft tripwire-loss faults muddying the outcome.
    let a = curve_profile(
        "a",
        vec![ComfortSourceRef {
            key: "a".into(),
            ema_tau_s: None,
            required: None,
        }],
        Vec::new(),
        SlewRates {
            up: 100.0,
            down: 100.0,
        },
    );
    let b = curve_profile(
        "b",
        vec![ComfortSourceRef {
            key: "b".into(),
            ema_tau_s: None,
            required: None,
        }],
        Vec::new(),
        SlewRates {
            up: 100.0,
            down: 100.0,
        },
    );
    let mut sim = Sim::new(control, vec![a, b]);

    // t0 cold start; t1 both healthy at 58.
    let r0 = sim.tick(0.0, &[("a", 50.0), ("b", 50.0)], &[]);
    assert_eq!(r0[0].outcome, Outcome::NoCommand);
    assert_eq!(r0[1].outcome, Outcome::NoCommand);
    let r1 = sim.tick(1.0, &[("a", 50.0), ("b", 50.0)], &[]);
    assert!(f64_eq(r1[0].pwm().unwrap(), 58.0));
    assert!(f64_eq(r1[1].pwm().unwrap(), 58.0));

    // t2..t10: source a goes quiet (within bound); b stays fresh.
    for i in 2..10 {
        let t = i as f64;
        let r = sim.tick(t, &[("b", 50.0)], &[]);
        assert!(!r[0].is_failsafe(), "t{t}");
        assert!(f64_eq(r[1].pwm().unwrap(), 58.0), "t{t}: B unaffected");
    }

    // t11: a is 10 s stale -> A fails to 100 %; B, healthy on the same
    // tick, keeps its normal 58.
    let r11 = sim.tick(11.0, &[("b", 50.0)], &[]);
    assert!(r11[0].is_failsafe());
    assert!(f64_eq(r11[0].pwm().unwrap(), 100.0));
    assert!(!r11[1].is_failsafe(), "B must not be affected by A's failure");
    assert!(f64_eq(r11[1].pwm().unwrap(), 58.0), "B keeps its normal command");
    assert!(r11[1].is_clean());
}

// -- 10.3 determinism --------------------------------------------------------

#[test]
fn deterministic_replay_identical_inputs() {
    // spec *Deterministic on identical input*: two simulations from
    // identical state fed identical injected timestamps/readings produce
    // identical `TickResult`s, tick for tick.
    let run = || -> Vec<TickResult> {
        let control = control_end_to_end();
        let profile = curve_profile(
            "p",
            vec![ComfortSourceRef {
                key: "g".into(),
                ema_tau_s: None,
                required: None,
            }],
            vec![TripwireRef { key: "h".into() }],
            SlewRates {
                up: 20.0,
                down: 7.0,
            },
        );
        let mut sim = Sim::new(control, vec![profile]);
        let mut out = Vec::new();
        // A scripted timeline covering: seeding, steady, latch, gap < 5 s,
        // gap > 5 s (clamped — a fault), transient gaps.
        #[allow(clippy::type_complexity)]
        let timeline: &[(f64, &[(&str, f64)], &[(&str, f64)])] = &[
            (0.0, &[("g", 50.0)], &[("h", 60.0)]),
            (1.0, &[("g", 55.0)], &[("h", 60.0)]),
            (2.0, &[("g", 50.0)], &[("h", 85.0)]),
            (3.0, &[("g", 30.0)], &[("h", 85.0)]),
            (4.0, &[("g", 35.0)], &[]),
            (5.0, &[("g", 40.0)], &[]),
            (7.0, &[], &[]),
            (30.0, &[("g", 60.0)], &[("h", 85.0)]),
        ];
        for (t, comfort, trip) in timeline {
            out.push(sim.tick(*t, comfort, trip)[0].clone());
        }
        out
    };

    let a = run();
    let b = run();
    assert_eq!(
        a.len(),
        b.len(),
        "both replays run the same number of ticks"
    );
    for (i, (ra, rb)) in a.iter().zip(b.iter()).enumerate() {
        assert_eq!(ra.outcome, rb.outcome, "tick {i}: outcomes differ");
        assert_eq!(ra.faults.len(), rb.faults.len(), "tick {i}: fault count");
        for (fa, fb) in ra.faults.iter().zip(rb.faults.iter()) {
            assert_eq!(fa.source, fb.source, "tick {i}: fault source");
            assert_eq!(fa.kind, fb.kind, "tick {i}: fault kind");
            assert_eq!(fa.severity, fb.severity, "tick {i}: fault severity");
            assert_eq!(fa.message, fb.message, "tick {i}: fault message");
        }
    }

    // Sanity on the replayed content itself: the 23 s gap at t=30 is
    // reported (clamped) and did not fail-safe (comfort is fresh).
    let last = a.last().unwrap();
    assert!(
        last.faults.iter().any(|f| f.kind == FaultKind::ClampedTickGap),
        "t=30: the 23 s gap must be reported: {:?}",
        last
    );
    assert!(!last.is_failsafe());
}

// -- 10.4 failsafe bypasses the slew limiter ----------------------------------

#[test]
fn failsafe_bypasses_slew() {
    // spec *Fail-safe bypasses the slew limiter*: 20 % commanded, 1 s at
    // 20 %/s up would allow 40 % — the fail-safe emits 100 % on the tick.
    let control = control_end_to_end();
    let profile = curve_profile(
        "p",
        vec![ComfortSourceRef {
            key: "g".into(),
            ema_tau_s: None,
            required: None,
        }],
        Vec::new(),
        SlewRates {
            up: 20.0,
            down: 7.0,
        },
    );
    let mut sim = Sim::new(control, vec![profile]);

    // t0: cold start (seed g=40 → curve target 42).
    let r0 = sim.tick(0.0, &[("g", 40.0)], &[]);
    assert_eq!(r0[0].outcome, Outcome::NoCommand);

    // t1: command 20 % (from 0 at 20 %/s the 42 % target is not reachable).
    let r1 = sim.tick(1.0, &[("g", 40.0)], &[]);
    assert!(f64_eq(r1[0].pwm().unwrap(), 20.0), "t1: {:?}", r1[0]);
    assert!(!r1[0].is_failsafe());

    // t2..t10: comfort goes quiet but stays within the 10 s bound; the
    // output climbs at 20 %/s to the 42 % target and holds.
    for i in 2..10 {
        let t = i as f64;
        let r = sim.tick(t, &[], &[]);
        assert!(!r[0].is_failsafe(), "t{t}: within bound");
        assert!(r[0].is_clean(), "t{t}: {:?}", r[0]);
    }
    assert!(f64_eq(sim.pwm(0).unwrap(), 42.0), "steady at the 42 % target");

    // t11: the sample is now 10 s old (>= the bound) -> fail-safe. Without
    // the bypass, one tick at 20 %/s up from 42 % would emit 62 %; the spec
    // demands 100 % on the tick.
    let r11 = sim.tick(11.0, &[], &[]);
    assert!(r11[0].is_failsafe(), "t11: {:?}", r11[0]);
    assert!(
        f64_eq(r11[0].pwm().unwrap(), 100.0),
        "failsafe must emit 100 % on the tick (not 62 % via slew): {:?}",
        r11[0]
    );
    assert!(r11[0]
        .faults
        .iter()
        .any(|f| f.source == "g" && f.kind == FaultKind::ComfortSourceStale));
}

// -- 10.5 cross-cutting: one evaluation, many consumers ------------------------

#[test]
fn one_evaluation_many_consumers() {
    // spec *One evaluation, many consumers*: a single tripwire source shared
    // by three profiles; when it clears, all three revert on the same tick.
    let mut control = FanControlConfig::default();
    control.sensor_sources.insert("g".into(), comfort_src(1.5, true));
    control.sensor_sources.insert("h".into(), trip_src(80.0, 5.0, 90.0));
    let mk = |id: &str| {
        curve_profile(
            id,
            vec![ComfortSourceRef {
                key: "g".into(),
                ema_tau_s: None,
                required: None,
            }],
            vec![TripwireRef { key: "h".into() }],
            SlewRates {
                up: 100.0,
                down: 100.0,
            },
        )
    };
    let mut sim = Sim::new(control, vec![mk("a"), mk("b"), mk("c")]);

    // t0..t2: latch at 85; every profile is floored to 90.
    let _ = sim.tick(0.0, &[("g", 50.0)], &[("h", 60.0)]);
    let r1 = sim.tick(1.0, &[("g", 50.0)], &[("h", 85.0)]);
    for (i, r) in r1.iter().enumerate() {
        assert!(
            f64_eq(r.pwm().unwrap(), 90.0),
            "t1 profile {i} floored to 90: {:?}",
            r
        );
    }

    // t3: cools below threshold - hysteresis; all three profiles revert to
    // the mode-derived 58 on the same tick — no per-profile divergence.
    let r3 = sim.tick(3.0, &[("g", 50.0)], &[("h", 73.0)]);
    for (i, r) in r3.iter().enumerate() {
        assert!(
            f64_eq(r.pwm().unwrap(), 58.0),
            "t3 profile {i} reverts to 58 together: {:?}",
            r
        );
        assert!(r.is_clean(), "t3 profile {i}: {:?}", r);
    }
}
