//! Time-corrected exponential moving average (design D3).
//!
//! `state ← state + (1 − e^(−Δt/τ)) · (x − state)`; the first sample *seeds*
//! the state rather than blending with zero. This is the discrete-exact EMA:
//! correct for any Δt (a 900 ms stall must not skew the filter, and neither
//! must a longer one — long gaps are clamped to `max_gap` and reported via
//! [`EmaUpdate::clamped`], design D3).

use std::time::Duration;

/// Result of one EMA update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EmaUpdate {
    /// The smoothed value after the update.
    pub value: f64,
    /// The smoothed value before this update; `None` when this sample seeded
    /// the state (first-ever sample).
    pub prev: Option<f64>,
    /// The (possibly clamped) time step actually integrated.
    pub dt: Duration,
    /// True when the raw tick delta was clamped to the gap bound (log-worthy,
    /// design D3). Always false on a seeding update.
    pub clamped: bool,
}

/// An unseeded exponential moving average with time-corrected steps.
///
/// `None` until seeded: the first sample sets the state exactly (never
/// zero-padded).
#[derive(Debug, Clone, Default)]
pub struct Ema {
    value: Option<f64>,
    last: Option<Duration>,
    samples: u32,
}

impl Ema {
    /// Unseeded filter.
    pub const fn new() -> Self {
        Self {
            value: None,
            last: None,
            samples: 0,
        }
    }

    /// False until the first sample has seeded the state.
    pub const fn is_seeded(&self) -> bool {
        self.value.is_some()
    }

    /// The current smoothed value, `None` until seeded.
    pub const fn value(&self) -> Option<f64> {
        self.value
    }

    /// Number of samples consumed so far (dT/dt is disabled below 2; spec
    /// *Tripwire Detection*).
    pub const fn samples(&self) -> u32 {
        self.samples
    }

    /// Timestamp of the last consumed sample.
    pub const fn last_update(&self) -> Option<Duration> {
        self.last
    }

    /// Consume a sample at injected timestamp `now`, advancing (or seeding)
    /// the filter with time constant `tau_s`. If the raw gap since the last
    /// sample exceeds `max_gap`, the filter integrates with the clamped gap
    /// and returns [`EmaUpdate::clamped`].
    ///
    /// A non-monotonic clock (now < last) integrates zero time and leaves the
    /// state unchanged, so the result stays deterministic.
    pub fn update(&mut self, now: Duration, tau_s: f64, x: f64, max_gap: Duration) -> EmaUpdate {
        self.samples += 1;
        let prev = self.value;
        let last = self.last;
        match (prev, last) {
            (None, _) => {
                // First sample seeds the state — never blended with zero.
                self.value = Some(x);
                self.last = Some(now);
                EmaUpdate {
                    value: x,
                    prev: None,
                    dt: Duration::ZERO,
                    clamped: false,
                }
            }
            (Some(s), Some(t)) => {
                let raw = now.saturating_sub(t);
                let dt = raw.min(max_gap);
                let clamped = raw > max_gap;
                let alpha = 1.0 - (-dt.as_secs_f64() / tau_s).exp();
                let v = s + alpha * (x - s);
                self.value = Some(v);
                self.last = Some(now);
                EmaUpdate {
                    value: v,
                    prev: Some(s),
                    dt,
                    clamped,
                }
            }
            // (Some, None) is unreachable: seeding sets both together.
            _ => unreachable!("EMA state and timestamp are seeded together"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dur(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    #[test]
    fn first_sample_seeds_exactly() {
        // spec *First sample seeds the filter*
        let mut e = Ema::new();
        assert!(!e.is_seeded());
        let u = e.update(dur(0.0), 1.5, 70.0, dur(5.0));
        assert_eq!(u.value, 70.0, "first sample must seed the state, not blend with 0");
        assert_eq!(u.prev, None);
        assert!(e.is_seeded());
        assert_eq!(e.value(), Some(70.0));
        assert_eq!(e.samples(), 1);
    }

    #[test]
    fn one_step_at_tau_follows_discrete_exact_formula() {
        // Design D3 formula: 50 + (1 − e^(−1)) · (70 − 50) = 62.642… %.
        // NOTE: the spec scenario's "60.7" figure is arithmetically
        // inconsistent with its own α = 1 − e^(−Δt/τ) at Δt=τ (which is
        // 0.632, giving 62.64); the design D3 formula is the binding
        // contract and is asserted here.
        let mut e = Ema::new();
        e.update(dur(0.0), 1.0, 50.0, dur(5.0));
        let u = e.update(dur(1.0), 1.0, 70.0, dur(5.0));
        let expected = 50.0 + (1.0 - (-1.0_f64).exp()) * 20.0;
        assert!((u.value - expected).abs() < 1e-9, "{} vs {}", u.value, expected);
        assert!((u.value - 62.6424).abs() < 0.001);
        assert_eq!(u.prev, Some(50.0));
        assert!(!u.clamped);
    }

    #[test]
    fn zero_delta_leaves_state_unchanged() {
        let mut e = Ema::new();
        e.update(dur(0.0), 1.0, 50.0, dur(5.0));
        let u = e.update(dur(0.0), 1.0, 70.0, dur(5.0));
        assert_eq!(u.value, 50.0, "Δt=0 must integrate no time");
        assert_eq!(u.prev, Some(50.0));
    }

    #[test]
    fn multi_step_converges_to_reading() {
        let mut e = Ema::new();
        e.update(dur(0.0), 1.0, 50.0, dur(5.0));
        for i in 1..=40 {
            e.update(dur(i as f64), 1.0, 70.0, dur(5.0));
        }
        // Residual after 40 unit-τ steps is 20·e^-40 ≈ 3.7e-16 — but f64
        // rounding accumulates, so allow a loose 1e-6 "converged" band.
        assert!((e.value().unwrap() - 70.0).abs() < 1e-6, "must converge to the constant reading");
    }

    #[test]
    fn constant_input_is_stable() {
        let mut e = Ema::new();
        e.update(dur(0.0), 2.0, 45.0, dur(5.0));
        for i in 1..=5 {
            let u = e.update(dur(i as f64), 2.0, 45.0, dur(5.0));
            assert!((u.value - 45.0).abs() < 1e-9, "a constant input must not drift");
        }
    }

    #[test]
    fn gap_clamped_to_max_gap_and_reported() {
        // A 30 s gap with max_gap 5 s integrates 5 s of smoothing, not 30.
        let mut e = Ema::new();
        e.update(dur(0.0), 1.0, 50.0, dur(5.0));
        let u = e.update(dur(30.0), 1.0, 70.0, dur(5.0));
        assert!(u.clamped);
        let expected_unclamped = 50.0 + (1.0 - (-30.0_f64).exp()) * 20.0; // ≈ 70.0
        assert!((u.value - expected_unclamped).abs() > 0.1, "clamp must change the result");
        let expected_clamped = 50.0 + (1.0 - (-5.0_f64).exp()) * 20.0;
        assert!((u.value - expected_clamped).abs() < 1e-9, "{} vs {}", u.value, expected_clamped);
    }

    #[test]
    fn backdated_timestamp_is_deterministic_no_crash() {
        let mut e = Ema::new();
        e.update(dur(10.0), 1.0, 50.0, dur(5.0));
        let u = e.update(dur(5.0), 1.0, 70.0, dur(5.0));
        assert_eq!(u.value, 50.0, "backdated tick must not corrupt the state");
    }
}
