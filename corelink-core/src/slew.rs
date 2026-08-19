//! Asymmetric slew-rate limiting (spec *Slew-Rate Limiting*).
//!
//! Stateless by design: the position is seeded from the injected
//! last-commanded PWM every tick (design D2 — the core stores no output
//! position), so mode switches and daemon-supplied manual overrides cannot
//! desync the limiter.

use std::time::Duration;

/// The result of one tick's slew limiting: the PWM to actually emit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlewResult {
    /// The emitted PWM (%) after applying the rate limit.
    pub pwm: f64,
}

/// Step the current position toward the target, bounded by an asymmetric
/// rate limit.
///
/// - `position`: the last commanded PWM (injected by the daemon each tick).
/// - `target`: the effective target PWM (after protection floor and
///   min-duty floor, before the limiter).
/// - `up_rate`: maximum allowed change per second when increasing.
/// - `down_rate`: maximum allowed change per second when decreasing.
/// - `dt`: the injected tick delta.
///
/// Returns the next position to feed back to the daemon. Moves at most
/// `rate × dt` toward the target; if the target is within reach the limiter
/// does not add artificial delay (spec *Small correction unimpeded*).
pub fn step_toward(position: f64, target: f64, up_rate: f64, down_rate: f64, dt: Duration) -> SlewResult {
    let dt_s = dt.as_secs_f64();
    let delta = target - position;
    if delta.abs() < f64::EPSILON {
        // Already at target — no movement.
        return SlewResult {
            pwm: position.clamp(0.0, 100.0),
        };
    }
    let rate = if delta > 0.0 { up_rate } else { down_rate };
    let max_step = rate * dt_s;
    let step = delta.abs().min(max_step).copysign(delta);
    SlewResult {
        pwm: (position + step).clamp(0.0, 100.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dur(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    #[test]
    fn upward_rate_limited() {
        // spec *Upward rate-limited*: 20% → 80%, up 20 %/s, dt=1s → 40%.
        let r = step_toward(20.0, 80.0, 20.0, 5.0, dur(1.0));
        assert!((r.pwm - 40.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn downward_slower_than_upward() {
        // spec *Downward slower than upward*: 80% → 10%, up 20, down 5, dt=1 → 75%.
        let r = step_toward(80.0, 10.0, 20.0, 5.0, dur(1.0));
        assert!((r.pwm - 75.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn small_correction_unimpeded() {
        // spec *Small correction unimpeded*: target within one tick's range.
        // 20% → 35%, up 20 %/s, dt=1s → max step 20, delta 15 ≤ 20 → reach 35.
        let r = step_toward(20.0, 35.0, 20.0, 5.0, dur(1.0));
        assert!((r.pwm - 35.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn zero_delta_is_noop() {
        let r = step_toward(20.0, 80.0, 20.0, 5.0, Duration::ZERO);
        assert!((r.pwm - 20.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn at_target_is_noop() {
        let r = step_toward(45.0, 45.0, 20.0, 5.0, dur(2.0));
        assert!((r.pwm - 45.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn multi_tick_ramp() {
        // Simulate a 20 %/s up ramp over 4 seconds from 0 → target 100.
        let mut pos = 0.0;
        for _ in 0..4 {
            let r = step_toward(pos, 100.0, 20.0, 5.0, dur(1.0));
            pos = r.pwm;
        }
        assert!((pos - 80.0).abs() < 1e-9, "got {pos}");
    }

    #[test]
    fn result_clamped_to_range() {
        // Target above 100 % clamps the *position*: from 0 % we may take up
        // to 100 %/s × 2 s = 200 points, so we reach the clamped target.
        let r = step_toward(0.0, 150.0, 100.0, 50.0, dur(2.0));
        assert!((r.pwm - 100.0).abs() < 1e-9, "got {}", r.pwm);
        // Target below 0 % clamps the *position*: from 100 % at 50 %/s down
        // we need 2 s to reach the clamped target of 0 %.
        let r = step_toward(100.0, -50.0, 100.0, 50.0, dur(2.0));
        assert!((r.pwm - 0.0).abs() < 1e-9, "got {}", r.pwm);
        // A single tick cannot overshoot the slew budget.
        let r = step_toward(100.0, -50.0, 100.0, 50.0, dur(1.0));
        assert!((r.pwm - 50.0).abs() < 1e-9, "got {}", r.pwm);
    }

    #[test]
    fn fractional_delta() {
        // 0.5 s at 20 %/s up: 10 → 20 (max step 10, delta 100) → 20.
        let r = step_toward(10.0, 100.0, 20.0, 5.0, dur(0.5));
        assert!((r.pwm - 20.0).abs() < 1e-9, "got {}", r.pwm);
    }
}
