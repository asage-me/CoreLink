//! PID controller for `target_temp` mode (design D4).
//!
//! `u = Kp·e + Ki·∫e·dt + Kd·de/dt`, with error = folded °C − setpoint °C
//! (positive when hot) and output clamped to 0–100 %.
//!
//! - **Derivative on error** (not on measurement): there is no cleaner PV
//!   signal, and the input is EMA-smoothed anyway.
//! - **Anti-windup: back-calculation clamping** — while the output is at a
//!   0–100 % clamp bound, the integrator is clamped to the value that would
//!   have produced the saturated output, so no debt accumulates and a
//!   reversing error gets an immediate response (spec *Anti-windup under
//!   saturation*).
//! - One `Pid` instance per profile; state is never shared (spec *Independent
//!   integrator per profile*).

use std::time::Duration;

use crate::config::PidGains;

/// A per-profile PID controller with back-calculation anti-windup.
#[derive(Debug, Clone)]
pub struct Pid {
    gains: PidGains,
    integral: f64,
    last_error: Option<f64>,
}

impl Pid {
    /// Fresh controller (integral 0, no derivative memory).
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            integral: 0.0,
            last_error: None,
        }
    }

    /// The accumulated integral (°C·s), for telemetry/tests.
    pub fn integral(&self) -> f64 {
        self.integral
    }

    /// The gains in effect.
    pub fn gains(&self) -> &PidGains {
        &self.gains
    }

    /// Advance one tick.
    ///
    /// `error` = folded °C − setpoint °C (positive when hot). `dt` must be
    /// the gap-clamped tick delta (design D3); a zero delta integrates and
    /// differentiates nothing. Returns the output duty % clamped to 0–100.
    pub fn update(&mut self, error: f64, dt: Duration) -> f64 {
        let dt_s = dt.as_secs_f64();

        // Derivative on error; 0 until a previous error exists (first-tick
        // rule) or when no time has elapsed.
        let derivative = match (self.last_error, dt_s > 0.0) {
            (Some(prev), true) => (error - prev) / dt_s,
            _ => 0.0,
        };
        self.last_error = Some(error);

        // Integral: accumulate ∫e·dt. Skipped entirely when Ki = 0 so a
        // proportional-only controller carries no debt (spec *Ramps up when
        // hot* asserts a zero integral).
        if dt_s > 0.0 && self.gains.ki > 0.0 {
            self.integral += error * dt_s;
        }

        let p = self.gains.kp * error;
        let i = self.gains.ki * self.integral;
        let d = self.gains.kd * derivative;
        let u_raw = p + i + d;
        let u_sat = u_raw.clamp(0.0, 100.0);

        // Back-calculation anti-windup (design D4): when the output
        // saturates, hold the integral at the value that would have produced
        // the saturated output. Because the saturation side is monotone in
        // the integral (Ki > 0), this is a one-sided clamp; it also winds
        // the integral down as soon as the error reverses enough to leave
        // saturation, so there is no integral-debt recovery delay.
        if self.gains.ki > 0.0 {
            if u_raw > 100.0 {
                let cap = (100.0 - p - d) / self.gains.ki;
                if self.integral > cap {
                    self.integral = cap;
                }
            } else if u_raw < 0.0 {
                let floor = (0.0 - p - d) / self.gains.ki;
                if self.integral < floor {
                    self.integral = floor;
                }
            }
        }

        u_sat
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dur(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    fn gains(kp: f64, ki: f64) -> PidGains {
        PidGains {
            kp,
            ki,
            kd: 0.0,
        }
    }

    #[test]
    fn proportional_only_response_with_zero_ki() {
        // spec *Ramps up when hot*: setpoint 55, folded 75 → error +20 °C,
        // Ki=0 → output = Kp×20 clamped to 0–100.
        let mut pid = Pid::new(gains(2.0, 0.0));
        let out = pid.update(20.0, dur(1.0));
        assert!((out - 40.0).abs() < 1e-9, "Kp=2 × 20 °C = 40 %");
        // no integral debt builds with Ki = 0
        assert_eq!(pid.integral(), 0.0);
    }

    #[test]
    fn proportional_clamps_at_bounds() {
        let mut pid = Pid::new(gains(2.0, 0.0));
        assert_eq!(pid.update(60.0, dur(1.0)), 100.0, "hot error clamps to 100");
        assert_eq!(pid.update(-60.0, dur(1.0)), 0.0, "cold error clamps to 0");
    }

    #[test]
    fn reaches_and_holds_setpoint_is_steady() {
        // spec *Reaches and holds setpoint*: with error ~0 the output is
        // steady and the controller's own terms induce no oscillation.
        let mut pid = Pid::new(gains(2.0, 0.2));
        // Drive the integral to a plausible holding value with a hot start,
        // then settle at the setpoint.
        for _ in 0..10 {
            pid.update(10.0, dur(1.0));
        }
        let first = pid.update(0.0, dur(1.0));
        for _ in 0..10 {
            let out = pid.update(0.0, dur(1.0));
            assert!((out - first).abs() < 1e-9, "output must be steady at zero error");
        }
    }

    #[test]
    fn independent_integrators_across_instances() {
        // spec *Independent integrator per profile*: identical histories →
        // identical outputs; a divergent history on one instance does not
        // perturb the other.
        let mut a = Pid::new(gains(2.0, 0.2));
        let mut b = Pid::new(gains(2.0, 0.2));
        let mut c = Pid::new(gains(2.0, 0.2));
        // Verify A, B and C produce identical outputs for identical histories.
        for _ in 0..5 {
            let oa = a.update(15.0, dur(1.0));
            let ob = b.update(15.0, dur(1.0));
            let oc = c.update(15.0, dur(1.0));
            assert!((oa - ob).abs() < 1e-12, "identical histories → identical outputs");
            assert!((oa - oc).abs() < 1e-12, "identical histories → identical outputs");
        }
        // Now diverge B: A and C must continue producing identical outputs.
        let _ = b.update(-30.0, dur(1.0));
        let oa = a.update(15.0, dur(1.0));
        let oc = c.update(15.0, dur(1.0));
        assert!((oa - oc).abs() < 1e-12, "divergent history on one profile must not affect the other");
    }

    #[test]
    fn anti_windup_under_saturation() {
        // spec *Anti-windup under saturation*: saturate at 100 % for 20 s
        // with persistently positive error; once the error reverses, the
        // output must not keep a recovery delay attributable to accumulated
        // integral debt.
        let mut pid = Pid::new(gains(2.0, 0.2)); // defaults Kp=2, Ki=0.2, Kd=0
        const DT: f64 = 1.0;

        // Reference: a plain PI integrator WITHOUT anti-windup, same history.
        // (We cannot use a second `Pid` — this one also anti-winds.)
        let mut unwound_i = 0.0;
        let unwound_update = |unwound_i: &mut f64, e: f64| {
            *unwound_i += e * DT;
            let u = 2.0 * e + 0.2 * *unwound_i; // p + i, Kd = 0
            u.clamp(0.0, 100.0)
        };

        // Phase 1: error +20 °C (setpoint 55, folded 75). The proportional
        // term is 40 %, so the integral grows until the output saturates.
        let mut ticks = 0;
        while ticks < 15 {
            let u = pid.update(20.0, dur(DT));
            unwound_update(&mut unwound_i, 20.0);
            ticks += 1;
            if u >= 100.0 {
                break;
            }
        }
        assert!(pid.update(20.0, dur(DT)) >= 100.0, "sanity: saturating");
        unwound_update(&mut unwound_i, 20.0);

        // Phase 2: 20 s of persistent saturation at 100 %.
        for _ in 0..20 {
            let u = pid.update(20.0, dur(DT));
            assert_eq!(u, 100.0, "stays saturated while error persists");
            unwound_update(&mut unwound_i, 20.0);
        }

        let debt = unwound_i - pid.integral();
        assert!(debt > 10.0, "without anti-windup the integral would hold debt (debt={debt})");

        // Phase 3: error reverses to −20 °C. An anti-wound controller must
        // drop promptly; an unwound one stays pinned near 100 %.
        let u_wound = pid.update(-20.0, dur(DT));
        let u_unwound = unwound_update(&mut unwound_i, -20.0);
        assert!(
            u_wound < u_unwound,
            "wound output {u_wound} must be below the debt-pinned output {u_unwound}"
        );
        assert!(u_wound <= 50.0, "no integral-debt recovery delay (got {u_wound})");
    }

    #[test]
    fn zero_delta_is_a_noop_on_integrator_and_derivative() {
        let mut pid = Pid::new(gains(2.0, 0.2));
        let first = pid.update(20.0, Duration::ZERO);
        let second = pid.update(20.0, Duration::ZERO);
        assert!((first - second).abs() < 1e-12);
        assert_eq!(pid.integral(), 0.0, "zero delta integrates nothing");
    }

    #[test]
    fn derivative_on_error_kicks_on_second_update() {
        let mut pid = Pid::new(PidGains {
            kp: 0.0,
            ki: 0.0,
            kd: 1.0,
        });
        let first = pid.update(10.0, dur(1.0));
        assert_eq!(first, 0.0, "no derivative on the first update");
        let second = pid.update(11.0, dur(1.0));
        assert!((second - 1.0).abs() < 1e-9, "de/dt = (11−10)/1 s = 1 °C/s → +1 %");
    }
}
