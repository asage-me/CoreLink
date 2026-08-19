//! Piecewise-linear fan-curve lookup (design D5): sorted points, linear
//! interior, clamped ends.

use crate::config::CurvePoint;

/// A validated, sorted fan curve.
///
/// Construction sorts the points by temperature; `Curve::lookup` is a
/// linear scan (n is small, ~5–15 points).
#[derive(Debug, Clone)]
pub struct Curve {
    /// Sorted by `temp_c`, strictly increasing (checked by config
    /// validation, but also re-asserted here so the type is safe
    /// standalone).
    points: Vec<CurvePoint>,
}

impl Curve {
    /// Build a curve from user-defined points.
    ///
    /// # Panics
    ///
    /// Fewer than 2 points, duplicate temperatures, or a non-finite
    /// temperature. (These are all rejected earlier by config validation;
    /// the panic is a defence-in-depth invariant, not an expected path.)
    pub fn new(points: Vec<CurvePoint>) -> Self {
        let mut points = points;
        assert!(points.len() >= 2, "curve requires at least 2 points");
        assert!(points.iter().all(|p| p.temp_c.is_finite()), "curve point temperature must be finite");
        points.sort_by(|a, b| a.temp_c.total_cmp(&b.temp_c));
        for w in points.windows(2) {
            assert!(w[1].temp_c > w[0].temp_c, "curve point temperatures must be strictly increasing");
        }
        Self { points }
    }

    /// PWM % target for a folded temperature.
    ///
    /// - Interior: linear interpolation between the bracketing points.
    /// - At or below the lowest point: that point's %.
    /// - At or above the highest point: that point's % (clamped).
    pub fn lookup(&self, temp_c: f64) -> f64 {
        debug_assert!(self.points.len() >= 2);
        let pts = &self.points;
        if temp_c <= pts[0].temp_c {
            return pts[0].pwm;
        }
        let last = pts.len() - 1;
        if temp_c >= pts[last].temp_c {
            return pts[last].pwm;
        }
        // Find the first point strictly above temp_c (points are sorted).
        let hi = pts.iter().position(|p| p.temp_c > temp_c).unwrap();
        let lo = hi - 1;
        let (t0, pwm0) = (pts[lo].temp_c, pts[lo].pwm);
        let (t1, pwm1) = (pts[hi].temp_c, pts[hi].pwm);
        let f = (temp_c - t0) / (t1 - t0);
        pwm0 + f * (pwm1 - pwm0)
    }

    /// The stored points (sorted), for inspection.
    pub fn points(&self) -> &[CurvePoint] {
        &self.points
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_point_curve() -> Curve {
        Curve::new(vec![
            CurvePoint { temp_c: 40.0, pwm: 20.0 },
            CurvePoint { temp_c: 70.0, pwm: 80.0 },
        ])
    }

    #[test]
    fn interior_interpolation() {
        // spec *Interior interpolation*: points (40,20) and (70,80),
        // folded 55 °C → 50 %.
        let c = two_point_curve();
        let v = c.lookup(55.0);
        assert!((v - 50.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn clamped_at_high_end_when_above_all_points() {
        // spec *Clamped at ends*: folded 200 °C (above all points),
        // highest point (90 °C, 80%) → mode target 80 %.
        let c = Curve::new(vec![
            CurvePoint {
                temp_c: 20.0,
                pwm: 20.0,
            },
            CurvePoint {
                temp_c: 90.0,
                pwm: 80.0,
            },
        ]);
        let v = c.lookup(200.0);
        assert!((v - 80.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn clamped_at_low_end_when_below_all_points() {
        // Below the lowest point (40 °C, 20%) → that point's 20 %.
        let c = two_point_curve();
        let v = c.lookup(10.0);
        assert!((v - 20.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn clamped_at_high_end_of_two_point_curve() {
        let c = two_point_curve();
        let v = c.lookup(80.0);
        assert!((v - 80.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn clamped_at_low_end_of_two_point_curve() {
        let c = two_point_curve();
        let v = c.lookup(30.0);
        assert!((v - 20.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn interior_multi_point() {
        let c = Curve::new(vec![
            CurvePoint {
                temp_c: 20.0,
                pwm: 20.0,
            },
            CurvePoint {
                temp_c: 50.0,
                pwm: 60.0,
            },
            CurvePoint {
                temp_c: 80.0,
                pwm: 90.0,
            },
        ]);
        // 35 °C → 20 + (60-20) * (35-20)/(50-20) = 20 + 40 * 0.5 = 40
        let v = c.lookup(35.0);
        assert!((v - 40.0).abs() < 1e-9, "got {v}");
        // 65 °C → 60 + (90-60) * (65-50)/(80-50) = 60 + 30 * 0.5 = 75
        let v = c.lookup(65.0);
        assert!((v - 75.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn unsorted_input_is_sorted_on_construction() {
        let c = Curve::new(vec![
            CurvePoint {
                temp_c: 70.0,
                pwm: 80.0,
            },
            CurvePoint {
                temp_c: 40.0,
                pwm: 20.0,
            },
        ]);
        assert!((c.lookup(55.0) - 50.0).abs() < 1e-9, "got {}", c.lookup(55.0));
    }

    #[test]
    #[should_panic]
    fn fewer_than_two_points_panics() {
        Curve::new(vec![CurvePoint {
            temp_c: 50.0,
            pwm: 40.0,
        }]);
    }

    #[test]
    #[should_panic]
    fn duplicate_temperatures_panic() {
        Curve::new(vec![
            CurvePoint {
                temp_c: 40.0,
                pwm: 20.0,
            },
            CurvePoint {
                temp_c: 40.0,
                pwm: 30.0,
            },
        ]);
    }
}
