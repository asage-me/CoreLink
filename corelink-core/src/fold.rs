//! Fold layer: reduce the comfort drivers' smoothed values to the single
//! scalar the mode controller consumes (spec *Fold Rule*). Tripwire sensors
//! never enter the fold — they are excluded by the pipeline, not by count.

use crate::config::FoldRule;

impl FoldRule {
    /// Reduce a non-empty iterator of smoothed values to one scalar.
    ///
    /// `Max` (default, conservative — the hottest driver wins) and `Avg`
    /// (equal-weight arithmetic mean) both always produce a value within the
    /// observed range of their inputs.
    ///
    /// # Panics
    ///
    /// Empty input is a caller error: the pipeline only folds usable
    /// (present or still-fresh) sources and fail-safes before folding when
    /// none are usable.
    pub fn fold(&self, values: impl Iterator<Item = f64>) -> f64 {
        let values: Vec<f64> = values.collect();
        assert!(!values.is_empty(), "FoldRule::fold: empty input is a caller error");
        match self {
            FoldRule::Max => values.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            FoldRule::Avg => values.iter().sum::<f64>() / values.len() as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::config::FoldRule;

    #[test]
    fn default_is_max() {
        assert_eq!(FoldRule::default(), FoldRule::Max);
    }

    #[test]
    fn default_fold_is_max_of_drivers() {
        // spec *Default fold is max*: 55 and 78 → 78
        let v = FoldRule::Max.fold([55.0, 78.0].into_iter());
        assert_eq!(v, 78.0);
    }

    #[test]
    fn explicit_avg_fold() {
        // spec *Explicit avg fold*: 55 and 70 → 62.5
        let v = FoldRule::Avg.fold([55.0, 70.0].into_iter());
        assert!((v - 62.5).abs() < 1e-12);
    }

    #[test]
    fn fold_is_within_observed_range() {
        let v_max = FoldRule::Max.fold([10.0, 20.0, 33.0].into_iter());
        let v_avg = FoldRule::Avg.fold([10.0, 20.0, 33.0].into_iter());
        assert!((10.0..=33.0).contains(&v_max), "{v_max} out of range");
        assert!((10.0..=33.0).contains(&v_avg), "{v_avg} out of range");
    }

    #[test]
    #[should_panic(expected = "caller error")]
    fn empty_input_is_caller_error() {
        FoldRule::Avg.fold(std::iter::empty::<f64>());
    }

    #[test]
    fn single_value_fold() {
        let v = FoldRule::Max.fold([42.0].into_iter());
        assert_eq!(v, 42.0);
        let v = FoldRule::Avg.fold([42.0].into_iter());
        assert_eq!(v, 42.0);
    }
}
