//! Time-corrected exponential moving average (design D3).
//!
//! `state ← state + (1 − e^(−Δt/τ)) · (x − state)`; the first sample *seeds*
//! the state rather than blending with zero.
