//! The per-profile tick pipeline: `ProfileHandle::tick(&TickInput) -> TickResult`
//! (design D1), with the D6 pipeline ordering and the D7
//! health/cold-start state machine.
