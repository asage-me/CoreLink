## 1. Workspace scaffolding

- [x] 1.1 `git init` the repo and create the Cargo workspace (root `Cargo.toml` with `corelink-core` member)
- [x] 1.2 Create `corelink-core` crate with module skeleton (`lib.rs`, `config.rs`, `ema.rs`, `curve.rs`, `pid.rs`, `slew.rs`, `tripwire.rs`, `profile.rs`) and dev-dependency-free `Cargo.toml` (only `serde` with derive)

## 2. Config value types + validation

- [x] 2.1 Serde types per design D8: `FanProfileConfig` (mode, static percent, curve points, setpoint, gains, min_duty, fold, slew rates, comfort sources, tripwire refs), `TripwireConfig`, `CurvePoint`, `PidGains`, `FoldRule`, `SlewRates`
- [x] 2.2 Pure `validate(&FanProfileConfig) -> ValidationErrors` covering mode-field coherence (curve requires ≥2 distinct-temp points; static_percent must not declare comfort sources but may declare tripwire/display; target_temp requires setpoint + gains; device_memory needs no sensor declarations), tau/hysteresis/rates/min_duty bounds; unit tests for every rejection and the happy paths (spec: *Mode is exclusive*, *Missing comfort_driver rejected*)

## 3. EMA module

- [x] 3.1 First-sample-seeds behavior (spec: *First sample seeds the filter*); `None` until seeded
- [x] 3.2 Time-corrected update `state + (1 − e^(−Δt/τ))·(x − state)` with injected `Duration` delta (spec: *EMA follows injected time*); tick-gap clamping per design D3
- [x] 3.3 Unit tests: seeding, one-step math at Δt=τ (60.7 °C case), multi-step convergence, gap clamp

## 4. Fold module

- [x] 4.1 `FoldRule` enum (`Max` default, `Avg`) reducing a non-empty iterator of smoothed values
- [x] 4.2 Unit tests: default max (55/78 → 78), avg (55/70 → 62.5), empty input is a caller error (spec: *Fold Rule* scenarios)

## 5. Curve lookup

- [x] 5.1 Sorted-points storage for ≥2 distinct-temp points (design D5); piecewise-linear interpolation; clamped ends (spec: *Curve Mode* — interior 55 °C → 50 %, above-max clamps)
- [x] 5.2 Unit tests for both spec scenarios plus below-min clamping

## 6. PID controller

- [x] 6.1 Per-profile `Pid` struct: `update(err, Δt) -> f64` output clamped [0,100], gains from config
- [x] 6.2 Anti-windup back-calculation per design D4 (spec: *Anti-windup under saturation* — 20 s saturated then reversed error, no integral-debt recovery delay)
- [x] 6.3 Unit tests: proportional-only response with Ki=0 (*Ramps up when hot*), settle behavior, independent integrators across two instances (*Independent integrator per profile*)

## 7. Tripwire state (shared)

- [x] 7.1 `TripwireState` per source: own EMA, latch with hysteresis clear, dT/dt check disabled until 2 samples (spec: *Tripwire Detection* — threshold trip, 40→80 spike trip, no false first-sample trip)
- [x] 7.2 Latch persistence: holds while above threshold−hysteresis, clears at or below it (spec: *Tripwire Latching* — 85/82/75 °C sequence)
- [x] 7.3 `TripwireEval` produced per source per tick: `online` flag + `latched` (last-known when offline) for the profile pipeline (design D2)

## 8. Slew-rate limiter

- [x] 8.1 `step_toward(position, target, rate, Δt)` with separate up/down rates; position initialized from the injected last-commanded PWM every tick (spec: *upward limited*, *downward slower*, *small correction unimpeded*)
- [x] 8.2 Unit tests including the 20→40 (up 20 %/s @1 s) and 80→75 (down 5 %/s @1 s) cases

## 9. Profile pipeline (the `tick()`)

- [x] 9.1 `ProfileHandle` + `TickInput`/`TickResult`/`Outcome` types per design D1; no-command vs command vs failsafe outcomes
- [x] 9.2 Mode gate + per-mode controllers: `static_percent` (constant duty; with declared tripwire(s) the protection floor can raise it while latched; with none, constant and fault-less — no failsafe path at all), `curve` (lookup, no integrator), `target_temp` (PID), `device_memory` (always NoCommand; faults become report-only, spec *device_memory never commands*)
- [x] 9.3 Health/cold-start state machine per design D7: first seed tick → NoCommand withhold; required source missing/stale → failsafe 100%; never-arrived sole source → 100% after cold start; cold start with an absent-but-other-fresh source (spec: *Required-vs-Optional*, *Cold-Start* scenarios)
- [x] 9.4 Pipeline ordering per design D6: health → filter updates → fold → mode controller → protection floor (`max` of latched tripwires, spec *Protection Floor*) → min-duty floor (exempt: static, failsafe) → slew → emit
- [x] 9.5 Soft tripwire-loss handling (spec: *Soft Handling of a Lost Tripwire Source*: no 100%, soft fault, last-known latch state holds the floor, unlatched-absent applies no floor)
- [x] 9.6 Fault events attached to `TickResult.faults` for: missing/stale/never-arrived comfort source, lost tripwire, tick-gap clamp — never silent (spec: *Fail-Safe to 100%*)

## 10. Integration + determinism tests

- [ ] 10.1 End-to-end scripted tick sequences (no hardware): cold start → steady curve operation → tripwire latch → protection floor holds while comfort drops → tripwire clears → tripwire source goes lost (soft) → comfort source stale → failsafe 100%
- [ ] 10.2 Per-port isolation test: profile A failsafe, profile B unchanged, same tick (spec: *Per-port isolation*)
- [ ] 10.3 Determinism test: replay identical inputs from identical state → identical `TickResult`s (spec: *Deterministic on identical input*)
- [ ] 10.4 Failsafe bypass test: 20 % commanded + 1 s @ 20 %/s up → 100 % (not 40 %) on the failsafe tick (spec: *Fail-safe bypasses the slew limiter*)
- [ ] 10.5 Full `cargo test -p corelink-core` green + `cargo clippy` clean
