## Context

Greenfield Rust workspace (no code yet). This change creates `corelink-core`, the pure-logic center of CoreLink's Phase 1 fan control. Upstream of it (later changes): the daemon owns hwmon reads, USB HID writes (per-channel speed commands, per-channel RPM reads — see OpenLinkHub's Commander implementation for protocol shape), config hot-reload, and fault surfacing. Downstream: the GUI edits the config value types defined here.

The core is invoked at most once per second per profile by the daemon. Inputs are fully injected; the crate must have zero non-dev dependencies.

## Goals / Non-Goals

**Goals:**
- A single, deterministic per-tick entry point per profile that satisfies every requirement in `specs/fan-control/spec.md`.
- Fully unit-testable: injected timestamps, injected readings, no hidden state that tests can't set or observe.
- A config value model (serde types) complete enough that the GUI and daemon changes can build against it without further core edits.
- An outcome type that makes "command" vs "no command" vs "failsafe" impossible to conflate at the type level, with structured fault events.

**Non-Goals:**
- No I/O, no async, no threads, no logging, no config file parsing from disk (only typed values + validation).
- No knowledge of devices, hwmon, PCIe BDF, or USB channels (that's config/daemon territory; the core sees opaque `s2`-keyed reading lists).
- No PID auto-tuning, no gain scheduling, no RPM-chase mode, no multi-fan coordination (one port = one profile; the daemon keeps them independent).
- No default gain *recommendation engine* — defaults exist, but choosing them is the user's job (documented in config comments).

## Decisions

### D1: One `tick()` entry point per profile, explicit state objects

```rust
pub struct ProfileHandle { /* sealed per-profile state */ }

pub struct TickInput<'a> {
    pub now: Duration,                 // injected monotonic timestamp
    pub readings: &'a Readings,        // map of source key -> Option<f64>
    pub last_commanded_pwm: f64,       // daemon feedback, 0.0–100.0
}

pub enum Outcome {
    Command { pwm: f64, failsafe: bool },
    NoCommand,                          // cold-start tick, or device_memory mode
}

pub struct TickResult {
    pub outcome: Outcome,
    pub faults: Vec<Fault>,             // Fault { source, kind, severity, message }
}

impl ProfileHandle {
    pub fn tick(&mut self, input: &TickInput) -> TickResult;
}
```

Rationale: one function = one place to reason about ordering, and tests call exactly what the daemon calls. `Outcome` being an enum (not `Option<f64>` + flags) makes the "no command" vs "failsafe" distinction unrepresentable-as-wrong. Alternatives considered: a global `ControlEngine::tick_all()` (rejected — the spec's one-profile-one-port scoping is cleaner as N independent handles, and the daemon can interleave devices naturally), and returning state deltas to the daemon (rejected — state stays on the core side; the daemon only feeds back the one fact it uniquely owns: last-commanded PWM).

### D2: State ownership — which state lives where

| State | Owner | Shared? |
|---|---|---|
| Per-comfort-driver EMA | `ProfileHandle` | no |
| Fold + mode controller state (curve is stateless; PID integrator only for `target_temp`) | `ProfileHandle` | no |
| Slew-limiter position | **not stored** — re-derived from `last_commanded_pwm` each tick | n/a |
| Per-source tripwire EMA + latch + hysteresis | **shared `TripwireStateMap`** (one entry per source key) | yes — built once by the daemon, handed to every profile referencing that source |
| Cold-start / last-sample bookkeeping | `ProfileHandle` | no |

The one genuinely shared object is tripwire state (`"a fact about the system, not per-profile state"`). It's a `HashMap<SourceKey, TripwireState>` owned by the daemon's tick scheduler: daemon evaluates all tripwire sources once per tick (cheap), then calls `profile.tick(..., &evaluated_tripwires)`. This keeps `ProfileHandle::tick` pure-with-shared-readonly-input — no interior mutability, no locking, trivially single-threaded. `SourceKey` is an opaque `u64`/`String` chosen by the daemon (e.g., normalized PCIe BDF); the core never inspects it.

Slew position is stateless-by-design (spec: "seeded from the injected last-commanded PWM, not an internal memory of the last emit") — this is what makes mode switches and daemon-supplied manual overrides desync-safe.

```rust
pub struct TripwireEval {
    pub key: SourceKey,
    pub latched: bool,   // last-known state when no reading this tick
    pub online: bool,    // did a reading arrive this tick? (false -> soft fault)
}
```

### D3: EMA — exponential form, time-corrected

`state ← state + (1 − e^(−Δt/τ)) · (x − state)`, with the first sample *setting* state (not blending). This is the discrete-exact EMA: correct for any Δt (the daemon's 1 Hz nominally, but a 900 ms stall must not skew the filter). A fixed-α-per-tick variant would treat a stall tick the same as a normal one — rejected.

**Tick-gap clamp:** if Δt > `max_tick_gap` (config, default 5 s), the filter clamps Δt to the bound (log-worthy fault) and PID uses the same clamped Δt. Without this, a systemd stall followed by one tick would produce one gigantic `e^(−Δt/τ)` jump. Mitigated, logged, still deterministic.

### D4: PID — form, anti-windup, and the "configured per mode" unification

Controller: `u = Kp·e + Ki·∫e·dt + Kd·de/dt`, error = folded °C − setpoint °C (positive when hot), output clamped [0, 100].

- **Derivative on error** (not on measurement) — there's no cleaner PV signal, and the input is EMA-smoothed anyway.
- **Anti-windup: back-calculation clamping** — when `u` saturates, the integrator is pulled toward `sat(u_raw − Kp·e)/Kp` (the value that would have produced the saturated output). This satisfies the spec's wind-down-on-reverse-error scenario with one mechanism; a plain clamp leaves stale integral at the bound.
- **Mode unification** (per the "PID for everything, configurable by type" agreement): one `Controller` struct with gain fields; `curve` mode instantiates it with `Kp = 1, Ki = 0, Kd = 0` and feeds it a *zeroed error against the curve lookup as moving setpoint* — which, concretely, means curve mode bypasses the struct entirely and sends the lookup straight to the output stage (the spec's "gain-1 follower" is the design consequence: an integrator on top of a user-drawn control law is a bug, not a feature). So "configurable by type" is realized as: the *mode* selects which controller instance runs, and only `target_temp` has tunable gains + active integral.

Defaults: Kp = 2 %/°C, Ki = 0.2 %/°C·s, Kd = 0 — sane starting values for a 1 Hz loop with a 250 ms PWM step; documented in config schema comments as "start low, raise Kp if it sags, add Ki if it creeps."

### D5: Curve lookup — sorted points, linear interior, clamped ends

Config stores `Vec<(temp_c: f64, pwm: f64)>`; validation sorts by temp, rejects duplicate temps and non-monotone-able data (duplicates), and requires ≥ 1 point (a single point = constant). Lookup is a two-pointer scan (n is small, ~5–15 points) — no interpolation library. Values below the lowest point's temp clamp to that point's %; above the highest, clamp to the highest % (spec scenario "Clamped at ends"). Temps are allowed in 0–100 °C by config validation; the spec's 200 °C scenario is handled by clamping regardless.

### D6: Protection floor + min-duty + slew — strict ordering

The per-tick pipeline (this ordering is the spec's contract, fixed once here):

```
 1. Mode gate: device_memory                -> NoCommand (+ report-only faults)
 2. Sensor health (curve/target_temp):
      all comfort drivers missing/stale     -> Command{pwm:100, failsafe:true} (+ faults)
      first-ever-seed tick (cold start)     -> NoCommand   (withhold, not failsafe)
 3. Update comfort EMAs (per driver); update shared tripwire EMAs + latches
      (from evaluated tripwires in D2); apply staleness; emit soft faults
      for offline tripwires (state = last known)
 4. Fold (max default) -> folded scalar
 5. Mode controller:
      static_percent  -> fixed %  (skips comfort path 2–4: no comfort EMAs, no fold;
                                   still declares a static duty)
      curve           -> lookup(folded)
      target_temp     -> PID(tick)
 6. Protection floor:  out = max(out, max protection_target of latched tripwires)
      (applies to ALL modes that own the port — including static_percent, where
       a *declared* tripwire is the only protection it can have; a static profile
       with no declared tripwires keeps its constant duty and no faults)
 7. Min-duty floor:    out = max(out, min_duty)          // curve/target_temp only
 8. Slew limit:        out = step_toward(last_commanded_pwm, out, rate(dir) × Δt)
 9. Emit Command{pwm: clamp0100(out), failsafe:false}
```

Step 2 runs **before** any filter update: a fail-safe tick must not advance EMA/PID state with nothing in them, and a NoCommand cold-start tick must be the *only* tick that seeds and stays silent.

### D7: Failsafe, cold-start, required-vs-optional sources — one state machine per profile

```
enum ProfileInit { Cold, Running }                       // per profile
enum SourceLastSeen { Never, OkAt(Duration), StaleAt(Duration) }  // per comfort source

missing/required source rules (config: comfort source required: bool = true):
  Cold  + all sources fresh this tick         -> seed EMAs, outcome=NoCommand, init->Running
  Cold  + any required source absent, no source fresh this tick
                                              -> stays Cold, outcome=NoCommand,
                                                 fault "source never arrived"
  Cold  + required source absent but another fresh -> seed fresh ones, outcome=NoCommand,
                                                 fault, init->Running* (*from next tick it's
                                                 Running with that source missing -> failsafe)
  Running + any required source missing/past staleness -> Command{100, failsafe}
  Running + all sources fresh                 -> normal pipeline from step 3
```

This is the spec's "Required-vs-Optional" + "Cold-Start" requirements collapsed into one explicit machine (easier to test: each transition is a unit test). Optional sources (e.g., an auxiliary thermistor that may not be plugged in) missing → soft fault, no failsafe; the fold simply folds what's present.

### D8: Config value model (serde) — the contract for GUI and daemon

```rust
#[derive(Serialize, Deserialize)] pub struct FanProfileConfig {
    pub id: String,
    pub mode: Mode,                       // static_percent | curve | target_temp | device_memory
    pub static_percent: Option<f64>,      // 0..=100, required iff mode=static_percent
    pub curve_points: Option<Vec<CurvePoint>>, // required iff mode=curve
    pub setpoint_c: Option<f64>,          // required iff mode=target_temp
    pub gains: Option<PidGains>,          // Kp, Ki, Kd; default Some(...)
    pub min_duty: f64,                    // default 0.0
    pub fold: FoldRule,                   // Max (default) | Avg
    pub slew: SlewRates,                  // up 20%/s, down 7%/s defaults
    pub comfort: Vec<ComfortSourceConfig>,// key, ema_tau_s, required (default true)
    pub tripwires: Vec<TripwireRefConfig> // key, protection_target_f64
}
// per-source blocks live in a sibling `sensor_sources` table in the same config,
// so the daemon owns one entry per physical source:
pub struct TripwireConfig { pub threshold_c: f64, pub spike_rate_c_per_s: f64,
                            pub ema_tau_s: f64, pub hysteresis_c: f64 }
```

Validation (in-core, pure): mode-field coherence (curve requires **≥2 distinct-temp points**; target_temp requires setpoint + gains; static_percent requires a value and **must not declare comfort sources** — tripwire/display declarations are allowed and are the only protection a static port can have), point count, tau > 0, hysteresis > 0, 0 ≤ min_duty ≤ 100, rates > 0, fold values in bounds. Violations → `ValidationErrors` (Vec of `Path + message`), never panics.

**Alternative considered:** store tripwire tuning per *(source, profile)* pair. Rejected — spec says tripwire state is a shared fact about the system; per-profile tuning would silently diverge latches between profiles sharing a source, which is exactly the confusion the spec's "one evaluation, many consumers" rule forbids. Per-source tuning it is.

### D9: Types and numeric hygiene

- All math in `f64` (temps, PWM, rates, gains). `f32` saves nothing at this scale and invites clamping drift.
- `Duration` (not `f64` seconds or `Instant`) for injected timestamps — ordering, subtraction, and the "no system clock" property all fall out of the type.
- PWM is `f64` inside the core; the daemon rounds to its protocol's step (e.g., `round()` to a whole percent for Commander) — rounding policy is daemon's, because it depends on firmware, which the core must not know.
- `#[must_use]` on `TickResult`; no `Result` in `tick` (the spec's failures all surface as faults + failsafe outcomes; config-time problems surface at construction).

## Risks / Trade-offs

- **[PID defaults are guesses; a badly tuned `target_temp` profile can oscillate]** → Defaults bias low (sluggish, not ringing); the slew limiter caps the physical consequence of any oscillation to rate-limited wobble; the GUI (later change) will show the measured-vs-setpoint trace so the user can tune visibly. Documented in the config comments.
- **[A daemon stall (Δt jump) is clamped, so protection reacts late after a stall]** → The clamp bounds the *filter* damage; fail-safe staleness uses the *true* gap (staleness check happens in step 2, before clamp), so after a long stall a genuinely hot sensor triggers failsafe/protection immediately on the first tick after recovery, not after several.
- **[Shared `TripwireEval` means the daemon must evaluate tripwires before profiles; a daemon bug could mis-sequence this]** → The API shape enforces it: `ProfileHandle::tick` takes the evaluated map by reference and there is no other path to tripwire updates. Unit tests pin the "latch survives a missing reading tick" behavior against the map.
- **[Config value types are the first cross-cutting contract; later changes (daemon, GUI) will pressure them]** → Kept minimal-but-complete (mode coherence is validated, not assumed), serde-only, no device vocabulary in field names — the same constraint as the core. Renames before archive if exploration surfaces better words.
- **[One profile = one port is enforced by convention, not the core]** → The core can't know ports; the daemon maps profiles to channels (later change) and validation there will reject two profiles claiming one channel. Documented in both changes' proposals.

## Migration Plan

Greenfield: no deployment, no rollback. `git init` + first commit lands with the workspace and this crate; the crate builds and tests pass standalone (`cargo test -p corelink-core`) with no hardware present. Subsequent changes (daemon, GUI) depend on `corelink-core` and cannot break it without failing its test suite.

## Resolved Questions (2026-08-17)

- **Curve x-axis bounds:** free choice in 0–100 °C with a **≥2-point minimum** (a single point would degenerate into `static_percent` and is rejected by validation). The GUI (later change) will default new curves to a sensible starting shape (~20 °C floor point through ~90 °C at max duty).
- **`static_percent` + tripwire:** a static profile declares protection *only if the user wires it up*: static with no sensors = a constant, faultless duty forever; static **with declared tripwire(s)** = constant duty with the protection floor able to raise it while latched. Config validation allows tripwire/display declarations on `static_percent` but rejects comfort sources; the GUI (later change) surfaces a visible note that a latched tripwire overrides the fixed duty.
- **PID defaults tuning (Kp=2, Ki=0.2, Kd=0):** deferred — deliberately tuned in the **daemon change**, once the real achieved tick period is measured on hardware (nominally 1 Hz, but the Firmware 2.x ACK-retry budget affects it). This change ships the defaults as documented starting values only.
- **Firmware 2.x ACK race (plan.md Phase 1):** daemon-scoped — the 8×400 ms retry loop lives around speed writes in the daemon, which owns the actual `Δt` the core receives. Core is unaffected beyond receiving the injected timestamps it is given.
