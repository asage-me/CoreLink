# fan-control specification

## Purpose

CoreLink's pure-logic fan control core: one deterministic per-tick pipeline
per fan profile, driven entirely by injected timestamps, sensor readings, and
the daemon's last-commanded PWM — zero I/O, zero async, no device topology.
This capability specifies the behavior contract (smoothing, folding, modes,
tripwire protection, slew limiting, per-port fail-safe, and fault reporting)
implemented by the corelink-core crate.

## Requirements


### Requirement: Pure Core Contract
The control core SHALL be a pure, synchronous module with no I/O dependencies: no file access, no USB, no device or hwmon concepts, and no internal clock. Per tick, per fan profile, the core SHALL consume an injected timestamp, one scalar reading per declared contributing sensor, and the last commanded PWM (fed back by the daemon), and SHALL emit exactly one outcome: a PWM command, the fail-safe command (100%), or an explicit "no command". Device topology (many-to-many mapping of sensor sources to profiles) SHALL exist only in configuration; the core SHALL consume pre-resolved per-profile input lists and SHALL NOT distinguish which physical device a reading originated from.

#### Scenario: Deterministic on identical input
- **WHEN** the core is invoked twice from the same prior state with identical injected timestamps, readings, and last-commanded PWM,
- **THEN** both invocations emit identical outcomes and identical next states.

#### Scenario: No time or I/O access
- **WHEN** the core is compiled and linked,
- **THEN** it performs no file, network, or USB access and reads no system time; time advances only through injected timestamps.

#### Scenario: Single outcome per profile per tick
- **WHEN** the core is invoked for a profile,
- **THEN** it emits exactly one of: a PWM command, a fail-safe 100% command, or "no command", with any fault events attached to that single outcome.

### Requirement: Sensor Roles
Each contributing sensor in a fan profile SHALL be declared with exactly one role: `comfort_driver`, `tripwire`, or `display_only`. The role SHALL be the complete declaration of how that sensor's value may influence the profile's output. A `display_only` sensor's value SHALL have no effect on any emitted command. Every temperature-driven profile SHALL declare at least one `comfort_driver`; a profile missing one is a configuration-validation error, not a runtime condition.

#### Scenario: display_only is inert
- **WHEN** a profile has one comfort driver at 50 °C and one display_only sensor at 200 °C,
- **THEN** the emitted command equals the one produced with the display_only sensor absent.

#### Scenario: Missing comfort_driver rejected
- **WHEN** configuration for a temperature-driven profile declares no comfort_driver,
- **THEN** validation fails with an error that names the profile.

### Requirement: Fan Profile Modes
Each fan profile SHALL run in exactly one mode: `static_percent`, `curve`, `target_temp`, or `device_memory`. `static_percent` emits a fixed duty cycle with no comfort-driver dependency: it MAY declare `tripwire` and `display_only` sources, and the protection floor applies while a declared tripwire is latched; with no tripwires declared its duty is constant under all sensor conditions and it cannot generate any fault. A `static_percent` profile SHALL NOT declare `comfort_driver` sources. `curve` and `target_temp` are temperature-driven and consume comfort-driver readings. `device_memory` never emits a PWM command.

#### Scenario: static_percent ignores sensors
- **WHEN** a `static_percent` profile at 45% has all its comfort drivers missing,
- **THEN** it still emits 45% (no fail-safe, no command-withhold) — the mode requires no sensor input.

#### Scenario: Mode is exclusive
- **WHEN** a profile's configuration names `curve`,
- **THEN** its `target_temp` setpoint and PID gain fields are inert (not read by the pipeline).

#### Scenario: device_memory never commands
- **WHEN** a `device_memory` profile is ticked — under any sensor condition (healthy, missing, stale),
- **THEN** the core emits a "no command" outcome (never a PWM value, never the fail-safe 100%), and any faults are emitted as report-only events labeled as affecting a device-managed port.

### Requirement: Comfort-Driven Smoothing
In `curve` and `target_temp` modes, each comfort driver SHALL pass through an exponential moving average with a configurable time constant τ (suggested default 1–2 s) evaluated on the injected timestamp delta. The first sample received for a driver SHALL seed the EMA state (never zero-padded). The comfort drivers' smoothed values SHALL be reduced to a single scalar by the profile's declared fold rule.

#### Scenario: First sample seeds the filter
- **WHEN** a comfort driver's first reading is 70 °C,
- **THEN** the smoothed value used on that tick is 70 °C, not 0 °C or a blend with 0 °C.

#### Scenario: EMA follows injected time
- **WHEN** the smoothed value is 50 °C and a new reading of 70 °C arrives with injected Δt = τ,
- **THEN** the smoothed value becomes 60.7 °C ± 0.1 (α = 1 − e^(−Δt/τ)).

### Requirement: Fold Rule
The fold rule SHALL reduce the comfort drivers' smoothed values to the single scalar consumed by the mode controller. The default fold rule SHALL be `max`; declared rules MUST produce a value within the observed range of their inputs, and `avg` SHALL be an equal-weight arithmetic mean. Tripwire sensors SHALL be excluded from the fold regardless of count.

#### Scenario: Default fold is max
- **WHEN** a profile has two comfort drivers reading 55 °C and 78 °C with fold `max` (the default),
- **THEN** the folded scalar is 78 °C.

#### Scenario: Explicit avg fold
- **WHEN** the fold rule is `avg` and the smoothed drivers read 55 °C and 70 °C,
- **THEN** the folded scalar is 62.5 °C.

### Requirement: Curve Mode
In `curve` mode the PWM target SHALL be computed from a piecewise-linear lookup of the folded scalar over a user-defined set of at least two (temperature °C, PWM %) points spanning the 0–100 °C / 0–100 % space (a single constant point degenerates to `static_percent`, so one point is rejected by validation). Between points the lookup SHALL be linear; below the lowest point's temperature the target SHALL equal the lowest point's %, and above the highest it SHALL equal the highest's. The mode controller in this mode SHALL be a gain-1 follower: no integral or derivative term is active, and the curve value passes to the output stage directly.

#### Scenario: Interior interpolation
- **WHEN** curve points include (40 °C, 20%) and (70 °C, 80%) and the folded value is 55 °C,
- **THEN** the mode target is 50%.

#### Scenario: Clamped at ends
- **WHEN** the folded value is 200 °C (above all points) and the highest point is (90 °C, 80%),
- **THEN** the mode target is 80%.

#### Scenario: Curve has no integrator state
- **WHEN** a `curve` profile's target jumps between ticks,
- **THEN** there is no accumulated integral term in its output stage; movement between targets is governed only by the slew limiter.

### Requirement: Target-TEMP Mode (PID)
In `target_temp` mode a per-profile PID controller SHALL drive the folded scalar toward a configured setpoint temperature: error = folded °C − setpoint °C (positive when hot), output = duty cycle %. Gains Kp, Ki, and Kd SHALL be configurable per profile, with the integrator active and accumulated as Ki·∫e·dt. The integrator SHALL be held (anti-windup) whenever the controller output is at its 0–100% clamp bound, and SHALL be wound down when the sign of the error reverses (back-calculation or clamping, implementation detail). The controller output SHALL be the mode command before the protection floor, min-duty floor, and slew limiter.

#### Scenario: Ramps up when hot
- **WHEN** setpoint is 55 °C, the folded value is 75 °C, and Ki is zero,
- **THEN** the output tracks the proportional response (Kp × 20 °C) clamped to 0–100%.

#### Scenario: Anti-windup under saturation
- **WHEN** the controller saturates at 100% for 20 s (error persistently positive),
- **THEN** the accumulated integral is held at its bound such that, once the error reverses, the output does not overshoot with a recovery delay attributable to accumulated integral debt.

#### Scenario: Reaches and holds setpoint
- **WHEN** the system settles such that the folded value equals the setpoint,
- **THEN** the error is ~0 and the output is steady (no oscillation induced by the controller's own terms).

#### Scenario: Independent integrator per profile
- **WHEN** two `target_temp` profiles receive identical folded-temperature histories,
- **THEN** each integrates independently; one profile's integral state does not influence the other's output.

### Requirement: Tripwire Detection
Each declared `tripwire` sensor SHALL be evaluated independently on every tick it receives a reading. The sensor SHALL have its own light low-pass EMA (configurable τ, suggested default shorter than the comfort-driver's). Two independent trip conditions SHALL be checked: (a) absolute — smoothed value ≥ configured threshold; (b) rate-of-change — (current smoothed − previous smoothed) / Δt ≥ configured spike rate. The dT/dt condition SHALL be disabled until the tripwire has received at least two samples.

#### Scenario: Absolute threshold trips
- **WHEN** the smoothed tripwire value reaches or exceeds its configured threshold,
- **THEN** the tripwire's protection state is latched for that tick.

#### Scenario: dT/dt spike trips
- **WHEN** the tripwire rises from a smoothed 40 °C to a smoothed 80 °C between consecutive ticks with spike rate 20 °C/s,
- **THEN** the protection state is latched for that tick.

#### Scenario: dT/dt disabled before two samples
- **WHEN** the tripwire receives one sample of 80 °C and threshold is 90 °C,
- **THEN** the dT/dt check is not applied (no previous sample), so no false trip occurs.

### Requirement: Tripwire Latching and Cross-Profile Sharing
A latched tripwire's protection state SHALL persist until its smoothed value falls at or below `threshold − hysteresis`, where hysteresis is a configured positive value; a drop above that level SHALL NOT clear the latch. Tripwire state SHALL be evaluated once per contributing sensor source and shared across every profile that references that source: one evaluation, many consumers; a single source's state transition affects all listing profiles on the same tick.

#### Scenario: Latch survives a dip within hysteresis
- **WHEN** a tripwire latched at 85 °C (threshold 80, hysteresis 5) dips to 82 °C,
- **THEN** the protection state remains latched.

#### Scenario: Clear below hysteresis
- **WHEN** the smoothed value of a latched tripwire falls to exactly `threshold − hysteresis` or below,
- **THEN** the protection state clears on that tick.

#### Scenario: One evaluation, many consumers
- **WHEN** a tripwire source referenced by three profiles clears its latch,
- **THEN** all three profiles' protection floor reverts to mode-derived values on the same tick, with no per-profile divergence.

### Requirement: Protection Floor
While any tripwire referenced by a profile is latched, the profile's output SHALL be the maximum of (mode-derived output) and (highest protection target configured among the currently latched tripwires). The protection target is a profile- or source-configured PWM% value. While latched, the protection floor SHALL override the curve value, static value, and PID output in all temperature-driven and static modes alike; it SHALL NOT override the fail-safe 100%.

#### Scenario: Tripwire raises a calm curve
- **WHEN** comfort gives a mode target of 30% and a latched tripwire's protection target is 90%,
- **THEN** the effective output before the slew limiter is 90%.

#### Scenario: Floor persists while latched
- **WHEN** the comfort/PID output would drop to 20% while the tripwire remains latched at protection target 90%,
- **THEN** the effective output stays 90% for every tick until the latch clears.

#### Scenario: Protection does not suppress failsafe
- **WHEN** a profile's tripwire is latched (protection target 90%) but on that tick its comfort drivers have all gone stale,
- **THEN** the output is the fail-safe 100% (not the 90% floor).

### Requirement: Slew-Rate Limiting
All non-fail-safe commands SHALL pass through an asymmetric slew-rate limiter using the injected tick delta: the emitted value SHALL move at most `rate × Δt` from the last commanded PWM toward the effective target, where the upward rate (suggested default 15–25 %/s) and downward rate (suggested default 5–10 %/s) are independently configurable per profile. The limiter's current position SHALL be initialized from, and each tick seeded by, the injected last-commanded PWM — not from zero and not from an internal memory of the last emit.

#### Scenario: Upward rate-limited
- **WHEN** last commanded PWM is 20%, effective target is 80%, upward rate is 20 %/s, Δt = 1 s,
- **THEN** the emitted command is 40%.

#### Scenario: Downward slower than upward
- **WHEN** last commanded PWM is 80%, effective target is 10%, upward 20 %/s, downward 5 %/s, Δt = 1 s,
- **THEN** the emitted command is 75%.

#### Scenario: Small correction unimpeded
- **WHEN** the effective target lies within one tick's reachable range,
- **THEN** the emitted command reaches the target on that tick (the limiter does not add artificial delay to small corrections).

### Requirement: Minimum-Duty Floor
In `curve` and `target_temp` modes, the final output SHALL not fall below a configurable per-profile minimum-duty value (config default 0 — off is allowed unless the user says otherwise). The minimum-duty SHALL NOT apply to an explicit `static_percent` value (the user's explicit choice wins) nor to the fail-safe command (which bypasses all rate/limit stages).

#### Scenario: Curve output floored
- **WHEN** a `curve` profile's lookup gives 3% and its minimum-duty is 10%,
- **THEN** the emitted (pre-slew) value is 10%.

#### Scenario: Static percent not floored
- **WHEN** a `static_percent` profile is configured to 0% (explicit) and its minimum-duty is 10%,
- **THEN** the emitted value is 0% — the explicit user value wins.

### Requirement: Fail-Safe to 100% (Per-Port)
In `curve` and `target_temp` modes, if on a tick no usable measurement is available — all comfort drivers missing, a daemon-reported read error for a contributing source, the newest sample older than the configured staleness bound, or filter state uninitialized at the point a real decision is required — the core SHALL emit 100% for that profile on that tick, bypassing the mode controller, the protection floor, the minimum-duty, and the slew limiter. Because each profile owns exactly one port, only that profile's port is affected; other profiles' outputs SHALL be unchanged by this profile's failure. The fail-safe condition SHALL be attached to the tick's outcome as a fault event and SHALL NOT be silent.

#### Scenario: All drivers missing
- **WHEN** a `curve` profile's tick receives no readings for any of its comfort drivers,
- **THEN** it emits 100% with a fault event naming the missing sources.

#### Scenario: Stale sample triggers fail-safe
- **WHEN** the newest comfort-driver sample is at least the configured staleness bound old relative to the injected timestamp,
- **THEN** that profile emits 100% with a staleness fault event.

#### Scenario: Per-port isolation
- **WHEN** profile A fails its fail-safe and profile B is healthy on the same tick,
- **THEN** A emits 100% and B emits its normal slew-limited command; B is unaffected by A's failure.

#### Scenario: Fail-safe bypasses the slew limiter
- **WHEN** a profile at 20% commanded PWM meets the fail-safe condition with upward rate 20 %/s and Δt = 1 s,
- **THEN** the emitted command is 100% on that tick (not 40%).

### Requirement: Cold-Start Withholds Command (One Tick)
A profile whose filter state was just seeded on that tick (its comfort drivers have each received their first-ever sample) SHALL emit a "no command" outcome — not a PWM value and not the fail-safe. This holds for at most one tick per profile lifecycle. From the second tick onward (filter states are now initialized) the normal mode logic and fail-safe apply. The daemon SHALL treat "no command" as "do not touch this port this tick" and SHALL NOT count that tick toward staleness of that profile's sources. Fail-safe 100% applies from the first tick that a previously-healthy source is missing, not from the very first tick of an otherwise-healthy cold start.

#### Scenario: Second tick emits a command
- **WHEN** a cold-started profile with one healthy comfort driver receives tick 2,
- **THEN** it emits a real PWM command (mode-derived or fail-safe per the other requirements).

#### Scenario: Cold start is not immediately fail-safe
- **WHEN** a cold-started `curve` profile receives its first tick with one healthy comfort driver and one source that has never delivered (declared but absent from tick 1),
- **THEN** the first tick emits "no command" (filter-seeding) — not immediately 100%, because the missing source has never been healthy; the staleness/fail-safe logic for that source activates from the moment it was last observed healthy (or immediately on tick 1 if the configuration marks it as required for a healthy tick — see "Required-Source" clause below).

### Requirement: Required-vs-Optional Sensor Sources
A comfort driver MAY be marked in config as required (default: all comfort drivers are required in `curve` and `target_temp` modes). A required source that has been observed healthy at least once and then stopped arriving MUST be treated as missing from the moment its staleness bound is exceeded, triggering the fail-safe. A source that has never delivered a reading is treated under the cold-start rule; it SHALL raise a "source-never-arrived" fault event on every tick while absent from a required declaration, and if it is the only comfort driver of that profile, the profile's first decision tick MUST be treated as missing → 100%.

#### Scenario: Required source lost after health
- **WHEN** a required comfort driver was healthy, is then silent past the staleness bound,
- **THEN** the profile emits 100% (fail-safe) from the first tick past the bound.

#### Scenario: Never-arrived sole source
- **WHEN** a profile's only declared comfort driver (required) has never delivered a reading and the cold-start withhold tick has passed,
- **THEN** the profile emits 100% with a "source missing" fault event.

#### Scenario: Never-arrived alongside healthy source
- **WHEN** a profile has two required comfort drivers, one healthy from tick 1 and one never-arrived,
- **THEN** after the cold-start withhold the profile emits 100% (the required source is missing), with a fault event naming the never-arrived source.

### Requirement: Soft Handling of a Lost Tripwire Source
If a tripwire source's readings stop arriving while the profile's comfort drivers remain fresh, the core SHALL NOT invoke the fail-safe. On that tick (and each subsequent tick while the tripwire is still missing), the core SHALL emit a soft fault event naming the lost tripwire source and the affected profiles, SHALL treat that tripwire's protection state as its last known latched state (it does not silently clear on absence), and SHALL continue to emit normal mode-derived output. The fail-safe 100% is applied only by missing comfort drivers (per the Fail-Safe requirement), never by a missing tripwire alone.

#### Scenario: Lost tripwire, healthy comfort
- **WHEN** a tripwire source stops arriving but the profile's comfort drivers are fresh,
- **THEN** the tick emits a soft fault event and a normal mode-derived (slew-limited) command — not 100%.

#### Scenario: Missing tripwire stays latched
- **WHEN** a latched tripwire's source stops reading and comfort is healthy,
- **THEN** it is still counted as latched for that tick: the protection floor still applies, and a soft fault event rides along.

#### Scenario: Missing tripwire that was clear
- **WHEN** an unlatched tripwire's source stops reading and comfort is healthy,
- **THEN** it is counted as unlatched for that tick (last known state was clear), and a soft fault event rides along. No false protection floor is applied from an absence.
