## Why

Phase 1 requires smooth, noise-resistant, and still responsive fan control. Exploration (2026-08-16) showed the original plan was wrong in two ways:

1. **"One 2-second moving average for everything"** — the GPU sensors have very different dynamics (Edge/Mem are slow and stable; Hotspot can jump 40→80 °C in under a second). One shared slow average delays real protection by ~2 s on a genuine hot event, while one fast enough to avoid snaps wouldn't stop acoustic fan slamming.
2. **Sensors and fans are many-to-many** — one fan may need several temperature sources, and one source may drive several fans — yet the control math must stay trivially testable: one folded scalar temperature in, one PWM out.

## What Changes

- New pure-logic Rust crate `corelink-core` holding the entire fan control pipeline. No device names, no hwmon access, no USB — timestamps, sensor readings, and the last commanded PWM are injected per tick. The daemon (a later change) calls the core synchronously once per poll tick, per fan profile.
- **Sensor roles replace count-based smoothing.** Each contributing sensor is declared in config with a role: `comfort_driver` (feeds the user-drawn fan curve through a light EMA, τ ≈ 1–2 s; first sample seeds the EMA state — never zero/padded), `tripwire` (protection only — absolute threshold or dT/dt spike bypasses the comfort curve; it never enters the curve; dT/dt is disabled until 2 samples exist), or `display_only` (never touches the fan path). The role declaration *is* the complete description of how a sensor's value may reach the single PID input.
- **Fold layer.** Per fan profile, a config-declared fold rule reduces the comfort drivers to the single scalar temperature the core consumes. `max` is the default fold (conservative: the hottest driver wins); `avg` and other rules are declared in config. Tripwire state is evaluated per tripwire source and shared across all profiles that reference it, since it is a fact about the system, not per-profile state.
- **Per-profile pipeline with explicit control modes.** Each profile runs in one of `static_percent` (fixed duty cycle, no comfort sensors — optionally protected by declared tripwires), `curve` (piecewise-linear graph: 0–100 °C → 0–100 %), `target_temp` (PID driving the measured/folded temperature to a configured setpoint), or `device_memory` (CoreLink does not own the port; emits no commands). A single PID controller type is used in every temperature-driven mode and is **configured by mode**: in `curve` it is a gain-1 follower of the curve lookup (integrator and derivative inoperative — the curve is the user's own control law), and in `target_temp` it is a full closed loop on temperature toward the setpoint (tunable gains, active integrator with anti-windup). In `static_percent` the controller is bypassed. The only process variable is temperature (no fan/RPM feedback). Per tick: comfort drivers → EMA → fold (default `max`) → mode controller, with the protection override applied as a uniform PWM floor while a referenced tripwire is latched (`final = max(mode_output, protection_target)`). Every non-failsafe command passes through an asymmetric slew-rate limiter (≈15–25 %/s up, ≈5–10 %/s down, both configurable); a per-profile configurable minimum-duty floor applies to `curve` and `target_temp` outputs (explicit `static_percent` values and the failsafe are exempt). Per-profile state: EMA filters + PID integrator (active only in `target_temp`). The last commanded PWM is an *input* the daemon feeds back — the core never remembers its own output position, so mode switches or manual overrides cannot desync the output.
- **Failsafe: fail to 100% PWM, per-port.** In the temperature-driven modes, if on a tick no usable measurement is available — all comfort drivers missing, a daemon-reported read error, or the newest sample stale past a configured bound — the port is driven to 100% *immediately*, bypassing the mode controller and slew limiter (full speed is not ramped). A one-tick cold-start withhold (first-ever samples only) precedes this; a declared-required comfort source that never arrives triggers it from the profile's first decision tick. Scope is per-port (Commander fan control is per-channel in the protocol): only the port assigned to the failed profile is affected — all other ports on the same device are untouched. Ports in Device Memory Mode are not owned by CoreLink (the onboard device profile drives them), so a DMM profile's faults are *reported* (daemon log + GUI fault event), never acted on — the core emits no command on those ports by contract.
- **First-tick correctness.** No uninitialized filter state ever masquerades as a real reading: EMA seeds with the first actual sample; an explicit "insufficient data" state exists; dT/dt is disabled until 2 samples exist.
- **Report, don't swallow.** A lost tripwire source while comfort drivers are healthy degrades that profile's protection and emits a reportable fault event; it does not by itself command 100%.
- No hardware I/O in this change: the crate is fully unit-testable (injected clock, injected feedback).

## Capabilities

### New Capabilities

- `fan-control`: pure per-tick fan control core — role-driven sensor smoothing, multi-driver fold, protection tripwire, PID, asymmetric slew limiting, and per-port failsafe to 100% — that reduces a set of injected sensor readings to per-profile PWM output with no I/O dependencies.

### Modified Capabilities

(none — this is a new capability; `openspec/specs/` is empty)

## Impact

- `Cargo.toml` (new): workspace root with `corelink-core` member.
- `corelink-core/` (new crate): modules `ema`, `fold`, `tripwire`, `pid`, `slew`, `profile` (pipeline + per-profile state), fan-profile config value types, plus unit tests.
- No changes to existing code (greenfield repo).
- No external dependencies beyond `serde` (config value types) and `serde_json` if config-parsing tests need them.
