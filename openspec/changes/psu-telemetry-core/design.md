## Context

Phase 1 (`fan-control-core`) is archived and complete: a zero-I/O pure crate `corelink-core` that the (as-yet unbuilt) daemon will call per fan tick. Phase 2 of `plan.md` is **Power Monitoring & Telemetry**: poll the HX1500i PSU for voltages, currents, wattages, and temps, using the Linux `corsair-psu` hwmon interface first and raw 64-byte HID requests if the driver is absent. Exploration (2026-08) verified the device protocol against two independent, working implementations:

- **Linux mainline driver** `drivers/hwmon/corsair-psu.c` (Wilken Gottwalt) — the authoritative reference: it is the very interface plan.md says to check first, it defines the request/reply layout, the echo contract, the `unsupported` semantics (echo command byte = 0), the LINEAR11 conversion, and per-command ABI scales.
- **OpenLinkHub** `src/devices/psuhid/psuhid.go` + `src/common/common.go` — a user-space proof that the same read/write cadence works reliably against this exact device family, including per-transfer re-init.

The daemon (`corelinkd`) does not exist yet, so — mirroring Phase 1's split — this change builds **only the pure logic**: packet construction, reply classification, decoding, the poll plan, plausibility, and the normalized snapshot. All I/O (hidapi handles, the 1 Hz loop, hwmon file reads, staleness, re-init policy enforcement, systemd) belongs to the later daemon change, which will be written against the exact API fixed by this design.

Constraints:
- The user's machine runs Linux with the HX1500i; iCUE may or may not be installed (a concurrent HID client is a realistic failure mode, and the spec's `mismatch` verdict exists for it).
- The HX1500i is `0x1b1c:0x1c1f` (one PID covers Legacy, 2023 and 2025 series). It is a single-PSU setup, but the design must not preclude other Corsair i-series PSUs (same protocol family).
- The crate must stay testable without hardware: golden vectors for the decode, exact byte asserts for packets, and in-memory reply fixtures for the snapshot builder.

## Goals / Non-Goals

**Goals:**
- A pure, synchronous, zero-dependency-logic crate `corelink-psu` whose public API is exactly what the daemon needs to run one poll cycle and map hwmon channel values — nothing that requires a running PSU to implement or test.
- The LINEAR11 decode matching the Linux driver byte-for-byte (signed mantissa), with the register table that fixes each register's native unit (V, A, W, °C, RPM).
- One `PsuSnapshot` type with per-field `Option` + typed events, produced identically from raw replies or hwmon channel values, so the two transport paths can never diverge numerically.
- The per-cycle transfer sequence as inspectable data (19 steps), including the per-cycle re-init.
- Plausibility validation with per-field ranges (defaulted, daemon-overridable) so a corrupt reply is a fault event, never a plotted number.
- Device identity (model table, product-string parsing for both legacy and 2023+ layouts, stable id) and the init-only critical-threshold capture, self-describing for the later GUI/telemetry phases.

**Non-Goals:**
- No I/O of any kind: no hidapi, no filesystem, no clock, no threads, no logging. No USB polling loop, no staleness tracking (daemon).
- **No writes to the PSU.** The kernel driver's own comment warns misconfiguration "resets or shuts down" the unit; the design constructs only read-style and init/rail-select requests, and the snapshot path never emits fan-mode or fan-PWM write packets (0xF0/0x3B as *writes* are absent by construction — fan mode exists only as an optional 0xF0 *read*).
- No smoothing, no time-series, no database (Phase 5). No multi-PSU fan coordination (there is one PSU; the model table simply does not close the door on more).
- No hwmon *probing* logic (scanning `/sys/class/hwmon/*/name`). The core maps channel values it is handed; the daemon decides hwmon-vs-raw.
- No config parsing from disk — serde types only, per Phase 1 convention.

## Decisions

### D1: Crate shape and public API — data-in / snapshot-out, one cycle's worth

The crate is a single module with a small, flat public surface (everything `#[must_use]`, no `Result` in the hot path — errors are *events on the snapshot*, matching Phase 1's "faults ride the outcome" convention):

```rust
// packet.rs
pub fn read_packet(cmd: u8) -> [u8; 64]            // [0x03, cmd, 0, 0…]
pub fn init_packet() -> [u8; 64]                   // [0xFE, 0x03, 0, 0…] (swapped order)
pub fn rail_select_packet(rail: Rail) -> [u8; 64]  // [0x02, 0x00, rail, 0…]

// plan.rs — the 19-step per-cycle sequence, as data
pub struct PlanStep { pub kind: StepKind /* Init | SelectRail(Rail) | Read(Reg) */, }
pub fn poll_plan() -> [PlanStep; 19]

// verdict.rs — classification of one request/reply pair (pure)
pub enum Verdict {
    Matched { value: u16 },   // length+command echo ok; value = LE u16 at reply[2..4]
    Unsupported,              // echo length ok, echoed command byte == 0
    Mismatch,                 // otherwise — retry candidate, never data
}
pub fn classify(req: &[u8; 64], reply: &[u8; 64]) -> Verdict

// snapshot.rs
pub struct PsuSnapshot { … Option<f64> fields, Option<FanMode>, Thresholds, DeviceId, Vec<PsuEvent> }
pub impl PsuSnapshot {
    pub fn from_raw(responses: &RawResponses, model: &PsuModel) -> PsuSnapshot
    pub fn from_hwmon(values: &HwmonChannels, model: &PsuModel) -> PsuSnapshot
}

// thresholds.rs — init-only capture, daemon-driven
pub fn thresholds_from(crit_readings: &ThresholdReadings) -> Thresholds

// identity.rs
pub fn model_for(vid: u16, pid: u16) -> Option<&'static PsuModel>
pub fn parse_product_string(raw: &[u8]) -> ParsedIdentity
pub fn device_id(model: &PsuModel, raw_product: Option<impl AsRef<[u8]>>) -> DeviceId
```

`RawResponses` is a plain struct with one `Option<[u8; 64]>` per data register in the poll plan (plus optional `init_reply` for identity, optional `fan_mode_reply`). The daemon fills it by walking `poll_plan()`, calling the hidapi device, and running each pair through `classify` (retrying `Mismatch` on the *device* per its budget, converting non-`Matched` verdicts to `None` fields). Alternatives considered: (a) handing the crate `&mut SomeHidIo` behind a trait — rejected, it drags I/O concepts into a crate Phase 1 deliberately kept free of, and the daemon's retry policy is a daemon concern; (b) a streaming "feed replies one at a time" builder — rejected, it invites partial-snapshot state and the whole point is one atomic per-cycle conversion.

Rationale: the daemon's cycle becomes *mechanical*: plan → transfer → classify → fill struct → `from_raw`. Every correctness-relevant behavior (decoding, ranges, identity, events) is inside `corelink-psu` and unit-testable; every policy-relevant behavior (retry budget, timing, staleness) stays in the daemon, where only it has facts.

### D2: LINEAR11 — signed mantissa, kernel-exact, native units fixed per register

The kernel's decoder:

```c
const int exp  = ((s16)val) >> 11;                                    // 5-bit two's-complement
const int mant = ((s16)((val & 0x7ff) << 5)) >> 5;                    // 11-bit two's-complement
s64 result = mant * scale;                                             // scale = ABI units, see below
if (exp >= 0) result *= (1UL << exp); else result >>= -exp;
return clamp(result, LONG_MIN, LONG_MAX);
```

`corelink-psu` implements the mantissa/exp/quantity part **exactly** (signed 11-bit mantissa, signed 5-bit exponent, exact powers of two — no `f64::powf`, integer math only, then one final conversion to `f64`), *not* OpenLinkHub's variant, which masks the mantissa as unsigned (`val & 0x7FF` with no sign extension — a latent bug for negative mantissas that never fires on positive PSU readings but is not a reference to copy). For every register on the HX1500i the quantity is non-negative in practice, so the two implementations agree numerically; we take the correct one.

The kernel's per-command `scale` (×1000 for V/A/°C, ×1,000,000 for W, ×1 for RPM/fan) converts the native quantity into the **hwmon ABI units** (mV, mA, milli-°C, µW) it publishes — it is *not* part of the device protocol. Therefore:

- **Raw path:** `quantity` *is* already in the register's native unit (V, A, **°C**, W, RPM). No scaling.
- **hwmon path:** read mV/mA/milli-°C/µW and divide exactly by the kernel's scales (÷1000, ÷1000, ÷1000, ÷1,000,000).

This is provably consistent: both paths invert/apply the same constants the kernel uses, so "hwmon-first, raw-fallback" can never change a number by a factor of 1000. The temperature-native-unit question (is the raw quantity °C, or K?) resolves to **°C** by the kernel's own behavior: `hwmon` publishes milli-degrees *Celsius* (ABI), the driver applies ×1000 with **no 273,150 offset** — so the raw quantity must already be °C. This remains the one item the live two-path diff (daemon phase) re-confirms on real hardware before any value is considered final.

Golden vectors in tests are generated by transcribing the kernel's C function line-for-line for a spread of raw words (exponent +/−, mantissa +/−, 0, 0x7FF, 0x800, 0xFFFF) per per-register scale; the Rust decoder and the transcribed reference are both checked in, so any drift is a test failure, not a live surprise.

### D3: The 19-step poll plan, with the rail-sequencing subtleties made explicit

```
 1. Init            [0xFE, 0x03, 0]   — no data consumed (identity may come from it, see D6)
 2. Read FAN_RPM    0x90
 3. Read TEMP_VRM   0x8D
 4. Read TEMP_CASE  0x8E
 5. Read IN_VOLTAGE 0x88
 6. Read IN_CURRENT 0x89     — HX1500i: expected Unsupported → field None + event (not a fault)
 7. Read TOTAL_W    0xEE
 8. SelectRail(12V) [0x02, 0x00, 0x00]
 9. Read RAIL_V     0x8B     10. Read RAIL_A 0x8C     11. Read RAIL_W 0x96
12. SelectRail(5V)  … 13-15. V/A/W
16. SelectRail(3.3V) … 17-19. V/A/W
```

Details the daemon needs, fixed here:
- **Init cadence:** once per cycle, at the head (OpenLinkHub's proven behavior on this family; self-heals after standby/sleep and any desync; 19 transfers at 1 Hz is trivial). The daemon MAY relax this later (kernel-style: probe + resume only) behind a config knob — the plan shape makes that a data change, not a logic change.
- **Rail-select is the only *write-style* request** (length byte 0x02). Its reply carries no value; the daemon must treat a `Mismatch` echo on the select leniently (proceed — the subsequent rail reads are the real validation) while a hard transport error is a cycle fault. Read requests (length 0x03) follow the strict `classify` rule.
- **Ordering matters:** per-rail reads are meaningless until that rail is selected (single shared sense path on the firmware). The plan is the single source of truth for that ordering; the daemon walks it and cannot interleave or reorder.

### D4: Snapshot model — `Option` per field, events per absence, SI/native units

```rust
pub struct PsuSnapshot {
    pub input_voltage_v:  Option<f64>,
    pub input_current_a:  Option<f64>,        // None on HX1500i (no 0x89) — healthy
    pub total_power_w:    Option<f64>,
    pub rail_12v: Rail { voltage_v, current_a, power_w },   // each Option<f64>
    pub rail_5v:  Rail,  pub rail_3v3: Rail,
    pub temp_vrm_c: Option<f64>,
    pub temp_case_c: Option<f64>,
    pub fan_rpm: Option<f64>,
    pub fan_mode: Option<FanMode>,            // Auto | FixedDuty(f64) — observation only
    pub thresholds: Thresholds,               // init-only, Option per entry
    pub identity: DeviceIdentity,             // model, parsed product, stable DeviceId
    pub events: Vec<PsuEvent>,
}
pub enum PsuEvent { UnsupportedField{field}, ReplyMismatch{field}, DecodeFault{field, raw, why},
                    Plausibility{field, value, range}, DegradedSnapshot{present, total},
                    ProductStringUnavailable, … }
```

- Fields are `f64` in native units (V, A, W, °C, RPM), not `f32` — Phase 1 convention, and the GUI/telemetry layer will want it.
- **Every `None` has a named event**; conversely every event names a field, so the daemon/GUI can never display a `None` without an explanation. Silence is not a state.
- **Degraded-snapshot gauge:** if ≤ 4 of the 15 core fields are `Some`, a `DegradedSnapshot` event rides along (the device is effectively offline or being fought over) — the snapshot is still returned, because partial data + events is more useful to the daemon than nothing.
- `fan_mode` (optional, set from the optional 0xF0 read the daemon performs outside the cycle) is **excluded** from the degraded gauge and from plausibility gating — it's state, not a sensor.
- `PsuSnapshot` derives `Clone, Debug, PartialEq` and `serde::{Serialize, Deserialize}` (dev-only `serde_json` in tests) so Phase 5's logging and the GUI build against it without a core change — same serde-only dependency stance as `corelink-core`.

Alternatives considered: (a) a `f64` + `valid: bool` pair per field — rejected, `Option` makes "no value" unrepresentable-as-wrong at the type level, as Phase 1's `Outcome` enum does for commands; (b) separate `Raw` and `Hwmon` snapshot structs — rejected, the whole value of two paths is one guaranteed-identical type.

### D5: Plausibility ranges — defaulted in-core, overridable by the daemon

A `Ranges` struct (public constructor with the spec's defaults; setters for daemon/model overrides) is applied to every `Some` candidate before it enters the snapshot:

| Field | Range | Note |
|---|---|---|
| input_voltage | 90–265 V | |
| input_current | 0–40 A | |
| +12 V rail vol | 4.0 – 13.2 V | |
| +5 V rail vol | 4.5 – 5.6 V | |
| +3.3 V rail vol | 3.0 – 4.0 V | |
| rail currents / powers | ≥ 0; power ≤ 2200 W | negative current ⇒ DecodeFault (signed-mantissa guard, D2) |
| total_power | 0 – 3000 W | headroom over 1500 W rated |
| temperatures | > 1 °C and < 150 °C | the `> 1` sanity gate from both reference drivers, made explicit |
| fan_rpm | 0 – 12 000 | |

An out-of-range candidate becomes `None` + `Plausibility{field, value, range}`. The ranges are deliberately conservative (a genuinely healthy HX1500i sits far inside them); they catch *decode/corruption* failures, not load transients.

### D6: Identity — both product-string layouts, stable id

- Model table (static, `&'static PsuModel`): `(0x1b1c, 0x1c1f)` → HX1500i, rated 1500 W, rails {12, 5, 3.3}. Unknown (vid,pid) ⇒ daemon treats the PSU as unsupported (event) — the core never invents a model.
- `parse_product_string`: NUL-truncate, strip `CORSAIR ` prefix and ` PSU` suffix; on 2023+ units (vendor==product merged string) the parse yields the same cleaned name — verified against OpenLinkHub's handling, which exists precisely because the 2023 series changed this.
- **Stable id:** md5 of the cleaned product string (OpenLinkHub's scheme — `iCUE` interop and future correlation with other tools fall out for free); fallback `vid:pid` when the string is unreadable (+ `ProductStringUnavailable` event). The product string is taken from the optional init reply on the raw path; on the hwmon path the daemon supplies it from the device it already knows (or the fallback applies).
- Unknown-model handling is a *daemon* decision; the core just answers `None` from `model_for`.

### D7: Thresholds — init-only, self-describing, same units as data

`Thresholds` holds `Option`s for: temperature crit (0x4F, ×2 sensors), per-rail voltage hcrit (0x40) / lcrit (0x44), per-rail current hcrit (0x46) — the exact set the kernel probes at `probe()` in `corsairpsu_get_criticals`. `thresholds_from()` maps a daemon-collected `ThresholdReadings` (same `RawResponses`-style flat struct, replies already classified) into the threshold area with the **same** LINEAR11 decode and native units as the data fields (V, A, °C), so the GUI can render "this PSU says its +12 V rail dies above X" without a second unit system. `Unsupported` reads produce `None` per entry — mirroring the kernel, which *probes* support by attempting the read. These thresholds are also the seed data for future PSU-temperature tripwires as fan-profile sensor sources (plan item 5) — pure data now, behavior later.

### D8: hwmon path — flat channel input, kernel-inverse scaling

```rust
pub struct HwmonChannels {
    pub in_volts_mv: [Option<f64>; 4],     // ch1 = mains DC, ch2-4 = rails 12/5/3.3
    pub curr_milli_a: [Option<f64>; 4],    // ch1 = input (absent on HX1500i)
    pub power_uw: [Option<f64>; 4],        // ch1 = total, ch2-4 = rails
    pub temp_milli_c: [Option<f64>; 2],    // ch1 = VRM, ch2 = case
    pub fan_rpm: Option<f64>,
}
```

`from_hwmon` divides each by the kernel's scale (1000/1000/1,000,000/1000/1), applies the same `Ranges`, the same degraded gauge, and emits the same event vocabulary — an absent or `NaN` channel is `UnsupportedField`-equivalent and `None`, never 0. The channel numbering matches the kernel driver's `hwmon_info` layout exactly (power0=total first because the kernel puts total power at channel 0 and rails at 1..3; mirrored 1-based here). This is also the *oracle* path: on the user's box, running hwmon with the driver and raw with the driver blacklisted, the two snapshots must match; the daemon-phase task for that is already implied by D2's open question.

## Risks / Trade-offs

- **Temperature native unit (°C vs K) is inferred, not measured** (kernel ×1000 with no offset ⇒ °C, ABI confirms milli-°C). → The live two-path diff in the daemon phase is the confirmation gate; if it shows 273.15° off, the fix is one constant in one place.
- **OpenLinkHub's Go decode differs from the kernel's** (unsigned mantissa, missing scale multiply). → Deliberately not copied; golden vectors are kernel-derived (D2). If a *real* HX1500i reply ever disagrees with both, the kernel (maintained, upstream) wins.
- **HX1500i lacks input current (0x89)** — every healthy snapshot carries one `None` + event. → This is the spec's `unsupported` path working as designed; the daemon logs it at info-once, not as an alarm (event severity field).
- **Concurrent HID client (iCUE) interleave** — replies can carry another client's request echo. → `Mismatch` verdict + daemon retry budget + eventual fault event (spec); the daemon owns the budget, the core owns the classification. Documented as "close other PSU software while CoreLink owns the device", per Phase 1's stance on single-writer-per-device.
- **Per-cycle re-init is 1 extra transfer per second** the kernel wouldn't do. → Proven behavior on this exact family (OpenLinkHub), self-healing after sleep; cost is negligible and relaxable behind a config knob without a core change (D3).
- **19 transfers per 1 s cycle** at ~250 ms each = ~5 s of device time budget vs 1 s wall clock — the transfers are fast (sub-millisecond on USB), so fine in practice; still, the daemon MUST budget a full-cycle time and, if it overruns, drop (and fault) the cycle rather than queue unboundedly. → Noted for the daemon design; the plan being data makes the cycle trivially measurable.
- **Plausibility ranges are guesses at the edges** (265 V in, 3000 W total) chosen for headroom, not measured. → Overridable per model (D5); a live calibration pass can tighten them after the first on-hardware weeks without breaking the contract (fields are Option either way).

## Migration Plan

None — greenfield addition: new crate `corelink-psu`, new workspace member, no existing code touched. Rollback = remove the member from the workspace.

## Open Questions

- **Live confirmation of the temperature native unit and of actual per-register exponents** — deferred to the daemon phase's two-path diff on real hardware (the only honest way to validate against the actual firmware, Legacy vs 2023 vs 2025).
- **Fan-mode read cadence in the daemon** (0xF0 once per cycle vs. on-change) — daemon policy; the core already models it as an optional field (D4).
- Whether to add the remaining i-series PIDs (HX1200i 2025 = 0x1c27, etc.) to the model table in *this* change — leaning no: one user, one PSU; the table's API (D6) makes it additive later.
