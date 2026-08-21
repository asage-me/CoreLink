# Tasks — psu-telemetry-core

## 1. Scaffolding

- [ ] 1.1 Add `corelink-psu` to the workspace `Cargo.toml` members; create the crate with `Cargo.toml` (`serde` derive dep, `serde_json` dev-dep) and an empty `src/lib.rs` documenting the module map with design-decision references (D1–D8)
- [ ] 1.2 `cargo build -p corelink-psu` and `cargo clippy -p corelink-psu` clean; `lib.rs` module map + crate-level doc (pure-core contract, no I/O)

## 2. Packet construction (`packet.rs`) — spec: HID Packet Construction

- [ ] 2.1 `read_packet(cmd) -> [u8; 64]` (`[0x03, cmd, 0, …]`), `init_packet()` (`[0xFE, 0x03, 0, …]` swapped order), `rail_select_packet(Rail) -> [u8; 64]` (`[0x02, 0x00, rail, …]`); `Rail` enum (12V/5V/3V3)
- [ ] 2.2 Tests: exact byte asserts for each packet (including all-tail-zero property), rail value 0/1/2

## 3. Register table & LINEAR11 decode (`linear11.rs`, `registers.rs`) — spec: LINEAR11 Decode With Per-Register Units

- [ ] 3.1 `Reg` enum / register table: command byte, name, native unit (V/A/W/°C/RPM), rail-scoped flag; `decode_linear11(u16) -> i32` — kernel-exact: signed 11-bit mantissa, signed 5-bit exponent, integer power-of-two arithmetic, single final conversion to `f64` (no `f64::powf`)
- [ ] 3.2 Check in a transcribed line-for-line Rust port of the kernel's `corsairpsu_linear11_to_long` as a test-only reference (dev scope), with golden vectors: mantissa +/−, exponent +/−, 0, 0x7FF, 0x800, 0xC00, 0xFFFF per scale
- [ ] 3.3 Tests: decoder == transcribed reference for every golden vector; native-unit spot checks (12 V input at mantissa 3/exp 2 → 12.0; 1500 W total; 2500 RPM; 45 °C); negative quantity preserved (not clamped)

## 4. Echo validation (`verdict.rs`) — spec: Response Echo Validation

- [ ] 4.1 `classify(req: &[u8; 64], reply: &[u8; 64]) -> Verdict` with `Verdict::Matched { value: u16 }` (LE u16 at reply[2..4]), `Unsupported` (echoed command byte 0), `Mismatch`; length-and-command echo rule per design D1
- [ ] 4.2 Tests: matched 0x90 reply (value extraction), zero-command-echo unsupported, mismatched length, mismatched command, reply shorter-echo cases

## 5. Plausibility ranges (`ranges.rs`) — spec: Plausibility Validation

- [ ] 5.1 `Ranges` struct with the spec defaults (in-V 90–265; rail V 4.0–13.2 / 4.5–5.6 / 3.0–4.0; temp >1 & <150 °C; total W 0–3000; rail W 0–2200; RPM 0–12000; in-A 0–40; rail A ≥ 0) — public defaults + daemon-overridable setters
- [ ] 5.2 `check(field, value, &ranges) -> Option<Plausibility>` + temperature >1 °C gate; tests: in-range passes untouched, out-of-range yields fault, temp ≤ 1 gated to None, negative current flagged

## 6. Poll plan (`plan.rs`) — spec: Poll Cycle Plan As Data

- [ ] 6.1 `PlanStep { kind: StepKind /* Init | SelectRail(Rail) | Read(Reg) */ }` and `poll_plan() -> [PlanStep; 19]` in the exact design D3 order (init head; 12 V → 5 V → 3.3 V rail blocks; select precedes its V/A/W reads)
- [ ] 6.2 Tests: first step init, last step 3.3 V watts, rail order, exactly 19 steps, two enumerations identical (determinism)

## 7. Identity (`identity.rs`) — spec: Device Identity

- [ ] 7.1 Static model table with HX1500i `(0x1b1c, 0x1c1f)` (name, rated 1500 W, rails {12, 5, 3.3}); `model_for(vid, pid)`
- [ ] 7.2 `parse_product_string(raw: &[u8]) -> ParsedIdentity`: NUL-truncate, strip `"CORSAIR "` / `" PSU"` (both legacy and 2023+ merged layouts); `DeviceId` = md5 of cleaned product string, fallback `(vid, pid)` with `ProductStringUnavailable` note
- [ ] 7.3 Tests: known-vid/pid hit, unknown → None, legacy string cleaning, 2023+ merged string, empty → fallback id, NUL-terminated garbage

## 8. Snapshot core (`snapshot.rs`) — spec: Normalized Snapshot, Raw HID Data Path

- [ ] 8.1 `PsuSnapshot` (Option per field, native units, `fan_mode: Option<FanMode>`, `thresholds`, `identity`, `events: Vec<PsuEvent>`) + `PsuEvent` enum (severity field) + `RawResponses` flat struct (one `Option<[u8; 64]>` per data register, optional init/fan-mode replies); derives `Clone, Debug, PartialEq, Serialize, Deserialize`
- [ ] 8.2 `from_raw(&RawResponses, &PsuModel, Ranges) -> PsuSnapshot`: decode → plausibility → field fill, every `None` paired with a named event; negative decode = DecodeFault; unsupported/mismatched → None + event; fan-mode read maps 0 → Auto, 1–100 → FixedDuty (excluded from gauge and gating)
- [ ] 8.3 Degraded-snapshot gauge: ≤ 4 of 15 core fields `Some` ⇒ `DegradedSnapshot { present, total }` event; tests: healthy full set (HX1500i in-A absent = healthy None), single unsupported field, out-of-range → None + Plausibility, corrupt decode → None + DecodeFault, degenerate (2/15) flagged

## 9. Init thresholds (`thresholds.rs`) — spec: Critical Thresholds Captured At Init

- [ ] 9.1 `ThresholdReadings` flat struct + `Thresholds` (Option per entry: temp crit ×2, rail V hcrit/lcrit, rail A hcrit); `thresholds_from(&ThresholdReadings) -> Thresholds` using the same LINEAR11 decode and native units as data fields; unsupported read → `None` per entry with event
- [ ] 9.2 Tests: supported threshold populated in data units (e.g. 85 °C), unsupported → None, no cross-contamination into data fields

## 10. hwmon path (`hwmon.rs`) — spec: hwmon Data Path

- [ ] 10.1 `HwmonChannels` (kernel channel numbering: in ch2–4 = 12/5/3.3, curr ch1 = input, power ch1 = total ch2–4 = rails, temp ch1 = VRM ch2 = case, fan ch1) and `from_hwmon(&HwmonChannels, &PsuModel, Ranges) -> PsuSnapshot` — kernel-inverse scaling (÷1000 / ÷1000 / ÷1,000,000 / ÷1000 / ×1), same ranges, same gauge, same event vocabulary; absent/NaN channel = None + event
- [ ] 10.2 Tests: mV→V (12010 mV → 12.01 V), µW→W (1,200,000 µW → 1.2 W), milli-°C→°C; absent `curr1` channel → None + event

## 11. Path equivalence + determinism (integration) — spec: Single snapshot type / Decode matches kernel reference

- [ ] 11.1 Equivalence tests: for a fixture set of physical values, build a `RawResponses` whose encoded replies would produce them (hand-assembled reply words via the LINEAR11 encode of the same fixed-point format) and an `HwmonChannels` with the identical physical values; assert field-by-field equality across both `from_raw` and `from_hwmon` (the decode-oracle property)
- [ ] 11.2 Determinism: identical input ⇒ identical snapshot (fields + events, including event order)
- [ ] 11.3 serde round-trip: snapshot serializes/deserializes losslessly (serde_json, dev-dep) — Phase 5/GUI contract
- [ ] 11.4 `cargo test -p corelink-psu`, `cargo clippy -p corelink-psu`, `cargo fmt --check` all green; crate README.md with a one-paragraph daemon-consumer note (how to walk the plan, classify, retry budget, call the two constructors)
