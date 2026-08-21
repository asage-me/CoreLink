# psu-telemetry specification

## Purpose

CoreLink's pure-logic power-supply telemetry core for the Corsair HX1500i: a set of
deterministic, I/O-free functions that turn either raw 64-byte HID replies or
`corsair-psu` hwmon channel values into one normalized snapshot of voltages,
currents, wattages, temperatures, and fan RPM — with per-field optionality,
plausibility validation, and typed fault events, so a missing or corrupt reading
is always loud, never a silently-zeroed number. This capability specifies the
behavior contract implemented by the `corelink-psu` crate. All hardware I/O
(hidapi, the 1 Hz poll loop, hwmon file reads, staleness tracking) lives in the
later daemon change; this crate only moves bytes and math.

## ADDED Requirements

### Requirement: Pure Core Contract
The PSU telemetry core SHALL be a pure, synchronous module with no I/O dependencies: no file access, no USB/hidapi, no system clock, no threads, and no logging. It SHALL accept raw reply bytes or hwmon channel values as injected input and SHALL emit a snapshot (structured value) plus a typed list of fault events. Every function SHALL produce identical output for identical input, and the crate SHALL construct no HID write command of any kind.

#### Scenario: Deterministic on identical input
- **WHEN** the snapshot builder is invoked twice with the same raw replies (or the same hwmon values),
- **THEN** both invocations emit identical snapshot fields and identical fault events.

#### Scenario: No I/O or clock access
- **WHEN** the crate is compiled and linked,
- **THEN** it performs no file, network, or USB access and reads no system time; all values advance only through injected input.

#### Scenario: No write command is ever constructed
- **WHEN** the crate is exercised over its full public API with any combination of inputs,
- **THEN** it produces only the read-style request layouts defined by the HID Packet Construction requirement and never a fan-mode, fan-PWM, or any other write request.

### Requirement: HID Packet Construction
The core SHALL construct 64-byte little-endian HID request reports (`[u8; 64]`) with all bytes after the command/param bytes zeroed. The read request SHALL be `[0x03, CMD, 0]` (length, command, zero param). The init request SHALL be `[0xFE, 0x03, 0]`, using the device's swapped length/command byte order. The rail-select request SHALL be `[0x02, 0x00, RAIL]` where RAIL ∈ {0 = +12 V, 1 = +5 V, 2 = +3.3 V}.

#### Scenario: Read packet exact layout
- **WHEN** a read packet is built for command `0x90`,
- **THEN** `buf[0]==0x03`, `buf[1]==0x90`, `buf[2]==0x00`, and every byte from index 3 through 63 is `0x00`.

#### Scenario: Init packet uses swapped bytes
- **WHEN** an init packet is built,
- **THEN** `buf[0]==0xFE` and `buf[1]==0x03` (length byte and command byte swapped relative to a normal command), with the remainder zeroed.

#### Scenario: Rail-select packet
- **WHEN** a rail-select packet is built for the +5 V rail,
- **THEN** `buf[0]==0x02`, `buf[1]==0x00`, `buf[2]==0x01`, with the remainder zeroed.

### Requirement: LINEAR11 Decode With Per-Register Units
Numeric HID replies carry a 16-bit little-endian value at reply offset 2–3. The core SHALL decode it as an SMBus LINEAR11 quantity — an 11-bit two's-complement mantissa and a two's-complement exponent (in the upper 5 bits of the same 16-bit word) such that `quantity = mantissa × 2^exponent` — byte-for-byte the LINEAR11 part of the Linux `corsair-psu` driver's `corsairpsu_linear11_to_long` reference. The quantity is already in the register's native unit, which SHALL be fixed per register: volts (input voltage and all rail voltages), amps (input current and all rail currents), watts (total and rail power), degrees Celsius (VRM and case temperature), and RPM (fan). No further scaling is applied on the raw data path — the kernel's per-command ×1000 (V/A/°C) and ×1,000,000 (W) multipliers are hwmon ABI conversions to milli/micro units, and the hwmon data path undoes exactly those. A negative quantity for a voltage, power, current, temperature, or RPM field SHALL be invalid: it is handled as a plausibility fault with the field set to `None`, and SHALL NOT be silently clamped to 0.0.

#### Scenario: Voltage — mantissa with exponent
- **WHEN** the input-voltage register's raw 16-bit word carries mantissa 3 and exponent 2 (quantity 3 × 2^2 = 12),
- **THEN** the snapshot reports `12.0` V (the register's native unit is volts).

#### Scenario: Power — native watts
- **WHEN** the total-power register's raw word carries mantissa 1500 and exponent 0 (quantity 1500),
- **THEN** the snapshot reports `1500.0` W (the kernel's ×1,000,000 in this case is only its hwmon µW conversion).

#### Scenario: Fan RPM native
- **WHEN** the fan register's raw word carries mantissa 1250 and exponent 1 (quantity 1250 × 2 = 2500),
- **THEN** the snapshot reports `2500.0` RPM.

#### Scenario: Temperature — native degrees Celsius
- **WHEN** a temperature register's raw word decodes to quantity 45,
- **THEN** the snapshot reports `45.0` °C, and the > 1 °C plausibility gate applies to that Celsius value.

#### Scenario: Decode matches the kernel reference
- **WHEN** the decoder is run over its golden vector set,
- **THEN** every result equals the quantity the Linux `corsairpsu_linear11_to_long` reference produces for the same raw 16-bit input (the mantissa × 2^exponent part, before that reference's per-command ABI scale), with the golden vectors derived from that C code rather than hand-approximated.

### Requirement: Response Echo Validation
For every reply the core SHALL classify it against the request that produced it. If the reply echoes the request's length and command bytes, the reply SHALL be `matched` and its data SHALL be accepted. If the reply's echoed command byte is zero (the kernel-documented signal that the device does not implement that command), the reply SHALL be classified `unsupported` and SHALL NOT be retried — the field is recorded as absent. If neither the length nor the command echoes the request, the reply SHALL be classified `mismatch` and treated as transient (e.g. a concurrent client such as iCUE interleaving), to be retried by the caller up to its budget and then surfaced as a fault — it SHALL never be silently accepted as data.

#### Scenario: Matched reply accepted
- **WHEN** a reply reads `[0x03, 0x90, 0x80, 0x09, ...]` for a read request `[0x03, 0x90, 0x00]`,
- **THEN** the validation verdict is `matched` and bytes 2–3 are read as the value.

#### Scenario: Zero command echo is unsupported
- **WHEN** a reply echoes the length but carries a zero command byte in the command position,
- **THEN** the verdict is `unsupported` and no retry is signaled.

#### Scenario: Mismatched reply is transient
- **WHEN** a reply's length or command byte does not equal the request,
- **THEN** the verdict is `mismatch`, signaling a retry (or a fault after the caller's retry budget), never an accepted value.

### Requirement: Raw HID Data Path
Given the ordered raw replies collected by walking the Poll Cycle Plan, the core SHALL produce a normalized `PsuSnapshot` whose voltage, current, power, temperature, and fan-RPM fields each carry a value in the register's native unit (V, A, W, °C, RPM respectively). Fields whose reply was `unsupported`, `mismatched` past the retry budget, or failed to decode SHALL be `None` with a corresponding typed event — never `0.0`. No additional unit scaling is applied on this path beyond the LINEAR11 decode itself. The core SHALL additionally accept an optional fan-mode reply (command 0xF0; an observation-only read the daemon performs outside the core cycle) and, when provided, SHALL set the snapshot's fan-mode field to automatic (raw value 0) or the fixed duty percentage (raw value 1–100); the fan-mode field is excluded from the degraded-snapshot gauge and from plausibility gating.

#### Scenario: Healthy full reply set yields populated snapshot
- **WHEN** a PSU model in which every register in the plan is supported returns a matched, in-plausibility-range reply for each,
- **THEN** the snapshot's corresponding fields are all `Some` and the event list contains no missing-field events; for the HX1500i, which does not implement input current (0x89), the input-current field is `None` with a missing-field event even in an otherwise healthy snapshot.

#### Scenario: Fan mode is an optional observation
- **WHEN** a fan-mode reply (0xF0) with decoded value 0 is provided,
- **THEN** the snapshot's fan-mode field reports automatic, and no other field or event is affected by its presence or absence.

#### Scenario: Unsupported field is None with an event
- **WHEN** the input-current register returns an `unsupported` echo,
- **THEN** the input-current field is `None` and a missing-field event names that field; no other field is affected.

#### Scenario: Corrupt decode is a fault, not zero
- **WHEN** a power reply decodes to a negative or otherwise non-physical quantity within the echo-matched bytes,
- **THEN** the field is set to `None` and a decode/implausibility fault event is emitted — the raw bytes are never reported as `0.0`.

### Requirement: Poll Cycle Plan As Data
The core SHALL expose the per-cycle transfer sequence as an inspectable plan of (kind, register) steps, in fixed order: init; fan (0x90); VRM temperature (0x8D); case temperature (0x8E); input voltage (0x88); input current (0x89); total power (0xEE); then, for each rail 12/5/3.3 in that order: rail-select (0x00) followed by volts (0x8B), amps (0x8C), watts (0x96). The plan SHALL include the init step at the head of every cycle (re-initialize before the cycle's reads), reflecting the self-healing re-init cadence. The plan SHALL be data (not embedded I/O), so the daemon cannot drift out of sync with what the core expects, and tests can assert the exact sequence and count.

#### Scenario: Plan starts with init and ends with the 3.3 V rail block
- **WHEN** the plan is enumerated,
- **THEN** its first step is the init step and its last step is the 3.3 V rail's watts read, with the three rail blocks ordered 12 V, 5 V, 3.3 V.

#### Scenario: Plan step count is stable
- **WHEN** the plan is enumerated,
- **THEN** it contains exactly: 1 init + 1 fan + 2 temps + 1 input-V + 1 input-A + 1 total-power + 3×(1 rail-select + 3 rail reads) = 19 steps.

#### Scenario: Plan is deterministic and side-effect free
- **WHEN** the plan is requested twice,
- **THEN** both enumerations are identical and no I/O occurs.

### Requirement: Plausibility Validation
Every `Some` snapshot field SHALL be checked against a per-register plausibility range before being emitted. Suggested ranges (configurable): input voltage 90–265 V; output +12 V 4.0–13.2 V, +5 V 4.5–5.6 V, +3.3 V 3.0–4.0 V; each temperature > 1 °C and < 150 °C; total power 0–3000 W; per-rail power 0–2200 W; fan RPM 0–12000; input current 0–40 A. A value outside its range SHALL not be emitted as data: the field SHALL be set to `None` and a typed plausibility fault event carrying the field, the offending value, and the range SHALL be emitted.

#### Scenario: Out-of-range value is faulted, not plotted
- **WHEN** the +12 V rail decodes to 120 V (outside 4.0–13.2 V),
- **THEN** the field is `None` and a plausibility fault event names the +12 V rail, the value 120 V, and the allowed range.

#### Scenario: Temperature sanity gate
- **WHEN** a temperature reply decodes to a value of 0 °C or below after the > 1 °C gate,
- **THEN** it is treated as absent (`None`) with an event, matching the reference drivers' `temp > 1` gate.

#### Scenario: In-range value passes untouched
- **WHEN** a field's value lies within its plausibility range,
- **THEN** it is emitted unmodified and no plausibility event is raised for it.

### Requirement: Normalized Snapshot With Per-Field Optionality
The core SHALL represent a PSU telemetry sample as a single snapshot type usable by both data paths, in which every measurable field is an `Option`-typed value in the field's native unit: input voltage (V), input current (A, optional per model), total power (W), +12 V / +5 V / +3.3 V rail voltage (V), current (A), and power (W) each, VRM temperature (°C), case temperature (°C), fan speed (RPM), and fan mode (an optional observation-only field, automatic or fixed duty %). A field SHALL be `None` — never `0.0` — when it is unsupported, unread, or implausible, and each such `None` SHALL be accompanied by a typed event. When four or fewer of the snapshot's fifteen core fields (input voltage, input current, total power, three rails × voltage/current/power, two temperatures, fan RPM) are `Some`, the core SHALL emit a degraded-snapshot fault rather than presenting an almost-empty snapshot as healthy.

#### Scenario: Field None never implies zero
- **WHEN** a field is absent (unsupported, mismatched past budget, or implausible),
- **THEN** it is stored as `None` with an event, and no downstream consumer can mistake it for a genuine 0.0 reading.

#### Scenario: Degenerate snapshot is flagged
- **WHEN** a snapshot has only the fan RPM and one temperature populated (two of fifteen fields),
- **THEN** a degraded-snapshot fault event is emitted alongside the partial data.

#### Scenario: Single snapshot type for both paths
- **WHEN** a snapshot is built from raw HID replies and one is built from hwmon channel values for the same physical readings,
- **THEN** both are the same struct with the same SI units, so the two paths cannot emit divergent numbers.

### Requirement: Device Identity
The core SHALL map (vendor ID, product ID) to a device model entry. The HX1500i SHALL be encoded as `0x1b1c:0x1c1f` (one product ID covers the Legacy, 2023, and 2025 series), with a rated wattage and its rail set. Product and vendor strings SHALL be parsed from their HID replies with NUL termination stripped; the legacy layout (separate "CORSAIR " product prefix and " PSU" suffix) and the 2023+ layout (vendor and product merged into a single string) SHALL both be handled. The core SHALL derive a stable device identifier from the parsed product identity, falling back to the (VID, PID) pair when the product string is unreadable.

#### Scenario: HX1500i recognized
- **WHEN** identity is resolved for `(0x1b1c, 0x1c1f)`,
- **THEN** the model entry reports the HX1500i name, its rated watts, and its rail set.

#### Scenario: Legacy product string cleaned
- **WHEN** a product string contains the "CORSAIR " prefix and/or a " PSU" suffix followed by NUL bytes,
- **THEN** the parsed product name excludes both decorations and all NUL bytes.

#### Scenario: Unreadable product falls back to id
- **WHEN** the product string reply is absent or empty,
- **THEN** the device identifier falls back to the (VID, PID) pair and a note records that the product string was unavailable.

### Requirement: Critical Thresholds Captured At Init
The core SHALL provide a function to build the snapshot's threshold area from the critical and low-critical threshold reads (temperature 0x4F, read once per temperature sensor; per-rail voltage high 0x40 and low 0x44, and per-rail current 0x46, each read per rail with the rail selected first), which the daemon performs once at startup. Each threshold SHALL be an optional SI value: present when a matched, in-range reply is read, `None` with an event when the device reports the threshold as unsupported. The raw decoded quantities SHALL be converted to the same SI units as their corresponding data fields.

#### Scenario: Supported threshold is populated
- **WHEN** a temperature high-critical reply matches and is in range,
- **THEN** the snapshot's corresponding threshold is `Some`, in the same units as the temperature field.

#### Scenario: Unsupported threshold is None
- **WHEN** a low-critical voltage reply is echoed `unsupported`,
- **THEN** the corresponding threshold is `None` with an event, and the data fields are unaffected.

### Requirement: hwmon Data Path
The core SHALL also construct the same normalized snapshot from `corsair-psu` hwmon channel values, as the preferred source when the Linux driver is present: input voltage `in` channels (millivolts ÷ 1000 → V), current `curr` channels (milliamperes ÷ 1000 → A), power `power` channels (microwatts ÷ 1,000,000 → W), temperature `temp` channels (millidegrees ÷ 1000 → °C), and fan RPM (`fan`, RPM). The field-to-rail and field-to-scalar mapping SHALL match the Raw HID Data Path exactly, so the two paths are interchangeable and can be diffed on live hardware as a decode oracle.

#### Scenario: hwmon millivolts to volts
- **WHEN** the +12 V hwmon `in` channel reads 12010 mV,
- **THEN** the snapshot reports `12.01` V for that rail.

#### Scenario: hwmon microwatts to watts
- **WHEN** the hwmon total-power channel (`power1`) reads 1,500,000,000 µW,
- **THEN** the snapshot reports `1500.0` W for that field.

#### Scenario: hwmon and raw paths agree
- **WHEN** a hwmon snapshot and a raw-HID snapshot are built for the same physical readings,
- **THEN** every present field carries the same value (within decode precision), confirming the two paths cannot diverge.

#### Scenario: Absent hwmon channel is None with an event
- **WHEN** a channel attribute the kernel hid as unsupported is absent or unreadable (e.g. `curr1_input` on a unit without input-current sensing),
- **THEN** the corresponding field is `None` with a missing-field event — the hwmon-path analogue of the raw path's `unsupported` verdict — and no other field is affected.
