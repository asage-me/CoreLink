# corelink-core

The pure fan-control core of CoreLink Phase 1.

- **No I/O.** No file access, no USB, no device/hwmon concepts, no internal
  clock. Timestamps, per-source sensor readings, and the last-commanded PWM
  are injected per tick; the core emits exactly one outcome per profile per
  tick.
- **One dependency**: `serde` (derive) for the config value types.
- **No async, no panics on tick.** Faults are structured events attached to
  the result; config problems surface at `validate()`.

The daemon (a later change) owns hwmon reads and USB writes, evaluates
tripwire sources once per tick into a shared map, and calls
`ProfileHandle::tick` synchronously, once per poll tick, per fan profile.
