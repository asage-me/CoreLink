//! Per-source tripwire state: own EMA, absolute + dT/dt trip conditions,
//! latch with hysteresis clear. Shared (read-only) across profiles (design D2).
