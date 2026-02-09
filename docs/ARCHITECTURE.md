# Repository layout (refactored)

This project is a PlatformIO/Arduino ESP32 firmware.

## Top level
- `src/` — firmware sources (organized by domain)
- `include/` — project public headers (namespaced under `bldc_rig/`)
- `scripts/` — PlatformIO build hooks (inject git SHA, build time, dirty bit)
- `docs/` — documentation

## Source domains
- `src/app/` — application entrypoints (`setup()` / `loop()`), CLI command routing
- `src/logging/` — CSV/JSON logging + run summary aggregation
- `src/core/` — version/build metadata, core utilities

## Header namespace
All project headers live under:
- `include/bldc_rig/...`

This avoids collisions with third-party libraries and makes include paths stable.
