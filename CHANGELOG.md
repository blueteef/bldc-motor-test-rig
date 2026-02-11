# Changelog

## v1.5.1 — 2026-02-11
### Fixed
- **TFT white screen**: moved `rigSensorsBegin()` after I2C/SPI init; removed double `Wire.begin()` from INA226 driver.
- **SPI bus contention**: all logging modules (CSV, JSON, run summary) now properly deselect both TFT and SD chip-select lines after SD operations.
- Duplicate `requestAutoStopLogging()` calls in sweep stop and manual timer.
- Dead JSON writes before file was opened (moved to after `logJsonOpen()`).
- Hardcoded version strings replaced with `GIT_SHA` / `FW_VERSION` macros.
- `logJsonWriteColumns` signature mismatch (`const char**` vs `const char* const*`) that caused weak fallback to be used.

### Added
- **INA226 power data in logging pipeline**: vin_v, iin_a, pin_w, energy_wh columns in CSV, JSON, RUN_SUMMARY.CSV, and RUN_SUMMARY_SHORT.CSV.
- Power summary stats (min/max/mean) in JSON run summaries.

### Changed
- Pinned dependencies: `espressif32@6.10.0`, `Adafruit ILI9341@1.6.1`, `Adafruit GFX Library@1.11.11`.
- Removed orphaned flat headers from `include/` root (duplicated in `include/bldc_rig/logging/`).
- Removed weak linker shim for `logJsonWriteColumns`.

## 1.5.0_repo_refactor — 2026-02-07
### Changed
- Reorganized codebase into an **industry-standard, domain-oriented layout**:
  - `src/app/` for application entrypoints
  - `src/logging/` for logging and summaries
  - `src/core/` for version/build metadata
- Namespaced project headers under `include/bldc_rig/` and updated all includes accordingly.
- Firmware version bumped to `1.5.0_repo_refactor`.

### Notes
- Functional behavior is intended to be unchanged from `1.4.1_json_finalize_fix`; this is primarily a maintainability/refactor release.
