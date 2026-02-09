# Changelog (refactored layout)

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
