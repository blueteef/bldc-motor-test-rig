// log_run_summary.h
#pragma once
#include <Arduino.h>
#include <FS.h>

struct RunSummary {
  // Identity + provenance
  uint32_t run_id = 0;
  String   run_file;     // "RUN0001.CSV"
  String   json_file;    // "RUN0001.JSON"
  String   git_sha;      // short
  String   fw_version;   // e.g., "v0.2.1"
  String   schema_version = "1";

  // Timing
  uint32_t start_epoch = 0;   // 0 if unknown
  uint32_t start_ms = 0;
  uint32_t duration_ms = 0;
  uint32_t samples_csv = 0;
  float    log_hz_nominal = 50.0f;

  // Command + config
  String   mode;              // "manual"|"sweep"|...
  String   target;            // "throttle"|"rpm"|...
  int32_t  throttle_min = 0;
  int32_t  throttle_max = 0;
  int32_t  throttle_step = 0;
  int32_t  ramp_up_ms = 0;
  int32_t  hold_ms = 0;
  int32_t  ramp_down_ms = 0;
  int32_t  ppr = 1;
  int32_t  esc_pin = 25;
  int32_t  rpm_pin = -1;

  // End state / safety
  String   end_reason = "ok";   // ok|abort|fault|...
  String   fault_code = "none"; // none|overcurrent|...
  // Optional floats: use NAN when not available
  float    min_vbat_v = NAN;
  float    max_temp_c = NAN;

  // RPM summary
  float    rpm_min = NAN;
  float    rpm_max = NAN;
  float    rpm_avg = NAN;
  float    rpm_p95 = NAN;
  float    rpm_std = NAN;

  // Electrical summary (future)
  float    vbus_min_v = NAN;
  float    vbus_max_v = NAN;
  float    ibus_min_a = NAN;
  float    ibus_max_a = NAN;
  float    pwr_max_w  = NAN;
  float    energy_wh  = NAN;

  // Vibration summary
  float    vib_rms_g = NAN;
  float    vib_max_g = NAN;
  String   vib_axis_max_g; // "x|y|z" or ""

  // Thrust / torque summary (future)
  float    thrust_max_n = NAN;
  float    thrust_avg_n = NAN;
  float    torque_max_nm = NAN;

  String   notes; // user-provided / metadata
};

namespace RunSummaryLog {
  // Create file + write header if missing/empty
  bool ensureFileWithHeader(fs::FS &fs, const char* path);

  // Append one row
  bool append(fs::FS &fs, const char* path, const RunSummary &s);

  // The header (canonical)
  const char* header();
}
