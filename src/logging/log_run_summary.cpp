// log_run_summary.cpp
#include "bldc_rig/logging/log_run_summary.h"
#include <math.h>

// SPI CS pins — set by caller via init or passed through
static int gTftCs = -1;
static int gSdCs  = -1;

static inline void spiSelectSd() {
  if (gTftCs >= 0) digitalWrite(gTftCs, HIGH);
  if (gSdCs >= 0)  digitalWrite(gSdCs, LOW);
}
static inline void spiDeselectSd() {
  if (gSdCs >= 0) digitalWrite(gSdCs, HIGH);
}

// --- Helpers ---
static inline bool is_nan(float v) { return isnan(v); }

static String csvEscape(const String& in) {
  // Quote only when needed; escape quotes by doubling them
  bool needsQuotes = false;
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == ',' || c == '"' || c == '\n' || c == '\r') { needsQuotes = true; break; }
  }
  if (!needsQuotes) return in;

  String out = "\"";
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"') out += "\"\"";
    else out += c;
  }
  out += "\"";
  return out;
}

static void printOptFloat(File &f, float v, int decimals = 3) {
  if (is_nan(v)) return;        // blank field
  f.print(v, decimals);
}

namespace RunSummaryLog {

const char* header() {
  return
    "run_id,run_file,json_file,git_sha,fw_version,schema_version,"
    "start_epoch,start_ms,duration_ms,samples_csv,log_hz_nominal,"
    "mode,target,throttle_min,throttle_max,throttle_step,ramp_up_ms,hold_ms,ramp_down_ms,"
    "ppr,esc_pin,rpm_pin,"
    "end_reason,fault_code,min_vbat_v,max_temp_c,"
    "rpm_min,rpm_max,rpm_avg,rpm_p95,rpm_std,"
    "vbus_min_v,vbus_max_v,ibus_min_a,ibus_max_a,pwr_max_w,energy_wh,"
    "vib_rms_g,vib_max_g,vib_axis_max_g,"
    "thrust_max_n,thrust_avg_n,torque_max_nm,"
    "notes";
}

bool ensureFileWithHeader(fs::FS &fs, const char* path, int tftCsPin, int sdCsPin) {
  gTftCs = tftCsPin;
  gSdCs  = sdCsPin;

  // If missing -> create and write header
  spiSelectSd();
  bool exists = fs.exists(path);
  spiDeselectSd();

  if (!exists) {
    spiSelectSd();
    File f = fs.open(path, FILE_WRITE);
    if (!f) { spiDeselectSd(); return false; }
    f.println(header());
    f.close();
    spiDeselectSd();
    return true;
  }

  // If exists but empty -> write header
  spiSelectSd();
  File f = fs.open(path, FILE_READ);
  if (!f) { spiDeselectSd(); return false; }
  size_t sz = f.size();
  f.close();
  spiDeselectSd();

  if (sz == 0) {
    spiSelectSd();
    File fw = fs.open(path, FILE_WRITE);
    if (!fw) { spiDeselectSd(); return false; }
    fw.println(header());
    fw.close();
    spiDeselectSd();
  }

  return true;
}

bool append(fs::FS &fs, const char* path, const RunSummary &s, int tftCsPin, int sdCsPin) {
  if (!ensureFileWithHeader(fs, path, tftCsPin, sdCsPin)) return false;

  spiSelectSd();
  File f = fs.open(path, FILE_APPEND);
  if (!f) { spiDeselectSd(); return false; }

  // Keep order EXACTLY as header.
  f.print(s.run_id); f.print(',');
  f.print(csvEscape(s.run_file)); f.print(',');
  f.print(csvEscape(s.json_file)); f.print(',');
  f.print(csvEscape(s.git_sha)); f.print(',');
  f.print(csvEscape(s.fw_version)); f.print(',');
  f.print(csvEscape(s.schema_version)); f.print(',');

  f.print(s.start_epoch); f.print(',');
  f.print(s.start_ms); f.print(',');
  f.print(s.duration_ms); f.print(',');
  f.print(s.samples_csv); f.print(',');
  f.print(s.log_hz_nominal, 2); f.print(',');

  f.print(csvEscape(s.mode)); f.print(',');
  f.print(csvEscape(s.target)); f.print(',');
  f.print(s.throttle_min); f.print(',');
  f.print(s.throttle_max); f.print(',');
  f.print(s.throttle_step); f.print(',');
  f.print(s.ramp_up_ms); f.print(',');
  f.print(s.hold_ms); f.print(',');
  f.print(s.ramp_down_ms); f.print(',');

  f.print(s.ppr); f.print(',');
  f.print(s.esc_pin); f.print(',');
  f.print(s.rpm_pin); f.print(',');

  f.print(csvEscape(s.end_reason)); f.print(',');
  f.print(csvEscape(s.fault_code)); f.print(',');
  printOptFloat(f, s.min_vbat_v, 3); f.print(',');
  printOptFloat(f, s.max_temp_c, 2); f.print(',');

  printOptFloat(f, s.rpm_min, 1); f.print(',');
  printOptFloat(f, s.rpm_max, 1); f.print(',');
  printOptFloat(f, s.rpm_avg, 1); f.print(',');
  printOptFloat(f, s.rpm_p95, 1); f.print(',');
  printOptFloat(f, s.rpm_std, 2); f.print(',');

  printOptFloat(f, s.vbus_min_v, 3); f.print(',');
  printOptFloat(f, s.vbus_max_v, 3); f.print(',');
  printOptFloat(f, s.ibus_min_a, 3); f.print(',');
  printOptFloat(f, s.ibus_max_a, 3); f.print(',');
  printOptFloat(f, s.pwr_max_w, 2); f.print(',');
  printOptFloat(f, s.energy_wh, 4); f.print(',');

  printOptFloat(f, s.vib_rms_g, 3); f.print(',');
  printOptFloat(f, s.vib_max_g, 3); f.print(',');
  f.print(csvEscape(s.vib_axis_max_g)); f.print(',');

  printOptFloat(f, s.thrust_max_n, 3); f.print(',');
  printOptFloat(f, s.thrust_avg_n, 3); f.print(',');
  printOptFloat(f, s.torque_max_nm, 4); f.print(',');

  f.println(csvEscape(s.notes));

  f.close();
  spiDeselectSd();
  return true;
}

} // namespace
