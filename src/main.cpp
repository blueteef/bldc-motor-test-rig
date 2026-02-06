#include <Arduino.h>
#include <driver/ledc.h>

/*
  ESP32 MOTOR TEST RIG – IMU + SD LOGGING + NO-FLICKER UI + ESC MOTOR CONTROL
  FULL SWAP + STRICT CLI (GNU-ish) + ESC CALIBRATE + PWM FREQ CONTROL

  FIXES INCLUDED (latest):
    - STRICT CLI: NO side-effects on parse/validation errors.
    - SWEEP STOP FIX: when a sweep completes or user runs `sweep stop`,
        * it now forces thrTarget=0.0 (ramps down via rampRate)
        * so ESC output actually returns to MIN and motor stops
    - Ramp/Tri/Step completion paths call sweepStop() only (no leaving thrTarget at ramp_stop)
    - Default ESC PWM frequency = 200 Hz
    - ESC calibration state machine: esc calibrate / esc calibrate stop

  SAFETY NOTE:
    - `sweep stop` ramps throttle down to 0 (soft stop).
    - `motor estop` is immediate output low + disarm (hard stop).
    - Props OFF during development/testing.

  COMMANDS:
    help [motor|sweep|log|esc]
    status [motor|esc]

    log start
    log stop

    base capture [--windows N]

    set motor "text..."
    set notes "text..."

    motor arm
    motor disarm
    motor estop
    motor set --thr f           (0..1)
    motor set --us  n           (1000..2000)
    motor manual --thr f --sec s [--no-log]

    sweep stop
    sweep step --start a --stop b --step c --hold s [--ramp r]
    sweep ramp --start a --stop b --dur s
    sweep tri  --min a --max b --period s --cycles n

    esc set --freq Hz           (50..500)
    esc calibrate
    esc calibrate stop
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>

// ----------------- PINOUT -----------------
#define TFT_CS 5
#define TFT_DC 26
#define TFT_RST 4

#define SD_CS 13
#define TOUCH_CS 14

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define I2C_SDA 21
#define I2C_SCL 22

// ESC PWM
#define ESC_PIN 25
static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_MAX_US = 2000;

// Default ESC frequency
static constexpr uint32_t ESC_FREQ_HZ_DEFAULT = 200;
static constexpr uint8_t ESC_LEDC_RES_BITS = 16;
static constexpr uint8_t ESC_LEDC_CH = 0;

// ----------------- DISPLAY -----------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ----------------- IMU -----------------
#define IMU_ADDR 0x68

static constexpr uint32_t SAMPLE_US = 4000; // 250 Hz
static constexpr uint32_t UI_MS = 100;      // 10 Hz UI
static constexpr uint32_t LOG_MS = 20;      // 50 Hz logging
static constexpr uint32_t MOTOR_MS = 20;    // 50 Hz motor

// Gravity separation LPF
static constexpr float DT_S = 0.004f;
static constexpr float FC_G_HZ = 1.5f;
static constexpr float PI_F = 3.1415926f;
static constexpr float RC = 1.0f / (2.0f * PI_F * FC_G_HZ);
static constexpr float ALPHA_G = DT_S / (RC + DT_S);

// RMS window
static constexpr int RMS_N = 250; // 1s @ 250 Hz

// Baseline capture
static constexpr int BASELINE_WINDOWS_DEFAULT = 3;

// Redraw thresholds
static constexpr float AX_EPS = 0.03f;
static constexpr float VIB_EPS = 0.01f;
static constexpr float THR_EPS = 0.01f;

// ----------------- STATE -----------------
File logFile;
bool loggingEnabled = false;
bool sdOk = false;
uint32_t runNumber = 0;

char motorId[48] = "UNKNOWN";
char notes[96] = "";

// run metadata snapshot
uint32_t runStartMs = 0;
float baselineAtRun = 0.0f;
bool baselineValidAtRun = false;

// IMU values (m/s^2)
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float dx = 0, dy = 0, dz = 0;
float vibInst = 0;
float vibRms = 0;
float vibNet = 0;

double sumSq = 0.0;
int rmsCount = 0;
volatile bool vibRmsUpdated = false;
uint8_t whoami = 0;

// baseline
float vibBaseline = 0.0f;
bool baselineValid = false;
bool baselineCaptureActive = false;
float baselineSum = 0.0f;
int baselineCount = 0;
int baselineTargetWindows = BASELINE_WINDOWS_DEFAULT;

// rpm placeholder
int32_t rpmValue = -1;

// ----------------- MOTOR CONTROL -----------------
enum MotorMode : uint8_t
{
  MOTOR_OFF = 0,
  MOTOR_MANUAL,
  MOTOR_SWEEP_STEP,
  MOTOR_SWEEP_RAMP,
  MOTOR_SWEEP_TRI,
  MOTOR_SETUP_MANUAL_TIMER
};

bool motorArmed = false;
MotorMode motorMode = MOTOR_OFF;

float thrCmd = 0.0f;
float thrTarget = 0.0f;
float rampRate = 0.20f; // thr units/sec

bool manualTimerActive = false;
uint32_t manualStopMs = 0;

// step sweep
float step_start = 0.0f, step_stop = 0.0f, step_step = 0.0f;
float step_hold_s = 0.0f;
float step_current = 0.0f;
uint32_t step_phaseStartMs = 0;
bool step_holding = false;

// ramp sweep
float ramp_start = 0.0f, ramp_stop = 0.0f;
float ramp_dur_s = 0.0f;
uint32_t ramp_startMs = 0;

// tri sweep
float tri_min = 0.0f, tri_max = 0.0f;
float tri_period_s = 0.0f;
int tri_cycles = 0;
uint32_t tri_startMs = 0;

// ----------------- ESC PWM RUNTIME SETTINGS -----------------
uint32_t escFreqHz = ESC_FREQ_HZ_DEFAULT;

// ESC calibration state machine
enum EscCalState : uint8_t
{
  ESC_CAL_OFF = 0,
  ESC_CAL_HIGH,
  ESC_CAL_LOW,
  ESC_CAL_DONE
};
EscCalState escCalState = ESC_CAL_OFF;
uint32_t escCalT0 = 0;
static constexpr uint32_t ESC_CAL_HIGH_MS = 15000; // hold HIGH for 15s
static constexpr uint32_t ESC_CAL_LOW_MS = 15000;  // then LOW for 15s

// ----------------- UI LAYOUT -----------------
static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 240;

static constexpr int MARGIN_X = 8;
static constexpr int COL_W = 152;
static constexpr int COL_GAP = 8;
static constexpr int COL1_X = MARGIN_X;
static constexpr int COL2_X = MARGIN_X + COL_W + COL_GAP;

static constexpr int HEADER_Y1 = 6;
static constexpr int HEADER_Y2 = 26;

static constexpr int BODY_Y0 = 52;
static constexpr int ROW_H = 22; // size 2

static constexpr int LABEL_W_PIX = 84;
static constexpr int VALUE_XOFF = LABEL_W_PIX;

static constexpr int META_Y0 = 184;
static constexpr int META_LINE = 10;
static constexpr int META_PAD = 2;

static constexpr int STATUS_H = 20;
static constexpr int STATUS_Y = SCREEN_H - STATUS_H;

// colors
static constexpr uint16_t C_BG = ILI9341_BLACK;
static constexpr uint16_t C_HDR = ILI9341_CYAN;
static constexpr uint16_t C_OK = ILI9341_GREEN;
static constexpr uint16_t C_WARN = ILI9341_ORANGE;
static constexpr uint16_t C_BAD = ILI9341_RED;
static constexpr uint16_t C_LABEL = ILI9341_WHITE;
static constexpr uint16_t C_VALUE = ILI9341_WHITE;
static constexpr uint16_t C_TEXT = ILI9341_YELLOW;

// ----------------- I2C REG IO -----------------
bool writeReg(uint8_t r, uint8_t v)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(r);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t r, uint8_t *b, size_t n)
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0)
    return false;
  const uint8_t req = (uint8_t)n;
  if (Wire.requestFrom((uint8_t)IMU_ADDR, req) != req)
    return false;
  for (size_t i = 0; i < n; i++)
    b[i] = Wire.read();
  return true;
}

int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

// ----------------- UI HELPERS -----------------
void printFixedPadded(const char *s, int widthChars)
{
  char buf[120];
  int n = (int)strlen(s);
  if (n > widthChars)
    n = widthChars;
  memcpy(buf, s, n);
  for (int i = n; i < widthChars; i++)
    buf[i] = ' ';
  buf[widthChars] = '\0';
  tft.print(buf);
}

void drawLabel2(int x, int y, const char *label)
{
  tft.setTextSize(2);
  tft.setCursor(x, y + 3);
  tft.setTextColor(C_LABEL, C_BG);
  tft.print(label);
}

void drawValueText2Fixed(int x, int y, const char *text, int widthChars, uint16_t color)
{
  tft.setTextSize(2);
  tft.setCursor(x, y + 3);
  tft.setTextColor(color, C_BG);
  printFixedPadded(text, widthChars);
}

void clearValueRect2(int x, int y, int w, int h) { tft.fillRect(x, y, w, h, C_BG); }

void drawValueText2(int x, int y, const char *text, uint16_t color)
{
  tft.setTextSize(2);
  tft.setCursor(x, y + 3);
  tft.setTextColor(color, C_BG);
  tft.print(text);
}

void drawMetaLine1Fixed(int y, const char *label, const char *value, int valueChars)
{
  tft.setTextSize(1);
  tft.setCursor(8, y);
  tft.setTextColor(C_LABEL, C_BG);
  tft.print(label);
  tft.setCursor(60, y);
  tft.setTextColor(C_TEXT, C_BG);
  printFixedPadded(value, valueChars);
}

void statusBar(const char *msg, uint16_t color)
{
  tft.setTextSize(2);
  tft.setCursor(8, STATUS_Y + 3);
  tft.setTextColor(color, C_BG);
  printFixedPadded(msg, 28);
}

// ----------------- UTILS -----------------
float clamp01(float x)
{
  if (x < 0)
    return 0;
  if (x > 1)
    return 1;
  return x;
}
int clampInt(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

const char *motorModeName(MotorMode m)
{
  switch (m)
  {
  case MOTOR_OFF:
    return "OFF";
  case MOTOR_MANUAL:
    return "MANUAL";
  case MOTOR_SWEEP_STEP:
    return "STEP";
  case MOTOR_SWEEP_RAMP:
    return "RAMP";
  case MOTOR_SWEEP_TRI:
    return "TRI";
  case MOTOR_SETUP_MANUAL_TIMER:
    return "SETUP";
  default:
    return "UNK";
  }
}

const char *escCalName(EscCalState s)
{
  switch (s)
  {
  case ESC_CAL_OFF:
    return "OFF";
  case ESC_CAL_HIGH:
    return "HIGH";
  case ESC_CAL_LOW:
    return "LOW";
  case ESC_CAL_DONE:
    return "DONE";
  default:
    return "UNK";
  }
}

// ----------------- ESC PWM (ESP32 core 3.x) -----------------
uint32_t usToDuty(uint32_t us)
{
  const uint32_t period_us = 1000000UL / (uint32_t)escFreqHz;
  const uint32_t maxDuty = (1UL << ESC_LEDC_RES_BITS) - 1UL;
  if (us >= period_us)
    us = period_us - 1;
  uint64_t duty = (uint64_t)us * (uint64_t)maxDuty / (uint64_t)period_us;
  if (duty > maxDuty)
    duty = maxDuty;
  return (uint32_t)duty;
}

int thrToUs(float thr)
{
  thr = clamp01(thr);
  return (int)lroundf((float)ESC_MIN_US + thr * (float)(ESC_MAX_US - ESC_MIN_US));
}

void escWriteUs(int us)
{
  us = clampInt(us, ESC_MIN_US, ESC_MAX_US);
  const uint32_t duty = usToDuty((uint32_t)us);
  ledcWrite(ESC_LEDC_CH, duty); // legacy: (channel, duty)
}

int currentEscUs()
{
  if (escCalState == ESC_CAL_HIGH)
  {
    return ESC_MAX_US;
  }
  if (escCalState == ESC_CAL_LOW)
  {
    return ESC_MIN_US;
  }
  if (!motorArmed)
  {
    return ESC_MIN_US;
  }
  return thrToUs(thrCmd);
}

bool escSetFrequency(uint32_t hz)
{
  if (hz < 50 || hz > 500)
  {
    Serial.println("ERR esc_freq_out_of_range (50..500)");
    return false;
  }

  const uint32_t actual = ledcChangeFrequency(ESC_LEDC_CH, hz, ESC_LEDC_RES_BITS);
  if (actual == 0)
  {
    Serial.println("ERR esc_freq_change_failed");
    return false;
  }

  escFreqHz = actual;
  escWriteUs(currentEscUs());

  Serial.print("OK esc_freq=");
  Serial.println(escFreqHz);

  char msg[28];
  snprintf(msg, sizeof(msg), "ESC FREQ %luHz", (unsigned long)escFreqHz);
  statusBar(msg, C_OK);
  return true;
}

// ----------------- SD HELPERS -----------------
uint32_t findNextRunNumber()
{
  uint32_t maxRun = 0;
  File root = SD.open("/");
  if (!root)
    return 1;
  File f = root.openNextFile();
  while (f)
  {
    const char *name = f.name();
    if (!f.isDirectory())
    {
      const char *p = name;
      if (p[0] == '/')
        p++;
      if (strlen(p) == 11 && strncmp(p, "RUN", 3) == 0 && strncmp(p + 7, ".CSV", 4) == 0)
      {
        char numStr[5] = {0};
        memcpy(numStr, p + 3, 4);
        uint32_t n = (uint32_t)atoi(numStr);
        if (n > maxRun)
          maxRun = n;
      }
    }
    f = root.openNextFile();
  }
  return maxRun + 1;
}

void writeRunHeader(File &f)
{
  f.println("# rig=esp32_motor_dyno");
  f.println("# fw=1.4_sweep_stop_fix");
  f.print("# motor_id=");
  f.println(motorId);
  f.print("# notes=");
  f.println(notes);
  f.print("# imu_whoami=0x");
  f.println(whoami, HEX);
  f.print("# esc_pin=");
  f.println(ESC_PIN);
  f.print("# esc_freq_hz=");
  f.println(escFreqHz);
  f.print("# esc_min_us=");
  f.println(ESC_MIN_US);
  f.print("# esc_max_us=");
  f.println(ESC_MAX_US);
  f.println("# sample_hz=250");
  f.println("# log_hz=50");
  f.print("# vib_window_n=");
  f.println(RMS_N);
  f.print("# start_ms=");
  f.println(runStartMs);
  f.print("# baseline_valid=");
  f.println(baselineValidAtRun ? "true" : "false");
  f.print("# baseline_vibrms=");
  f.println(baselineAtRun, 6);
  f.println("timestamp_ms,ax_mps2,ay_mps2,az_mps2,mag_mps2,vib_inst_mps2,vib_rms_mps2,vib_net_mps2,thr_cmd,esc_us,rpm");
  f.flush();
}

bool startLogging()
{
  if (!sdOk)
  {
    statusBar("SD NOT READY", C_BAD);
    Serial.println("ERR sd_not_ready");
    return false;
  }
  if (loggingEnabled)
  {
    Serial.println("ERR log_already_running");
    return false;
  }
  if (escCalState != ESC_CAL_OFF)
  {
    Serial.println("ERR esc_cal_active");
    return false;
  }

  runStartMs = millis();
  baselineAtRun = vibBaseline;
  baselineValidAtRun = baselineValid;

  char fname[16];
  snprintf(fname, sizeof(fname), "/RUN%04lu.CSV", (unsigned long)runNumber);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  logFile = SD.open(fname, FILE_WRITE);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, LOW);

  if (!logFile)
  {
    statusBar("LOG OPEN FAIL", C_BAD);
    Serial.println("ERR log_open_fail");
    return false;
  }

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  writeRunHeader(logFile);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, LOW);

  loggingEnabled = true;

  char msg[28];
  snprintf(msg, sizeof(msg), "LOGGING RUN %04lu", (unsigned long)runNumber);
  statusBar(msg, C_OK);

  Serial.print("OK log_started file=");
  Serial.println(fname);
  return true;
}

void stopLogging()
{
  if (!loggingEnabled)
  {
    Serial.println("ERR log_not_running");
    return;
  }

  uint32_t stopMs = millis();
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  logFile.print("# stop_ms=");
  logFile.println(stopMs);
  logFile.flush();
  logFile.close();
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, LOW);

  loggingEnabled = false;
  statusBar("LOG STOPPED", C_WARN);
  Serial.println("OK log_stopped");

  runNumber++;
}

// ----------------- BASELINE -----------------
void startBaselineCapture(int windows)
{
  if (windows < 1)
    windows = 1;
  if (windows > 30)
    windows = 30;
  baselineTargetWindows = windows;

  baselineCaptureActive = true;
  baselineSum = 0.0f;
  baselineCount = 0;

  statusBar("CAPTURING BASE...", C_WARN);
  Serial.print("OK baseline_start windows=");
  Serial.println(baselineTargetWindows);
}

void onNewVibRmsComputed()
{
  if (baselineValid)
  {
    float v = vibRms - vibBaseline;
    vibNet = (v > 0.0f) ? v : 0.0f;
  }
  else
  {
    vibNet = vibRms;
  }

  if (baselineCaptureActive)
  {
    baselineSum += vibRms;
    baselineCount++;
    if (baselineCount >= baselineTargetWindows)
    {
      vibBaseline = baselineSum / (float)baselineTargetWindows;
      baselineValid = true;
      baselineCaptureActive = false;
      statusBar("BASELINE SET", C_OK);
      Serial.print("OK baseline value=");
      Serial.println(vibBaseline, 6);
    }
    else
    {
      Serial.print("OK baseline_progress ");
      Serial.print(baselineCount);
      Serial.print("/");
      Serial.println(baselineTargetWindows);
    }
  }
}

// ----------------- MOTOR CONTROL -----------------
void sweepStop();

void motorDisarm(const char *reason)
{
  motorArmed = false;
  motorMode = MOTOR_OFF;
  thrTarget = 0;
  thrCmd = 0;
  manualTimerActive = false;
  step_holding = false;
  escWriteUs(ESC_MIN_US);

  statusBar(reason ? reason : "MOTOR DISARM", C_BAD);
  Serial.print("OK motor_disarm reason=");
  Serial.println(reason ? reason : "(none)");
}

void motorArm()
{
  motorArmed = true;
  motorMode = MOTOR_MANUAL;
  thrTarget = 0;
  thrCmd = 0;
  manualTimerActive = false;
  escWriteUs(ESC_MIN_US);

  statusBar("MOTOR ARMED", C_OK);
  Serial.println("OK motor_arm");
}

// FIXED: sweepStop now forces throttle target to 0 so ESC ramps down and stops.
void sweepStop()
{
  motorMode = MOTOR_MANUAL;
  step_holding = false;
  manualTimerActive = false;

  thrTarget = 0.0f; // <-- critical stop behavior
  // thrCmd slews down to 0 via rampRate in motorUpdateTick()

  Serial.println("OK sweep_stop");
  statusBar("SWEEP STOP", C_WARN);
}

void sweepStartStep(float start, float stop, float step, float hold_s, float ramp)
{
  if (!motorArmed)
  {
    Serial.println("ERR not_armed");
    statusBar("ARM FIRST", C_WARN);
    return;
  }
  manualTimerActive = false;

  step_start = clamp01(start);
  step_stop = clamp01(stop);
  step_step = (step > 0.0f) ? step : 0.01f;
  step_hold_s = (hold_s > 0.0f) ? hold_s : 1.0f;
  rampRate = (ramp > 0.0f) ? ramp : rampRate;

  step_current = step_start;
  thrTarget = step_current;
  step_holding = false;
  step_phaseStartMs = millis();
  motorMode = MOTOR_SWEEP_STEP;

  Serial.println("OK sweep_step_start");
  statusBar("SWEEP STEP", C_OK);
}

void sweepStartRamp(float start, float stop, float dur_s)
{
  if (!motorArmed)
  {
    Serial.println("ERR not_armed");
    statusBar("ARM FIRST", C_WARN);
    return;
  }
  manualTimerActive = false;

  ramp_start = clamp01(start);
  ramp_stop = clamp01(stop);
  ramp_dur_s = (dur_s > 0.1f) ? dur_s : 1.0f;
  ramp_startMs = millis();
  motorMode = MOTOR_SWEEP_RAMP;

  Serial.println("OK sweep_ramp_start");
  statusBar("SWEEP RAMP", C_OK);
}

void sweepStartTri(float tmin, float tmax, float period_s, int cycles)
{
  if (!motorArmed)
  {
    Serial.println("ERR not_armed");
    statusBar("ARM FIRST", C_WARN);
    return;
  }
  manualTimerActive = false;

  tri_min = clamp01(tmin);
  tri_max = clamp01(tmax);
  if (tri_max < tri_min)
  {
    float tmp = tri_min;
    tri_min = tri_max;
    tri_max = tmp;
  }
  tri_period_s = (period_s > 0.2f) ? period_s : 2.0f;
  tri_cycles = (cycles > 0) ? cycles : 1;
  tri_startMs = millis();
  motorMode = MOTOR_SWEEP_TRI;

  Serial.println("OK sweep_tri_start");
  statusBar("SWEEP TRI", C_OK);
}

void startManualTimed(float thr, float sec, bool noLog)
{
  if (!motorArmed)
  {
    Serial.println("ERR not_armed");
    statusBar("ARM FIRST", C_WARN);
    return;
  }
  if (sec <= 0.0f || sec > 3600.0f)
  {
    Serial.println("ERR bad_sec");
    return;
  }
  if (noLog && loggingEnabled)
  {
    Serial.println("ERR logging_on_stop_first");
    statusBar("STOP LOG FIRST", C_WARN);
    return;
  }

  thrTarget = clamp01(thr);
  motorMode = MOTOR_SETUP_MANUAL_TIMER;
  manualTimerActive = true;
  manualStopMs = millis() + (uint32_t)(sec * 1000.0f);

  Serial.print("OK motor_manual thr=");
  Serial.print(thrTarget, 3);
  Serial.print(" sec=");
  Serial.print(sec, 1);
  Serial.print(" no_log=");
  Serial.println(noLog ? "true" : "false");

  statusBar("MANUAL SETUP", C_OK);
}

// -------- ESC Calibration --------
void escCalStart()
{
  if (motorArmed)
  {
    Serial.println("ERR esc_cal_requires_disarm");
    statusBar("DISARM FIRST", C_WARN);
    return;
  }
  escCalState = ESC_CAL_HIGH;
  escCalT0 = millis();
  escWriteUs(ESC_MAX_US);

  Serial.println("OK esc_cal_start");
  Serial.println("INFO plug ESC battery DURING HIGH window (15s)");
  Serial.println("INFO then LOW window (15s), then DONE");
  statusBar("ESC CAL: HIGH", C_WARN);
}

void escCalStop()
{
  escCalState = ESC_CAL_OFF;
  escWriteUs(ESC_MIN_US);
  Serial.println("OK esc_cal_stop");
  statusBar("ESC CAL STOP", C_BAD);
}

void escCalTick(uint32_t nowMs)
{
  if (escCalState == ESC_CAL_OFF)
    return;

  uint32_t dt = nowMs - escCalT0;

  if (escCalState == ESC_CAL_HIGH)
  {
    escWriteUs(ESC_MAX_US);
    if (dt >= ESC_CAL_HIGH_MS)
    {
      escCalState = ESC_CAL_LOW;
      escCalT0 = nowMs;
      statusBar("ESC CAL: LOW", C_WARN);
      Serial.println("OK esc_cal_phase low");
    }
    return;
  }

  if (escCalState == ESC_CAL_LOW)
  {
    escWriteUs(ESC_MIN_US);
    if (dt >= ESC_CAL_LOW_MS)
    {
      escCalState = ESC_CAL_DONE;
      escCalT0 = nowMs;
      statusBar("ESC CAL: DONE", C_OK);
      Serial.println("OK esc_cal_done");
      Serial.println("INFO power-cycle ESC, then arm normally");
    }
    return;
  }

  if (escCalState == ESC_CAL_DONE)
  {
    escWriteUs(ESC_MIN_US);
  }
}

void motorUpdateTick(uint32_t nowMs)
{
  // Calibration overrides motor control
  if (escCalState != ESC_CAL_OFF)
  {
    escCalTick(nowMs);
    return;
  }

  if (!motorArmed)
  {
    escWriteUs(ESC_MIN_US);
    return;
  }

  if (manualTimerActive && (int32_t)(nowMs - manualStopMs) >= 0)
  {
    manualTimerActive = false;
    // when manual ends, fall back to manual mode and ramp to zero
    motorMode = MOTOR_MANUAL;
    thrTarget = 0.0f;
    Serial.println("OK manual_done");
    statusBar("MANUAL DONE", C_WARN);
  }

  switch (motorMode)
  {
  case MOTOR_SWEEP_STEP:
  {
    float err = fabsf(thrCmd - thrTarget);
    bool atTarget = (err < 0.01f);
    if (!step_holding)
    {
      if (atTarget)
      {
        step_holding = true;
        step_phaseStartMs = nowMs;
      }
    }
    else
    {
      float held_s = (float)(nowMs - step_phaseStartMs) / 1000.0f;
      if (held_s >= step_hold_s)
      {
        float next = step_current + step_step;
        if (next > step_stop + 1e-6f)
        {
          sweepStop(); // will force thrTarget=0
        }
        else
        {
          step_current = next;
          thrTarget = step_current;
          step_holding = false;
          step_phaseStartMs = nowMs;
        }
      }
    }
  }
  break;

  case MOTOR_SWEEP_RAMP:
  {
    float t = (float)(nowMs - ramp_startMs) / 1000.0f;
    float u = t / ramp_dur_s;
    if (u >= 1.0f)
    {
      sweepStop(); // will force thrTarget=0
    }
    else
    {
      thrTarget = ramp_start + (ramp_stop - ramp_start) * u;
    }
  }
  break;

  case MOTOR_SWEEP_TRI:
  {
    float elapsed_s = (float)(nowMs - tri_startMs) / 1000.0f;
    float total_s = tri_period_s * (float)tri_cycles;
    if (elapsed_s >= total_s)
    {
      sweepStop(); // will force thrTarget=0
    }
    else
    {
      float phase = fmodf(elapsed_s, tri_period_s) / tri_period_s;
      float tri = (phase < 0.5f) ? (phase * 2.0f) : ((1.0f - phase) * 2.0f);
      thrTarget = tri_min + (tri_max - tri_min) * tri;
    }
  }
  break;

  default:
    break;
  }

  // slew limit thrCmd toward thrTarget
  float dt = (float)MOTOR_MS / 1000.0f;
  float maxDelta = rampRate * dt;
  float diff = thrTarget - thrCmd;
  if (diff > maxDelta)
    diff = maxDelta;
  else if (diff < -maxDelta)
    diff = -maxDelta;

  thrCmd = clamp01(thrCmd + diff);
  escWriteUs(thrToUs(thrCmd));
}

// ----------------- UI STATIC -----------------
void drawStaticUI()
{
  tft.fillScreen(C_BG);
  tft.setTextSize(2);

  tft.setCursor(8, HEADER_Y1);
  tft.setTextColor(C_HDR, C_BG);
  tft.print("MOTOR RIG v0.1");

  tft.setCursor(8, HEADER_Y2);
  tft.setTextColor(C_OK, C_BG);
  tft.print("IMU ONLINE");

  tft.setCursor(200, HEADER_Y2);
  tft.setTextColor(C_LABEL, C_BG);
  tft.print("WHO:0x");
  tft.print(whoami, HEX);

  // left labels
  drawLabel2(COL1_X, BODY_Y0 + 0 * ROW_H, "AX:");
  drawLabel2(COL1_X, BODY_Y0 + 1 * ROW_H, "AY:");
  drawLabel2(COL1_X, BODY_Y0 + 2 * ROW_H, "AZ:");
  drawLabel2(COL1_X, BODY_Y0 + 3 * ROW_H, "VIB I:");
  drawLabel2(COL1_X, BODY_Y0 + 4 * ROW_H, "VIB R:");

  // right labels
  drawLabel2(COL2_X, BODY_Y0 + 0 * ROW_H, "SD:");
  drawLabel2(COL2_X, BODY_Y0 + 1 * ROW_H, "RUN:");
  drawLabel2(COL2_X, BODY_Y0 + 2 * ROW_H, "MODE:");
  drawLabel2(COL2_X, BODY_Y0 + 3 * ROW_H, "US:");
  drawLabel2(COL2_X, BODY_Y0 + 4 * ROW_H, "RPM:");

  tft.fillRect(0, META_Y0, SCREEN_W, (STATUS_Y - META_Y0), C_BG);
  tft.fillRect(0, STATUS_Y, SCREEN_W, STATUS_H, C_BG);
  statusBar("BOOT", C_LABEL);
}

// ----------------- STRICT CLI -----------------
static constexpr size_t CLI_LINE_MAX = 220;
static constexpr int CLI_ARGV_MAX = 24;
char cliLine[CLI_LINE_MAX];
size_t cliLen = 0;

static inline char upc(char c)
{
  if (c >= 'a' && c <= 'z')
    return (char)(c - 32);
  return c;
}

bool streqi(const char *a, const char *b)
{
  while (*a && *b)
  {
    if (upc(*a) != upc(*b))
      return false;
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';
}

bool isFlag(const char *a) { return a && a[0] == '-' && a[1] != '\0'; }

int tokenize(char *s, char *argv[], int argvMax)
{
  int argc = 0;
  while (*s)
  {
    while (*s == ' ' || *s == '\t')
      s++;
    if (!*s)
      break;
    if (argc >= argvMax)
      return -1;

    if (*s == '"')
    {
      s++;
      argv[argc++] = s;
      while (*s && *s != '"')
        s++;
      if (*s != '"')
        return -2;
      *s = '\0';
      s++;
    }
    else
    {
      argv[argc++] = s;
      while (*s && *s != ' ' && *s != '\t')
        s++;
      if (*s)
      {
        *s = '\0';
        s++;
      }
    }
  }
  return argc;
}

const char *optValue(int argc, char *argv[], const char *shortFlag, const char *longFlag, bool &ok)
{
  ok = true;
  for (int i = 0; i < argc; i++)
  {
    if (shortFlag && streqi(argv[i], shortFlag))
    {
      if (i + 1 >= argc)
      {
        ok = false;
        return nullptr;
      }
      if (isFlag(argv[i + 1]))
      {
        ok = false;
        return nullptr;
      }
      return argv[i + 1];
    }
    if (longFlag && streqi(argv[i], longFlag))
    {
      if (i + 1 >= argc)
      {
        ok = false;
        return nullptr;
      }
      if (isFlag(argv[i + 1]))
      {
        ok = false;
        return nullptr;
      }
      return argv[i + 1];
    }
  }
  return nullptr;
}

bool hasFlag(int argc, char *argv[], const char *shortFlag, const char *longFlag)
{
  for (int i = 0; i < argc; i++)
  {
    if (shortFlag && streqi(argv[i], shortFlag))
      return true;
    if (longFlag && streqi(argv[i], longFlag))
      return true;
  }
  return false;
}

bool checkNoUnknownFlags(int argc, char *argv[], const char *allowed[], int allowedN)
{
  for (int i = 0; i < argc; i++)
  {
    if (!isFlag(argv[i]))
      continue;
    bool ok = false;
    for (int k = 0; k < allowedN; k++)
    {
      if (streqi(argv[i], allowed[k]))
      {
        ok = true;
        break;
      }
    }
    if (!ok)
    {
      Serial.print("ERR unknown_flag ");
      Serial.println(argv[i]);
      return false;
    }
  }
  return true;
}

bool parseFloat(const char *s, float &out)
{
  if (!s || !*s)
    return false;
  char *end = nullptr;
  out = strtof(s, &end);
  return end && *end == '\0';
}

bool parseInt(const char *s, int &out)
{
  if (!s || !*s)
    return false;
  char *end = nullptr;
  long v = strtol(s, &end, 10);
  if (!(end && *end == '\0'))
    return false;
  out = (int)v;
  return true;
}

// -------- HELP --------
void helpMain()
{
  Serial.println("HELP");
  Serial.println("  help [motor|sweep|log|esc]");
  Serial.println("  status [motor|esc]");
  Serial.println("  log start | log stop");
  Serial.println("  base capture [--windows N]");
  Serial.println("  set motor \"text\" | set notes \"text\"");
  Serial.println("  motor arm|disarm|estop");
  Serial.println("  motor set --thr f | motor set --us n");
  Serial.println("  motor manual --thr f --sec s [--no-log]");
  Serial.println("  sweep stop");
  Serial.println("  sweep step --start a --stop b --step c --hold s [--ramp r]");
  Serial.println("  sweep ramp --start a --stop b --dur s");
  Serial.println("  sweep tri  --min a --max b --period s --cycles n");
  Serial.println("  esc set --freq Hz");
  Serial.println("  esc calibrate | esc calibrate stop");
}

void helpMotor()
{
  Serial.println("HELP MOTOR");
  Serial.println("  motor arm");
  Serial.println("  motor disarm");
  Serial.println("  motor estop");
  Serial.println("  motor set --thr f     (0..1)");
  Serial.println("  motor set --us  n     (1000..2000)");
  Serial.println("  motor manual --thr f --sec s [--no-log]");
}

void helpSweep()
{
  Serial.println("HELP SWEEP");
  Serial.println("  sweep stop");
  Serial.println("  sweep step --start a --stop b --step c --hold s [--ramp r]");
  Serial.println("  sweep ramp --start a --stop b --dur s");
  Serial.println("  sweep tri  --min a --max b --period s --cycles n");
}

void helpLog()
{
  Serial.println("HELP LOG");
  Serial.println("  log start");
  Serial.println("  log stop");
}

void helpEsc()
{
  Serial.println("HELP ESC");
  Serial.println("  status esc");
  Serial.println("  esc set --freq Hz            (50..500)");
  Serial.println("  esc calibrate                (run BEFORE plugging ESC battery)");
  Serial.println("  esc calibrate stop           (abort)");
}

// -------- STATUS --------
void printMotorStatus()
{
  Serial.print("MOTOR armed=");
  Serial.print(motorArmed ? "true" : "false");
  Serial.print(" mode=");
  Serial.print(motorModeName(motorMode));
  Serial.print(" thr_target=");
  Serial.print(thrTarget, 4);
  Serial.print(" thr_cmd=");
  Serial.print(thrCmd, 4);
  Serial.print(" esc_us=");
  Serial.print(currentEscUs());
  Serial.print(" rpm=");
  Serial.println(rpmValue);
}

void printEscStatus()
{
  Serial.print("ESC freq_hz=");
  Serial.print(escFreqHz);
  Serial.print(" us=");
  Serial.print(currentEscUs());
  Serial.print(" cal_state=");
  Serial.println(escCalName(escCalState));
}

void printStatus()
{
  Serial.print("STATUS sd=");
  Serial.print(sdOk ? "ok" : "fail");
  Serial.print(" run=");
  Serial.print(runNumber);
  Serial.print(" log=");
  Serial.print(loggingEnabled ? "on" : "off");
  Serial.print(" baseline=");
  if (baselineValid)
    Serial.print(vibBaseline, 6);
  else
    Serial.print("nan");
  Serial.print(" vibrms=");
  Serial.print(vibRms, 6);
  Serial.print(" vibnet=");
  Serial.print(vibNet, 6);
  Serial.print(" motor=\"");
  Serial.print(motorId);
  Serial.print("\"");
  Serial.print(" notes=\"");
  Serial.print(notes);
  Serial.println("\"");
}

// -------- DISPATCH (STRICT / NO SIDE EFFECTS ON ERR) --------
void handleCliLine(char *line)
{
  char *argv[CLI_ARGV_MAX];
  int argc = tokenize(line, argv, CLI_ARGV_MAX);
  if (argc == -1)
  {
    Serial.println("ERR too_many_args");
    return;
  }
  if (argc == -2)
  {
    Serial.println("ERR missing_quote");
    return;
  }
  if (argc == 0)
    return;

  if (streqi(argv[0], "HELP"))
  {
    if (argc == 1)
    {
      helpMain();
      return;
    }
    if (streqi(argv[1], "MOTOR"))
    {
      helpMotor();
      return;
    }
    if (streqi(argv[1], "SWEEP"))
    {
      helpSweep();
      return;
    }
    if (streqi(argv[1], "LOG"))
    {
      helpLog();
      return;
    }
    if (streqi(argv[1], "ESC"))
    {
      helpEsc();
      return;
    }
    Serial.println("ERR help_topic");
    return;
  }

  if (streqi(argv[0], "STATUS"))
  {
    if (argc == 1)
    {
      printStatus();
      Serial.println("OK");
      return;
    }
    if (argc == 2 && streqi(argv[1], "MOTOR"))
    {
      printMotorStatus();
      Serial.println("OK");
      return;
    }
    if (argc == 2 && streqi(argv[1], "ESC"))
    {
      printEscStatus();
      Serial.println("OK");
      return;
    }
    Serial.println("ERR status_syntax");
    return;
  }

  // LOG (STRICT; no flags; no side effects on ERR)
  if (streqi(argv[0], "LOG"))
  {
    if (argc != 2)
    {
      Serial.println("ERR log_syntax");
      return;
    }
    if (isFlag(argv[1]))
    {
      Serial.println("ERR log_syntax");
      return;
    }

    if (streqi(argv[1], "START"))
    {
      (void)startLogging();
      return;
    }
    if (streqi(argv[1], "STOP"))
    {
      stopLogging();
      return;
    }
    Serial.println("ERR log_syntax");
    return;
  }

  if (streqi(argv[0], "BASE"))
  {
    if (argc < 2 || !streqi(argv[1], "CAPTURE"))
    {
      Serial.println("ERR base_syntax");
      return;
    }
    const char *allowed[] = {"--windows", "-w"};
    if (!checkNoUnknownFlags(argc, argv, allowed, 2))
      return;

    bool ok = true;
    int windows = BASELINE_WINDOWS_DEFAULT;
    const char *wv = optValue(argc, argv, "-w", "--windows", ok);
    if (!ok)
    {
      Serial.println("ERR windows_requires_value");
      return;
    }
    if (wv)
    {
      if (!parseInt(wv, windows))
      {
        Serial.println("ERR windows_not_int");
        return;
      }
    }
    startBaselineCapture(windows);
    return;
  }

  if (streqi(argv[0], "SET"))
  {
    if (argc < 3)
    {
      Serial.println("ERR set_syntax");
      return;
    }
    if (streqi(argv[1], "MOTOR"))
    {
      strncpy(motorId, argv[2], sizeof(motorId) - 1);
      motorId[sizeof(motorId) - 1] = '\0';
      Serial.println("OK");
      statusBar("FIELD UPDATED", C_OK);
      return;
    }
    if (streqi(argv[1], "NOTES"))
    {
      strncpy(notes, argv[2], sizeof(notes) - 1);
      notes[sizeof(notes) - 1] = '\0';
      Serial.println("OK");
      statusBar("FIELD UPDATED", C_OK);
      return;
    }
    Serial.println("ERR set_target");
    return;
  }

  // ESC commands
  if (streqi(argv[0], "ESC"))
  {
    if (argc < 2)
    {
      Serial.println("ERR esc_syntax");
      return;
    }

    if (streqi(argv[1], "SET"))
    {
      const char *allowed[] = {"--freq", "-f"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 2))
        return;

      bool ok = true;
      const char *fv = optValue(argc, argv, "-f", "--freq", ok);
      if (!ok)
      {
        Serial.println("ERR freq_requires_value");
        return;
      }
      if (!fv)
      {
        Serial.println("ERR esc_set_requires_freq");
        return;
      }

      int hz;
      if (!parseInt(fv, hz))
      {
        Serial.println("ERR freq_not_int");
        return;
      }
      (void)escSetFrequency((uint32_t)hz);
      return;
    }

    if (streqi(argv[1], "CALIBRATE"))
    {
      if (argc == 2)
      {
        escCalStart();
        return;
      }
      if (argc == 3 && streqi(argv[2], "STOP"))
      {
        escCalStop();
        return;
      }
      Serial.println("ERR esc_cal_syntax");
      return;
    }

    Serial.println("ERR esc_subcommand");
    return;
  }

  // MOTOR commands
  if (streqi(argv[0], "MOTOR"))
  {
    if (argc < 2)
    {
      Serial.println("ERR motor_syntax");
      return;
    }

    if (streqi(argv[1], "ARM"))
    {
      if (argc != 2)
      {
        Serial.println("ERR motor_arm_syntax");
        return;
      }
      if (escCalState != ESC_CAL_OFF)
      {
        Serial.println("ERR esc_cal_active");
        return;
      }
      motorArm();
      return;
    }
    if (streqi(argv[1], "DISARM"))
    {
      if (argc != 2)
      {
        Serial.println("ERR motor_disarm_syntax");
        return;
      }
      motorDisarm("MOTOR DISARM");
      return;
    }
    if (streqi(argv[1], "ESTOP"))
    {
      if (argc != 2)
      {
        Serial.println("ERR motor_estop_syntax");
        return;
      }
      motorDisarm("E-STOP");
      return;
    }

    if (streqi(argv[1], "SET"))
    {
      const char *allowed[] = {"--thr", "-t", "--us", "-u"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 4))
        return;

      bool okT = true, okU = true;
      const char *tv = optValue(argc, argv, "-t", "--thr", okT);
      const char *uv = optValue(argc, argv, "-u", "--us", okU);
      if (!okT)
      {
        Serial.println("ERR thr_requires_value");
        return;
      }
      if (!okU)
      {
        Serial.println("ERR us_requires_value");
        return;
      }

      if ((tv && uv) || (!tv && !uv))
      {
        Serial.println("ERR motor_set_requires_thr_or_us");
        return;
      }
      if (!motorArmed)
      {
        Serial.println("ERR not_armed");
        statusBar("ARM FIRST", C_WARN);
        return;
      }

      if (tv)
      {
        float t;
        if (!parseFloat(tv, t))
        {
          Serial.println("ERR thr_not_float");
          return;
        }
        thrTarget = clamp01(t);
        motorMode = MOTOR_MANUAL;
        manualTimerActive = false;
        Serial.println("OK");
        return;
      }

      if (uv)
      {
        int us;
        if (!parseInt(uv, us))
        {
          Serial.println("ERR us_not_int");
          return;
        }
        us = clampInt(us, ESC_MIN_US, ESC_MAX_US);
        thrTarget = clamp01((float)(us - ESC_MIN_US) / (float)(ESC_MAX_US - ESC_MIN_US));
        motorMode = MOTOR_MANUAL;
        manualTimerActive = false;
        Serial.println("OK");
        return;
      }
    }

    if (streqi(argv[1], "MANUAL"))
    {
      const char *allowed[] = {"--thr", "-t", "--sec", "-s", "--no-log"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 5))
        return;

      bool okT = true, okS = true;
      const char *tv = optValue(argc, argv, "-t", "--thr", okT);
      const char *sv = optValue(argc, argv, "-s", "--sec", okS);
      bool noLog = hasFlag(argc, argv, nullptr, "--no-log");

      if (!okT)
      {
        Serial.println("ERR thr_requires_value");
        return;
      }
      if (!okS)
      {
        Serial.println("ERR sec_requires_value");
        return;
      }
      if (!tv || !sv)
      {
        Serial.println("ERR manual_requires_thr_and_sec");
        return;
      }

      float t, s;
      if (!parseFloat(tv, t))
      {
        Serial.println("ERR thr_not_float");
        return;
      }
      if (!parseFloat(sv, s))
      {
        Serial.println("ERR sec_not_float");
        return;
      }

      startManualTimed(t, s, noLog);
      return;
    }

    Serial.println("ERR motor_subcommand");
    return;
  }

  // SWEEP commands
  if (streqi(argv[0], "SWEEP"))
  {
    if (argc < 2)
    {
      Serial.println("ERR sweep_syntax");
      return;
    }

    if (streqi(argv[1], "STOP"))
    {
      if (argc != 2)
      {
        Serial.println("ERR sweep_stop_syntax");
        return;
      }
      sweepStop();
      return;
    }

    if (streqi(argv[1], "STEP"))
    {
      const char *allowed[] = {"--start", "-a", "--stop", "-b", "--step", "-p", "--hold", "-h", "--ramp", "-r"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 10))
        return;

      bool ok = true;
      const char *sa = optValue(argc, argv, "-a", "--start", ok);
      if (!ok)
      {
        Serial.println("ERR start_requires_value");
        return;
      }
      const char *sb = optValue(argc, argv, "-b", "--stop", ok);
      if (!ok)
      {
        Serial.println("ERR stop_requires_value");
        return;
      }
      const char *sp = optValue(argc, argv, "-p", "--step", ok);
      if (!ok)
      {
        Serial.println("ERR step_requires_value");
        return;
      }
      const char *sh = optValue(argc, argv, "-h", "--hold", ok);
      if (!ok)
      {
        Serial.println("ERR hold_requires_value");
        return;
      }
      const char *sr = optValue(argc, argv, "-r", "--ramp", ok);
      if (!ok)
      {
        Serial.println("ERR ramp_requires_value");
        return;
      }

      if (!sa || !sb || !sp || !sh)
      {
        Serial.println("ERR sweep_step_requires_start_stop_step_hold");
        return;
      }

      float a, b, c, hold, ramp;
      if (!parseFloat(sa, a) || !parseFloat(sb, b) || !parseFloat(sp, c) || !parseFloat(sh, hold))
      {
        Serial.println("ERR sweep_step_bad_number");
        return;
      }
      if (sr)
      {
        if (!parseFloat(sr, ramp))
        {
          Serial.println("ERR ramp_not_float");
          return;
        }
      }
      else
        ramp = rampRate;

      sweepStartStep(a, b, c, hold, ramp);
      return;
    }

    if (streqi(argv[1], "RAMP"))
    {
      const char *allowed[] = {"--start", "-a", "--stop", "-b", "--dur", "-d"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 6))
        return;

      bool ok = true;
      const char *sa = optValue(argc, argv, "-a", "--start", ok);
      if (!ok)
      {
        Serial.println("ERR start_requires_value");
        return;
      }
      const char *sb = optValue(argc, argv, "-b", "--stop", ok);
      if (!ok)
      {
        Serial.println("ERR stop_requires_value");
        return;
      }
      const char *sd = optValue(argc, argv, "-d", "--dur", ok);
      if (!ok)
      {
        Serial.println("ERR dur_requires_value");
        return;
      }

      if (!sa || !sb || !sd)
      {
        Serial.println("ERR sweep_ramp_requires_start_stop_dur");
        return;
      }

      float a, b, dur;
      if (!parseFloat(sa, a) || !parseFloat(sb, b) || !parseFloat(sd, dur))
      {
        Serial.println("ERR sweep_ramp_bad_number");
        return;
      }
      sweepStartRamp(a, b, dur);
      return;
    }

    if (streqi(argv[1], "TRI"))
    {
      const char *allowed[] = {"--min", "-m", "--max", "-x", "--period", "-p", "--cycles", "-c"};
      if (!checkNoUnknownFlags(argc, argv, allowed, 8))
        return;

      bool ok = true;
      const char *smin = optValue(argc, argv, "-m", "--min", ok);
      if (!ok)
      {
        Serial.println("ERR min_requires_value");
        return;
      }
      const char *smax = optValue(argc, argv, "-x", "--max", ok);
      if (!ok)
      {
        Serial.println("ERR max_requires_value");
        return;
      }
      const char *sper = optValue(argc, argv, "-p", "--period", ok);
      if (!ok)
      {
        Serial.println("ERR period_requires_value");
        return;
      }
      const char *scyc = optValue(argc, argv, "-c", "--cycles", ok);
      if (!ok)
      {
        Serial.println("ERR cycles_requires_value");
        return;
      }

      if (!smin || !smax || !sper || !scyc)
      {
        Serial.println("ERR sweep_tri_requires_min_max_period_cycles");
        return;
      }

      float a, b, per;
      int cyc;
      if (!parseFloat(smin, a) || !parseFloat(smax, b) || !parseFloat(sper, per) || !parseInt(scyc, cyc))
      {
        Serial.println("ERR sweep_tri_bad_number");
        return;
      }
      sweepStartTri(a, b, per, cyc);
      return;
    }

    Serial.println("ERR sweep_subcommand");
    return;
  }

  Serial.println("ERR unknown_command");
}

void pollCli()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r')
      continue;

    if (c == '\n')
    {
      cliLine[cliLen] = '\0';
      cliLen = 0;
      handleCliLine(cliLine);
      continue;
    }

    if (cliLen < CLI_LINE_MAX - 1)
    {
      cliLine[cliLen++] = c;
    }
    else
    {
      cliLen = 0;
      Serial.println("ERR line_too_long");
    }
  }
}

// ----------------- SETUP -----------------
void setup()
{
  Serial.begin(115200);

  pinMode(SD_CS, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TFT_CS);

  tft.begin();
  tft.setRotation(3);
  tft.setTextWrap(false);
  tft.fillScreen(C_BG);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // WHO_AM_I
  uint8_t w = 0;
  if (readRegs(0x75, &w, 1))
    whoami = w;

  // IMU config
  writeReg(0x6B, 0x00);
  delay(50);
  writeReg(0x1C, 0x10); // accel +/-8g
  writeReg(0x1B, 0x08); // gyro +/-500 dps

  // SD init
  sdOk = SD.begin(SD_CS, SPI, 25000000);
  if (sdOk)
    runNumber = findNextRunNumber();

  // LEDC init (legacy API for widest compatibility)
  ledcSetup(ESC_LEDC_CH, ESC_FREQ_HZ_DEFAULT, ESC_LEDC_RES_BITS);
  ledcAttachPin(ESC_PIN, ESC_LEDC_CH);
  bool ledcOk = true;

  escFreqHz = ESC_FREQ_HZ_DEFAULT;
  escWriteUs(ESC_MIN_US);

  escFreqHz = ESC_FREQ_HZ_DEFAULT;
  escWriteUs(ESC_MIN_US);

  drawStaticUI();

  Serial.println();
  Serial.println("READY (STRICT CLI)");
  helpMain();
  Serial.println();
}

// ----------------- LOOP -----------------
void loop()
{
  static uint32_t ns = micros();
  static uint32_t nu = millis();
  static uint32_t nl = millis();
  static uint32_t nm = millis();

  // ---- IMU sampling @ 250 Hz ----
  if ((int32_t)(micros() - ns) >= 0)
  {
    ns += SAMPLE_US;

    uint8_t d[14];
    if (readRegs(0x3B, d, 14))
    {
      int16_t rax = be16(&d[0]);
      int16_t ray = be16(&d[2]);
      int16_t raz = be16(&d[4]);

      ax = (rax / 4096.0f) * 9.80665f;
      ay = (ray / 4096.0f) * 9.80665f;
      az = (raz / 4096.0f) * 9.80665f;

      gx += ALPHA_G * (ax - gx);
      gy += ALPHA_G * (ay - gy);
      gz += ALPHA_G * (az - gz);

      dx = ax - gx;
      dy = ay - gy;
      dz = az - gz;

      vibInst = sqrtf(dx * dx + dy * dy + dz * dz);

      sumSq += (double)vibInst * (double)vibInst;
      if (++rmsCount >= RMS_N)
      {
        vibRms = (float)sqrt(sumSq / (double)RMS_N);
        sumSq = 0.0;
        rmsCount = 0;
        vibRmsUpdated = true;
      }
    }
  }

  if (vibRmsUpdated)
  {
    vibRmsUpdated = false;
    onNewVibRmsComputed();
  }

  // ---- Motor update @ 50 Hz ----
  if ((int32_t)(millis() - nm) >= 0)
  {
    nm += MOTOR_MS;
    motorUpdateTick(millis());
  }

  // ---- UI refresh @ 10 Hz ----
  if ((int32_t)(millis() - nu) >= 0)
  {
    nu += UI_MS;

    static float ax_d = 999, ay_d = 999, az_d = 999, vi_d = 999, vr_d = 999;
    static bool sdOk_d = 2;
    static uint32_t run_d = 0xFFFFFFFF;
    static int escus_d = -9999;
    static int rpm_d = -9999;
    static MotorMode mode_d = (MotorMode)255;
    static float thr_d = 999;
    static char motor_d[48] = {0};
    static char notes_d[96] = {0};
    static char stat_d[160] = {0};

    const int vx1 = COL1_X + VALUE_XOFF;
    const int vx2 = COL2_X + VALUE_XOFF;
    const int vy0 = BODY_Y0;

    const int leftValueW = (COL1_X + COL_W) - vx1;
    const int valueH = ROW_H;
    const int rightValueWChars = 10;

    char buf[24];

    // left values (bounded)
    if (fabsf(ax - ax_d) > AX_EPS)
    {
      ax_d = ax;
      clearValueRect2(vx1, vy0 + 0 * ROW_H, leftValueW, valueH);
      snprintf(buf, sizeof(buf), "%.2f", ax_d);
      drawValueText2(vx1, vy0 + 0 * ROW_H, buf, C_VALUE);
    }
    if (fabsf(ay - ay_d) > AX_EPS)
    {
      ay_d = ay;
      clearValueRect2(vx1, vy0 + 1 * ROW_H, leftValueW, valueH);
      snprintf(buf, sizeof(buf), "%.2f", ay_d);
      drawValueText2(vx1, vy0 + 1 * ROW_H, buf, C_VALUE);
    }
    if (fabsf(az - az_d) > AX_EPS)
    {
      az_d = az;
      clearValueRect2(vx1, vy0 + 2 * ROW_H, leftValueW, valueH);
      snprintf(buf, sizeof(buf), "%.2f", az_d);
      drawValueText2(vx1, vy0 + 2 * ROW_H, buf, C_VALUE);
    }
    if (fabsf(vibInst - vi_d) > VIB_EPS)
    {
      vi_d = vibInst;
      clearValueRect2(vx1, vy0 + 3 * ROW_H, leftValueW, valueH);
      snprintf(buf, sizeof(buf), "%.3f", vi_d);
      drawValueText2(vx1, vy0 + 3 * ROW_H, buf, C_VALUE);
    }
    if (fabsf(vibRms - vr_d) > VIB_EPS)
    {
      vr_d = vibRms;
      clearValueRect2(vx1, vy0 + 4 * ROW_H, leftValueW, valueH);
      snprintf(buf, sizeof(buf), "%.3f", vr_d);
      drawValueText2(vx1, vy0 + 4 * ROW_H, buf, C_VALUE);
    }

    // right values
    if (sdOk != sdOk_d)
    {
      sdOk_d = sdOk;
      drawValueText2Fixed(vx2, vy0 + 0 * ROW_H, sdOk ? "OK" : "FAIL", 6, sdOk ? C_OK : C_BAD);
    }
    if (runNumber != run_d)
    {
      run_d = runNumber;
      char rbuf[10];
      snprintf(rbuf, sizeof(rbuf), "%04lu", (unsigned long)runNumber);
      drawValueText2Fixed(vx2, vy0 + 1 * ROW_H, rbuf, 6, C_VALUE);
    }
    if (motorMode != mode_d)
    {
      mode_d = motorMode;
      char mbuf[14];
      if (escCalState != ESC_CAL_OFF)
      {
        snprintf(mbuf, sizeof(mbuf), "C %s", escCalName(escCalState));
      }
      else
      {
        snprintf(mbuf, sizeof(mbuf), "%c %s", motorArmed ? 'A' : 'D', motorModeName(motorMode));
      }
      drawValueText2Fixed(vx2, vy0 + 2 * ROW_H, mbuf, rightValueWChars,
                          (escCalState != ESC_CAL_OFF) ? C_WARN : (motorArmed ? C_OK : C_WARN));
    }

    int esc_us = currentEscUs();
    if (esc_us != escus_d || fabsf(thrCmd - thr_d) > THR_EPS)
    {
      escus_d = esc_us;
      thr_d = thrCmd;
      char ubuf[14];
      snprintf(ubuf, sizeof(ubuf), "%d", esc_us);
      drawValueText2Fixed(vx2, vy0 + 3 * ROW_H, ubuf, rightValueWChars, C_VALUE);
    }

    if (rpmValue != rpm_d)
    {
      rpm_d = rpmValue;
      char rbuf2[14];
      if (rpmValue < 0)
        snprintf(rbuf2, sizeof(rbuf2), "N/A");
      else
        snprintf(rbuf2, sizeof(rbuf2), "%ld", (long)rpmValue);
      drawValueText2Fixed(vx2, vy0 + 4 * ROW_H, rbuf2, rightValueWChars, C_VALUE);
    }

    // meta
    if (strcmp(motorId, motor_d) != 0)
    {
      strncpy(motor_d, motorId, sizeof(motor_d) - 1);
      motor_d[sizeof(motor_d) - 1] = '\0';
      drawMetaLine1Fixed(META_Y0 + 0 * META_LINE + META_PAD, "MOTOR:", motor_d, 43);
    }
    if (strcmp(notes, notes_d) != 0)
    {
      strncpy(notes_d, notes, sizeof(notes_d) - 1);
      notes_d[sizeof(notes_d) - 1] = '\0';
      drawMetaLine1Fixed(META_Y0 + 1 * META_LINE + META_PAD, "NOTES:", notes_d, 43);
    }

    char stat[160];
    if (baselineValid)
    {
      snprintf(stat, sizeof(stat), "BL:%.3f NET:%.3f LOG:%s THR:%.2f F:%luHz",
               vibBaseline, vibNet, loggingEnabled ? "ON" : "OFF", thrCmd, (unsigned long)escFreqHz);
    }
    else
    {
      snprintf(stat, sizeof(stat), "BL:---- NET:---- LOG:%s THR:%.2f F:%luHz",
               loggingEnabled ? "ON" : "OFF", thrCmd, (unsigned long)escFreqHz);
    }
    if (strcmp(stat, stat_d) != 0)
    {
      strncpy(stat_d, stat, sizeof(stat_d) - 1);
      stat_d[sizeof(stat_d) - 1] = '\0';
      drawMetaLine1Fixed(META_Y0 + 2 * META_LINE + META_PAD, "STAT:", stat_d, 43);
    }
  }

  // ---- logging @ 50 Hz ----
  if ((int32_t)(millis() - nl) >= 0)
  {
    nl += LOG_MS;

    if (loggingEnabled && logFile)
    {
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      int esc_us = currentEscUs();

      char line[280];
      int n = snprintf(line, sizeof(line),
                       "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%ld\n",
                       (unsigned long)millis(),
                       ax, ay, az,
                       mag,
                       vibInst,
                       vibRms,
                       vibNet,
                       thrCmd,
                       esc_us,
                       (long)rpmValue);

      digitalWrite(TFT_CS, HIGH);
      digitalWrite(SD_CS, LOW);
      logFile.write((const uint8_t *)line, (size_t)n);
      digitalWrite(SD_CS, HIGH);
      digitalWrite(TFT_CS, LOW);

      static uint16_t flushCtr = 0;
      if (++flushCtr >= (1000 / LOG_MS))
      {
        digitalWrite(TFT_CS, HIGH);
        digitalWrite(SD_CS, LOW);
        logFile.flush();
        digitalWrite(SD_CS, HIGH);
        digitalWrite(TFT_CS, LOW);
        flushCtr = 0;
      }
    }
  }

  // ---- CLI ----
  pollCli();
}
