#include "app/rig_sensors.h"

#include "sensors/tach/tach_pcnt.h"
#include "sensors/power/ina226.h"

// ---- Configuration ----

// Tach: TCRT5000 DO -> GPIO 27, 1 reflective mark => 1 pulse per rev
static constexpr int TACH_GPIO = 27;
static constexpr int TACH_PPR  = 1;

// Tach behavior tuning
static constexpr uint32_t TACH_WINDOW_MS    = 250;
static constexpr uint32_t TACH_MIN_PULSE_US = 500;
static constexpr uint32_t TACH_TIMEOUT_MS   = 1000;

// Power: INA226 I2C address
static constexpr uint8_t INA226_ADDR = 0x40;

// Your shunt: 20A, 75mV => 0.00375 ohms
static constexpr float INA226_SHUNT_OHMS = 0.00375f;

// Power timeout behavior (optional)
static constexpr uint32_t POWER_TIMEOUT_MS = 1000;

static TachPcnt gTach;
static Ina226   gPower;
static RigSensorSnapshot gSnap;

// Track last good power update
static uint32_t gLastPowerOkMs = 0;

bool rigSensorsBegin() {
  // Tach init
  gTach.setWindowMs(TACH_WINDOW_MS);
  gTach.setMinPulseUs(TACH_MIN_PULSE_US);

  if (!gTach.begin(TACH_GPIO, TACH_PPR)) {
    gSnap.flags |= RIG_SENS_TACH_MISSING;
  }

  // Power init (INA226 on default I2C pins)
  if (!gPower.begin(INA226_ADDR, INA226_SHUNT_OHMS)) {
    gSnap.flags |= RIG_SENS_POWER_MISSING;
  }

  gLastPowerOkMs = 0;
  return true;
}

void rigSensorsUpdate() {
  const uint32_t now = millis();
  gSnap.ms = now;

  // ---- Tach ----
  gTach.update();
  gSnap.rpm = gTach.rpm();
  gSnap.tach_hz = gTach.hz();

  const uint32_t lastPulse = gTach.lastPulseSeenMs();
  if (lastPulse == 0 || (now - lastPulse) > TACH_TIMEOUT_MS) {
    gSnap.flags |= RIG_SENS_TACH_MISSING;
    gSnap.rpm = 0.0f;
    gSnap.tach_hz = 0.0f;
  } else {
    gSnap.flags &= ~RIG_SENS_TACH_MISSING;
  }

  // ---- Power (INA226) ----
  if (gPower.update()) {
    const auto& p = gPower.get();
    if (p.ok) {
      gSnap.vin_v = p.bus_v;
      gSnap.iin_a = p.current_a;
      gSnap.pin_w = p.power_w;
      gSnap.energy_wh = p.energy_wh;

      gLastPowerOkMs = now;
      gSnap.flags &= ~RIG_SENS_POWER_MISSING;
    }
  }

  // If power hasn’t updated recently, mark missing (but keep last values)
  if (gLastPowerOkMs == 0 || (now - gLastPowerOkMs) > POWER_TIMEOUT_MS) {
    gSnap.flags |= RIG_SENS_POWER_MISSING;
  }
}

const RigSensorSnapshot& rigSensorsGet() {
  return gSnap;
}
