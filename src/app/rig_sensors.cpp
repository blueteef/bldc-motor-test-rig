#include "app/rig_sensors.h"
#include "sensors/tach/tach_pcnt.h"

// ---- Configuration ----
// TCRT5000 DO -> GPIO 27, 1 reflective mark => 1 pulse per rev
static constexpr int TACH_GPIO = 27;
static constexpr int TACH_PPR  = 1;

// Behavior tuning
static constexpr uint32_t TACH_WINDOW_MS    = 250;  // update period
static constexpr uint32_t TACH_MIN_PULSE_US = 500;  // chatter/glitch reject
static constexpr uint32_t TACH_TIMEOUT_MS   = 1000; // no pulses -> missing

static TachPcnt gTach;
static RigSensorSnapshot gSnap;

bool rigSensorsBegin() {
  gTach.setWindowMs(TACH_WINDOW_MS);
  gTach.setMinPulseUs(TACH_MIN_PULSE_US);

  if (!gTach.begin(TACH_GPIO, TACH_PPR)) {
    // If PCNT init fails, we still "run" but flag tach missing forever
    gSnap.flags |= RIG_SENS_TACH_MISSING;
    return false;
  }
  return true;
}

void rigSensorsUpdate() {
  gTach.update();

  const uint32_t now = millis();
  gSnap.ms = now;

  gSnap.rpm = gTach.rpm();
  gSnap.tach_hz = gTach.hz();

  // Missing pulse detection (useful for UI + logging)
  const uint32_t lastPulse = gTach.lastPulseSeenMs();
  if (lastPulse == 0 || (now - lastPulse) > TACH_TIMEOUT_MS) {
    gSnap.flags |= RIG_SENS_TACH_MISSING;
    gSnap.rpm = 0.0f;
    gSnap.tach_hz = 0.0f;
  } else {
    gSnap.flags &= ~RIG_SENS_TACH_MISSING;
  }
}

const RigSensorSnapshot& rigSensorsGet() {
  return gSnap;
}
