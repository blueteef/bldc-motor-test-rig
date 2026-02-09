#pragma once
#include <Arduino.h>

// Flags you can log / show in UI
enum RigSensorFlags : uint32_t {
  RIG_SENS_OK            = 0,
  RIG_SENS_TACH_MISSING  = (1u << 0),
  RIG_SENS_POWER_MISSING = (1u << 1),
};

struct RigSensorSnapshot {
  uint32_t ms = 0;

  // Tach
  float rpm = 0.0f;
  float tach_hz = 0.0f;

  // Power (INA226)
  float vin_v = 0.0f;      // bus voltage
  float iin_a = 0.0f;      // current
  float pin_w = 0.0f;      // power
  float energy_wh = 0.0f;  // integrated energy

  uint32_t flags = RIG_SENS_OK;
};

bool rigSensorsBegin();
void rigSensorsUpdate();

// Access latest stable snapshot
const RigSensorSnapshot& rigSensorsGet();
