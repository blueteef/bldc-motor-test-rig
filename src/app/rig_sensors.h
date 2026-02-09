#pragma once
#include <Arduino.h>

// Flags you can log / show in UI
enum RigSensorFlags : uint32_t {
  RIG_SENS_OK           = 0,
  RIG_SENS_TACH_MISSING = (1u << 0),
};

struct RigSensorSnapshot {
  uint32_t ms = 0;

  // Tach
  float rpm = 0.0f;
  float tach_hz = 0.0f;

  // Future: power sensors, etc.
  // float vin_v = 0.0f;
  // float iin_a = 0.0f;
  // float pin_w = 0.0f;

  uint32_t flags = RIG_SENS_OK;
};

bool rigSensorsBegin();
void rigSensorsUpdate();

// Access latest stable snapshot
const RigSensorSnapshot& rigSensorsGet();
