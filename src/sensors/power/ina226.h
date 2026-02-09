#pragma once
#include <Arduino.h>

struct Ina226Reading {
  uint32_t ms = 0;

  float bus_v = 0.0f;     // volts (VBUS)
  float shunt_v = 0.0f;   // volts (VSHUNT, signed)
  float current_a = 0.0f; // amps
  float power_w = 0.0f;   // watts (computed)
  float energy_wh = 0.0f; // watt-hours integrated

  bool ok = false;
};

class Ina226 {
public:
  // addr: I2C address (0x40)
  // shunt_ohms: shunt resistance in ohms (your 20A/75mV => 0.00375)
  // sda/scl: optional if you use non-default I2C pins; pass -1 to use defaults
  bool begin(uint8_t addr, float shunt_ohms, int sda = -1, int scl = -1, uint32_t i2c_hz = 400000);

  // call at ~10–50 Hz
  bool update();

  const Ina226Reading& get() const { return r; }

  // If you later want to use INA226 current/power regs (calibrated),
  // set these before begin() and I can give you the calibrated path.
  void setExpectedMaxCurrentA(float a) { expected_max_a = a; }

private:
  uint8_t i2c_addr = 0x40;
  float r_shunt = 0.00375f;

  // Optional: used only if you later enable calibration mode
  float expected_max_a = 50.0f;

  uint32_t last_ms = 0;
  float energy_wh = 0.0f;

  Ina226Reading r;

  bool read16(uint8_t reg, uint16_t &v);
  bool write16(uint8_t reg, uint16_t v);

  bool configureDefaults();
};
