#include "ina226.h"
#include <Wire.h>

static inline int16_t s16(uint16_t x) { return (int16_t)x; }

// INA226 registers
static constexpr uint8_t REG_CONFIG   = 0x00;
static constexpr uint8_t REG_SHUNT_V  = 0x01;
static constexpr uint8_t REG_BUS_V    = 0x02;

// LSBs per datasheet
static constexpr float BUS_LSB_V   = 0.00125f;     // 1.25 mV/bit
static constexpr float SHUNT_LSB_V = 0.0000025f;   // 2.5 uV/bit

bool Ina226::begin(uint8_t addr, float shunt_ohms, int sda, int scl, uint32_t i2c_hz) {
  i2c_addr = addr;
  r_shunt = (shunt_ohms > 0.0f) ? shunt_ohms : 0.00375f;

  if (sda >= 0 && scl >= 0) {
    Wire.begin(sda, scl);
  } else {
    Wire.begin();
  }
  Wire.setClock(i2c_hz);

  // Basic presence check: read config
  uint16_t cfg = 0;
  if (!read16(REG_CONFIG, cfg)) return false;

  // Configure sane defaults (averaging/conv times/mode). Even if this fails,
  // raw reads still work; so we don't hard-fail on config write.
  (void)configureDefaults();

  energy_wh = 0.0f;
  last_ms = millis();
  r = Ina226Reading{};
  return true;
}

bool Ina226::configureDefaults() {
  // CONFIG bits: AVG[11:9], VBUSCT[8:6], VSHCT[5:3], MODE[2:0]
  // AVG=16 (0b100), CT=1.1ms (0b100), MODE=111 (shunt+bus continuous)
  uint16_t cfg =
      (0b100 << 9) |   // AVG=16
      (0b100 << 6) |   // VBUSCT=1.1ms
      (0b100 << 3) |   // VSHCT=1.1ms
      (0b111);         // MODE=continuous shunt+bus

  return write16(REG_CONFIG, cfg);
}

bool Ina226::update() {
  Ina226Reading out{};
  out.ms = millis();

  uint16_t raw_bus = 0;
  uint16_t raw_shunt = 0;

  if (!read16(REG_BUS_V, raw_bus)) { r = out; return false; }
  if (!read16(REG_SHUNT_V, raw_shunt)) { r = out; return false; }

  out.bus_v = (float)raw_bus * BUS_LSB_V;
  out.shunt_v = (float)s16(raw_shunt) * SHUNT_LSB_V;

  // Current from shunt voltage (works immediately, no calibration register needed)
  if (r_shunt > 0.0f) out.current_a = out.shunt_v / r_shunt;

  // Power computed (you can later switch to INA226 POWER reg if you want)
  out.power_w = out.bus_v * out.current_a;

  // Integrate energy (Wh)
  const uint32_t now = out.ms;
  const float dt_h = (now - last_ms) / 3600000.0f;
  if (dt_h > 0.0f && dt_h < 1.0f) energy_wh += out.power_w * dt_h;
  out.energy_wh = energy_wh;
  last_ms = now;

  out.ok = true;
  r = out;
  return true;
}

bool Ina226::write16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write((uint8_t)(v >> 8));
  Wire.write((uint8_t)(v & 0xFF));
  return (Wire.endTransmission() == 0);
}

bool Ina226::read16(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t n = Wire.requestFrom((int)i2c_addr, 2);
  if (n != 2) return false;

  const uint16_t hi = (uint16_t)Wire.read();
  const uint16_t lo = (uint16_t)Wire.read();
  v = (hi << 8) | lo;
  return true;
}
