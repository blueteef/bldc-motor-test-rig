#include "tach_pcnt.h"

// ESP-IDF PCNT driver (Arduino-ESP32 exposes these headers)
#include "driver/pcnt.h"

static constexpr pcnt_unit_t  TACH_PCNT_UNIT  = PCNT_UNIT_0;
static constexpr pcnt_channel_t TACH_PCNT_CH  = PCNT_CHANNEL_0;

// We count rising edges on the pulse pin.
// Control pin unused.
static constexpr int TACH_CTRL_GPIO = PCNT_PIN_NOT_USED;

// Safe limits for the 16-bit signed PCNT counter
static constexpr int16_t PCNT_H_LIM =  32767;
static constexpr int16_t PCNT_L_LIM = -32768;

bool TachPcnt::begin(int gpio, int pulsesPerRev) {
  _gpio = gpio;
  _ppr = (pulsesPerRev <= 0) ? 1 : pulsesPerRev;

  pinMode(_gpio, INPUT_PULLUP); // DO is often open-collector; pullup helps

  // Configure PCNT
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = _gpio;
  pcnt_config.ctrl_gpio_num  = TACH_CTRL_GPIO;
  pcnt_config.unit           = TACH_PCNT_UNIT;
  pcnt_config.channel        = TACH_PCNT_CH;

  // Count on positive edge only
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;

  // No control mode
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

  pcnt_config.counter_h_lim = PCNT_H_LIM;
  pcnt_config.counter_l_lim = PCNT_L_LIM;

  if (pcnt_unit_config(&pcnt_config) != ESP_OK) return false;

  // Glitch filter: rejects pulses shorter than filter threshold
  //
  // filter_value is in APB clock cycles. APB is typically 80 MHz on ESP32.
  // cycles = us * 80
  // Example: 500 us => 40,000 cycles
  //
  // Max allowed by hardware is limited; if you set too high, it may clamp.
  uint16_t filter_cycles = 0;
  {
    const uint32_t cycles = (uint32_t)(_minPulseUs * 80UL);
    filter_cycles = (cycles > 1023U) ? 1023U : (uint16_t)cycles; // common hw cap
  }
  pcnt_set_filter_value(TACH_PCNT_UNIT, filter_cycles);
  pcnt_filter_enable(TACH_PCNT_UNIT);

  // Reset & start
  pcnt_counter_pause(TACH_PCNT_UNIT);
  pcnt_counter_clear(TACH_PCNT_UNIT);
  pcnt_counter_resume(TACH_PCNT_UNIT);

  _windowStartMs = millis();
  _lastUpdateMs = _windowStartMs;
  _lastPulseSeenMs = 0;
  _hz = 0.0f;
  _rpm = 0.0f;

  _configured = true;
  return true;
}

void TachPcnt::update() {
  if (!_configured) return;

  const uint32_t nowMs = millis();
  const uint32_t elapsed = nowMs - _windowStartMs;
  if (elapsed < _windowMs) return;

  int16_t count = 0;
  pcnt_get_counter_value(TACH_PCNT_UNIT, &count);

  // Clear for next window
  pcnt_counter_clear(TACH_PCNT_UNIT);

  const float windowSec = (elapsed > 0) ? (float)elapsed / 1000.0f : 0.0f;
  const float pulses = (float)count;
  const float hz = (windowSec > 0.0f) ? (pulses / windowSec) : 0.0f;
  const float rpm = (hz / (float)_ppr) * 60.0f;

  // pulse seen tracking (useful for timeout logic / UI)
  if (count > 0) _lastPulseSeenMs = nowMs;

  // EMA smoothing
  const float alpha = 0.25f;
  _hz  = (_hz  == 0.0f) ? hz  : (_hz  + alpha * (hz  - _hz));
  _rpm = (_rpm == 0.0f) ? rpm : (_rpm + alpha * (rpm - _rpm));

  // Timeout -> force to 0 if no pulses recently
  if (_lastPulseSeenMs != 0 && (nowMs - _lastPulseSeenMs) > 1000) {
    _hz = 0.0f;
    _rpm = 0.0f;
  }

  _lastUpdateMs = nowMs;
  _windowStartMs = nowMs;
}
