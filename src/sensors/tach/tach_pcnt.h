#pragma once
#include <Arduino.h>

class TachPcnt {
public:
  // gpio: PCNT input pin (use GPIO 27)
  // pulsesPerRev: 1 for one reflective mark
  bool begin(int gpio, int pulsesPerRev = 1);

  // Call frequently. Internally updates RPM every windowMs.
  void update();

  float rpm() const { return _rpm; }
  float hz() const { return _hz; }

  void setWindowMs(uint32_t ms) { _windowMs = ms; }
  void setMinPulseUs(uint32_t us) { _minPulseUs = us; } // glitch filter threshold

  uint32_t lastUpdateMs() const { return _lastUpdateMs; }
  uint32_t lastPulseSeenMs() const { return _lastPulseSeenMs; }

private:
  bool _configured = false;

  int _gpio = -1;
  int _ppr = 1;

  uint32_t _windowMs = 250;
  uint32_t _minPulseUs = 500; // TCRT5000 chatter control

  uint32_t _windowStartMs = 0;
  uint32_t _lastUpdateMs = 0;
  uint32_t _lastPulseSeenMs = 0;

  float _hz = 0.0f;
  float _rpm = 0.0f;
};
