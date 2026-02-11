# BLDC Motor Test Rig

ESP32-based dynamometer / test stand for brushless DC motors. Logs vibration (IMU), RPM (tachometer), and electrical power (INA226) to SD card in CSV and JSON formats, with a live TFT display and serial CLI.

## Hardware

| Component | Part | Connection |
|---|---|---|
| MCU | ESP32 DevKit | — |
| Display | ILI9341 320x240 TFT | SPI (CS=5, DC=26, RST=4) |
| SD Card | SPI SD module | SPI (CS=13) |
| IMU | MPU-6050 / ICM-20x | I2C 0x68 (SDA=21, SCL=22) |
| Power Sensor | INA226 + 20A/75mV shunt | I2C 0x40 |
| Tachometer | TCRT5000 reflective | GPIO 27 (PCNT) |
| ESC | Standard PWM ESC | GPIO 25 (50-500 Hz) |

## Features

- 250 Hz IMU sampling, 50 Hz logging to CSV
- Live vibration RMS with baseline subtraction
- INA226 bus voltage, current, power, and integrated energy
- Tachometer RPM via hardware pulse counter
- Sweep modes: step, ramp, triangle
- JSON run metadata with full provenance (git SHA, build time)
- Run summaries in both detailed (`RUN_SUMMARY.CSV`) and short formats
- Serial CLI with GNU-style flags (`motor arm`, `sweep step --start 0.1 --stop 0.5 --step 0.1 --hold 2`)

## Build

Requires [PlatformIO](https://platformio.org/).

```
pio run          # build
pio run -t upload  # flash
pio device monitor -b 115200  # serial monitor
```

## Project Layout

```
src/app/           Application entrypoint, CLI, UI
src/logging/       CSV, JSON, run summary writers
src/sensors/power/ INA226 driver
src/sensors/tach/  PCNT tachometer driver
include/bldc_rig/  Project headers (namespaced)
scripts/           Build hooks (git SHA injection)
docs/              Architecture docs
```

## TODO

### Next Priority
- [ ] **Add HX711 load cell for thrust measurement** (10kg HX711 module)
  - Wire HX711 DAT/CLK to available GPIOs
  - Add `src/sensors/loadcell/` driver (tare, calibration, filtered reads)
  - Integrate into `RigSensorSnapshot` (thrust_g / thrust_n fields)
  - Add thrust columns to CSV/JSON logging pipeline
  - Add thrust to TFT display and run summaries
  - Calibrate with known weights

### Backlog
- [ ] Add temperature sensor (motor/ESC thermocouple or DS18B20)
- [ ] RPM P95 and standard deviation in run summaries
- [ ] Wi-Fi/BLE for wireless data export
- [ ] Web dashboard for real-time monitoring
- [ ] Configurable log rate (currently fixed 50 Hz)
- [ ] NTP time sync for absolute timestamps
