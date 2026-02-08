# trotten-desk-motor
ESP32 firmware to control an IKEA Trotten desk motor using a BTS7960 driver and a VL53L1X time‑of‑flight sensor for height feedback.

## Features
- Manual up/down control with smooth PWM ramp.
- Two height presets with short‑press toggle.
- Safety limits (min/max height) and measurement error watchdog.
- Sensor and motor driver disabled when idle to reduce noise and power use.

## Hardware
- ESP32 dev board
- BTS7960 motor driver
- VL53L1X time‑of‑flight distance sensor (Pololu library included in lib/vl53l1x)
- 3 push buttons (active‑LOW, using internal pull‑ups)

## Wiring
====================== ESP32 ======================

        GPIO21  -------------------- SDA  VL53L1X
        GPIO22  -------------------- SCL  VL53L1X

        GPIO18  -------------------- R_EN  BTS7960
        GPIO19  -------------------- L_EN  BTS7960
        GPIO32  -------------------- RPWM  BTS7960
        GPIO33  -------------------- LPWM  BTS7960

        GPIO27  ----[   BUTTON UP   ]--.
        GPIO14  ----[  BUTTON DOWN  ]--+---- (shorted to GND when pressed)
        GPIO26  ----[ BUTTON PRESET ]--'

====================================================

## Behavior
- Holding DOWN moves the desk down; holding UP moves it up.
- Motor ramps from a minimum kick PWM to full speed over 3 seconds.
- Short press on PRESET toggles between two target heights and moves automatically.
- Any button press during preset movement cancels the automatic move.
- Movement stops if the measured height exceeds limits or too many sensor errors occur.

## Configuration
All key parameters are in src/main.cpp:
- Height limits: MIN_HEIGHT_CM, MAX_HEIGHT_CM
- Presets: PRESET_LOW_CM, PRESET_HIGH_CM, PRESET_TOL_CM
- PWM: PWM_FREQ, PWM_RES, PWM_MIN_START, RAMP_MS
- Debounce and press timing: DEBOUNCE_MS, SHORT_PRESS_MIN_MS, SHORT_PRESS_MAX_MS

## Build and Upload (PlatformIO)
This project uses the Arduino framework with the esp32dev target.

Typical commands:
- Build: platformio run
- Upload: platformio run -t upload
- Monitor: platformio device monitor

## Notes
- The VL53L1X library is vendored in lib/vl53l1x.
- Height conversion and calibration offset are implemented in src/main.cpp; adjust if your sensor mounting differs.
