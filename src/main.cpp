/*
  ESP32 + BTS7960 + VL53L1X

  Buttons (INPUT_PULLUP, active LOW):
    UP     = GPIO 14
    DOWN   = GPIO 27
    PRESET = GPIO 26

  Motor driver (BTS7960):
    R_EN_PIN = 18
    L_EN_PIN = 19
    RPWM_PIN = 32
    LPWM_PIN = 33

  Logic:
  - When idle, motor and sensor are "off" (driver EN pins LOW, VL53L1X stopped in software).
  - Holding DOWN: move down (counter-clockwise). Stop when <= MIN_HEIGHT_CM or button released.
  - Holding UP: move up (clockwise). Stop when >= MAX_HEIGHT_CM or button released.
  - PRESET (short press): toggles target between PRESET_LOW_CM and PRESET_HIGH_CM and moves to it.
  - Any button press during preset move stops motion.
  - Smooth acceleration to full speed in RAMP_MS milliseconds (also for manual moves).
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>

// -------- Pins --------
constexpr uint8_t UP_BTN_PIN     = 27;
constexpr uint8_t DOWN_BTN_PIN   = 14;
constexpr uint8_t PRESET_BTN_PIN = 26;

constexpr uint8_t R_EN_PIN       = 18;
constexpr uint8_t L_EN_PIN       = 19;
constexpr uint8_t RPWM_PIN       = 32;
constexpr uint8_t LPWM_PIN       = 33;

// -------- Height constants (centimeters) --------
constexpr float MIN_HEIGHT_CM    = 70.0f;
constexpr float MAX_HEIGHT_CM    = 110.0f;
constexpr float PRESET_LOW_CM    = 73.0f;
constexpr float PRESET_HIGH_CM   = 99.0f;
constexpr float PRESET_TOL_CM    = 0.1f;   // tolerance for reaching preset
constexpr float PRESET_TOGGLE_TOL_CM    = 5.0f;   // tolerance for reaching preset


constexpr int MEASUREMENT_ERRORS_THRESHOLD = 50;

// -------- PWM / ramp --------
constexpr uint32_t PWM_FREQ       = 20000; // 20 kHz — less audible noise
constexpr uint8_t  PWM_RES        = 10;    // 10-bit (0..1023)
constexpr uint16_t PWM_MAX        = (1 << PWM_RES) - 1;
constexpr uint32_t RAMP_MS        = 3000;  // ramp time to full speed
constexpr uint16_t PWM_MIN_START  = 500;   // initial kick to overcome static friction

// LEDC channels
constexpr uint8_t RPWM_CH = 0;
constexpr uint8_t LPWM_CH = 1;

// -------- Button timing / debounce --------
constexpr uint32_t DEBOUNCE_MS         = 30;
constexpr uint32_t SHORT_PRESS_MIN_MS  = 40;
constexpr uint32_t SHORT_PRESS_MAX_MS  = 600;

// -------- Logging --------
constexpr uint8_t MAX_LOG_LEVEL = 0;

void logMessage(const String &message, uint8_t level = 0) {
  if (level > MAX_LOG_LEVEL) return;
  Serial.println(message);
}

void logValue(const String &label, float value, uint8_t level = 0) {
  if (level > MAX_LOG_LEVEL) return;
  Serial.print(label);
  Serial.print(": ");
  Serial.println(value);
}

// -------- ToF sensor --------
VL53L1X tof;
bool sensorActive = false;

// -------- Movement states --------
enum class MoveMode {
  IDLE,
  MANUAL_UP,
  MANUAL_DOWN,
  GOTO_PRESET
};

MoveMode mode = MoveMode::IDLE;

enum class PresetSel {
  LOW_PRESET,
  HIGH_PRESET
};

PresetSel currentPreset = PresetSel::LOW_PRESET;
float targetHeightCm = PRESET_LOW_CM;

// Current measured height
float currentHeightCm = 0.0f;
int measurementErrorsCount = 0;

// -------- Debounced button class --------
struct DebouncedButton {
  uint8_t pin;
  bool stableState;         // debounced state (true = HIGH, false = LOW)
  bool lastStableState;
  uint32_t lastChangeMs;
  uint32_t pressStartMs;
  bool shortClickFlag;

  DebouncedButton(uint8_t pin_)
    : pin(pin_),
      stableState(true),
      lastStableState(true),
      lastChangeMs(0),
      pressStartMs(0),
      shortClickFlag(false) {}

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    stableState = digitalRead(pin);
    lastStableState = stableState;
    lastChangeMs = millis();
  }

  void update(uint32_t nowMs) {
    bool raw = digitalRead(pin);

    if (raw != stableState && (nowMs - lastChangeMs) >= DEBOUNCE_MS) {
      lastStableState = stableState;
      stableState = raw;
      lastChangeMs = nowMs;

      // Detect press and release with duration
      if (stableState == LOW && lastStableState == HIGH) {
        // Button just pressed
        pressStartMs = nowMs;
      } else if (stableState == HIGH && lastStableState == LOW) {
        // Button just released
        uint32_t pressDuration = nowMs - pressStartMs;
        if (pressDuration >= SHORT_PRESS_MIN_MS && pressDuration <= SHORT_PRESS_MAX_MS) {
          shortClickFlag = true;
        }
      }
    }
  }

  bool isPressed() const {
    return stableState == LOW; // active LOW
  }

  bool wasShortClicked() {
    if (shortClickFlag) {
      shortClickFlag = false;
      return true;
    }
    return false;
  }
};

// -------- Buttons --------
DebouncedButton btnUp(UP_BTN_PIN);
DebouncedButton btnDown(DOWN_BTN_PIN);
DebouncedButton btnPreset(PRESET_BTN_PIN);

// -------- Sensor control (start/stop without cutting power) --------
void sensorStart() {
  if (!sensorActive) {
    tof.startContinuous(50); // interval in ms
    sensorActive = true;
  }
}

void sensorStop() {
  if (sensorActive) {
    tof.stopContinuous();
    sensorActive = false;
  }
}

// -------- Motor driver control --------
void motorEnable(bool enable) {
  digitalWrite(R_EN_PIN, enable ? HIGH : LOW);
  digitalWrite(L_EN_PIN, enable ? HIGH : LOW);
  logMessage("motorEnable", 3);
  logValue("enable", enable ? 1.0f : 0.0f, 3);
}

void motorStopHard() {
  // Stop PWM and disable EN — full stop
  ledcWrite(RPWM_CH, 0);
  ledcWrite(LPWM_CH, 0);
  motorEnable(false);
  sensorStop();
  mode = MoveMode::IDLE;
  logMessage("motorStopHard", 3);
}

void motorDriveUp(uint16_t pwm) {
  motorEnable(true);
  ledcWrite(LPWM_CH, 0);
  ledcWrite(RPWM_CH, pwm);
}

void motorDriveDown(uint16_t pwm) {
  motorEnable(true);
  ledcWrite(RPWM_CH, 0);
  ledcWrite(LPWM_CH, pwm);
}

// -------- Ramp control --------
uint32_t rampStartMs = 0;
uint16_t currentPwm  = 0;

void startMove(MoveMode newMode, uint32_t nowMs) {
  if (mode != newMode) {
    mode = newMode;
    rampStartMs = nowMs;
    currentPwm = 0;
    sensorStart();
    logMessage("startMove");
  }
}

uint16_t computeRampPwm(uint32_t nowMs) {
  uint32_t dt = nowMs - rampStartMs;
  if (dt >= RAMP_MS) {
    return PWM_MAX;
  }

  // Linear ramp from PWM_MIN_START to PWM_MAX
  uint32_t delta = (uint32_t)(PWM_MAX - PWM_MIN_START) * dt / RAMP_MS;
  uint16_t pwm = PWM_MIN_START + (uint16_t)delta;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  return pwm;
}

// -------- Sensor reading --------
bool recoverSensorAfterTimeout() {
  Serial.println("VL53L1X timeout, trying to recover...");

  motorStopHard();
  sensorStop();

  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  if (!tof.init()) {
    Serial.println("VL53L1X re-init failed");
    return false;
  }

  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.setTimeout(100);
  tof.startContinuous(50);
  sensorActive = true;
  return true;
}

bool readHeightCm(float &heightCm) {
  if (!sensorActive) return false;
  if (!tof.dataReady()) return false;

  uint16_t mm = tof.read(); // in millimeters

  if (tof.timeoutOccurred()) {
    if (!recoverSensorAfterTimeout()) {
      return false;
    }
    return false;
  }

  // Simple hard filtering of bogus values
  if (mm < 650 || mm > 1200) {
    return false;
  }


  float newHeightCm = mm / 10.0f + 2.24f;

  // Reject identical readings if we already have a valid previous value
  if (newHeightCm == currentHeightCm) {
    return false;
  }

  heightCm = newHeightCm;
  logValue("heightCm", heightCm, 0);

  return true;
}

// -------- Measurement / safety handling --------
bool handleMeasurementResult(bool success) {
  if (!success) {
    measurementErrorsCount++;

    if (measurementErrorsCount > 10) {
      logValue("measurementErrorsCount", measurementErrorsCount,0);
    }
  } else {
    measurementErrorsCount = 0;
  }

  if (measurementErrorsCount > MEASUREMENT_ERRORS_THRESHOLD) {
    motorStopHard();
    logMessage("Measurement errors threshold exceeded", 0);
    measurementErrorsCount = 0;
    return true; // stopped
  }
  
  return false;
}

bool checkSafetyLimits() {
  // Only meaningful if we have a non-zero height
  if (currentHeightCm <= 0.0f) return false;

  if (currentHeightCm <= MIN_HEIGHT_CM &&
      (mode == MoveMode::MANUAL_DOWN ||
       (mode == MoveMode::GOTO_PRESET && targetHeightCm <= currentHeightCm))) {
    motorStopHard();
    logMessage("Too low, stopping", 0);
    return true;
  }

  if (currentHeightCm >= MAX_HEIGHT_CM &&
      (mode == MoveMode::MANUAL_UP ||
       (mode == MoveMode::GOTO_PRESET && targetHeightCm >= currentHeightCm))) {
    motorStopHard();
    logMessage("Too high, stopping", 0);
    return true;
  }

  return false;
}

// -------- Preset handling --------
void togglePresetAndStartAuto(uint32_t nowMs) {
  // currentPreset = (currentPreset == PresetSel::LOW_PRESET)
  //                 ? PresetSel::HIGH_PRESET
  //                 : PresetSel::LOW_PRESET;

  if (fabsf(currentHeightCm - PRESET_LOW_CM) <= PRESET_TOGGLE_TOL_CM) {
    currentPreset = PresetSel::HIGH_PRESET;
  }

  if (fabsf(currentHeightCm - PRESET_HIGH_CM) <= PRESET_TOGGLE_TOL_CM) {
    currentPreset = PresetSel::LOW_PRESET;
  }

  targetHeightCm = (currentPreset == PresetSel::LOW_PRESET)
                    ? PRESET_LOW_CM
                    : PRESET_HIGH_CM;

  startMove(MoveMode::GOTO_PRESET, nowMs);

  logMessage("Start preset move");
  logValue("targetHeightCm", targetHeightCm, 0);
}

// -------- User input handling --------
void handleUserInput(uint32_t nowMs) {
  bool upPressed = btnUp.isPressed();
  bool downPressed = btnDown.isPressed();
  bool presetShortClick = btnPreset.wasShortClicked();

  // If we are in automatic preset movement — any button press cancels it.
  if (mode == MoveMode::GOTO_PRESET) {
    if (upPressed || downPressed || btnPreset.isPressed()) {
      motorStopHard();
      logMessage("Auto movement cancelled by button", 0);
    }
    // Do not start new movements here, wait until next loop when in IDLE.
    return;
  }

  // Manual control has priority
  if (downPressed && !upPressed) {
    startMove(MoveMode::MANUAL_DOWN, nowMs);
    currentPreset = PresetSel::LOW_PRESET;
  } else if (upPressed && !downPressed) {
    startMove(MoveMode::MANUAL_UP, nowMs);
    currentPreset = PresetSel::HIGH_PRESET;
  } else {
    // If both released while in manual mode — stop
    if ((mode == MoveMode::MANUAL_UP || mode == MoveMode::MANUAL_DOWN) &&
        !upPressed && !downPressed) {
      logMessage("Manual movement stopped (buttons released)", 0);
      motorStopHard();
    }
  }

  // If we are idle (or manual already stopped) and got a short click on PRESET — start preset move.
  if (presetShortClick && mode == MoveMode::IDLE) {
    togglePresetAndStartAuto(nowMs);
  }
}

// -------- Automatic preset mode handling --------
void handleGotoPresetMode(bool hasValidHeight) {
  if (!hasValidHeight) {
    logMessage("No valid height, waiting...", 3);
    return;
  }

  if (fabsf(currentHeightCm - targetHeightCm) <= PRESET_TOL_CM) {
    motorStopHard();
    logMessage("Reached target height", 0);
    return;
  }

  if (currentPwm == 0) {
    // Ramp not started yet, nothing to drive
    return;
  }

  if (currentHeightCm < targetHeightCm) {
    motorDriveUp(currentPwm);
    logMessage("Driving Up (auto)", 3);
  } else {
    motorDriveDown(currentPwm);
    logMessage("Driving Down (auto)", 3);
  }
}

// -------- Motor control update --------
void updateMotorControl(uint32_t nowMs, bool hasValidHeight) {
  if (mode == MoveMode::IDLE) return;

  // Update PWM ramp
  currentPwm = computeRampPwm(nowMs);
  logValue("PWM", currentPwm, 2);

  switch (mode) {
    case MoveMode::MANUAL_UP:
      motorDriveUp(currentPwm);
      break;

    case MoveMode::MANUAL_DOWN:
      motorDriveDown(currentPwm);
      break;

    case MoveMode::GOTO_PRESET:
      handleGotoPresetMode(hasValidHeight);
      break;

    default:
      break;
  }
}

// -------- setup / loop --------
void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Init started...");

  // Buttons
  btnUp.begin();
  btnDown.begin();
  btnPreset.begin();

  // Motor driver EN pins
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  motorEnable(false);

  // LEDC PWM configuration
  ledcSetup(RPWM_CH, PWM_FREQ, PWM_RES);
  ledcSetup(LPWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM_PIN, RPWM_CH);
  ledcAttachPin(LPWM_PIN, LPWM_CH);
  ledcWrite(RPWM_CH, 0);
  ledcWrite(LPWM_CH, 0);

  // VL53L1X
  Wire.begin();           // default SDA/SCL for ESP32: GPIO 21/22
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  if (!tof.init()) {
    Serial.println("VL53L1X init failed! Check wiring.");
    // We can continue; sensor will stay inactive until init succeeds
  } else {
    tof.setDistanceMode(VL53L1X::Long);         // Long mode (up to ~4 m)
    tof.setMeasurementTimingBudget(50000);      // 50 ms budget
    tof.setTimeout(100);
    // Do not start continuous mode here; will start when movement begins
    Serial.println("Init finished");
  }
}

void loop() {
  uint32_t nowMs = millis();

  // 1. Update buttons
  btnUp.update(nowMs);
  btnDown.update(nowMs);
  btnPreset.update(nowMs);

  // 2. Handle input / state transitions
  handleUserInput(nowMs);

  // 3. If idle — nothing to do (sensor is off)
  if (mode == MoveMode::IDLE) {
    delay(3); // small sleep to avoid busy looping
    return;
  }

  // 4. Read height (if moving)
  float h;
  bool success = readHeightCm(h);
  if (success) {
    currentHeightCm = h;
  }

  // 5. Measurement errors (may stop the motor)
  if (handleMeasurementResult(success)) {
    return;
  }

  // 6. Safety limits
  if (success && checkSafetyLimits()) {
    return;
  }

  // 7. Update motor according to mode and ramp
  updateMotorControl(nowMs, success);

  // 8. Small pause to reduce I2C/CPU load
  delay(25);
}
