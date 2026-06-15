// ----------------- LTS RESPOOLER CONTROL BOARD CODE -----------------

#include <TMCStepper.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <string>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <WebServer.h>
#include <Update.h>

// --- Forward Declarations for PlatformIO C++ Compilation ---
void handleCommand(const std::string& cmd);
void sendStatus(bool forceSend = false);
void loadSettings();
void applySpeedTarget();
void setMotorCurrent(int strengthPercent);
int calculatePWM(int pct);
void setConnLed(uint8_t duty);
uint32_t stepFreqFromIntervalUs(unsigned long us);
void stepperSetFreq(uint32_t freqHz);
void stepperStart(uint32_t freqHz);
void stepperStop();
void reSanitizeDriver();
void playTone(unsigned int freqHz, unsigned long durationMs);
void playStepperJingle();
int fanPWMFromSpeed(int speedPercent);
void setServoAngle(int angle);
int getSpeedPercentFromInterval(unsigned long us);
void wifiScanTask(void* parameter);
void sendOTAUpdate();
void otaLedPulseTask(void* parameter);

#include "esp_bt.h"
#include "esp_coexist.h"
#include <driver/ledc.h>
#include "driver/gpio.h"
#include <freertos/semphr.h>

// ------------------------ Board Info ----------------------------
#define FIRMWARE_VERSION "1.2.1-Web"

#ifdef BOARD_VARIANT_CB
  #define BOARD_NAME "LTS CB (Control)"
  #define TMC_UART_RX 18
  #define TMC_UART_TX 17
  #define STEP_PIN 14
  #define DIR_PIN 21
  #define EN_PIN 40
  #define LED_PIN 9
  #define LED_CONN_PIN 47
  #define FILAMENT_PIN 11
  #define BUTTON_PIN 10
  #define FAN_PIN 13
  #define SERVO_PIN 12
#elif defined(BOARD_VARIANT_DB)
  #define BOARD_NAME "LTS DB (Driver)"
  #define TMC_UART_RX 21
  #define TMC_UART_TX 22
  #define STEP_PIN 14
  #define DIR_PIN 27
  #define EN_PIN 13
  #define LED_PIN 19
  #define LED_CONN_PIN 2
  #define FILAMENT_PIN 33
  #define BUTTON_PIN 25
  #define FAN_PIN 32
  #define SERVO_PIN 26
#elif defined(BOARD_VARIANT_S3)
  #define BOARD_NAME "LTS Generic S3"
  #define TMC_UART_RX 4
  #define TMC_UART_TX 4
  #define STEP_PIN 1
  #define DIR_PIN 2
  #define EN_PIN 3
  #define LED_PIN 5
  #define LED_CONN_PIN 6
  #define FILAMENT_PIN 7
  #define BUTTON_PIN 8
  #define FAN_PIN 9
  #define SERVO_PIN 10
#elif defined(BOARD_VARIANT_C3)
  #define BOARD_NAME "LTS Generic C3"
  #define TMC_UART_RX 3
  #define TMC_UART_TX 3
  #define STEP_PIN 0
  #define DIR_PIN 1
  #define EN_PIN 2
  #define LED_PIN 4
  #define LED_CONN_PIN 5
  #define FILAMENT_PIN 6
  #define BUTTON_PIN 7
  #define FAN_PIN 8
  #define SERVO_PIN 9
#else
  #define BOARD_NAME "LTS DB (Driver)"
  #define TMC_UART_RX 21
  #define TMC_UART_TX 22
  #define STEP_PIN 14
  #define DIR_PIN 27
  #define EN_PIN 13
  #define LED_PIN 19
  #define LED_CONN_PIN 2
  #define FILAMENT_PIN 33
  #define BUTTON_PIN 25
  #define FAN_PIN 32
  #define SERVO_PIN 26
#endif

// ------------------------ Motor Parameters ----------------------
#define MICROSTEPPING 8
#define R_SENSE 0.11f

// -------------------- Timing and Limits --------------------------
#define START_INTERVAL_US 700
#define DEFAULT_INTERVAL_US 116
#define TORQUE_CHECK_INTERVAL_MS 50
#define BUTTON_DEBOUNCE_MS 50
#define ACCEL_UPDATE_INTERVAL 20
#define ACCEL_STEP 5
#define LED_CONN_PULSE_MAX 60
#define STATUS_NOTIFY_INTERVAL 400
#define LED_BLINK_INTERVAL 1000
#define TORQUE_SG_IGNORE 20
#define TORQUE_SG_LIMIT_LOW 0.72
#define TORQUE_SG_LIMIT_MED 0.85
#define TORQUE_SG_LIMIT_HIGH 0.97
#define TORQUE_BELOW_MS 700
#define MIN_RSSI_THRESHOLD -90
#define FILAMENT_LOSS_CONFIRM_MS 1500
#define STEP_DUTY_ON 512

#define TARGET_WEIGHT_FACTOR_2 0.59f
#define TARGET_WEIGHT_FACTOR_3 0.34f

#define FAN_CHANNEL LEDC_CHANNEL_1
#define FAN_TIMER LEDC_TIMER_1

#define STEP_LEDC_CHANNEL LEDC_CHANNEL_3
#define STEP_LEDC_TIMER LEDC_TIMER_2

#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_TIMER LEDC_TIMER_3

// ------------------- BLE UUIDs -----------------------------------
#define SERVICE_UUID "9E05D06D-68A7-4E1F-A503-AE26713AC101"
#define CHARACTERISTIC_UUID "7CB2F1B4-7E3F-43D2-8C92-DF58C9A7B1A8"

// ------------------- Direct settings variables -------------------
int speedPercent = 85;                        // 50-100
int motorDirection = 0;                       // 0, 1
int ledBrightness = 50;                       // 0-100
bool useFilamentSensor = true;                // true, false
int motorStrength = 100;                      // 40-120
int targetWeight = 0;                         // 0-3
int torqueLimit = 0;                          // 0-3
int jingleStyle = 0;                          // 0-3
unsigned long calibrationAt80Speed = 895000;  // ms

String wifiSSID = "";      // string
String wifiPassword = "";  // string

int fanSpeed = 60;         // 0–100
bool fanAlwaysOn = false;  // true, false
unsigned long fanStopAfter = 0;

int servoAngleR = 5;        // deg
int servoAngleL = 175;      // deg
float servoStepMm = 1.75f;  // mm
bool servoHomeIsR = true;   // true, false

// ----------------- Board variant identifier -----------------
uint8_t boardVariant = 0;

// --------------- High-Speed mode state -----------------
bool highSpeedMode = false;
int savedTorqueLimitHS = 0;
unsigned long baseTargetStepIntervalMicros = DEFAULT_INTERVAL_US;

// -------------- Status variables and objects -----------------
HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, 0);
NimBLECharacteristic* pCharacteristic = nullptr;
Preferences prefs;
String serialRxLine;

float guideServoPos = 0.0f;
bool guideServoTowardsL = true;
float motorStepAcc = 0.0f;
unsigned long lastMotorStepAccTime = 0;
bool servoWasRunning = false;
char lastStateForServo = 'I';
bool servoHomingToHome = false;
unsigned long lastServoHomeStepTime = 0;
bool servoHomeAfterDonePending = false;
char lastStateForDone = 'I';
bool servoGotoActive = false;
char servoGotoSide = 0;

StaticJsonDocument<640> lastStatusDoc;

bool deviceConnected = false;
bool isMotorRunning = false;
bool shouldStartMotorNow = false;
bool filamentDetected = false;
bool pendingDirectionChange = false;
bool lastStableButtonState = HIGH;
bool ledState = false;
int newMotorDirection = 0;
int pwmValue = 0;
int triggerJingleNow = 0;
unsigned long delayStartUntil = 0;
unsigned long filamentLostSince = 0;
unsigned long stepIntervalMicros = START_INTERVAL_US;
unsigned long targetStepIntervalMicros = DEFAULT_INTERVAL_US;
unsigned long spoolingStartTime = 0;
unsigned long lastNotify = 0;
unsigned long lastLedToggle = 0;
unsigned long lastAccelUpdate = 0;
unsigned long lastConnLedTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastIntervalMicros = 0;
unsigned long totalEstimatedTime = 0;
unsigned long pausedElapsed = 0;
unsigned long buttonPressSince = 0;
bool buttonLongPressHandled = false;
bool buttonAwaitingDecision = false;
bool doneHoldActive = false;
unsigned long doneHoldStart = 0;
bool remHoldActive = false;
unsigned long remHoldExpiry = 0;
bool progHoldActive = false;
unsigned long progHoldExpiry = 0;
unsigned long remStartHoldUntil = 0;
float progress = 0.0;
bool lastIsMotorRunning = false;
bool lastFilamentDetected = false;
float lastProgress = 0.0;
int lastSpeedPercent = 100;
int lastChipTemperature = 0;
int lastRemainingTime = 0;
volatile bool otaInProgress = false;
SemaphoreHandle_t bleNotifyMutex = nullptr;
char currentState = 'I';

Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
int ledPulseValue = 0;
int ledPulseDirection = 1;
unsigned long lastLedPulseTime = 0;
uint16_t greenDitherAcc = 0;

// ---------- OTA LED pulse task handle and function ------------
TaskHandle_t otaLedPulseTaskHandle = nullptr;
TaskHandle_t wifiScanTaskHandle = nullptr;
void otaLedPulseTask(void* parameter) {
  pwmValue = calculatePWM(ledBrightness);
  int otaLedPulse = 0;
  int otaLedDir = 1;
  uint16_t blueDitherAcc = 0;

  while (otaInProgress) {
    otaLedPulse += otaLedDir * 1;
    if (otaLedPulse >= LED_CONN_PULSE_MAX) {
      otaLedPulse = LED_CONN_PULSE_MAX;
      otaLedDir = -1;
    } else if (otaLedPulse <= 0) {
      otaLedPulse = 0;
      otaLedDir = 1;
    }

    setConnLed((uint8_t)otaLedPulse);

    uint16_t pulse255 = (uint16_t)((otaLedPulse * 255 + (LED_CONN_PULSE_MAX / 2)) / LED_CONN_PULSE_MAX);
    uint16_t scaled16 = (uint16_t)pulse255 * (uint16_t)pwmValue;
    uint8_t baseVal = scaled16 / 255;
    uint8_t remainder = scaled16 % 255;
    blueDitherAcc = (uint16_t)(blueDitherAcc + remainder);
    if (blueDitherAcc >= 255) {
      baseVal++;
      blueDitherAcc -= 255;
    }

    led.setPixelColor(0, led.Color(0, 0, baseVal));
    led.show();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  setConnLed(LED_CONN_PULSE_MAX);
  led.setPixelColor(0, 0);
  led.show();
  vTaskDelete(NULL);
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialRxLine.trim();
      if (serialRxLine.length() > 0) {
        if (serialRxLine.length() > 512) {
          serialRxLine = "";
        } else {
          std::string cmd(serialRxLine.c_str());
          handleCommand(cmd);
          serialRxLine = "";
        }
      } else {
        serialRxLine = "";
      }
    } else {
      if (serialRxLine.length() < 512) serialRxLine += c;
    }
  }
}

void wifiScanTask(void* parameter) {
  WiFi.disconnect(true);
  int n = WiFi.scanNetworks();
  StaticJsonDocument<256> ssidDoc;
  JsonArray arr = ssidDoc.createNestedArray("SSID_LIST");
  for (int i = 0; i < n; i++) {
    int rssi = WiFi.RSSI(i);
    if (rssi > MIN_RSSI_THRESHOLD) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() > 0 && ssid.length() < 33) {
        arr.add(ssid);
      }
    }
  }
  if (deviceConnected && pCharacteristic) {
    String jsonOut;
    serializeJson(ssidDoc, jsonOut);
    if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
    pCharacteristic->setValue(jsonOut.c_str());
    pCharacteristic->notify();
    if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
  }
  WiFi.scanDelete();
  prefs.begin("respooler", true);
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
  prefs.end();
  if (ssid.length() == 0) {
    ssid = "SkyNet";
    pwd = "123456798aB!";
  }
  if (ssid.length() > 0 && pwd.length() > 0) {
    WiFi.begin(ssid.c_str(), pwd.c_str());
  }
  wifiScanTaskHandle = nullptr;
  vTaskDelete(NULL);
}

// -------------- Wi-Fi connect async status --------------
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartTime = 0;

// ------------------ Helper and utility functions ---------------
int calculatePWM(int brightnessPercent) {
  float gamma = 1.15f;
  float n = (float)constrain(brightnessPercent, 0, 100) / 100.0f;
  return (int)round(powf(n, gamma) * 255.0f);
}
int fanPWMFromSpeed(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}

inline void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (uint32_t)((((uint64_t)pulseUs) * ((1UL << 14) - 1) + 10000UL) / 20000UL);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);
}
inline void setConnLed(uint8_t duty) {
  duty = constrain(duty, 0, 255);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 255 - duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}
inline uint32_t stepFreqFromIntervalUs(unsigned long us) {
  if (us < 1) us = 1;
  return (uint32_t)(1000000UL / (2UL * us));
}
inline void stepperSetFreq(uint32_t freqHz) {
  if (freqHz < 1) freqHz = 1;
  ledc_set_freq(LEDC_LOW_SPEED_MODE, STEP_LEDC_TIMER, freqHz);
}
inline void stepperStart(uint32_t freqHz) {
  stepperSetFreq(freqHz);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, STEP_DUTY_ON);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL);
}
inline void stepperStop() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL);
}
int getSpeedPercentFromInterval(unsigned long us) {
  double t = (double)us;
  double p = 50.0 + ((170.0 - t) * 50.0 / (170.0 - 93.0));
  int percent = (int)round(p);
  return constrain(percent, 50, 100);
}

void applySpeedTarget() {
  if (highSpeedMode) {
    unsigned long adjusted = (unsigned long)round((double)baseTargetStepIntervalMicros / 1.4);
    if (adjusted < 50) adjusted = 50;
    targetStepIntervalMicros = adjusted;
  } else {
    targetStepIntervalMicros = baseTargetStepIntervalMicros;
  }
}

void loadSettings() {
  prefs.begin("respooler", true);
  boardVariant = prefs.getUChar("variant", 0);
  baseTargetStepIntervalMicros = prefs.getUInt("speed", DEFAULT_INTERVAL_US);
  speedPercent = getSpeedPercentFromInterval(baseTargetStepIntervalMicros);
  applySpeedTarget();
  motorDirection = prefs.getUInt("dir", 0);
  ledBrightness = prefs.getUInt("led", 50);
  useFilamentSensor = prefs.getBool("filamentSensor", true);
  motorStrength = prefs.getUInt("motorStrength", 100);
  torqueLimit = prefs.getUInt("torqueLimit", 0);
  highSpeedMode = prefs.getBool("hsMode", false);
  if (highSpeedMode) {
    savedTorqueLimitHS = torqueLimit;
    torqueLimit = 0;
  }
  jingleStyle = prefs.getUInt("jingle", 0);
  calibrationAt80Speed = prefs.getULong("cal80_093", 895000);
  fanSpeed = prefs.getUInt("fanSpeed", 60);
  fanAlwaysOn = prefs.getBool("fanAlways", false);
  targetWeight = prefs.getUInt("targetWeight", 0);

  servoAngleR = (int)prefs.getUInt("servoR", 5);
  servoAngleL = (int)prefs.getUInt("servoL", 175);
  servoStepMm = prefs.getFloat("servoStep", 1.75f);
  servoHomeIsR = prefs.getBool("servoHomeR", true);

  servoAngleR = constrain(servoAngleR, 0, 180);
  servoAngleL = constrain(servoAngleL, 0, 180);
  servoStepMm = constrain(servoStepMm, 0.05f, 20.0f);

  applySpeedTarget();
  prefs.end();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);
  digitalWrite(DIR_PIN, motorDirection);
}

void setMotorCurrent(int percent) {
  int strengthCurrent = map(percent, 40, 120, 350, 1400);
  driver.rms_current(strengthCurrent);
}

void reSanitizeDriver() {
  driver.GSTAT(0b111);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.microsteps(MICROSTEPPING);
  setMotorCurrent(motorStrength);
}

// ------------------- BLE status/error/settings -------------------
void sendStatus(bool forceSend) {
  if (!deviceConnected || !pCharacteristic) return;

  float progVal = progress;
  int remVal = 0;
  if (currentState == 'D') {
    progVal = 100.0f;
    remVal = 0;
  } else if (totalEstimatedTime > 0) {
    unsigned long elapsed;
    if (spoolingStartTime > 0) {
      elapsed = millis() - spoolingStartTime;
    } else if (pausedElapsed > 0) {
      elapsed = pausedElapsed;
    } else if (currentState == 'A') {
      elapsed = pausedElapsed;
    } else {
      elapsed = 0;
    }
    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);
    if (effectiveTotal == 0) {
      progVal = 0.0f;
      remVal = 0;
    } else if (elapsed >= effectiveTotal) {
      progVal = 100.0f;
      remVal = 0;
    } else {
      progVal = (100.0f * (float)elapsed) / (float)effectiveTotal;
      unsigned long remainingMs = effectiveTotal - elapsed;
      remVal = (int)(remainingMs / 1000UL);
    }
  }
  if (remHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < remHoldExpiry) {
      remVal = lastRemainingTime;
    } else {
      remHoldActive = false;
    }
  }
  if (progHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < progHoldExpiry) {
      progVal = lastProgress;
    } else {
      progHoldActive = false;
    }
  }
  if (currentState == 'R') {
    unsigned long nowMs = millis();
    if (nowMs < remStartHoldUntil) {
      remVal = lastRemainingTime;
    }
  }
  int chipTemp = (int)temperatureRead();

  StaticJsonDocument<640> doc;
  doc["STAT"] = String(currentState);
  doc["HAS_FIL"] = filamentDetected;
  doc["USE_FIL"] = useFilamentSensor;
  doc["PROG"] = progVal;
  doc["REM"] = remVal;
  doc["TEMP"] = chipTemp;
  doc["WIFI_SSID"] = wifiSSID.length() > 0 ? wifiSSID.c_str() : nullptr;
  doc["WIFI_OK"] = WiFi.status() == WL_CONNECTED;
  doc["FW"] = FIRMWARE_VERSION;
  doc["OTA_OK"] = nullptr;
  doc["SPD"] = speedPercent;
  doc["JIN"] = jingleStyle;
  doc["LED"] = ledBrightness;
  doc["DIR"] = motorDirection;
  doc["POW"] = motorStrength;
  doc["TRQ"] = highSpeedMode ? 0 : torqueLimit;
  doc["WGT"] = targetWeight;
  doc["DUR"] = calibrationAt80Speed / 1000;
  doc["HS"] = highSpeedMode;
  doc["FAN_SPD"] = fanSpeed;
  doc["FAN_ON"] = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter);
  doc["FAN_ALW"] = fanAlwaysOn;
  doc["SV_R"] = servoAngleR;
  doc["SV_L"] = servoAngleL;
  doc["SV_STP"] = servoStepMm;
  doc["SV_HOME"] = servoHomeIsR ? "R" : "L";
  doc["VAR"] = (boardVariant == 2) ? "PRO" : (boardVariant == 1) ? "STD"
                                                                 : "UNK";

  lastIsMotorRunning = isMotorRunning;
  lastFilamentDetected = filamentDetected;
  lastProgress = progVal;
  lastChipTemperature = chipTemp;
  lastRemainingTime = remVal;
  lastSpeedPercent = speedPercent;

  String jsonOut;
  serializeJson(doc, jsonOut);
  if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
  pCharacteristic->setValue(jsonOut.c_str());
  pCharacteristic->notify();
  if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
}

// ------------------- Stepper jingle/tones --------------------------
void playTone(unsigned int freqHz, unsigned long durationMs) {
  uint16_t prevCurrent = driver.rms_current();
  driver.rms_current(1400);
  unsigned long halfPeriod = 1000000UL / (2 * freqHz);
  unsigned long totalCycles = (durationMs * 1000UL) / (2 * halfPeriod);
  bool dirState = false;
  for (unsigned long i = 0; i < totalCycles; i++) {
    dirState = !dirState;
    digitalWrite(DIR_PIN, dirState);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(40);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(halfPeriod * 2 - 40);
  }
  driver.rms_current(prevCurrent);
}
void playStepperJingle() {
  if (jingleStyle == 0) return;
  delay(500);
  stepperStop();
  ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
  gpio_reset_pin((gpio_num_t)STEP_PIN);
  gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  {
    led.setPixelColor(0, led.Color(0, pwmValue, 0));
    led.show();
  }
  switch (jingleStyle) {
    case 1:  // Simple
      playTone(1000, 140);
      delay(20);
      playTone(800, 140);
      delay(20);
      playTone(1200, 400);
      break;
    case 2:  // Glissando
      playTone(523, 220);
      delay(20);
      playTone(587, 70);
      delay(20);
      playTone(659, 70);
      delay(20);
      playTone(698, 70);
      delay(20);
      playTone(784, 70);
      delay(20);
      playTone(880, 70);
      delay(20);
      playTone(988, 100);
      delay(20);
      playTone(1047, 400);
      break;
    case 3:  // Star Wars
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 500);
      break;
  }
  digitalWrite(EN_PIN, HIGH);
  {
    ledc_channel_config_t step_ledc_chan = {
      .gpio_num = STEP_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = STEP_LEDC_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = STEP_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&step_ledc_chan);
  }
  stepperStop();
}

// --------------------- BLE command handling -------------------------
void sendOTAUpdate() {
  if (otaInProgress) {
    return;
  }
  isMotorRunning = false;
  digitalWrite(STEP_PIN, LOW);
  stepperStop();
  digitalWrite(EN_PIN, HIGH);
  currentState = 'U';
  sendStatus(true);
  xTaskCreate([](void*) {
    const char* ota_url = "https://respooler.lts-design.com/Firmware/ControlBoard_V4_OTA.bin";
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == INADDR_NONE) {
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }

    otaInProgress = true;
    sendStatus(true);
    if (otaLedPulseTaskHandle == nullptr) {
      xTaskCreate(otaLedPulseTask, "OtaLedPulse", 1024, NULL, 2, &otaLedPulseTaskHandle);
    }

    http.begin(client, ota_url);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    bool otaSuccess = false;

    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      if (Update.begin(contentLength)) {
        WiFiClient* stream = http.getStreamPtr();
        while (http.connected() && Update.writeStream(*stream) > 0) {
          delay(1);
        }
        if (Update.end() && Update.isFinished()) {
          otaSuccess = true;
          if (deviceConnected && pCharacteristic) {
            StaticJsonDocument<64> doc;
            doc["OTA_OK"] = true;
            delay(100);
            sendStatus(true);
            String jsonOut;
            serializeJson(doc, jsonOut);
            if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
            pCharacteristic->setValue(jsonOut.c_str());
            pCharacteristic->notify();
            if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
            NimBLEDevice::deinit(true);
            delay(1000);
          }
          otaInProgress = false;
          otaLedPulseTaskHandle = nullptr;
          ESP.restart();
        }
      } else {
        if (deviceConnected && pCharacteristic) {
          StaticJsonDocument<64> doc;
          doc["OTA_OK"] = false;
          String jsonOut;
          serializeJson(doc, jsonOut);
          if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
          pCharacteristic->setValue(jsonOut.c_str());
          pCharacteristic->notify();
          if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
        }
        otaInProgress = false;
        otaLedPulseTaskHandle = nullptr;
        currentState = 'I';
        sendStatus(true);
        vTaskDelete(NULL);
      }
    } else {
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      otaInProgress = false;
      otaLedPulseTaskHandle = nullptr;
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }
    http.end();
    if (!otaSuccess) {
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
    otaInProgress = false;
    otaLedPulseTaskHandle = nullptr;
    currentState = 'I';
    sendStatus(true);
    vTaskDelete(NULL);
  },
              "OTAUpdateTask", 8192, NULL, 1, NULL);
}


// ----------------- Diagnostics & WebServer -----------------
#define MAX_LOG_LINES 50
String logLines[MAX_LOG_LINES];
int logHead = 0;
int logCount = 0;
SemaphoreHandle_t logMutex = nullptr;

void addToLog(const String& msg) {
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  Serial.println(msg);
  logLines[logHead] = msg;
  logHead = (logHead + 1) % MAX_LOG_LINES;
  if (logCount < MAX_LOG_LINES) logCount++;
  if (logMutex) xSemaphoreGive(logMutex);
}

String getResetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "Unbekannter Reset";
    case ESP_RST_POWERON:   return "Power-on Reset (Netzteil eingeschaltet)";
    case ESP_RST_EXT:       return "Externer Pin Reset";
    case ESP_RST_SW:        return "Software Reset";
    case ESP_RST_PANIC:     return "Software Panic / Absturz (Crash)";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog Reset";
    case ESP_RST_TASK_WDT:  return "Task Watchdog Reset";
    case ESP_RST_WDT:       return "Anderer Watchdog Reset";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep Wake Reset";
    case ESP_RST_BROWNOUT:  return "Brownout Reset (Spannungseinbruch! Netzteil zu schwach!)";
    case ESP_RST_SDIO:      return "SDIO Reset";
    default:                return "Undefinierter Reset-Grund";
  }
}

WebServer webServer(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>LTS Respooler Pro Console</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-base: #0a0e17;
            --bg-card: rgba(20, 27, 45, 0.65);
            --border-card: rgba(255, 255, 255, 0.08);
            --primary: #00f2fe;
            --secondary: #4facfe;
            --text-main: #f3f4f6;
            --text-muted: #9ca3af;
            --success: #10b981;
            --error: #ef4444;
            --warning: #f59e0b;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Outfit', sans-serif; }
        body {
            background-color: var(--bg-base);
            color: var(--text-main);
            min-height: 100vh;
            padding: 2rem 1rem;
            background-image: radial-gradient(circle at 10% 20%, rgba(0, 242, 254, 0.04) 0%, transparent 40%),
                              radial-gradient(circle at 90% 80%, rgba(79, 172, 254, 0.04) 0%, transparent 40%);
        }
        .container { max-width: 900px; margin: 0 auto; }
        header { text-align: center; margin-bottom: 2rem; }
        h1 {
            font-size: 2.25rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.25rem;
            letter-spacing: -0.03em;
        }
        .subtitle { color: var(--text-muted); font-size: 1rem; font-weight: 300; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 1.25rem; margin-bottom: 1.25rem; }
        .card {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 16px;
            padding: 1.25rem;
            backdrop-filter: blur(12px);
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.4);
            transition: transform 0.2s, box-shadow 0.2s;
        }
        .card:hover { transform: translateY(-2px); box-shadow: 0 12px 40px 0 rgba(0, 242, 254, 0.08); }
        .card-title {
            font-size: 1.1rem;
            font-weight: 600;
            margin-bottom: 1rem;
            border-bottom: 1px solid var(--border-card);
            padding-bottom: 0.5rem;
            display: flex;
            align-items: center;
            justify-content: space-between;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            color: var(--secondary);
        }
        .metric { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.75rem; }
        .metric-label { color: var(--text-muted); font-size: 0.9rem; }
        .metric-value { font-weight: 600; font-size: 1rem; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; margin-right: 0.5rem; }
        .status-dot.active { background-color: var(--success); box-shadow: 0 0 8px var(--success); animation: pulse 1.5s infinite; }
        .status-dot.inactive { background-color: var(--text-muted); }
        .status-dot.alarm { background-color: var(--error); box-shadow: 0 0 8px var(--error); animation: pulse 1s infinite; }
        @keyframes pulse {
            0% { opacity: 0.4; }
            50% { opacity: 1; }
            100% { opacity: 0.4; }
        }
        .progress-bar-container { background: rgba(255, 255, 255, 0.04); border-radius: 10px; height: 12px; width: 100%; overflow: hidden; margin: 0.5rem 0; border: 1px solid var(--border-card); }
        .progress-bar { height: 100%; background: linear-gradient(90deg, var(--primary), var(--secondary)); width: 0%; transition: width 0.3s ease; }
        .console {
            background: rgba(5, 7, 12, 0.8);
            border: 1px solid var(--border-card);
            border-radius: 12px;
            height: 220px;
            overflow-y: auto;
            padding: 0.75rem;
            font-family: 'Courier New', Courier, monospace;
            font-size: 0.85rem;
            color: #34d399;
            margin-bottom: 0.75rem;
            box-shadow: inset 0 2px 8px rgba(0,0,0,0.8);
        }
        .console-line { margin-bottom: 0.2rem; line-height: 1.2; }
        .btn-group { display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.5rem; }
        button {
            padding: 0.6rem 0.75rem;
            border-radius: 8px;
            border: none;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.9rem;
            transition: all 0.2s;
            text-align: center;
        }
        .btn-primary { background: linear-gradient(135deg, var(--primary), var(--secondary)); color: #05070c; }
        .btn-primary:hover { opacity: 0.9; box-shadow: 0 0 12px rgba(0, 242, 254, 0.3); }
        .btn-danger { background: var(--error); color: white; }
        .btn-danger:hover { box-shadow: 0 0 12px rgba(239, 68, 68, 0.3); }
        .btn-warning { background: var(--warning); color: black; }
        .btn-warning:hover { box-shadow: 0 0 12px rgba(245, 158, 11, 0.3); }
        .file-upload-label {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            border: 2px dashed var(--border-card);
            border-radius: 12px;
            padding: 1.5rem;
            cursor: pointer;
            transition: border-color 0.2s;
            color: var(--text-muted);
            text-align: center;
        }
        .file-upload-label:hover { border-color: var(--primary); color: var(--text-main); }
        .file-upload input[type="file"] { display: none; }
        .upload-prog-container { display: none; margin-top: 1rem; }
        .upload-status { font-size: 0.85rem; margin-top: 0.5rem; text-align: center; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>LTS Respooler Pro</h1>
            <div class="subtitle" id="fw-info">Management Console & Diagnostics</div>
        </header>
        <div class="grid">
            <div class="card">
                <div class="card-title">Live Status</div>
                <div class="metric">
                    <span class="metric-label">System-Zustand</span>
                    <span class="metric-value" id="sys-state" style="display:flex; align-items:center;">
                        <span class="status-dot inactive" id="state-dot"></span>
                        <span id="state-text">Lade...</span>
                    </span>
                </div>
                <div class="metric">
                    <span class="metric-label">Filament Sensor</span>
                    <span class="metric-value" id="filament-status">Lade...</span>
                </div>
                <div class="metric">
                    <span class="metric-label">ESP32 Temperatur</span>
                    <span class="metric-value" id="temp-status">Lade...</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Boot Grund</span>
                    <span class="metric-value" id="boot-status" style="font-size:0.8rem; text-align:right;">Lade...</span>
                </div>
            </div>
            
            <div class="card">
                <div class="card-title">Wickel-Fortschritt</div>
                <div class="metric">
                    <span class="metric-label">Fortschritt</span>
                    <span class="metric-value" id="progress-percent">0%</span>
                </div>
                <div class="progress-bar-container">
                    <div class="progress-bar" id="progress-bar"></div>
                </div>
                <div class="metric">
                    <span class="metric-label">Restlaufzeit</span>
                    <span class="metric-value" id="rem-time">0s</span>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Steuerung</div>
                <div class="btn-group">
                    <button class="btn-primary" onclick="sendCmd('START')">Start</button>
                    <button class="btn-warning" onclick="sendCmd('PAUSE')">Pause</button>
                    <button class="btn-danger" onclick="sendCmd('STOP')">Stop</button>
                </div>
            </div>
        </div>

        <div class="card" style="margin-bottom: 1.25rem;">
            <div class="card-title">
                <span>Echtzeit-Diagnose-Logs</span>
                <button onclick="clearLogs()" style="padding:0.25rem 0.5rem; font-size:0.75rem; background:rgba(255,255,255,0.05); color:var(--text-muted); border:1px solid var(--border-card);">Logs leeren</button>
            </div>
            <div class="console" id="console">
                <div class="console-line"><span class="console-msg">Warte auf Log-Daten...</span></div>
            </div>
        </div>

        <div class="card">
            <div class="card-title">Firmware Update (OTA)</div>
            <label class="file-upload-label" id="drop-zone">
                <span id="upload-label-text">Klicke oder ziehe die .bin Firmware-Datei hierher</span>
                <input type="file" id="firmware-file" accept=".bin">
            </label>
            <div class="upload-prog-container" id="upload-prog-container">
                <div class="progress-bar-container">
                    <div class="progress-bar" id="upload-bar" style="width: 0%"></div>
                </div>
                <div class="upload-status" id="upload-status">Lade hoch...</div>
            </div>
        </div>
    </div>

    <script>
        function sendCmd(cmdName) {
            fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ CMD: cmdName })
            })
            .then(res => res.json())
            .catch(err => console.error('Fehler beim Senden des Befehls:', err));
        }

        function updateStatus() {
            fetch('/status')
            .then(res => res.json())
            .then(data => {
                document.getElementById('fw-info').innerText = `Konsole & Diagnosedaten (FW: ${data.FW} | Board: ${data.VAR})`;
                const stateText = document.getElementById('state-text');
                const stateDot = document.getElementById('state-dot');
                stateText.innerText = stateMap(data.STAT);
                
                stateDot.className = 'status-dot';
                if (data.STAT === 'R') {
                    stateDot.classList.add('active');
                } else if (data.STAT === 'A') {
                    stateDot.classList.add('alarm');
                } else {
                    stateDot.classList.add('inactive');
                }

                const filStatus = document.getElementById('filament-status');
                if (data.HAS_FIL) {
                    filStatus.innerText = 'Filament eingelegt';
                    filStatus.style.color = 'var(--success)';
                } else {
                    filStatus.innerText = 'Fehlt / Runout';
                    filStatus.style.color = 'var(--error)';
                }

                document.getElementById('temp-status').innerText = data.TEMP + ' °C';
                document.getElementById('boot-status').innerText = data.BOOT_REASON || 'Unbekannt';

                const prog = parseFloat(data.PROG).toFixed(1);
                document.getElementById('progress-percent').innerText = prog + '%';
                document.getElementById('progress-bar').style.width = prog + '%';
                
                const rem = data.REM;
                if (rem > 0) {
                    const m = Math.floor(rem / 60);
                    const s = rem % 60;
                    document.getElementById('rem-time').innerText = `${m}m ${s}s`;
                } else {
                    document.getElementById('rem-time').innerText = '--';
                }
            })
            .catch(err => console.error('Fehler beim Abrufen des Status:', err));
        }

        function stateMap(stat) {
            switch(stat) {
                case 'I': return 'Bereit (Idle)';
                case 'P': return 'Pausiert';
                case 'A': return 'Auto-Stop (StallGuard)';
                case 'D': return 'Fertig (Done)';
                case 'R': return 'Spult (Running)';
                case 'U': return 'OTA Update...';
                default: return 'Unbekannt (' + stat + ')';
            }
        }

        function updateLogs() {
            fetch('/logs')
            .then(res => res.json())
            .then(data => {
                const con = document.getElementById('console');
                let html = '';
                
                if (!data.logs || data.logs.length === 0) {
                    con.innerHTML = '<div class="console-line"><span class="console-msg">Keine Log-Einträge vorhanden.</span></div>';
                    return;
                }

                data.logs.forEach(line => {
                    let cls = 'console-msg';
                    if (line.includes('stalled') || line.includes('runout') || line.includes('Brownout') || line.includes('Reset') || line.includes('Absturz')) {
                        cls += ' error';
                    } else if (line.includes('Connected') || line.includes('success') || line.includes('fertig') || line.includes('started')) {
                        cls += ' success';
                    } else if (line.includes('changed') || line.includes('Pausiert') || line.includes('State')) {
                        cls += ' warn';
                    }
                    html += `<div class="console-line"><span class="${cls}">${line}</span></div>`;
                });
                
                const shouldScroll = con.scrollHeight - con.clientHeight - con.scrollTop < 40;
                con.innerHTML = html;
                if (shouldScroll) {
                    con.scrollTop = con.scrollHeight;
                }
            })
            .catch(err => console.error('Fehler beim Abrufen der Logs:', err));
        }

        function clearLogs() {
            document.getElementById('console').innerHTML = '';
        }

        const fileInput = document.getElementById('firmware-file');
        const dropZone = document.getElementById('drop-zone');
        const labelText = document.getElementById('upload-label-text');
        const progContainer = document.getElementById('upload-prog-container');
        const uploadBar = document.getElementById('upload-bar');
        const uploadStatus = document.getElementById('upload-status');

        fileInput.addEventListener('change', (e) => {
            const file = e.target.files[0];
            if (file) uploadFile(file);
        });

        function uploadFile(file) {
            const xhr = new XMLHttpRequest();
            const formData = new FormData();
            formData.append('update', file);

            progContainer.style.display = 'block';
            labelText.innerText = file.name;
            uploadStatus.innerText = 'Lade hoch...';

            xhr.upload.addEventListener('progress', (e) => {
                if (e.lengthComputable) {
                    const percent = (e.loaded / e.total) * 100;
                    uploadBar.style.width = percent + '%';
                    uploadStatus.innerText = `Lade hoch: ${Math.round(percent)}%`;
                }
            });

            xhr.addEventListener('load', () => {
                if (xhr.status === 200) {
                    uploadStatus.innerText = 'Erfolgreich! Gerät startet neu...';
                    uploadStatus.style.color = 'var(--success)';
                } else {
                    uploadStatus.innerText = 'Fehler beim Hochladen!';
                    uploadStatus.style.color = 'var(--error)';
                }
            });

            xhr.addEventListener('error', () => {
                uploadStatus.innerText = 'Fehler beim Upload!';
                uploadStatus.style.color = 'var(--error)';
            });

            xhr.open('POST', '/update');
            xhr.send(formData);
        }

        dropZone.addEventListener('dragover', (e) => {
            e.preventDefault();
            dropZone.style.borderColor = 'var(--primary)';
        });
        dropZone.addEventListener('dragleave', () => {
            dropZone.style.borderColor = 'var(--border-card)';
        });
        dropZone.addEventListener('drop', (e) => {
            e.preventDefault();
            dropZone.style.borderColor = 'var(--border-card)';
            const file = e.dataTransfer.files[0];
            if (file && file.name.endsWith('.bin')) {
                uploadFile(file);
            } else {
                alert('Bitte nur .bin Dateien hochladen!');
            }
        });

        setInterval(updateStatus, 1000);
        setInterval(updateLogs, 1500);
        updateStatus();
        updateLogs();
    </script>
</body>
</html>
)rawliteral";

void handleStatus() {
  StaticJsonDocument<1024> doc;
  float progVal = progress;
  int remVal = 0;
  if (currentState == 'D') {
    progVal = 100.0f;
    remVal = 0;
  } else if (totalEstimatedTime > 0) {
    unsigned long elapsed = 0;
    if (spoolingStartTime > 0) elapsed = millis() - spoolingStartTime;
    else if (pausedElapsed > 0) elapsed = pausedElapsed;
    else if (currentState == 'A') elapsed = pausedElapsed;
    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);
    
    if (effectiveTotal == 0) {
      progVal = 0.0f;
      remVal = 0;
    } else if (elapsed >= effectiveTotal) {
      progVal = 100.0f;
      remVal = 0;
    } else {
      progVal = (100.0f * (float)elapsed) / (float)effectiveTotal;
      remVal = (int)((effectiveTotal - elapsed) / 1000UL);
    }
  }
  if (remHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < remHoldExpiry) remVal = lastRemainingTime;
    else remHoldActive = false;
  }
  if (progHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < progHoldExpiry) progVal = lastProgress;
    else progHoldActive = false;
  }
  if (currentState == 'R') {
    unsigned long nowMs = millis();
    if (nowMs < remStartHoldUntil) remVal = lastRemainingTime;
  }
  
  doc["STAT"] = String(currentState);
  doc["HAS_FIL"] = filamentDetected;
  doc["USE_FIL"] = useFilamentSensor;
  doc["PROG"] = progVal;
  doc["REM"] = remVal;
  doc["TEMP"] = (int)temperatureRead();
  doc["WIFI_SSID"] = wifiSSID.length() > 0 ? wifiSSID.c_str() : nullptr;
  doc["WIFI_OK"] = WiFi.status() == WL_CONNECTED;
  doc["FW"] = FIRMWARE_VERSION;
  doc["SPD"] = speedPercent;
  doc["JIN"] = jingleStyle;
  doc["LED"] = ledBrightness;
  doc["DIR"] = motorDirection;
  doc["POW"] = motorStrength;
  doc["TRQ"] = highSpeedMode ? 0 : torqueLimit;
  doc["WGT"] = targetWeight;
  doc["DUR"] = calibrationAt80Speed / 1000;
  doc["HS"] = highSpeedMode;
  doc["FAN_SPD"] = fanSpeed;
  doc["FAN_ON"] = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter);
  doc["FAN_ALW"] = fanAlwaysOn;
  doc["SV_R"] = servoAngleR;
  doc["SV_L"] = servoAngleL;
  doc["SV_STP"] = servoStepMm;
  doc["SV_HOME"] = servoHomeIsR ? "R" : "L";
  doc["VAR"] = (boardVariant == 2) ? "PRO" : (boardVariant == 1) ? "STD" : "UNK";
  doc["BOOT_REASON"] = getResetReasonString();
  uint32_t ioin = driver.IOIN();
  doc["TMC_OK"] = (ioin != 0 && ioin != 0xFFFFFFFF);
  doc["TMC_IOIN"] = "0x" + String(ioin, HEX);
  doc["HEAP"] = ESP.getFreeHeap();
  
  String jsonOut;
  serializeJson(doc, jsonOut);
  webServer.send(200, "application/json", jsonOut);
}

void handleLogs() {
  String json = "{\"logs\":[";
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  int idx = (logHead - logCount + MAX_LOG_LINES) % MAX_LOG_LINES;
  for (int i = 0; i < logCount; i++) {
    String escaped = logLines[idx];
    escaped.replace("\"", "\\\"");
    escaped.replace("\n", "\\n");
    escaped.replace("\r", "\\r");
    json += "\"" + escaped + "\"";
    if (i < logCount - 1) json += ",";
    idx = (idx + 1) % MAX_LOG_LINES;
  }
  if (logMutex) xSemaphoreGive(logMutex);
  json += "]}";
  webServer.send(200, "application/json", json);
}

void handleWebCommand() {
  if (webServer.hasArg("plain")) {
    String body = webServer.arg("plain");
    handleCommand(body.c_str());
    webServer.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    webServer.send(400, "application/json", "{\"status\":\"error\"}");
  }
}

void handleCommand(const std::string& cmd) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, cmd.c_str());
  if (error) return;

  if (doc.containsKey("CMD")) {
    String command = doc["CMD"].as<String>();
    if (command == "START") {
      if (!isMotorRunning) {
        shouldStartMotorNow = true;
        delayStartUntil = millis();
      }
    } else if (command == "STOP") {
      if (otaInProgress) {
        return;
      }
      if (currentState == 'A') {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        shouldStartMotorNow = false;
        pendingDirectionChange = false;
        triggerJingleNow = 0;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        filamentLostSince = 0;
        stepperStop();
        ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
        gpio_reset_pin((gpio_num_t)STEP_PIN);
        gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
        digitalWrite(STEP_PIN, LOW);
        digitalWrite(EN_PIN, HIGH);
        {
          ledc_channel_config_t step_ledc_chan = {
            .gpio_num = STEP_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = STEP_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = STEP_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
          };
          ledc_channel_config(&step_ledc_chan);
        }
        currentState = 'I';
        sendStatus(true);
      } else {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        currentState = 'I';
        sendStatus(true);
      }
    } else if (command == "PAUSE") {
      if (isMotorRunning) {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else if (command == "WIFI_SCAN") {
      if (wifiScanTaskHandle == nullptr) {
        xTaskCreate(wifiScanTask, "WiFiScan", 4096, NULL, 1, &wifiScanTaskHandle);
      }
    } else if (command == "OTA") {
      if (!otaInProgress) {
        sendOTAUpdate();
      }
    } else if (command == "WIFI_CONNECT") {
      if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        WiFi.disconnect(true);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        wifiConnectInProgress = true;
        wifiConnectStartTime = millis();
      }
    }
  }
  if (doc.containsKey("SET")) {
    JsonObject set = doc["SET"].as<JsonObject>();
    bool changed = false;
    bool servoEndpointsChanged = false;
    if (set.containsKey("VAR")) {
      JsonVariant vj = set["VAR"];
      uint8_t newVar = boardVariant;
      if (vj.is<const char*>()) {
        const char* s = vj.as<const char*>();
        if (s && s[0]) {
          String up = String(s);
          up.toUpperCase();
          if (up == "PRO") newVar = 2;
          else if (up == "STD" || up == "STANDARD") newVar = 1;
          else if (up == "UNK" || up == "UNKNOWN") newVar = 0;
        }
      } else {
        int vi = vj.as<int>();
        if (vi < 0) vi = 0;
        if (vi > 2) vi = 2;
        newVar = (uint8_t)vi;
      }
      if (newVar != boardVariant) {
        boardVariant = newVar;
        changed = true;
      }
    }
    if (set.containsKey("DIR")) {
      int dir = set["DIR"];
      if (dir != motorDirection) {
        if (isMotorRunning) {
          pendingDirectionChange = true;
          newMotorDirection = dir;
          isMotorRunning = false;
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          digitalWrite(STEP_PIN, LOW);
        } else {
          motorDirection = dir;
          digitalWrite(DIR_PIN, motorDirection);
          prefs.begin("respooler", false);
          prefs.putUInt("dir", motorDirection);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("LED")) {
      ledBrightness = constrain((int)set["LED"], 0, 100);
      pwmValue = calculatePWM(ledBrightness);
      prefs.begin("respooler", false);
      prefs.putUInt("led", ledBrightness);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("USE_FIL")) {
      useFilamentSensor = (int)set["USE_FIL"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("filamentSensor", useFilamentSensor);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("POW")) {
      motorStrength = constrain((int)set["POW"], 40, 120);
      setMotorCurrent(motorStrength);
      prefs.begin("respooler", false);
      prefs.putUInt("motorStrength", motorStrength);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("TRQ")) {
      int newTrq = constrain((int)set["TRQ"], 0, 3);
      if (highSpeedMode) {
        savedTorqueLimitHS = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", savedTorqueLimitHS);
        prefs.end();
        torqueLimit = 0;
      } else {
        torqueLimit = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", torqueLimit);
        prefs.end();
      }
      changed = true;
    }
    if (set.containsKey("JIN")) {
      int newJingleStyle = constrain((int)set["JIN"], 0, 3);
      if (newJingleStyle != jingleStyle) {
        jingleStyle = newJingleStyle;
        prefs.begin("respooler", false);
        prefs.putUInt("jingle", jingleStyle);
        prefs.end();
        if (!isMotorRunning) triggerJingleNow = jingleStyle;
        changed = true;
      }
    }
    if (set.containsKey("DUR")) {
      unsigned long dur = (unsigned long)set["DUR"];
      if (dur >= 10 && dur <= 20000) {
        calibrationAt80Speed = dur * 1000UL;
        prefs.begin("respooler", false);
        prefs.putULong("cal80_093", calibrationAt80Speed);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("WGT")) {
      int newWgt = constrain((int)set["WGT"], 0, 3);
      targetWeight = newWgt;
      prefs.begin("respooler", false);
      prefs.putUInt("targetWeight", (unsigned int)targetWeight);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("SPD")) {
      int speed = constrain((int)set["SPD"], 50, 100);
      double span = 170.0 - 93.0;
      double baseInterval = 170.0 - ((double)(speed - 50) * span / 50.0);
      baseTargetStepIntervalMicros = (unsigned long)lround(baseInterval);
      speedPercent = speed;
      prefs.begin("respooler", false);
      prefs.putUInt("speed", baseTargetStepIntervalMicros);
      prefs.end();
      applySpeedTarget();
      changed = true;
    }
    if (set.containsKey("HS")) {
      bool newHS = (int)set["HS"] != 0;
      if (newHS != highSpeedMode) {
        if (newHS) {
          highSpeedMode = true;
          savedTorqueLimitHS = torqueLimit;
          torqueLimit = 0;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", true);
          prefs.end();
        } else {
          highSpeedMode = false;
          torqueLimit = savedTorqueLimitHS;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", false);
          prefs.putUInt("torqueLimit", torqueLimit);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("FAN_SPD")) {
      fanSpeed = constrain((int)set["FAN_SPD"], 0, 100);
      prefs.begin("respooler", false);
      prefs.putUInt("fanSpeed", fanSpeed);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("FAN_ALW")) {
      fanAlwaysOn = (int)set["FAN_ALW"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("fanAlways", fanAlwaysOn);
      prefs.end();
      changed = true;
    }

    if (set.containsKey("SV_R")) {
      int r = constrain((int)set["SV_R"], 0, 180);
      if (r != servoAngleR) {
        servoAngleR = r;
        prefs.begin("respooler", false);
        prefs.putUInt("servoR", (unsigned int)servoAngleR);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'R') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_L")) {
      int l = constrain((int)set["SV_L"], 0, 180);
      if (l != servoAngleL) {
        servoAngleL = l;
        prefs.begin("respooler", false);
        prefs.putUInt("servoL", (unsigned int)servoAngleL);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'L') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_STP")) {
      float s = (float)set["SV_STP"].as<float>();
      s = constrain(s, 0.05f, 20.0f);
      if (fabsf(s - servoStepMm) > 0.0001f) {
        servoStepMm = s;
        prefs.begin("respooler", false);
        prefs.putFloat("servoStep", servoStepMm);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SV_HOME")) {
      const char* v = set["SV_HOME"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);
      if (c == 'R' || c == 'L') {
        bool homeR = (c == 'R');
        if (homeR != servoHomeIsR) {
          servoHomeIsR = homeR;
          prefs.begin("respooler", false);
          prefs.putBool("servoHomeR", servoHomeIsR);
          prefs.end();
          servoEndpointsChanged = true;
          changed = true;
        }
      }
    }
    if (set.containsKey("SV_GOTO") && !isMotorRunning && (currentState == 'I' || currentState == 'D')) {
      const char* v = set["SV_GOTO"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);

      if (c == 'L' || c == 'R' || c == 'H') {
        servoGotoSide = c;
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (servoEndpointsChanged && !isMotorRunning && (currentState == 'I' || currentState == 'D') && !servoGotoActive && !(servoGotoSide == 'L' || servoGotoSide == 'R')) {
      guideServoTowardsL = servoHomeIsR;
      servoHomingToHome = true;
      lastServoHomeStepTime = millis();
    }
    if (set.containsKey("WIFI_SSID")) {
      wifiSSID = set["WIFI_SSID"].as<String>();
      changed = true;
    }
    if (set.containsKey("WIFI_PASS")) {
      wifiPassword = set["WIFI_PASS"].as<String>();
      changed = true;
    }
    if (changed) {
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.putUChar("variant", boardVariant);
      prefs.end();
      sendStatus(true);
    }
  }
}

// ------------------- BLE callback classes ------------------------
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo&) override {
    deviceConnected = true;
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
    xTaskCreate([](void*) {
      delay(1100);
      sendStatus(true);
      vTaskDelete(NULL);
    },
                "delayedSend", 2048, NULL, 1, NULL);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo&, int) override {
    deviceConnected = pServer->getConnectedCount() > 0;
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
  }
};
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    std::string rx = pChar->getValue();
    if (!rx.empty()) handleCommand(rx);
  }
};

// -------------------------- Setup --------------------------------
void setup() {
  logMutex = xSemaphoreCreateMutex();
  addToLog("[System] LTS-Respooler startet... Reset-Grund: " + getResetReasonString());
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_coex_preference_set(ESP_COEX_PREFER_BT);
  pinMode(FILAMENT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_CONN_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = SERVO_LEDC_TIMER,
    .freq_hz = 50,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&servo_timer);

  ledc_channel_config_t servo_channel = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = SERVO_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = SERVO_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&servo_channel);

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = FAN_TIMER,
    .freq_hz = 35000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = FAN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = FAN_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = FAN_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);

  ledc_timer_config_t conn_led_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&conn_led_timer);

  ledc_channel_config_t conn_led_conf = {
    .gpio_num = LED_CONN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&conn_led_conf);

  ledc_timer_config_t step_ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = STEP_LEDC_TIMER,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&step_ledc_timer);

  ledc_channel_config_t step_ledc_chan = {
    .gpio_num = STEP_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = STEP_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = STEP_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&step_ledc_chan);

  led.begin();
  led.clear();
  led.show();

  Serial.begin(115200);
  TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
#if defined(BOARD_VARIANT_S3) || defined(BOARD_VARIANT_C3)
  // Single-wire UART configuration for TMC2209
  gpio_set_direction((gpio_num_t)TMC_UART_TX, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_pullup_en((gpio_num_t)TMC_UART_TX);
#endif

  driver.begin();
  uint32_t ioin = driver.IOIN();
  addToLog("[TMC2209] Initialisierung... IOIN: 0x" + String(ioin, HEX) + ((ioin == 0 || ioin == 0xFFFFFFFF) ? " (Verbindung FEHLGESCHLAGEN)" : " (Verbindung ERFOLGREICH)"));
  driver.toff(3);
  driver.microsteps(MICROSTEPPING);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.SGTHRS(40);
  driver.semin(0);
  driver.TCOOLTHRS(0xFFFFF);
  uint16_t cool = driver.COOLCONF();
  driver.COOLCONF(cool | (1 << 13));

  prefs.begin("respooler", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPassword = prefs.getString("pwd", "");
  prefs.end();

  if (wifiSSID.length() == 0) {
    wifiSSID = "SkyNet";
    wifiPassword = "123456798aB!";
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  }
  bleNotifyMutex = xSemaphoreCreateMutex();
  loadSettings();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);

  guideServoPos = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
  guideServoTowardsL = servoHomeIsR;
  setServoAngle((int)lroundf(guideServoPos));

  NimBLEDevice::init(BOARD_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P6);
  NimBLEDevice::setMTU(512);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName(BOARD_NAME);
  advData.addServiceUUID(SERVICE_UUID);

  std::string manufacturerData;
  if (boardVariant == 2) {
    manufacturerData = "\xFF\xFF\x02";
  } else {
    manufacturerData = "\xFF\xFF\x01";
  }
  advData.setManufacturerData(manufacturerData);

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  setConnLed(150);
  currentState = 'I';

  // Setup Web Server
  webServer.on("/", HTTP_GET, []() {
    webServer.send(200, "text/html", index_html);
  });
  webServer.on("/status", HTTP_GET, handleStatus);
  webServer.on("/logs", HTTP_GET, handleLogs);
  webServer.on("/api/command", HTTP_POST, handleWebCommand);
  webServer.on("/update", HTTP_POST, []() {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      addToLog("[OTA] Firmware-Upload startet: " + upload.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        addToLog("[OTA] Fehler: " + String(Update.errorString()));
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        addToLog("[OTA] Schreibfehler bei Chunk");
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        addToLog("[OTA] Upload fertig! Groesse: " + String(upload.totalSize) + " Bytes. Reboot...");
      } else {
        addToLog("[OTA] Fehler beim Beenden: " + String(Update.errorString()));
      }
    }
  });
  webServer.begin();
  addToLog("[System] WebServer gestartet auf Port 80.");
}

// ------------------- Main loop -----------------------------------
void loop() {
  webServer.handleClient();
  unsigned long now = millis();
  static char lastStateLogged = ' ';
  if (currentState != lastStateLogged) {
    String stateStr;
    switch (currentState) {
      case 'I': stateStr = "Bereit (Idle)"; break;
      case 'P': stateStr = "Pausiert"; break;
      case 'A': stateStr = "Auto-Stop (StallGuard)"; break;
      case 'D': stateStr = "Wickeln fertig (Done)"; break;
      case 'R': stateStr = "Rampe (Running)"; break;
      case 'U': stateStr = "OTA Update... Bitte warten"; break;
      default: stateStr = String(currentState); break;
    }
    addToLog("[System] Zustand gewechselt zu: " + stateStr);
    lastStateLogged = currentState;
  }
  static bool lastWifiConnected = false;
  bool currentWifiConnected = WiFi.status() == WL_CONNECTED;
  if (currentWifiConnected != lastWifiConnected) {
    lastWifiConnected = currentWifiConnected;
    if (currentWifiConnected) {
      addToLog("[System] WiFi verbunden! IP: " + WiFi.localIP().toString() + " RSSI: " + String(WiFi.RSSI()) + " dBm");
    } else {
      addToLog("[System] WiFi Verbindung verloren.");
    }
  }
  static bool lastFilamentLogState = false;
  if (filamentDetected != lastFilamentLogState) {
    lastFilamentLogState = filamentDetected;
    if (filamentDetected) {
      addToLog("[Sensor] Filament erkannt.");
    } else {
      addToLog("[Sensor] Filament nicht erkannt (fehlt oder Runout).");
    }
  }
  pollSerialCommands();

  // ------------------ Wi-Fi connect async check -----------------
  if (wifiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = true;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.end();
    } else if (millis() - wifiConnectStartTime > 10000) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
  }

  // ------------------ Filament sensor -----------------
  bool rawFilamentPresent = digitalRead(FILAMENT_PIN) == LOW;
  if (rawFilamentPresent) {
    filamentLostSince = 0;
    filamentDetected = true;
  } else {
    if (filamentLostSince == 0) {
      filamentLostSince = now;
    }
    if (filamentDetected && (now - filamentLostSince >= FILAMENT_LOSS_CONFIRM_MS)) {
      filamentDetected = false;
    }
  }

  if (useFilamentSensor && (currentState == 'P' || currentState == 'A') && !filamentDetected) {
    addToLog("[Sensor] Filament im Pausen- oder Alarmzustand verloren.");
    isMotorRunning = false;
    shouldStartMotorNow = false;
    pendingDirectionChange = false;
    triggerJingleNow = 0;
    remHoldActive = true;
    remHoldExpiry = millis() + 200;
    progHoldActive = true;
    progHoldExpiry = millis() + 200;
    progress = 0.0;
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    pausedElapsed = 0;
    stepperStop();
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    currentState = 'I';
    sendStatus(true);
  }

  // ------------------ Motor/LED state -----------------
  if (!otaInProgress) {
    if (isMotorRunning) {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }

        led.setPixelColor(0, led.Color(0, baseVal, 0));
        led.show();
      }
    } else if (currentState == 'P') {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }
        led.setPixelColor(0, led.Color(baseVal, baseVal / 2, 0));
        led.show();
      }
    } else if (currentState == 'A') {
      if (millis() - lastLedToggle > 400) {
        lastLedToggle = now;
        ledState = !ledState;
        led.setPixelColor(0, ledState ? led.Color(pwmValue, 0, 0) : 0);
        led.show();
      }
    } else if (useFilamentSensor && filamentDetected) {
      led.setPixelColor(0, led.Color(0, pwmValue, 0));
      led.show();
    } else {
      led.setPixelColor(0, led.Color(0, 0, pwmValue));
      led.show();
    }
  }

  // ----------------- Torque Auto-Stop -----------------
  static unsigned long lastTorqueCheck = 0;
  static unsigned long torqueBelowStartTime = 0;
  if (torqueLimit > 0 && isMotorRunning && stepIntervalMicros == targetStepIntervalMicros) {
    if (now - lastTorqueCheck >= TORQUE_CHECK_INTERVAL_MS) {
      lastTorqueCheck = now;
      uint16_t sg = driver.SG_RESULT();
      if (sg > TORQUE_SG_IGNORE) {

        float strengthScale = 1.0;
        int baseLimit = 350 * strengthScale;
        int limit = 999;
        if (torqueLimit == 1) limit = baseLimit * TORQUE_SG_LIMIT_LOW;
        else if (torqueLimit == 2) limit = baseLimit * TORQUE_SG_LIMIT_MED;
        else if (torqueLimit == 3) limit = baseLimit * TORQUE_SG_LIMIT_HIGH;

        int currSpeedPercent = getSpeedPercentFromInterval(stepIntervalMicros);
        if (currSpeedPercent > 94) {
          limit = limit * 0.95;
        } else if (currSpeedPercent > 85) {
          limit = limit * 0.97;
        } else if (currSpeedPercent < 65) {
          limit = limit * 0.98;
        }

        if (sg < limit) {
          if (torqueBelowStartTime == 0) torqueBelowStartTime = now;
          else if (now - torqueBelowStartTime >= TORQUE_BELOW_MS) {
            addToLog("[StallGuard] Blockade erkannt! SG-Wert: " + String(sg) + ", Limit: " + String(limit) + ". Motor stoppt.");
            isMotorRunning = false;
            currentState = 'A';
            sendStatus(true);
            digitalWrite(STEP_PIN, LOW);
            delay(100);
            digitalWrite(EN_PIN, HIGH);
            unsigned long elapsed = millis() - spoolingStartTime;
            spoolingStartTime = 0;
            pausedElapsed = elapsed;
            torqueBelowStartTime = 0;
            stepperStop();
            ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
            gpio_reset_pin((gpio_num_t)STEP_PIN);
            gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
            digitalWrite(EN_PIN, LOW);
            digitalWrite(STEP_PIN, LOW);
            delay(120);
            {
              led.setPixelColor(0, led.Color(pwmValue, 0, 0));
              led.show();
            }
            for (int i = 0; i < 3; i++) {
              playTone(850, 250);
              delay(100);
            }
            digitalWrite(EN_PIN, HIGH);
            {
              ledc_channel_config_t step_ledc_chan = {
                .gpio_num = STEP_PIN,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = STEP_LEDC_CHANNEL,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = STEP_LEDC_TIMER,
                .duty = 0,
                .hpoint = 0
              };
              ledc_channel_config(&step_ledc_chan);
            }
            stepperStop();
          }
        } else {
          torqueBelowStartTime = 0;
        }
      }
    }
  } else {
    torqueBelowStartTime = 0;
  }

  static unsigned long wgtFinishStart = 0;
  if (isMotorRunning && targetWeight >= 1 && targetWeight <= 3 && totalEstimatedTime > 0) {
    unsigned long elapsed = 0;
    if (spoolingStartTime > 0) elapsed = now - spoolingStartTime;
    else if (pausedElapsed > 0) elapsed = pausedElapsed;
    else if (currentState == 'A') elapsed = pausedElapsed;

    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);

    if (effectiveTotal == 0) {
      wgtFinishStart = 0;
    } else if (elapsed >= effectiveTotal) {
      if (!(useFilamentSensor && !filamentDetected)) {
        if (wgtFinishStart == 0) wgtFinishStart = now;
        if (now - wgtFinishStart >= 700) {
          isMotorRunning = false;
          digitalWrite(STEP_PIN, LOW);
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          totalEstimatedTime = 0;
          spoolingStartTime = 0;
          currentState = 'D';
          sendStatus(true);
          progress = 100.0f;
          sendStatus(true);
          doneHoldActive = true;
          doneHoldStart = millis();
          playStepperJingle();
          wgtFinishStart = 0;
        }
      } else {
        wgtFinishStart = 0;
      }
    } else {
      wgtFinishStart = 0;
    }
  }


  if (isMotorRunning && useFilamentSensor && !filamentDetected && filamentLostSince > 0 && now - filamentLostSince > FILAMENT_LOSS_CONFIRM_MS) {
    addToLog("[Sensor] Filament-Runout waehrend des Betriebs erkannt! Spulvorgang beendet.");
    isMotorRunning = false;
    digitalWrite(STEP_PIN, LOW);
    stepperStop();
    delay(40);
    digitalWrite(EN_PIN, HIGH);
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    currentState = 'D';
    sendStatus(true);
    progress = 100.0f;
    sendStatus(true);
    doneHoldActive = true;
    doneHoldStart = millis();
    delay(300);
    playStepperJingle();
  }
  if (doneHoldActive && (millis() - doneHoldStart >= 20000)) {
    progress = 0.0;
    currentState = 'I';
    sendStatus(true);
    doneHoldActive = false;
  }

  // ------- Direction change after stop -----------
  if (pendingDirectionChange && !isMotorRunning) {
    motorDirection = newMotorDirection;
    prefs.begin("respooler", false);
    prefs.putUInt("dir", motorDirection);
    prefs.end();
    digitalWrite(DIR_PIN, motorDirection);
    delayStartUntil = now + 1000;
    shouldStartMotorNow = true;
    pendingDirectionChange = false;
  }

  // ------------ Motor start logic ----------------
  if (shouldStartMotorNow && now >= delayStartUntil) {
    if (otaInProgress) {
      shouldStartMotorNow = false;
      return;
    }
    if (isMotorRunning) {
      shouldStartMotorNow = false;
    } else if (!useFilamentSensor || filamentDetected) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)targetStepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      totalEstimatedTime = calibrationAt80Speed * (80.0 / currSpeedPercent);
      if (pausedElapsed > 0) {
        spoolingStartTime = millis() - pausedElapsed;
        pausedElapsed = 0;
      } else {
        spoolingStartTime = millis();
      }
      currentState = 'R';
      sendStatus(true);
      remStartHoldUntil = millis() + 800;
      doneHoldActive = false;
      stepperStop();
      reSanitizeDriver();
      digitalWrite(DIR_PIN, motorDirection);
      digitalWrite(EN_PIN, LOW);
      digitalWrite(STEP_PIN, LOW);
      delay(15);
      {
        uint16_t prevI = driver.rms_current();
        driver.en_spreadCycle(true);
        int boosted = prevI + prevI / 4;
        if (boosted > 1600) boosted = 1600;
        driver.rms_current(boosted);
        stepperStart(stepFreqFromIntervalUs(1800));
        delay(150);
        driver.rms_current(prevI);
        driver.en_spreadCycle(false);
      }
      stepIntervalMicros = START_INTERVAL_US;
      lastAccelUpdate = now;
      isMotorRunning = true;
      addToLog("[Motor] Start gewaehlt. Ziel-Geschwindigkeit: " + String(speedPercent) + "%");
      stepperStart(stepFreqFromIntervalUs(stepIntervalMicros));
    }
    shouldStartMotorNow = false;
  }

  // ------------ Button handling with debounce ------------
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastStableButtonState && now - lastDebounceTime > BUTTON_DEBOUNCE_MS) {
    lastDebounceTime = now;

    if (reading == LOW) {
      if (!isMotorRunning) {
        if (currentState == 'P' || currentState == 'A') {
          buttonPressSince = now;
          buttonLongPressHandled = false;
          buttonAwaitingDecision = true;
        } else if (useFilamentSensor && !filamentDetected) {
          if (!otaInProgress) {
            for (int i = 0; i < 5; i++) {
              led.setPixelColor(0, led.Color(0, 0, pwmValue));
              led.show();
              delay(100);
              led.setPixelColor(0, 0);
              led.show();
              delay(100);
            }
          }
        } else {
          shouldStartMotorNow = true;
          delayStartUntil = now;
        }
      } else {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else {
      if (buttonAwaitingDecision) {
        if (!buttonLongPressHandled) {
          if (!(useFilamentSensor && !filamentDetected)) {
            shouldStartMotorNow = true;
            delayStartUntil = now;
          }
        }
        buttonAwaitingDecision = false;
      }
    }
    lastStableButtonState = reading;
  }
  if (buttonAwaitingDecision && !buttonLongPressHandled && (digitalRead(BUTTON_PIN) == LOW)) {
    if (now - buttonPressSince >= 1000) {
      remHoldActive = true;
      remHoldExpiry = millis() + 200;
      progHoldActive = true;
      progHoldExpiry = millis() + 200;
      isMotorRunning = false;
      progress = 0.0;
      totalEstimatedTime = 0;
      spoolingStartTime = 0;
      pausedElapsed = 0;
      stepperStop();
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      currentState = 'I';
      sendStatus(true);
      buttonLongPressHandled = true;
    }
  }

  // ----------- Motor acceleration (speed ramp) --------------
  if (isMotorRunning && now - lastAccelUpdate > ACCEL_UPDATE_INTERVAL) {
    lastAccelUpdate = now;
    if (stepIntervalMicros > targetStepIntervalMicros) {
      stepIntervalMicros -= ACCEL_STEP;
      if (stepIntervalMicros < targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    } else if (stepIntervalMicros < targetStepIntervalMicros) {
      stepIntervalMicros += ACCEL_STEP;
      if (stepIntervalMicros > targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    }
    if (stepIntervalMicros != lastIntervalMicros) {
      stepperSetFreq(stepFreqFromIntervalUs(stepIntervalMicros));
      lastIntervalMicros = stepIntervalMicros;
    }
    if (stepIntervalMicros == targetStepIntervalMicros) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)stepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      unsigned long newTotal = calibrationAt80Speed * (80.0 / currSpeedPercent);
      unsigned long elapsed = millis() - spoolingStartTime;
      if (totalEstimatedTime > 0) {
        float progressRatio = (float)elapsed / totalEstimatedTime;
        spoolingStartTime = millis() - (newTotal * progressRatio);
      }
      totalEstimatedTime = newTotal;
    }
  }

  // ------------- Connection LED animation ------------------
  if (!otaInProgress) {
    static int connLedPulse = 0;
    static int connLedDir = 2;

    if (!deviceConnected) {
      if (now - lastConnLedTime > 25) {
        lastConnLedTime = now;
        connLedPulse += connLedDir;
        if (connLedPulse >= LED_CONN_PULSE_MAX) {
          connLedPulse = LED_CONN_PULSE_MAX;
          connLedDir = -2;
        } else if (connLedPulse <= 0) {
          connLedPulse = 0;
          connLedDir = 2;
        }
        setConnLed(connLedPulse);
      }
    } else {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LED_CONN_PULSE_MAX);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
  }

  // ---------------------- Jingle trigger ---------------------
  if (triggerJingleNow != 0) {
    playStepperJingle();
    triggerJingleNow = 0;
  }

  // ----------- Restart BLE advertising if disconnected --------------
  if (!deviceConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::startAdvertising();
  }

  // ------------------ Guide servo movement -----------------
  if (isMotorRunning) {
    if (!servoWasRunning) {
      servoWasRunning = true;
      lastMotorStepAccTime = now;
      servoHomingToHome = false;
      servoGotoActive = false;
    } else {
      unsigned long dtMs = now - lastMotorStepAccTime;
      if (dtMs > 0) {
        lastMotorStepAccTime = now;

        float stepsPerSec = (float)stepFreqFromIntervalUs(stepIntervalMicros);
        motorStepAcc += stepsPerSec * ((float)dtMs / 1000.0f);
        float stepsPerSpoolRev = (200.0f * (float)MICROSTEPPING) * (52.0f / 18.0f);

        while (motorStepAcc >= stepsPerSpoolRev) {
          motorStepAcc -= stepsPerSpoolRev;

          float target = guideServoTowardsL ? (float)servoAngleL : (float)servoAngleR;
          float stepDeg = servoStepMm * (360.0f / (35.0f * 3.77f));
          if (stepDeg < 0.05f) stepDeg = 0.05f;

          if (fabsf(guideServoPos - target) <= stepDeg) {
            guideServoPos = target;
            setServoAngle((int)lroundf(guideServoPos));
            guideServoTowardsL = !guideServoTowardsL;
          } else {
            guideServoPos += (guideServoPos < target) ? stepDeg : -stepDeg;
            setServoAngle((int)lroundf(guideServoPos));
          }
        }
      }
    }
  } else {
    servoWasRunning = false;
  }

  if (currentState == 'I' && lastStateForServo != 'I') {
    guideServoTowardsL = servoHomeIsR;
    motorStepAcc = 0.0f;
    servoWasRunning = false;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
  }
  lastStateForServo = currentState;
  
  // ------------------ Respooler done ------------------
  if (currentState == 'D' && lastStateForDone != 'D') {
    servoHomeAfterDonePending = true;
  }
  if (currentState != 'D' && lastStateForDone == 'D') {
    servoHomeAfterDonePending = false;
  }
  lastStateForDone = currentState;

  if (servoHomeAfterDonePending && currentState == 'D' && !isMotorRunning) {
    guideServoTowardsL = servoHomeIsR;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
    servoHomeAfterDonePending = false;
  }

  // ------------------ Servo goto ------------------
  if (!isMotorRunning && servoGotoActive) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float target;
      if (servoGotoSide == 'L') target = (float)servoAngleL;
      else if (servoGotoSide == 'R') target = (float)servoAngleR;
      else if (servoGotoSide == 'H') target = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      else {
        servoGotoActive = false;
        return;
      }

      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - target) <= stepDeg) {
        guideServoPos = target;
        setServoAngle((int)lroundf(guideServoPos));
        servoGotoActive = false;
      } else {
        guideServoPos += (guideServoPos < target) ? stepDeg : -stepDeg;
        setServoAngle((int)lroundf(guideServoPos));
      }
    }
  }

  // ------------------ Servo homing ------------------
  if (!isMotorRunning && servoHomingToHome) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float targetHome = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - targetHome) <= stepDeg) {
        guideServoPos = targetHome;
        setServoAngle((int)lroundf(guideServoPos));
        servoHomingToHome = false;
      } else {
        guideServoPos += (guideServoPos < targetHome) ? stepDeg : -stepDeg;
        setServoAngle((int)lroundf(guideServoPos));
      }
    }
  }

  // ---------------- Fan control logic ----------------
  int fanPWM = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter) ? fanPWMFromSpeed(fanSpeed) : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, fanPWM);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  if (!isMotorRunning && fanPWM > 0 && fanStopAfter == 0) {
    fanStopAfter = millis() + 10000;
  }
  if (isMotorRunning) fanStopAfter = millis() + 10000;
  if (!fanAlwaysOn && !isMotorRunning && millis() >= fanStopAfter) {
    fanStopAfter = 0;
  }

  // ----------- Periodic Status-Update every 500 ms -----------
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 500) {
    sendStatus();
    lastStatusTime = millis();
  }
}
