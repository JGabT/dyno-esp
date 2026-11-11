/*
  dyno-esp.ino
  Arduino Nano ESP32 + E18-D80NK-N IR sensor + Daly BMS over BLE

  Open WiFi access point, live 5 Hz dyno dashboard.

  Measurement:
    - RPM from time between sensor pulses
    - Omega is moving-averaged (adjustable window) and low-pass filtered
    - Angular acceleration from a short linear fit of filtered omega
    - Torque / power from the inertial-dyno equations:
        tau_mech = inertia * angularAcceleration
        tau_loss = coast-down calibrated friction
        torque   = tau_mech + tau_loss
        power    = torque * angularVelocity

  Logging:
    - Every 200 ms sample is written to a CSV file in on-board flash
      (LittleFS or the Nano ESP32 FAT partition, whichever is available).
    - Browse and download runs at http://192.168.4.1/logs
    - A 5 Hz JSON telemetry line is also written to the USB serial port.

  BMS:
    - Daly smart BMS telemetry over BLE: pack voltage, current, SOC,
      cell max/min, temperatures, remaining capacity, MOS state.
    - Set BMS_MAC_ADDRESS to your BMS.

  The E18-D80NK-N is a 5 V NPN normally-open sensor. Use a voltage
  divider (for example 10 kOhm series + 20 kOhm to GND) between the
  black signal wire and the Nano ESP32 input. The Nano ESP32 is 3.3 V only.
  If the sensor output is open-collector without an internal pull-up,
  add 10 kOhm from the signal wire to 5 V.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <FFat.h>
#include <NimBLEDevice.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Hardware / network configuration
// ---------------------------------------------------------------------------
constexpr uint8_t SENSOR_PIN = D2;            // change to the pin you wire to
constexpr int SENSOR_EDGE = FALLING;          // NPN NO: LOW when target is seen
constexpr char AP_SSID[] = "Dyno-ESP32";      // open network, no password

// Measurement tuning
constexpr uint16_t DEFAULT_PPR = 1;           // sensor pulses per revolution
constexpr uint8_t DEFAULT_AVG_WINDOW = 8;     // omega moving-average samples
constexpr float DEFAULT_FRICTION = 0.0f;      // N*m, coast-down result

// Roller: solid cylinder, 43 kg, 0.15 m radius.
// I = 1/2 * m * r^2 = 0.5 * 43 * 0.15^2 = 0.48375 kg*m^2
constexpr float DEFAULT_ROLLER_MASS_KG = 43.0f;
constexpr float DEFAULT_ROLLER_RADIUS_M = 0.15f;
constexpr float DEFAULT_INERTIA =
    0.5f * DEFAULT_ROLLER_MASS_KG * DEFAULT_ROLLER_RADIUS_M *
    DEFAULT_ROLLER_RADIUS_M;

// E18-D80NK-N debounce and dropout limits. The sensor needs a short gap
// between targets; 2 ms allows up to 500 pulses/s while rejecting switch
// chatter. A long gap is treated as a stopped/restarted roller.
constexpr uint32_t MIN_PULSE_GAP_US = 2000;
constexpr uint32_t PULSE_TIMEOUT_US = 3000000;
constexpr uint32_t SPEED_WINDOW_US = 500000;  // slope window for alpha
constexpr float OMEGA_LPF_TAU_S = 0.05f;      // omega low-pass time constant
constexpr float ALPHA_LPF = 0.35f;            // alpha smoothing per update
constexpr uint8_t SPEED_HISTORY = 128;
constexpr uint8_t PULSE_QUEUE_SIZE = 64;
constexpr uint32_t CONTROL_INTERVAL_MS = 20;  // internal update rate, 50 Hz

// Coast-down friction calibration
constexpr uint32_t CALIB_DURATION_MS = 15000;
constexpr uint32_t CALIB_MIN_SAMPLES = 20;
constexpr uint32_t CALIB_MAX_SAMPLES = 250;
constexpr float CALIB_MIN_OMEGA = 10.0f;      // rad/s
constexpr float CALIB_MAX_OMEGA = 500.0f;     // rad/s
constexpr float CALIB_MIN_ALPHA = -0.01f;     // rad/s^2 (coasting)
constexpr float CALIB_MAX_BMS_CURRENT_A = 2.0f;

// Serial telemetry stream (one JSON line per 5 Hz sample)
constexpr bool SERIAL_TELEMETRY = true;
constexpr uint32_t SERIAL_STREAM_MS = 200;

// CSV logging
constexpr uint32_t LOG_INTERVAL_MS = 200;     // one row per 5 Hz sample
constexpr uint8_t LOG_BUFFER_ROWS = 10;       // flush to flash every 2 s
constexpr float LOG_CLEANUP_RATIO = 0.85f;    // remove oldest run above this

// Daly BMS over BLE
constexpr char BMS_MAC_ADDRESS[] = "d2:1a:01:01:c9:a7";
constexpr char SERVICE_UUID_MAIN[] = "0000fff0-0000-1000-8000-00805f9b34fb";
constexpr char CHAR_UUID_TX[] = "0000fff1-0000-1000-8000-00805f9b34fb";
constexpr char CHAR_UUID_RX[] = "0000fff2-0000-1000-8000-00805f9b34fb";
constexpr uint32_t BMS_QUERY_INTERVAL_MS = 120;
constexpr uint32_t BMS_SCAN_MS = 4000;
constexpr uint32_t BMS_SCAN_RETRY_MS = 6000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
struct Settings {
  float massKg;
  float radiusM;
  float inertia;      // computed from massKg and radiusM
  float friction;     // N*m, coast-down calibration or manual value
  uint16_t ppr;
  uint8_t avgWindow;
};

void saveSettings();

Settings settings;
Preferences preferences;
WebServer server(80);

struct DalyTelemetry {
  float totalVoltage = 0.0f;
  float current = 0.0f;
  float soc = 0.0f;
  uint16_t maxCellMv = 0;
  uint8_t maxCellNo = 0;
  uint16_t minCellMv = 0;
  uint8_t minCellNo = 0;
  float maxTemp = 0.0f;
  float minTemp = 0.0f;
  uint8_t bmsState = 0;
  bool chargeMos = false;
  bool dischargeMos = false;
  uint32_t remCapMah = 0;
  uint8_t cellCount = 12;
  uint16_t cycles = 0;
  uint16_t cellMv[48] = {0};
  bool cellBalancing[48] = {false};
} bmsData;

struct BmsDeviceInfo {
  NimBLEAddress address;
  bool found = false;
} bmsDevInfo;

struct LogRecord {
  uint32_t record;
  uint32_t runMs;
  uint32_t uptimeS;
  float rpm;
  float omega;
  float alpha;
  float torque;
  float power;
  float efficiency;
  bool efficiencyValid;
  bool sensor;
  uint32_t pulses;
  float revolutions;
  float peakRpm;
  float peakTorque;
  float peakPower;
  float inertia;
  float friction;
  uint16_t ppr;
  float massKg;
  float radiusM;
  uint8_t avgWindow;
  bool bmsConnected;
  uint32_t bmsAgeMs;
  float voltage;
  float current;
  float soc;
  uint16_t maxCellMv;
  uint8_t maxCellNo;
  uint16_t minCellMv;
  uint8_t minCellNo;
  float maxTemp;
  float minTemp;
  uint32_t remCapMah;
  uint16_t cycles;
  bool chargeMos;
  bool dischargeMos;
};

portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseQueue[PULSE_QUEUE_SIZE];
volatile uint8_t pulseHead = 0;
volatile uint8_t pulseTail = 0;
volatile uint32_t totalPulses = 0;
volatile uint32_t lastIsrUs = 0;

struct SpeedSample {
  uint32_t t;
  float omega;
};

SpeedSample speedHistory[SPEED_HISTORY];
uint8_t speedHead = 0;
uint8_t speedCount = 0;
SpeedSample filteredHistory[SPEED_HISTORY];
uint8_t filteredHead = 0;
uint8_t filteredCount = 0;
bool omegaFilterReady = false;
float omegaFiltered = 0.0f;
uint32_t lastFilterUs = 0;
bool haveLastPulse = false;
uint32_t lastPulseUs = 0;
uint32_t lastIntervalUs = 0;

float rpm = 0.0f;
float omega = 0.0f;       // rad/s
float alpha = 0.0f;       // rad/s^2
float torque = 0.0f;      // N*m
float power = 0.0f;       // W
float peakRpm = 0.0f;
float peakTorque = 0.0f;
float peakPower = 0.0f;
float efficiency = 0.0f;        // mechanical power / battery power, %
bool efficiencyValid = false;
uint32_t lastBmsVoltageMs = 0;
uint32_t lastControlMs = 0;
uint32_t runStartMs = 0;
uint32_t runStopMs = 0;
bool runActive = true;

// Coast-down friction calibration state
enum CalibState : uint8_t {
  CALIB_IDLE = 0,
  CALIB_RUNNING = 1,
  CALIB_OK = 2,
  CALIB_FAIL = 3
};
CalibState calibState = CALIB_IDLE;
bool calibActive = false;
uint32_t calibStartMs = 0;
uint32_t calibElapsedMs = 0;
uint32_t calibSamples = 0;
float calibTauSum = 0.0f;
float calibLastTau = 0.0f;
float calibResult = 0.0f;

uint32_t lastSerialMs = 0;

// Logging state
fs::FS *logFs = nullptr;
const char *logFsName = "none";
bool loggingReady = false;
char currentLogPath[32] = "";
uint32_t logStartMs = 0;
uint32_t logRecordNum = 0;
uint32_t lastLogMs = 0;
LogRecord logBuffer[LOG_BUFFER_ROWS];
uint8_t logBufferCount = 0;

// BLE client state
bool bleReady = false;
NimBLEClient *bmsClient = nullptr;
NimBLERemoteCharacteristic *bmsTxChar = nullptr;  // BMS -> ESP notifications
NimBLERemoteCharacteristic *bmsRxChar = nullptr;  // ESP -> BMS writes
volatile bool bmsConnected = false;
volatile bool bmsConnectRequested = false;

// Thread-safe BLE receive ring
uint8_t bleRxRing[1024];
volatile size_t bleRingHead = 0;
volatile size_t bleRingTail = 0;
portMUX_TYPE bleRingMux = portMUX_INITIALIZER_UNLOCKED;

// Daly frame assembly
uint8_t frameAssembleBuf[13];
size_t frameAssembleLen = 0;
bool frameBaseDetected = false;
int frameBase = 0;

// ---------------------------------------------------------------------------
// Sensor pulse capture
// ---------------------------------------------------------------------------
void IRAM_ATTR sensorISR() {
  uint32_t now = micros();
  if (now - lastIsrUs < MIN_PULSE_GAP_US) return;
  lastIsrUs = now;

  uint8_t next = (pulseHead + 1) % PULSE_QUEUE_SIZE;
  if (next != pulseTail) {
    pulseQueue[pulseHead] = now;
    pulseHead = next;
  }
  totalPulses++;
}

void pushSpeedSample(uint32_t t, float w) {
  speedHistory[speedHead] = {t, w};
  speedHead = (speedHead + 1) % SPEED_HISTORY;
  if (speedCount < SPEED_HISTORY) speedCount++;
}

void pushFilteredSample(uint32_t t, float w) {
  filteredHistory[filteredHead] = {t, w};
  filteredHead = (filteredHead + 1) % SPEED_HISTORY;
  if (filteredCount < SPEED_HISTORY) filteredCount++;
}

// Smooth the raw pulse-derived omega with a moving average, then a first-order
// low-pass, before the derivative is calculated. The window is in samples and
// is adjustable from the dashboard.
void filterOmega(uint32_t t) {
  uint8_t window = settings.avgWindow;
  if (window < 1) window = 1;
  if (window > speedCount) window = speedCount;
  if (window == 0) return;

  float sum = 0.0f;
  for (uint8_t i = 0; i < window; i++) {
    int idx = (int)speedHead - 1 - i;
    while (idx < 0) idx += SPEED_HISTORY;
    sum += speedHistory[idx].omega;
  }
  float average = sum / (float)window;

  if (!omegaFilterReady) {
    omegaFiltered = average;
    omegaFilterReady = true;
  } else {
    float dt = (t - lastFilterUs) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 1.0f / 50.0f;
    float k = 1.0f - expf(-dt / OMEGA_LPF_TAU_S);
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    omegaFiltered += k * (average - omegaFiltered);
  }
  lastFilterUs = t;
  pushFilteredSample(t, omegaFiltered);
}

void addPulse(uint32_t t) {
  if (!haveLastPulse) {
    haveLastPulse = true;
    lastPulseUs = t;
    lastIntervalUs = 0;
    return;
  }

  uint32_t dt = t - lastPulseUs;
  lastPulseUs = t;

  if (dt < MIN_PULSE_GAP_US) return;

  // Long gap: treat this as a restart and do not use the old speed history.
  if (dt > PULSE_TIMEOUT_US) {
    speedHead = 0;
    speedCount = 0;
    filteredHead = 0;
    filteredCount = 0;
    omegaFilterReady = false;
    lastFilterUs = 0;
    lastIntervalUs = 0;
    return;
  }

  lastIntervalUs = dt;
  if (settings.ppr == 0) return;

  // omega = 2*pi / (seconds per pulse / ppr)
  float w = (2.0f * PI * 1000000.0f) / ((float)dt * (float)settings.ppr);
  pushSpeedSample(t, w);
  filterOmega(t);
}

void processPulses() {
  for (;;) {
    uint32_t t;

    portENTER_CRITICAL(&pulseMux);
    if (pulseTail == pulseHead) {
      portEXIT_CRITICAL(&pulseMux);
      return;
    }
    t = pulseQueue[pulseTail];
    pulseTail = (pulseTail + 1) % PULSE_QUEUE_SIZE;
    portEXIT_CRITICAL(&pulseMux);

    addPulse(t);
  }
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------
void clearSpeedHistory() {
  speedHead = 0;
  speedCount = 0;
  filteredHead = 0;
  filteredCount = 0;
  omegaFilterReady = false;
  omegaFiltered = 0.0f;
  lastFilterUs = 0;
  haveLastPulse = false;
  lastPulseUs = 0;
  lastIntervalUs = 0;
  rpm = 0.0f;
  omega = 0.0f;
  alpha = 0.0f;
  torque = 0.0f;
  power = 0.0f;
}

// Coast-down calibration: while the roller is unpowered and slowing down,
// mechanical torque is I * alpha (negative), so bearing/friction loss is
// tau_loss = -I * alpha. Average it and store it as settings.friction.
void serviceCoastDown(uint32_t nowMs) {
  if (!calibActive) return;

  calibElapsedMs = nowMs - calibStartMs;

  uint32_t bmsAge = lastBmsVoltageMs == 0
                        ? 0xFFFFFFFFu
                        : (uint32_t)(nowMs - lastBmsVoltageMs);
  bool bmsFresh = bmsConnected && lastBmsVoltageMs != 0 && bmsAge < 3000;
  bool unpowered = !bmsFresh || fabsf(bmsData.current) <= CALIB_MAX_BMS_CURRENT_A;

  bool coasting = unpowered && isfinite(alpha) && alpha < CALIB_MIN_ALPHA &&
                  omega >= CALIB_MIN_OMEGA && omega <= CALIB_MAX_OMEGA;

  if (coasting) {
    float tauLoss = -settings.inertia * alpha;
    if (isfinite(tauLoss) && tauLoss > 0.0f && tauLoss < 50.0f) {
      calibTauSum += tauLoss;
      calibSamples++;
      calibLastTau = tauLoss;
    }
  }

  bool finished = calibElapsedMs >= CALIB_DURATION_MS ||
                  calibSamples >= CALIB_MAX_SAMPLES;

  if (finished) {
    calibActive = false;
    if (calibSamples >= CALIB_MIN_SAMPLES) {
      float result = calibTauSum / (float)calibSamples;
      if (!(result >= 0.0f) || result > 100.0f) result = DEFAULT_FRICTION;
      settings.friction = result;
      calibResult = result;
      calibState = CALIB_OK;
      saveSettings();
    } else {
      calibState = CALIB_FAIL;
    }
  }
}

void startCoastDown() {
  calibActive = true;
  calibState = CALIB_RUNNING;
  calibStartMs = millis();
  calibElapsedMs = 0;
  calibSamples = 0;
  calibTauSum = 0.0f;
  calibLastTau = 0.0f;
  calibResult = 0.0f;
}

void cancelCoastDown() {
  if (!calibActive) return;
  calibActive = false;
  calibState = CALIB_IDLE;
  calibElapsedMs = millis() - calibStartMs;
}

void updateTelemetry() {
  processPulses();

  uint32_t nowUs = micros();

  // If pulses stop, speed should fall to zero. Allow about three missed pulses.
  uint32_t stopTimeout = 500000UL;
  uint32_t threeIntervals = lastIntervalUs * 3UL;
  if (threeIntervals > stopTimeout) stopTimeout = threeIntervals;

  bool fresh = haveLastPulse &&
               ((uint32_t)(nowUs - lastPulseUs) <= stopTimeout);
  float rawOmega = 0.0f;
  float rawAlpha = 0.0f;

  if (fresh && filteredCount > 0) {
    uint8_t newestIdx = (filteredHead + SPEED_HISTORY - 1) % SPEED_HISTORY;
    uint32_t newestT = filteredHistory[newestIdx].t;
    float newestOmega = filteredHistory[newestIdx].omega;

    int n = 0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;

    // Walk backwards through the filtered samples inside the fit window.
    for (int i = 0; i < filteredCount; i++) {
      int idx = (int)filteredHead - 1 - i;
      while (idx < 0) idx += SPEED_HISTORY;

      uint32_t age = newestT - filteredHistory[idx].t;
      if (age > SPEED_WINDOW_US) break;

      double x = -(double)age / 1000000.0;
      double y = (double)filteredHistory[idx].omega;
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
      n++;
    }

    if (n >= 3) {
      double denom = (double)n * sxx - sx * sx;
      if (fabs(denom) > 1e-12) {
        double slope = ((double)n * sxy - sx * sy) / denom;
        double intercept = (sy - slope * sx) / (double)n;
        double ageNow = (double)(nowUs - newestT) / 1000000.0;

        rawOmega = (float)(intercept + slope * ageNow);
        if (isfinite(rawOmega) && isfinite(slope) && fabs(slope) < 20000.0) {
          rawAlpha = (float)slope;
        } else {
          rawOmega = newestOmega;
          rawAlpha = 0.0f;
        }
      } else {
        rawOmega = newestOmega;
      }
    } else {
      rawOmega = newestOmega;
    }
  }

  if (!(rawOmega > 0.0f) || !isfinite(rawOmega)) rawOmega = 0.0f;
  if (!isfinite(rawAlpha) || fabs(rawAlpha) > 20000.0f) rawAlpha = 0.0f;

  if (!fresh) {
    rawOmega = 0.0f;
    rawAlpha = 0.0f;
  }

  omega = rawOmega;
  rpm = omega * 60.0f / (2.0f * PI);

  if (fresh) {
    alpha += ALPHA_LPF * (rawAlpha - alpha);
  } else {
    alpha = 0.0f;
  }

  float tq = settings.inertia * alpha;
  if (omega > 1.0f) tq += settings.friction;
  if (tq < 0.0f) tq = 0.0f;

  torque = tq;
  power = torque * omega;

  serviceCoastDown(millis());

  if (fresh) {
    if (rpm > peakRpm) peakRpm = rpm;
    if (torque > peakTorque) peakTorque = torque;
    if (power > peakPower) peakPower = power;
  }

  // Efficiency is mechanical flywheel power divided by BMS battery power.
  // It is only meaningful while the dyno is accelerating and the BMS data is
  // fresh. Battery current is positive on discharge.
  uint32_t bmsAge = lastBmsVoltageMs == 0
                        ? 0xFFFFFFFFu
                        : (uint32_t)(millis() - lastBmsVoltageMs);
  bool bmsFresh = bmsConnected && lastBmsVoltageMs != 0 && bmsAge < 3000;
  float batteryPower = bmsData.totalVoltage * bmsData.current;

  if (bmsFresh && batteryPower > 5.0f && power > 0.0f) {
    float rawEfficiency = power / batteryPower * 100.0f;
    if (isfinite(rawEfficiency) && rawEfficiency >= 0.0f &&
        rawEfficiency <= 150.0f) {
      efficiency += 0.3f * (rawEfficiency - efficiency);
      efficiencyValid = true;
    } else {
      efficiency = 0.0f;
      efficiencyValid = false;
    }
  } else {
    efficiency = 0.0f;
    efficiencyValid = false;
  }
}

// ---------------------------------------------------------------------------
// Persistent settings
// ---------------------------------------------------------------------------
void loadSettings() {
  preferences.begin("dyno", true);
  bool hasMass = preferences.isKey("mass");
  bool hasRadius = preferences.isKey("radius");
  settings.massKg = preferences.getFloat("mass", DEFAULT_ROLLER_MASS_KG);
  settings.radiusM = preferences.getFloat("radius", DEFAULT_ROLLER_RADIUS_M);

  // Upgrade from the older firmware that stored only an inertia value.
  // Keep the same inertia by deriving an equivalent radius for the default
  // roller mass.
  if (!hasMass && !hasRadius) {
    float oldInertia = preferences.getFloat("inertia", DEFAULT_INERTIA);
    if (oldInertia > 0.0f && oldInertia <= 100.0f) {
      settings.massKg = DEFAULT_ROLLER_MASS_KG;
      float r2 = 2.0f * oldInertia / settings.massKg;
      if (r2 > 0.0f) settings.radiusM = sqrtf(r2);
    }
  }

  settings.friction = preferences.getFloat("friction", DEFAULT_FRICTION);
  settings.ppr = preferences.getUShort("ppr", DEFAULT_PPR);
  settings.avgWindow = (uint8_t)preferences.getUShort("avgWindow", DEFAULT_AVG_WINDOW);
  preferences.end();

  if (!(settings.massKg > 0.001f) || settings.massKg > 1000.0f) {
    settings.massKg = DEFAULT_ROLLER_MASS_KG;
  }
  if (!(settings.radiusM > 0.001f) || settings.radiusM > 10.0f) {
    settings.radiusM = DEFAULT_ROLLER_RADIUS_M;
  }
  if (!(settings.friction >= 0.0f) || settings.friction > 100.0f) {
    settings.friction = DEFAULT_FRICTION;
  }
  if (settings.ppr < 1 || settings.ppr > 1000) {
    settings.ppr = DEFAULT_PPR;
  }
  if (settings.avgWindow < 1 || settings.avgWindow > 64) {
    settings.avgWindow = DEFAULT_AVG_WINDOW;
  }

  settings.inertia = 0.5f * settings.massKg * settings.radiusM * settings.radiusM;
  if (!(settings.inertia > 0.0f) || settings.inertia > 100.0f) {
    settings.massKg = DEFAULT_ROLLER_MASS_KG;
    settings.radiusM = DEFAULT_ROLLER_RADIUS_M;
    settings.inertia = DEFAULT_INERTIA;
  }
}

void saveSettings() {
  preferences.begin("dyno", false);
  preferences.putFloat("mass", settings.massKg);
  preferences.putFloat("radius", settings.radiusM);
  preferences.putFloat("friction", settings.friction);
  preferences.putUShort("ppr", settings.ppr);
  preferences.putUShort("avgWindow", settings.avgWindow);
  preferences.end();
}

// ---------------------------------------------------------------------------
// CSV logging
// ---------------------------------------------------------------------------
const char LOG_HEADER[] =
    "Record,Run_ms,Uptime_s,RPM,Omega_rads,Alpha_rads2,Torque_Nm,Power_W,"
    "Efficiency_pct,Eff_Valid,"
    "Sensor,Pulses,Revolutions,Peak_RPM,Peak_Torque_Nm,Peak_Power_W,"
    "Inertia_kgm2,Friction_Nm,PPR,Roller_Mass_kg,Roller_Radius_m,Avg_Window,"
    "BMS_Connected,BMS_Age_ms,BMS_Voltage_V,BMS_Current_A,BMS_SOC_pct,"
    "BMS_MaxCell_mV,BMS_MaxCell_No,BMS_MinCell_mV,BMS_MinCell_No,"
    "BMS_MaxTemp_C,BMS_MinTemp_C,BMS_RemCap_mAh,BMS_Cycles,"
    "BMS_ChargeMOS,BMS_DischargeMOS";

void flushLogBuffer() {
  if (logBufferCount == 0) return;
  if (!loggingReady || logFs == nullptr || currentLogPath[0] == '\0') {
    logBufferCount = 0;
    return;
  }

  File f = logFs->open(currentLogPath, FILE_APPEND);
  if (!f) {
    loggingReady = false;
    logBufferCount = 0;
    return;
  }

  for (uint8_t i = 0; i < logBufferCount; i++) {
    const LogRecord &r = logBuffer[i];
    f.printf("%lu,%lu,%lu,%.1f,%.2f,%.2f,%.3f,%.1f,%.1f,%d,%d,%lu,%.2f,"
             "%.1f,%.3f,%.1f,%.5f,%.4f,%u,%.3f,%.4f,%u,"
             "%d,%lu,%.2f,%.2f,%.1f,"
             "%u,%u,%u,%u,"
             "%.1f,%.1f,%lu,%u,"
             "%d,%d\n",
             (unsigned long)r.record, (unsigned long)r.runMs,
             (unsigned long)r.uptimeS,
             r.rpm, r.omega, r.alpha, r.torque, r.power,
             r.efficiency, r.efficiencyValid ? 1 : 0,
             r.sensor ? 1 : 0, (unsigned long)r.pulses, r.revolutions,
             r.peakRpm, r.peakTorque, r.peakPower,
             r.inertia, r.friction, (unsigned)r.ppr,
             r.massKg, r.radiusM, (unsigned)r.avgWindow,
             r.bmsConnected ? 1 : 0, (unsigned long)r.bmsAgeMs,
             r.voltage, r.current, r.soc,
             (unsigned)r.maxCellMv, (unsigned)r.maxCellNo,
             (unsigned)r.minCellMv, (unsigned)r.minCellNo,
             r.maxTemp, r.minTemp, (unsigned long)r.remCapMah,
             (unsigned)r.cycles,
             r.chargeMos ? 1 : 0, r.dischargeMos ? 1 : 0);
  }

  f.close();
  logBufferCount = 0;
}

size_t logTotalBytes() {
  if (logFs == &LittleFS) return LittleFS.totalBytes();
  if (logFs == &FFat) return FFat.totalBytes();
  return 0;
}

size_t logUsedBytes() {
  if (logFs == &LittleFS) return LittleFS.usedBytes();
  if (logFs == &FFat) return FFat.usedBytes();
  return 0;
}

void rotateOldLogs() {
  if (logFs == nullptr) return;

  size_t total = logTotalBytes();
  size_t used = logUsedBytes();
  if (total == 0 || ((float)used / (float)total) <= LOG_CLEANUP_RATIO) return;

  String oldest;
  File root = logFs->open("/");
  if (!root) return;
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (!name.startsWith("/")) name = "/" + name;
    if (name.endsWith(".csv") && name != currentLogPath &&
        (oldest.length() == 0 || name < oldest)) {
      oldest = name;
    }
    file = root.openNextFile();
  }
  root.close();

  if (oldest.length() > 0) logFs->remove(oldest);
}

void startNewLog() {
  flushLogBuffer();
  logBufferCount = 0;
  logRecordNum = 0;
  currentLogPath[0] = '\0';

  if (!loggingReady || logFs == nullptr) return;

  rotateOldLogs();

  int idx = 1;
  for (; idx <= 9999; idx++) {
    snprintf(currentLogPath, sizeof(currentLogPath), "/run_%04d.csv", idx);
    if (!logFs->exists(currentLogPath)) break;
  }
  if (idx > 9999) {
    loggingReady = false;
    currentLogPath[0] = '\0';
    return;
  }

  File f = logFs->open(currentLogPath, FILE_WRITE);
  if (!f) {
    loggingReady = false;
    currentLogPath[0] = '\0';
    return;
  }
  f.println(LOG_HEADER);
  f.close();

  logStartMs = millis();
  lastLogMs = logStartMs;
}

void mountLogFs() {
  // LittleFS is more power-loss resistant; the Nano ESP32 default partition
  // has no spiffs partition, so fall back to the FAT partition in that case.
  if (LittleFS.begin(true)) {
    logFs = &LittleFS;
    logFsName = "LittleFS";
  } else if (FFat.begin(true)) {
    logFs = &FFat;
    logFsName = "FFat";
  } else {
    logFs = nullptr;
    logFsName = "none";
  }

  loggingReady = (logFs != nullptr);
  if (loggingReady) startNewLog();
}

void stopRun() {
  if (!runActive) return;
  flushLogBuffer();
  runStopMs = millis() - runStartMs;
  runActive = false;
}

bool formatLogFs() {
  if (logFs == nullptr) return false;

  flushLogBuffer();
  runActive = false;
  currentLogPath[0] = '\0';
  logRecordNum = 0;
  logBufferCount = 0;

  bool ok = false;
  if (logFs == &LittleFS) {
    LittleFS.end();
    ok = LittleFS.format();
    ok = LittleFS.begin(true) && ok;
  } else if (logFs == &FFat) {
    FFat.end();
    ok = FFat.format();
    ok = FFat.begin(true) && ok;
  }

  loggingReady = ok;
  if (ok) {
    runStartMs = millis();
    runStopMs = 0;
    runActive = true;
    startNewLog();
  }
  return ok;
}

void appendLogRecord() {
  if (!runActive || !loggingReady || logBufferCount >= LOG_BUFFER_ROWS) return;

  uint32_t pulses;
  portENTER_CRITICAL(&pulseMux);
  pulses = totalPulses;
  portEXIT_CRITICAL(&pulseMux);

  uint32_t now = millis();
  LogRecord &r = logBuffer[logBufferCount];
  r.record = ++logRecordNum;
  r.runMs = now - runStartMs;
  r.uptimeS = now / 1000UL;
  r.rpm = rpm;
  r.omega = omega;
  r.alpha = alpha;
  r.torque = torque;
  r.power = power;
  r.efficiency = efficiency;
  r.efficiencyValid = efficiencyValid;
  r.sensor = (digitalRead(SENSOR_PIN) == LOW);
  r.pulses = pulses;
  r.revolutions = pulses / (float)settings.ppr;
  r.peakRpm = peakRpm;
  r.peakTorque = peakTorque;
  r.peakPower = peakPower;
  r.inertia = settings.inertia;
  r.friction = settings.friction;
  r.ppr = settings.ppr;
  r.massKg = settings.massKg;
  r.radiusM = settings.radiusM;
  r.avgWindow = settings.avgWindow;
  r.bmsConnected = bmsConnected;
  r.bmsAgeMs = lastBmsVoltageMs == 0
                   ? 0
                   : (uint32_t)(now - lastBmsVoltageMs);
  r.voltage = bmsData.totalVoltage;
  r.current = bmsData.current;
  r.soc = bmsData.soc;
  r.maxCellMv = bmsData.maxCellMv;
  r.maxCellNo = bmsData.maxCellNo;
  r.minCellMv = bmsData.minCellMv;
  r.minCellNo = bmsData.minCellNo;
  r.maxTemp = bmsData.maxTemp;
  r.minTemp = bmsData.minTemp;
  r.remCapMah = bmsData.remCapMah;
  r.cycles = bmsData.cycles;
  r.chargeMos = bmsData.chargeMos;
  r.dischargeMos = bmsData.dischargeMos;

  logBufferCount++;
  if (logBufferCount >= LOG_BUFFER_ROWS) flushLogBuffer();
}

void resetStats() {
  portENTER_CRITICAL(&pulseMux);
  pulseHead = 0;
  pulseTail = 0;
  totalPulses = 0;
  lastIsrUs = micros();
  portEXIT_CRITICAL(&pulseMux);

  clearSpeedHistory();
  peakRpm = 0.0f;
  peakTorque = 0.0f;
  peakPower = 0.0f;
  efficiency = 0.0f;
  efficiencyValid = false;
  runStartMs = millis();
  runStopMs = 0;
  runActive = true;
  lastControlMs = runStartMs;
  startNewLog();
}

// ---------------------------------------------------------------------------
// Serial telemetry stream
// ---------------------------------------------------------------------------
// One JSON line per 5 Hz sample, with pulse-derived speed/torque and the
// matching BMS voltage/current. This is the "Serial" synchronization path;
// the browser dashboard polls /data at the same rate.
void streamSerialTelemetry(uint32_t nowMs) {
  if (!SERIAL_TELEMETRY) return;
  if ((uint32_t)(nowMs - lastSerialMs) < SERIAL_STREAM_MS) return;
  lastSerialMs = nowMs;

  uint32_t bmsAge = lastBmsVoltageMs == 0
                        ? 0
                        : (uint32_t)(nowMs - lastBmsVoltageMs);
  bool bmsFresh = bmsConnected && lastBmsVoltageMs != 0 && bmsAge < 3000;
  float voltage = bmsFresh ? bmsData.totalVoltage : 0.0f;
  float current = bmsFresh ? bmsData.current : 0.0f;
  float pElec = voltage * current;

  Serial.printf(
      "{\"t\":%lu,\"rpm\":%.1f,\"omega\":%.2f,\"alpha\":%.2f,"
      "\"torque\":%.3f,\"pMech\":%.1f,\"v\":%.2f,\"i\":%.2f,"
      "\"pElec\":%.1f,\"eff\":%.1f,\"effOk\":%d,"
      "\"friction\":%.4f,\"bmsAge\":%lu,\"calib\":%u}\n",
      (unsigned long)nowMs, rpm, omega, alpha, torque, power,
      voltage, current, pElec, efficiency, efficiencyValid ? 1 : 0,
      settings.friction, (unsigned long)bmsAge, (unsigned)calibState);
}

// ---------------------------------------------------------------------------
// Daly BMS over BLE
// ---------------------------------------------------------------------------
bool verifyChecksum(const uint8_t *pkt, size_t len) {
  if (len < 2) return false;
  uint8_t sum = 0;
  for (size_t i = 0; i < len - 1; ++i) sum += pkt[i];
  return (sum == pkt[len - 1]);
}

void parseDalyFrame(const uint8_t *frame) {
  if (frame[0] != 0xA5 || !verifyChecksum(frame, 13)) return;

  uint32_t nowMs = millis();
  uint8_t cmd = frame[2];
  const uint8_t *payload = frame + 4;

  switch (cmd) {
    case 0x90: {
      uint16_t total_raw = (payload[0] << 8) | payload[1];
      uint16_t current_raw = (payload[4] << 8) | payload[5];
      uint16_t soc_raw = (payload[6] << 8) | payload[7];

      bmsData.totalVoltage = total_raw / 10.0f;
      bmsData.current = ((int)current_raw - 30000) / 10.0f;
      bmsData.soc = soc_raw / 10.0f;
      lastBmsVoltageMs = nowMs;
      break;
    }
    case 0x91: {
      bmsData.maxCellMv = (payload[0] << 8) | payload[1];
      bmsData.maxCellNo = payload[2];
      bmsData.minCellMv = (payload[3] << 8) | payload[4];
      bmsData.minCellNo = payload[5];
      break;
    }
    case 0x92: {
      bmsData.maxTemp = payload[0] - 40.0f;
      bmsData.minTemp = payload[2] - 40.0f;
      break;
    }
    case 0x93: {
      bmsData.bmsState = payload[0];
      bmsData.chargeMos = (payload[1] == 1);
      bmsData.dischargeMos = (payload[2] == 1);
      bmsData.remCapMah = ((uint32_t)payload[4] << 24) |
                           ((uint32_t)payload[5] << 16) |
                           ((uint32_t)payload[6] << 8) | payload[7];
      break;
    }
    case 0x94: {
      if (payload[0] >= 1 && payload[0] <= 48) {
        bmsData.cellCount = payload[0];
      }
      bmsData.cycles = (payload[5] << 8) | payload[6];
      break;
    }
    case 0x95: {
      uint8_t rawFrameNo = payload[0];
      if (!frameBaseDetected) {
        frameBase = (rawFrameNo == 0) ? 0 : 1;
        frameBaseDetected = true;
      }

      int idx = (int)rawFrameNo - frameBase;
      if (idx >= 0 && idx < 16) {
        for (int i = 0; i < 3; ++i) {
          int cellIndex = idx * 3 + i;
          if (cellIndex < bmsData.cellCount && cellIndex < 48) {
            int p = 1 + i * 2;
            uint16_t mv = (payload[p] << 8) | payload[p + 1];
            bmsData.cellMv[cellIndex] = (mv == 0xFFFF) ? 0 : mv;
          }
        }
      }
      break;
    }
    case 0x97: {
      int count = bmsData.cellCount > 48 ? 48 : bmsData.cellCount;
      for (int i = 0; i < count; ++i) {
        int byteIndex = i / 8;
        int bitIndex = i % 8;
        bmsData.cellBalancing[i] = (payload[byteIndex] >> bitIndex) & 0x01;
      }
      break;
    }
  }
}

void processBleRingBuffer() {
  for (;;) {
    uint8_t b = 0;
    bool hasByte = false;

    portENTER_CRITICAL(&bleRingMux);
    if (bleRingHead != bleRingTail) {
      b = bleRxRing[bleRingTail];
      bleRingTail = (bleRingTail + 1) % sizeof(bleRxRing);
      hasByte = true;
    }
    portEXIT_CRITICAL(&bleRingMux);

    if (!hasByte) break;

    if (frameAssembleLen == 0) {
      if (b == 0xA5) frameAssembleBuf[frameAssembleLen++] = b;
    } else {
      frameAssembleBuf[frameAssembleLen++] = b;
      if (frameAssembleLen == 13) {
        parseDalyFrame(frameAssembleBuf);
        frameAssembleLen = 0;
      }
    }
  }
}

void bmsNotifyCallback(NimBLERemoteCharacteristic *pRemoteCharacteristic,
                       uint8_t *pData, size_t length, bool isNotify) {
  (void)pRemoteCharacteristic;
  (void)isNotify;
  if (length == 0 || pData == nullptr) return;

  portENTER_CRITICAL(&bleRingMux);
  for (size_t i = 0; i < length; ++i) {
    size_t nextHead = (bleRingHead + 1) % sizeof(bleRxRing);
    if (nextHead != bleRingTail) {
      bleRxRing[bleRingHead] = pData[i];
      bleRingHead = nextHead;
    }
  }
  portEXIT_CRITICAL(&bleRingMux);
}

class BmsScanCallbacks : public NimBLEScanCallbacks {
  void onDiscovered(const NimBLEAdvertisedDevice *dev) override {
    if (dev == nullptr) return;
    if (dev->getAddress().toString() == std::string(BMS_MAC_ADDRESS)) {
      bmsDevInfo.address = dev->getAddress();
      bmsDevInfo.found = true;
      bmsConnectRequested = true;
    }
  }
};
BmsScanCallbacks bmsScanCallbacks;

class BmsClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *client) override {
    (void)client;
    bmsConnected = true;
    frameBaseDetected = false;
  }

  void onDisconnect(NimBLEClient *client, int reason) override {
    (void)client;
    (void)reason;
    bmsConnected = false;
    bmsTxChar = nullptr;
    bmsRxChar = nullptr;
  }
};
BmsClientCallbacks bmsClientCallbacks;

void sendDalyRequest(uint8_t cmd, uint8_t arg = 0) {
  if (bmsRxChar == nullptr || !bmsConnected || bmsClient == nullptr ||
      !bmsClient->isConnected()) {
    return;
  }

  uint8_t pkt[13];
  pkt[0] = 0xA5;
  pkt[1] = 0x40;
  pkt[2] = cmd;
  pkt[3] = 0x08;
  memset(&pkt[4], 0x00, 8);
  pkt[4] = arg;

  uint8_t sum = 0;
  for (int i = 0; i < 12; ++i) sum += pkt[i];
  pkt[12] = sum;

  if (bmsRxChar->canWrite()) {
    bmsRxChar->writeValue(pkt, 13, false);
  }
}

bool discoverAndSubscribe() {
  if (bmsClient == nullptr || !bmsClient->isConnected()) return false;

  NimBLERemoteService *svc = bmsClient->getService(SERVICE_UUID_MAIN);
  if (svc == nullptr) return false;

  bmsTxChar = svc->getCharacteristic(CHAR_UUID_TX);
  bmsRxChar = svc->getCharacteristic(CHAR_UUID_RX);
  if (bmsTxChar == nullptr || bmsRxChar == nullptr) return false;

  if (bmsTxChar->canNotify()) {
    return bmsTxChar->subscribe(true, bmsNotifyCallback, false);
  }
  return false;
}

bool connectToBms() {
  if (!bmsDevInfo.found) return false;

  if (bmsClient != nullptr) {
    if (bmsClient->isConnected()) return true;
    bmsTxChar = nullptr;
    bmsRxChar = nullptr;
    NimBLEDevice::deleteClient(bmsClient);
    bmsClient = nullptr;
  }

  bmsClient = NimBLEDevice::createClient();
  if (bmsClient == nullptr) return false;

  bmsClient->setClientCallbacks(&bmsClientCallbacks, false);
  bmsClient->setConnectTimeout(3000);

  if (!bmsClient->connect(bmsDevInfo.address)) {
    NimBLEDevice::deleteClient(bmsClient);
    bmsClient = nullptr;
    return false;
  }

  if (!discoverAndSubscribe()) {
    bmsClient->disconnect();
    NimBLEDevice::deleteClient(bmsClient);
    bmsClient = nullptr;
    return false;
  }
  return true;
}

void startBmsScan(uint32_t durationMs) {
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&bmsScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(60);
  scan->setMaxResults(0);
  scan->start(durationMs, false);
}

void serviceBms(uint32_t now) {
  static uint32_t lastScanMs = 0;
  static uint32_t lastQueryMs = 0;
  static uint8_t queryStep = 0;

  if (!bleReady) return;

  // Do not start a blocking connect while the dyno is spinning: a failed
  // attempt would stall pulse processing and lose speed samples.
  if (bmsConnectRequested && !bmsConnected && rpm < 1.0f) {
    bmsConnectRequested = false;
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) scan->stop();
    connectToBms();
    lastScanMs = millis();
  }

  if (!bmsConnected) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (!scan->isScanning() && rpm < 1.0f &&
        (uint32_t)(now - lastScanMs) >= BMS_SCAN_RETRY_MS) {
      lastScanMs = now;
      startBmsScan(BMS_SCAN_MS);
    }
    return;
  }

  if (bmsClient == nullptr || !bmsClient->isConnected()) return;
  if ((uint32_t)(now - lastQueryMs) < BMS_QUERY_INTERVAL_MS) return;

  lastQueryMs = now;

  // Same non-overlapping sequence as the reference BMS sketch: interleave
  // 0x90 (pack V/A/SOC) with the slower-changing queries.
  switch (queryStep) {
    case 0:
    case 1:
    case 3:
    case 5:
    case 7:
      sendDalyRequest(0x90);
      break;
    case 2:
      sendDalyRequest(0x91);
      break;
    case 4:
      sendDalyRequest(0x92);
      break;
    case 6:
      sendDalyRequest(0x93);
      break;
    case 8:
      sendDalyRequest(0x94);
      break;
  }

  queryStep = (queryStep + 1) % 9;
}

// ---------------------------------------------------------------------------
// Web page
// ---------------------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>Dyno</title>
<style>
  :root{
    --bg:#f3f6fb; --card:#ffffff; --line:#d6dfeb; --text:#142033;
    --muted:#5d6b7e; --green:#0f9d58; --blue:#2563eb;
    --orange:#ea580c; --pink:#db2777; --red:#dc2626;
  }
  *{box-sizing:border-box}
  body{
    margin:0; min-height:100vh; color:var(--text);
    font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    background:
      radial-gradient(900px 500px at 15% -10%, #dbeafe 0%, transparent 60%),
      radial-gradient(700px 400px at 100% 0%, #e5e9ff 0%, transparent 55%),
      var(--bg);
  }
  header{
    display:flex; justify-content:space-between; gap:16px; align-items:center;
    padding:24px clamp(14px,4vw,40px) 10px;
  }
  h1{margin:0; font-size:clamp(24px,4vw,36px); letter-spacing:-1px}
  h1 span{color:var(--green)}
  .sub{display:flex; gap:8px; align-items:center; color:var(--muted); font-size:13px; margin-top:6px}
  .dot{width:9px; height:9px; border-radius:50%; background:var(--green); box-shadow:0 0 10px var(--green)}
  .dot.off{background:var(--red); box-shadow:0 0 10px var(--red)}
  .sensor{
    border:1px solid var(--line); background:rgba(255,255,255,.9);
    border-radius:12px; padding:8px 12px; text-align:right; font-size:12px;
    color:var(--muted); min-width:96px;
  }
  .sensor b{display:block; color:var(--text); font-size:15px; margin-top:2px}
  main{padding:8px clamp(14px,4vw,40px) 36px; max-width:1200px; margin:auto}
  .stats{
    display:grid; grid-template-columns:repeat(auto-fit,minmax(140px,1fr));
    gap:12px; margin:14px 0;
  }
  .stat,.card{
    background:linear-gradient(180deg,#ffffff,#f7faff);
    border:1px solid var(--line); border-radius:16px;
    box-shadow:0 12px 30px rgba(25,50,90,.08);
  }
  .stat{padding:14px 16px}
  .stat span{display:block; color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.08em}
  .stat strong{display:block; font-size:clamp(22px,3vw,30px); line-height:1.15; margin-top:6px; font-variant-numeric:tabular-nums}
  .stat small{display:block; color:var(--muted); font-size:11px; margin-top:4px}
  .peaks{color:var(--muted); font-size:12px; margin:0 2px 16px}
  .peaks b{color:var(--text); font-weight:600}
  .chart-grid{display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px}
  .card{padding:14px}
  .chart-title{display:flex; justify-content:space-between; align-items:baseline; margin:0 2px 8px; color:var(--muted); font-size:13px}
  .chart-title b{color:var(--text); font-size:16px; font-variant-numeric:tabular-nums}
  canvas{display:block; width:100%; height:150px}
  .setup{margin-top:14px; padding:0}
  summary{cursor:pointer; padding:14px 16px; font-weight:600}
  form{padding:0 16px 16px; display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:12px; align-items:end}
  label{display:flex; flex-direction:column; gap:6px; color:var(--muted); font-size:12px}
  input{
    width:100%; border:1px solid var(--line); border-radius:9px;
    background:#fff; color:var(--text); padding:10px 11px; font:inherit;
    outline:none;
  }
  input:focus{border-color:var(--blue)}
  .actions{grid-column:1/-1; display:flex; gap:10px; flex-wrap:wrap}
  button,a.btn{
    border:0; border-radius:10px; padding:10px 15px; font:inherit; font-weight:600;
    color:#fff; background:var(--green); cursor:pointer; text-decoration:none;
    display:inline-block;
  }
  button.secondary,a.btn.secondary{color:var(--text); background:#e7edf6}
  button:active,a.btn:active{transform:translateY(1px)}
  button:disabled{opacity:.45; cursor:not-allowed}
  .session{margin-top:14px}
  .links{display:flex; gap:10px; flex-wrap:wrap}
  .logmsg{margin:10px 0 0}
  .battery-head{display:flex; align-items:baseline; gap:8px; margin:22px 2px 0;
    color:var(--muted); font-size:13px}
  .battery-head b{color:var(--text); font-size:15px}
  .badge{font-size:11px; font-weight:600; letter-spacing:.04em; padding:2px 8px;
    border-radius:999px; background:#e7edf6; color:var(--muted)}
  .badge.on{background:rgba(15,157,88,.12); color:var(--green)}
  .msg{grid-column:1/-1; min-height:16px; margin:0; font-size:12px; color:var(--muted)}
  .msg.ok{color:var(--green)} .msg.err{color:var(--red)}
  .readout{grid-column:1/-1; border:1px dashed var(--line); border-radius:10px;
    padding:10px 12px; background:#f8fbff; color:var(--muted); font-size:12px}
  .readout b{color:var(--text); font-size:16px; font-variant-numeric:tabular-nums}
  .readout small{display:block; margin-top:2px; color:var(--muted)}
  .hint{padding:0 16px 16px; margin:0; color:var(--muted); font-size:12px; line-height:1.5}
  footer{padding:0 14px 28px; text-align:center; color:var(--muted); font-size:12px; line-height:1.6}
  footer b{color:var(--text)}
  .card.wide{grid-column:1/-1}
  .card.wide canvas{height:340px}
  .chart-note{color:var(--muted); font-size:11px; margin:-4px 2px 8px}
  .lg{font-weight:700}
  .lg.speed{color:#111111} .lg.current{color:#f59e0b}
  .lg.power{color:#2563eb} .lg.eff{color:#16a34a}
  @media(max-width:700px){
    .chart-grid{grid-template-columns:1fr}
    form{grid-template-columns:1fr}
    header{flex-wrap:wrap}
    canvas{height:130px}
    .card.wide canvas{height:270px}
  }
</style>
</head>
<body>
<header>
  <div>
    <h1>Dyno</h1>
    <div class="sub"><span id="dot" class="dot"></span><span id="status">Connecting...</span><span>·</span><span>5 Hz</span></div>
  </div>
  <div class="sensor">IR sensor<b id="sensor">—</b></div>
</header>

<main>
  <section class="stats">
    <div class="stat"><span>RPM</span><strong id="rpm">0</strong><small>rev / min</small></div>
    <div class="stat"><span>Torque</span><strong id="torque">0</strong><small>N·m estimated</small></div>
    <div class="stat"><span>Power</span><strong id="power">0</strong><small>W estimated</small></div>
    <div class="stat"><span>Efficiency</span><strong id="effStat">—</strong><small>mech / battery</small></div>
    <div class="stat"><span>Angular accel</span><strong id="alpha">0</strong><small>rad/s²</small></div>
    <div class="stat"><span>Pulses</span><strong id="pulses">0</strong><small>sensor count</small></div>
    <div class="stat"><span>Revolutions</span><strong id="revs">0</strong><small>pulses / PPR</small></div>
  </section>

  <p class="peaks">Session peaks: <b id="peakRpm">0</b> rpm · <b id="peakTorque">0</b> N·m · <b id="peakPower">0</b> W</p>

  <div class="battery-head">Battery (Daly BMS) <b id="bmsStatus">searching…</b> <span class="badge" id="bmsBadge">BLE</span></div>
  <section class="stats">
    <div class="stat"><span>Pack</span><strong id="bmsV">—</strong><small>V</small></div>
    <div class="stat"><span>Current</span><strong id="bmsA">—</strong><small>A · + discharge</small></div>
    <div class="stat"><span>Battery power</span><strong id="bmsW">—</strong><small>W · V × A</small></div>
    <div class="stat"><span>State of charge</span><strong id="bmsSoc">—</strong><small>%</small></div>
    <div class="stat"><span>Cell spread</span><strong id="bmsDelta">—</strong><small>mV · max − min</small></div>
    <div class="stat"><span>Pack temp</span><strong id="bmsTemp">—</strong><small>°C max</small></div>
  </section>
  <p class="peaks" id="bmsDetail">Cells: <b id="bmsCellRange">—</b> · Temps: <b id="bmsTempRange">—</b> °C · Remaining: <b id="bmsRem">—</b> Ah · Cycles: <b id="bmsCycles">—</b> · MOS: <b id="bmsMos">—</b></p>

  <section class="chart-grid">
    <article class="card">
      <div class="chart-title"><span>RPM</span><b id="rpmNow">0</b></div>
      <canvas id="chartRpm"></canvas>
    </article>
    <article class="card">
      <div class="chart-title"><span>Torque (N·m)</span><b id="torqueNow">0</b></div>
      <canvas id="chartTorque"></canvas>
    </article>
    <article class="card">
      <div class="chart-title"><span>Power (W)</span><b id="powerNow">0</b></div>
      <canvas id="chartPower"></canvas>
    </article>
    <article class="card">
      <div class="chart-title"><span>Angular accel (rad/s²)</span><b id="alphaNow">0</b></div>
      <canvas id="chartAlpha"></canvas>
    </article>
    <article class="card wide">
      <div class="chart-title"><span>Motor performance characteristics</span><b id="perfNow">—</b></div>
      <div class="chart-note">X: torque (N·m) derived from roller inertia · <span class="lg speed">speed</span>, <span class="lg current">current</span>, <span class="lg power">power</span> and <span class="lg eff">efficiency</span> each use their own Y axis · current and efficiency need a live BMS</div>
      <canvas id="chartPerf"></canvas>
    </article>
  </section>

  <section class="card session">
    <div class="chart-title"><span>Session log (CSV)</span><b id="logName">—</b></div>
    <div class="links">
      <button type="button" id="startBtn">Start run</button>
      <button type="button" id="stopBtn" class="secondary">Stop run</button>
      <button type="button" id="formatBtn" class="secondary">Format LittleFS</button>
    </div>
    <div class="links" style="margin-top:10px">
      <a class="btn secondary" href="/download?active=1">Download current CSV</a>
      <a class="btn secondary" href="/logs">All saved runs</a>
    </div>
    <p class="msg logmsg" id="logMsg">—</p>
    <p class="msg logmsg" id="ctrlMsg"></p>
  </section>

  <details class="card setup" open>
    <summary>Calibration &amp; settings</summary>
    <form id="cfg">
      <label>Roller mass (kg)
        <input id="mass" name="mass" type="number" step="0.1" min="0.001" max="1000" required>
      </label>
      <label>Roller radius (m)
        <input id="radius" name="radius" type="number" step="0.001" min="0.001" max="10" required>
      </label>
      <label>Pulses per revolution
        <input id="ppr" name="ppr" type="number" step="1" min="1" max="1000" required>
      </label>
      <label>Moving average window (samples)
        <input id="avgWindow" name="avgWindow" type="number" step="1" min="1" max="64" required>
      </label>
      <label>Friction torque (N·m)
        <input id="friction" name="friction" type="number" step="0.001" min="0" max="100" required>
      </label>
      <div class="readout">
        Effective inertia <b id="inertiaOut">—</b> kg·m²
        <small>solid cylinder: I = ½ m r²</small>
      </div>
      <div class="actions">
        <button type="submit">Save settings</button>
        <button type="button" class="secondary" id="calibBtn">Coast-down friction calibration</button>
        <button type="button" class="secondary" id="resetBtn">New run (reset counters)</button>
      </div>
      <p class="msg" id="calibMsg"></p>
      <p class="msg" id="formMsg"></p>
    </form>
    <p class="hint">
      Torque is estimated from the roller's angular acceleration:
      <b>τ = I × α + τ_loss</b>. Mass and radius set the inertia automatically;
      the moving-average window smooths omega before the derivative is taken.
      <b>Coast-down calibration:</b> click the button, spin the roller up with the
      motor unpowered, then let it coast. For 15 s the firmware averages
      <b>−I × α</b> while the roller slows and stores the result as friction torque.
      This is an inertial dyno estimate: it measures torque while speed changes,
      not steady-state torque. There is no load cell.
    </p>
  </details>
</main>

<footer>
  Connect to open WiFi <b>Dyno-ESP32</b> · open <b>http://192.168.4.1</b><br>
  Every run is saved on the ESP32 as CSV · <a href="/logs" style="color:var(--green)">browse saved runs</a><br>
  Keep this open network isolated. No password and no authentication.
</footer>

<script>
const MAX_POINTS = 150;             // 30 seconds at 5 Hz

class Spark {
  constructor(id, color) {
    this.canvas = document.getElementById(id);
    this.ctx = this.canvas.getContext('2d');
    this.color = color;
    this.data = [];
  }
  reset() {
    this.data = [];
    this.draw();
  }
  push(value) {
    value = Number(value);
    if (!Number.isFinite(value)) value = 0;
    this.data.push(value);
    if (this.data.length > MAX_POINTS) this.data.shift();
    this.scheduleDraw();
  }
  scheduleDraw() {
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => {
      this._raf = 0;
      this.draw();
    });
  }
  draw() {
    const canvas = this.canvas;
    const ctx = this.ctx;
    const rect = canvas.getBoundingClientRect();
    const w = Math.max(60, rect.width);
    const h = Math.max(60, rect.height);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const pw = Math.round(w * dpr);
    const ph = Math.round(h * dpr);

    if (canvas.width !== pw || canvas.height !== ph) {
      canvas.width = pw;
      canvas.height = ph;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = 'rgba(20,32,51,0.10)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = Math.round((h - 1) * i / 4) + 0.5;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (!this.data.length) return;

    let min = Math.min(0, ...this.data);
    let max = Math.max(0, ...this.data);
    if (!Number.isFinite(min) || !Number.isFinite(max)) return;
    if (max - min < 1e-9) max = min + 1;
    const pad = (max - min) * 0.08;
    max += pad;
    if (min < 0) min -= pad;

    const yOf = v => h - ((v - min) / (max - min)) * h;
    const x1 = w - 2;
    const x0 = 2;
    const step = (x1 - x0) / (MAX_POINTS - 1);
    const firstX = x1 - (this.data.length - 1) * step;
    const lastX = x1;
    const lastY = yOf(this.data[this.data.length - 1]);

    // Area fill
    ctx.beginPath();
    for (let i = 0; i < this.data.length; i++) {
      const x = firstX + i * step;
      const y = yOf(this.data[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.lineTo(lastX, h);
    ctx.lineTo(firstX, h);
    ctx.closePath();
    ctx.globalAlpha = 0.12;
    ctx.fillStyle = this.color;
    ctx.fill();
    ctx.globalAlpha = 1;

    // Line
    ctx.beginPath();
    for (let i = 0; i < this.data.length; i++) {
      const x = firstX + i * step;
      const y = yOf(this.data[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = this.color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.stroke();

    // Latest point
    ctx.beginPath();
    ctx.arc(lastX, lastY, 2.6, 0, Math.PI * 2);
    ctx.fillStyle = this.color;
    ctx.fill();

    // Scale labels
    ctx.fillStyle = 'rgba(20,32,51,0.55)';
    ctx.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText(max.toFixed(1), w - 5, 3);
    ctx.textBaseline = 'bottom';
    ctx.fillText(min.toFixed(1), w - 5, h - 3);
  }
}

class PerformanceChart {
  constructor(id) {
    this.canvas = document.getElementById(id);
    this.ctx = this.canvas.getContext('2d');
    this.data = [];
    this.maxPoints = 900;
    this.curves = [
      { key: 'rpm',     label: 'Speed',      color: '#111111' },
      { key: 'current', label: 'Current',    color: '#f59e0b' },
      { key: 'power',   label: 'Power',      color: '#2563eb' },
      { key: 'eff',     label: 'Efficiency', color: '#16a34a' }
    ];
  }
  reset() {
    this.data = [];
    this.draw();
  }
  push(values) {
    const torque = Number(values.torque);
    if (!Number.isFinite(torque) || torque < 0) return;
    const row = { torque };
    for (const c of this.curves) {
      const raw = values[c.key];
      if (raw === null || raw === undefined || raw === '') {
        row[c.key] = null;
      } else {
        const v = Number(raw);
        row[c.key] = Number.isFinite(v) ? v : null;
      }
    }
    this.data.push(row);
    if (this.data.length > this.maxPoints) this.data.shift();
    this.scheduleDraw();
  }
  scheduleDraw() {
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => {
      this._raf = 0;
      this.draw();
    });
  }
  binned(key, bins, torqueMax) {
    const acc = Array.from({ length: bins }, () => ({ sum: 0, n: 0 }));
    for (const row of this.data) {
      const v = row[key];
      if (v === null) continue;
      let b = Math.floor((row.torque / torqueMax) * bins);
      if (b < 0) b = 0;
      if (b >= bins) b = bins - 1;
      acc[b].sum += v;
      acc[b].n += 1;
    }
    return acc.map((a, i) => a.n > 0
      ? { x: (i + 0.5) / bins * torqueMax, y: a.sum / a.n }
      : null);
  }
  draw() {
    const canvas = this.canvas;
    const ctx = this.ctx;
    if (!canvas || !ctx) return;

    const rect = canvas.getBoundingClientRect();
    const w = Math.max(260, rect.width);
    const h = Math.max(180, rect.height);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const pw = Math.round(w * dpr);
    const ph = Math.round(h * dpr);
    if (canvas.width !== pw || canvas.height !== ph) {
      canvas.width = pw;
      canvas.height = ph;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const compact = w < 700;
    const padL = compact ? 52 : 64;
    const padT = 10;
    const padB = 46;
    const axisGap = compact ? 38 : 46;
    const rightBase = w - axisGap * 3 - 8;
    const plotX = padL;
    const plotRight = rightBase - 14;
    const plotW = Math.max(30, plotRight - plotX);
    const plotY = padT;
    const plotH = Math.max(40, h - padT - padB);
    const plotBottom = plotY + plotH;

    let maxTorque = 0, maxRpm = 0, maxCurrent = 0, maxPower = 0, maxEff = 0;
    for (const row of this.data) {
      if (row.torque > maxTorque) maxTorque = row.torque;
      if (row.rpm !== null && row.rpm > maxRpm) maxRpm = row.rpm;
      if (row.current !== null && row.current > maxCurrent) maxCurrent = row.current;
      if (row.power !== null && row.power > maxPower) maxPower = row.power;
      if (row.eff !== null && row.eff > maxEff) maxEff = row.eff;
    }

    const roundUp = (v, step, floor) => Math.max(floor, Math.ceil(v / step) * step);
    const niceStep = (range, targetTicks) => {
      const raw = Math.max(1e-9, range / targetTicks);
      const pow = Math.pow(10, Math.floor(Math.log10(raw)));
      const norm = raw / pow;
      let mult = 10;
      if (norm <= 1) mult = 1;
      else if (norm <= 2) mult = 2;
      else if (norm <= 5) mult = 5;
      return mult * pow;
    };
    const torqueRange = this.data.length ? Math.max(0.1, maxTorque * 1.08) : 0.8;
    const torqueTick = niceStep(torqueRange, 5);
    const torqueMax = Math.max(torqueTick, Math.ceil(torqueRange / torqueTick) * torqueTick);
    const speedMax = roundUp(maxRpm * 1.08, 500, 500);
    const currentMax = roundUp(maxCurrent * 1.08, 2, 2);
    const powerMax = roundUp(maxPower * 1.08, maxPower > 1000 ? 500 : 50, 50);
    const effMax = maxEff > 100 ? roundUp(maxEff * 1.08, 10, 10) : 100;

    const xOf = t => plotX + (t / torqueMax) * plotW;
    const speedY = v => plotBottom - (v / speedMax) * plotH;
    const currentY = v => plotBottom - (v / currentMax) * plotH;
    const powerY = v => plotBottom - (v / powerMax) * plotH;
    const effY = v => plotBottom - (v / effMax) * plotH;

    // Grid: horizontal Y divisions, vertical torque tick marks.
    ctx.strokeStyle = 'rgba(20,32,51,0.08)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = plotY + plotH * i / 5;
      ctx.beginPath(); ctx.moveTo(plotX, y); ctx.lineTo(plotRight, y); ctx.stroke();
    }
    for (let t = 0; t <= torqueMax + torqueTick * 0.001; t += torqueTick) {
      const x = xOf(t);
      ctx.beginPath(); ctx.moveTo(x, plotY); ctx.lineTo(x, plotBottom); ctx.stroke();
    }

    const mappings = {
      rpm: speedY,
      current: currentY,
      power: powerY,
      eff: effY
    };

    // Sort samples into torque bins so this is a characteristic curve,
    // not a time trace. Binning also smooths 5 Hz measurement noise.
    const bins = Math.max(12, Math.min(40, Math.round(plotW / 18)));
    for (const c of this.curves) {
      const points = this.binned(c.key, bins, torqueMax);
      const yOf = mappings[c.key];
      let started = false;
      ctx.beginPath();
      for (const p of points) {
        if (!p) continue;
        const x = xOf(p.x);
        const y = yOf(p.y);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = c.color;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();
    }

    // Axes
    ctx.strokeStyle = 'rgba(20,32,51,0.55)';
    ctx.fillStyle = 'rgba(20,32,51,0.66)';
    ctx.lineWidth = 1;
    ctx.font = (compact ? '8px' : '9px') + ' ui-monospace, SFMono-Regular, Menlo, monospace';

    const drawRightAxis = (x, maxVal, decimals) => {
      ctx.beginPath();
      ctx.moveTo(x, plotY);
      ctx.lineTo(x, plotBottom);
      ctx.stroke();
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      for (let i = 0; i <= 5; i++) {
        const y = plotY + plotH * i / 5;
        const value = maxVal * (5 - i) / 5;
        ctx.beginPath();
        ctx.moveTo(x - 3, y);
        ctx.lineTo(x + 3, y);
        ctx.stroke();
        ctx.fillText(value.toFixed(decimals), x + 5, y);
      }
    };

    // Left speed axis
    ctx.beginPath();
    ctx.moveTo(plotX, plotY);
    ctx.lineTo(plotX, plotBottom);
    ctx.stroke();
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 5; i++) {
      const y = plotY + plotH * i / 5;
      const value = speedMax * (5 - i) / 5;
      ctx.beginPath();
      ctx.moveTo(plotX - 3, y);
      ctx.lineTo(plotX + 3, y);
      ctx.stroke();
      ctx.fillText(value.toFixed(0), plotX - 6, y);
    }

    const currentAxisX = rightBase;
    const powerAxisX = rightBase + axisGap;
    const effAxisX = rightBase + axisGap * 2;
    drawRightAxis(currentAxisX, currentMax, currentMax <= 25 ? 1 : 0);
    drawRightAxis(powerAxisX, powerMax, 0);
    drawRightAxis(effAxisX, effMax, 0);

    // X ticks and labels
    const torqueDecimals = torqueTick < 0.1 ? 2 : (torqueTick < 10 ? 1 : 0);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillStyle = 'rgba(20,32,51,0.66)';
    for (let t = 0; t <= torqueMax + torqueTick * 0.001; t += torqueTick) {
      const x = xOf(t);
      ctx.beginPath();
      ctx.moveTo(x, plotBottom);
      ctx.lineTo(x, plotBottom + 3);
      ctx.stroke();
      ctx.fillText(t.toFixed(torqueDecimals), x, plotBottom + 7);
    }
    ctx.fillText('Torque (N·m)', plotX + plotW / 2, plotBottom + 24);

    // Axis titles
    const axisTitle = (text, x, angle) => {
      ctx.save();
      ctx.translate(x, plotY + plotH / 2);
      ctx.rotate(angle);
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = 'rgba(20,32,51,0.66)';
      ctx.fillText(text, 0, 0);
      ctx.restore();
    };
    axisTitle('Speed (rpm)', 11, -Math.PI / 2);
    axisTitle('Current (A)', currentAxisX + axisGap - 9, Math.PI / 2);
    axisTitle('Power (W)', powerAxisX + axisGap - 9, Math.PI / 2);
    axisTitle('Efficiency (%)', effAxisX + axisGap - 9, Math.PI / 2);

    // Legend (desktop only; the chart note carries the color key on phones)
    if (!compact && plotW > 240) {
      const lw = compact ? 105 : 118;
      const lx = plotRight - lw - 8;
      const ly = plotY + 8;
      const lh = this.curves.length * 14 + 8;
      ctx.fillStyle = 'rgba(255,255,255,0.94)';
      ctx.strokeStyle = 'rgba(20,32,51,0.15)';
      ctx.beginPath();
      ctx.rect(lx, ly, lw, lh);
      ctx.fill();
      ctx.stroke();
      ctx.font = (compact ? '9px' : '10px') + ' system-ui, sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      for (let i = 0; i < this.curves.length; i++) {
        const c = this.curves[i];
        const yy = ly + 12 + i * 14;
        ctx.strokeStyle = c.color;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(lx + 8, yy);
        ctx.lineTo(lx + 28, yy);
        ctx.stroke();
        ctx.fillStyle = 'rgba(20,32,51,0.82)';
        ctx.fillText(c.label, lx + 34, yy);
      }
    }

    if (!this.data.length) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = 'rgba(20,32,51,0.40)';
      ctx.font = '11px system-ui, sans-serif';
      ctx.fillText('press Start run to record', plotX + plotW / 2, plotY + plotH / 2);
    }
  }
}

const charts = {
  rpm: new Spark('chartRpm', '#0f9d58'),
  torque: new Spark('chartTorque', '#2563eb'),
  power: new Spark('chartPower', '#ea580c'),
  alpha: new Spark('chartAlpha', '#db2777')
};

const perfChart = new PerformanceChart('chartPerf');
charts.perf = perfChart;
perfChart.draw();

const $ = id => document.getElementById(id);
const form = $('cfg');
let formReady = false;
let lastCalibState = -1;
let resizeTimer = 0;
let tickTimer = 0;

function nf(value, digits) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : '—';
}

function setOnline(online, text) {
  $('status').textContent = text;
  $('dot').classList.toggle('off', !online);
}

function updateInertia() {
  const mass = Number($('mass').value);
  const radius = Number($('radius').value);
  const inertia = mass > 0 && radius > 0 ? 0.5 * mass * radius * radius : NaN;
  $('inertiaOut').textContent = Number.isFinite(inertia) ? inertia.toFixed(5) : '—';
  return inertia;
}

function initForm(d) {
  $('mass').value = d.mass;
  $('radius').value = d.radius;
  $('ppr').value = d.ppr;
  $('avgWindow').value = d.avgWindow;
  $('friction').value = d.friction;
  updateInertia();
  formReady = true;
}

['mass', 'radius'].forEach(id => {
  $(id).addEventListener('input', updateInertia);
});

function render(d) {
  $('rpm').textContent = nf(d.rpm, 0);
  $('torque').textContent = nf(d.torque, 2);
  $('power').textContent = nf(d.power, 0);
  $('alpha').textContent = nf(d.alpha, 0);
  $('pulses').textContent = Math.round(Number(d.pulses) || 0).toLocaleString();
  $('revs').textContent = nf(d.revolutions, 1);

  $('rpmNow').textContent = nf(d.rpm, 0);
  $('torqueNow').textContent = nf(d.torque, 2);
  $('powerNow').textContent = nf(d.power, 0);
  $('alphaNow').textContent = nf(d.alpha, 0);

  $('peakRpm').textContent = nf(d.peakRpm, 0);
  $('peakTorque').textContent = nf(d.peakTorque, 2);
  $('peakPower').textContent = nf(d.peakPower, 0);
  $('sensor').textContent = d.sensor ? 'Target' : 'Clear';

  const b = d.bms || {};
  const bmsLive = !!b.connected && !!b.fresh && Number(b.voltage) > 0;
  if (bmsLive) {
    $('bmsStatus').textContent = 'connected';
    $('bmsBadge').className = 'badge on';
    $('bmsV').textContent = nf(b.voltage, 2);
    $('bmsA').textContent = nf(b.current, 1);
    $('bmsW').textContent = nf(b.voltage * b.current, 0);
    $('bmsSoc').textContent = nf(b.soc, 1);
    $('bmsDelta').textContent = nf(b.maxCell - b.minCell, 0);
    $('bmsTemp').textContent = nf(b.maxTemp, 1);
    $('bmsCellRange').textContent = b.maxCell > 0 && b.minCell > 0
      ? (b.minCell / 1000).toFixed(3) + ' V (#' + Math.round(b.minCellNo) + ') – ' +
        (b.maxCell / 1000).toFixed(3) + ' V (#' + Math.round(b.maxCellNo) + ')'
      : '—';
    $('bmsTempRange').textContent = nf(b.minTemp, 1) + ' – ' + nf(b.maxTemp, 1);
    $('bmsRem').textContent = nf((b.remCapMah || 0) / 1000, 2);
    $('bmsCycles').textContent = Math.round(b.cycles || 0);
    $('bmsMos').textContent =
      (b.chargeMos ? 'CHG on' : 'CHG off') + ' / ' +
      (b.dischargeMos ? 'DIS on' : 'DIS off');
  } else {
    if (!d.bleReady) $('bmsStatus').textContent = 'BLE unavailable';
    else if (b.connected) $('bmsStatus').textContent = 'connected · waiting for data';
    else $('bmsStatus').textContent = 'searching…';
    $('bmsBadge').className = 'badge';
    ['bmsV', 'bmsA', 'bmsW', 'bmsSoc', 'bmsDelta', 'bmsTemp',
     'bmsCellRange', 'bmsTempRange', 'bmsRem', 'bmsCycles', 'bmsMos']
      .forEach(id => { $(id).textContent = '—'; });
  }

  $('logName').textContent = d.log || '—';
  if (d.logOk) {
    $('logMsg').textContent = (d.logged || 0) + ' samples saved · ' + d.log +
      (d.runActive ? ' · recording' : ' · stopped');
    $('logMsg').className = 'msg logmsg';
  } else {
    $('logMsg').textContent = 'CSV logging unavailable (no filesystem)';
    $('logMsg').className = 'msg logmsg err';
  }
  $('stopBtn').disabled = !d.runActive;
  $('formatBtn').textContent = 'Format ' + (d.logFs || 'storage');

  $('effStat').textContent = d.effOk ? nf(d.eff, 1) + '%' : '—';

  $('perfNow').textContent = d.runActive ? 'recording' : 'run stopped';

  const calibState = Number(d.calibState) || 0;
  const calibMsg = $('calibMsg');
  if (calibState === 1) {
    $('calibBtn').textContent = 'Cancel calibration';
    calibMsg.className = 'msg';
    calibMsg.textContent = 'Calibrating… spin the roller up and let it coast · ' +
      nf((Number(d.calibElapsedMs) || 0) / 1000, 1) + ' s · ' +
      Math.round(Number(d.calibSamples) || 0) + ' samples';
  } else {
    $('calibBtn').textContent = 'Coast-down friction calibration';
    if (calibState === 2) {
      calibMsg.className = 'msg ok';
      calibMsg.textContent = 'Friction calibrated: ' + nf(d.friction, 4) +
        ' N·m from ' + Math.round(Number(d.calibSamples) || 0) + ' coast samples.';
    } else if (calibState === 3) {
      calibMsg.className = 'msg err';
      calibMsg.textContent = 'Not enough coasting samples. Spin the roller up, then let it coast freely.';
    } else {
      calibMsg.className = 'msg';
      calibMsg.textContent = '';
    }
  }
  if (calibState === 2 && lastCalibState !== 2) {
    $('friction').value = Number(d.friction).toFixed(4);
    updateInertia();
  }
  lastCalibState = calibState;

  charts.rpm.push(d.rpm);
  charts.torque.push(d.torque);
  charts.power.push(d.power);
  charts.alpha.push(d.alpha);

  if (d.runActive && Number(d.torque) > 0.005) {
    charts.perf.push({
      torque: d.torque,
      rpm: d.rpm,
      current: bmsLive && Number(b.current) >= 0 ? b.current : null,
      power: d.power,
      eff: d.effOk ? d.eff : null
    });
  }

  if (!formReady) initForm(d);
}

async function tick() {
  try {
    const res = await fetch('/data', { cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const d = await res.json();
    render(d);
    setOnline(true, 'Live');
  } catch (err) {
    setOnline(false, 'No data - check WiFi');
  }
  // Back off while the tab is hidden so the ESP32 is not serving a screen
  // nobody is looking at.
  tickTimer = setTimeout(tick, document.hidden ? 1000 : 200);
}

form.addEventListener('submit', async event => {
  event.preventDefault();
  const msg = $('formMsg');
  msg.className = 'msg';
  msg.textContent = 'Saving...';

  try {
    const body = new URLSearchParams(new FormData(form));
    const res = await fetch('/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
    msg.className = 'msg ok';
    msg.textContent = 'Saved.';
  } catch (err) {
    msg.className = 'msg err';
    msg.textContent = err.message || 'Save failed';
  }
});

$('calibBtn').addEventListener('click', async () => {
  const msg = $('calibMsg');
  msg.className = 'msg';
  msg.textContent = 'Working...';
  try {
    const res = await fetch('/calibrate', { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
    msg.textContent = data.active
      ? 'Calibration started. Spin up the roller, then let it coast freely.'
      : 'Calibration cancelled.';
  } catch (err) {
    msg.className = 'msg err';
    msg.textContent = err.message || 'Calibration request failed';
  }
});

async function postAction(path, okText) {
  const msg = $('ctrlMsg');
  msg.className = 'msg logmsg';
  msg.textContent = 'Working...';
  try {
    const res = await fetch(path, { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
    msg.className = 'msg logmsg ok';
    msg.textContent = okText;
    return true;
  } catch (err) {
    msg.className = 'msg logmsg err';
    msg.textContent = err.message || 'Request failed';
    return false;
  }
}

$('startBtn').addEventListener('click', async () => {
  if (!(await postAction('/start', 'New run started.'))) return;
  Object.values(charts).forEach(c => c.reset());
});

$('stopBtn').addEventListener('click', async () => {
  await postAction('/stop', 'Run stopped.');
});

$('formatBtn').addEventListener('click', async () => {
  if (!confirm('Erase all saved CSV runs and format the log storage?')) return;
  if (!(await postAction('/format', 'Storage formatted. New run started.'))) return;
  Object.values(charts).forEach(c => c.reset());
});

$('resetBtn').addEventListener('click', async () => {
  if (!confirm('Start a new CSV log and reset counters, peaks and charts?')) return;
  const msg = $('formMsg');
  try {
    const res = await fetch('/reset', { method: 'POST' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    Object.values(charts).forEach(c => c.reset());
    msg.className = 'msg ok';
    msg.textContent = 'New run started.';
  } catch (err) {
    msg.className = 'msg err';
    msg.textContent = 'Reset failed';
  }
});

document.addEventListener('visibilitychange', () => {
  if (!document.hidden) {
    clearTimeout(tickTimer);
    tick();
  }
});

window.addEventListener('resize', () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => {
    Object.values(charts).forEach(c => c.draw());
  }, 120);
});

tick();
</script>
</body>
</html>
)HTML";

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  uint32_t pulses;
  portENTER_CRITICAL(&pulseMux);
  pulses = totalPulses;
  portEXIT_CRITICAL(&pulseMux);

  uint32_t seconds = millis() / 1000UL;
  float revolutions = pulses / (float)settings.ppr;
  int sensor = (digitalRead(SENSOR_PIN) == LOW) ? 1 : 0;

  uint32_t bmsAge = 0;
  bool bmsFresh = false;
  if (bmsConnected && lastBmsVoltageMs != 0) {
    bmsAge = (uint32_t)(millis() - lastBmsVoltageMs);
    bmsFresh = bmsAge < 3000;
  }
  uint32_t runMs = (uint32_t)(millis() - runStartMs);
  uint32_t runStop = runActive ? 0 : runStopMs;

  const char *logName = currentLogPath[0] != '\0' ? currentLogPath : "";
  float pElec = bmsFresh ? bmsData.totalVoltage * bmsData.current : 0.0f;

  static char payload[2048];  // BSS, not the 8 KB loop-task stack
  snprintf(payload, sizeof(payload),
           "{\"millis\":%lu,\"rpm\":%.1f,\"omega\":%.2f,\"alpha\":%.2f,"
           "\"torque\":%.3f,\"power\":%.1f,\"pMech\":%.1f,\"pElec\":%.1f,"
           "\"eff\":%.1f,\"effOk\":%d,"
           "\"pulses\":%lu,\"revolutions\":%.2f,"
           "\"peakRpm\":%.1f,\"peakTorque\":%.3f,\"peakPower\":%.1f,"
           "\"mass\":%.3f,\"radius\":%.4f,\"inertia\":%.5f,"
           "\"friction\":%.4f,\"ppr\":%u,\"avgWindow\":%u,"
           "\"sensor\":%d,\"uptime\":%lu,\"runMs\":%lu,"
           "\"runActive\":%d,\"runStopMs\":%lu,"
           "\"calibActive\":%d,\"calibState\":%u,\"calibElapsedMs\":%lu,"
           "\"calibSamples\":%lu,\"calibTorque\":%.4f,"
           "\"log\":\"%s\",\"logged\":%lu,\"logOk\":%d,\"logFs\":\"%s\","
           "\"bleReady\":%d,"
           "\"bms\":{\"connected\":%d,\"fresh\":%d,\"ageMs\":%lu,\"ms\":%lu,"
           "\"voltage\":%.2f,\"current\":%.2f,"
           "\"soc\":%.1f,\"maxCell\":%u,\"maxCellNo\":%u,"
           "\"minCell\":%u,\"minCellNo\":%u,"
           "\"maxTemp\":%.1f,\"minTemp\":%.1f,"
           "\"remCapMah\":%lu,\"cycles\":%u,"
           "\"chargeMos\":%d,\"dischargeMos\":%d}}",
           (unsigned long)millis(), rpm, omega, alpha,
           torque, power, power, pElec,
           efficiency, efficiencyValid ? 1 : 0,
           (unsigned long)pulses, revolutions,
           peakRpm, peakTorque, peakPower,
           settings.massKg, settings.radiusM, settings.inertia,
           settings.friction, (unsigned)settings.ppr, (unsigned)settings.avgWindow,
           sensor, (unsigned long)seconds, (unsigned long)runMs,
           runActive ? 1 : 0, (unsigned long)runStop,
           calibActive ? 1 : 0, (unsigned)calibState,
           (unsigned long)calibElapsedMs, (unsigned long)calibSamples,
           calibLastTau,
           logName, (unsigned long)logRecordNum, loggingReady ? 1 : 0,
           logFsName, bleReady ? 1 : 0,
           bmsConnected ? 1 : 0, bmsFresh ? 1 : 0, (unsigned long)bmsAge,
           (unsigned long)lastBmsVoltageMs,
           bmsData.totalVoltage, bmsData.current,
           bmsData.soc, (unsigned)bmsData.maxCellMv, (unsigned)bmsData.maxCellNo,
           (unsigned)bmsData.minCellMv, (unsigned)bmsData.minCellNo,
           bmsData.maxTemp, bmsData.minTemp,
           (unsigned long)bmsData.remCapMah, (unsigned)bmsData.cycles,
           bmsData.chargeMos ? 1 : 0, bmsData.dischargeMos ? 1 : 0);

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", payload);
}

String baseName(const String &path) {
  int slash = path.lastIndexOf('/');
  return slash >= 0 ? path.substring(slash + 1) : path;
}

void handleDownload() {
  flushLogBuffer();

  if (logFs == nullptr) {
    server.send(404, "text/plain", "CSV logging unavailable");
    return;
  }

  String name;
  if (server.hasArg("active")) {
    name = currentLogPath;
  } else if (server.hasArg("file")) {
    // Accept only a bare filename to prevent path traversal.
    name = "/" + baseName(server.arg("file"));
  }

  if (name.length() <= 1 || !name.endsWith(".csv") || !logFs->exists(name)) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }

  File f = logFs->open(name, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }

  String filename = baseName(name);
  server.sendHeader("Content-Disposition",
                    "attachment; filename=\"" + filename + "\"");
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleLogs() {
  flushLogBuffer();

  String html;
  html.reserve(4096);
  html += F("<!doctype html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<meta name='color-scheme' content='dark'><title>Dyno logs</title>"
            "<style>"
            ":root{--bg:#f3f6fb;--card:#ffffff;--line:#d6dfeb;--text:#142033;"
            "--muted:#5d6b7e;--green:#0f9d58;--blue:#2563eb}"
            "*{box-sizing:border-box}"
            "body{margin:0;min-height:100vh;color:var(--text);background:"
            "radial-gradient(900px 500px at 15% -10%,#dbeafe 0,transparent 60%),"
            "radial-gradient(700px 400px at 100% 0,#e5e9ff 0,transparent 55%),var(--bg);"
            "font-family:system-ui,-apple-system,'Segoe UI',Roboto,sans-serif;"
            "padding:24px clamp(14px,4vw,40px)}"
            "main{max-width:900px;margin:auto}"
            "h1{margin:0 0 4px;font-size:clamp(24px,4vw,32px)}"
            "h1 span{color:var(--green)}"
            ".sub{color:var(--muted);font-size:13px;margin-bottom:18px}"
            ".card{background:linear-gradient(180deg,#ffffff,#f7faff);"
            "border:1px solid var(--line);border-radius:16px;padding:16px;margin-bottom:14px;"
            "box-shadow:0 12px 30px rgba(25,50,90,.08)}"
            ".row{display:flex;justify-content:space-between;align-items:center;gap:12px;"
            "padding:10px 0;border-top:1px solid var(--line);flex-wrap:wrap}"
            ".row:first-of-type{border-top:0}"
            ".name{font-weight:600;word-break:break-all}"
            ".meta{color:var(--muted);font-size:12px}"
            "a.btn{background:var(--green);color:#fff;padding:9px 14px;"
            "border-radius:10px;text-decoration:none;font-weight:600;display:inline-block}"
            "a.btn.secondary{background:#e7edf6;color:var(--text)}"
            "a.btn:active{transform:translateY(1px)}"
            ".empty{color:var(--muted);font-size:13px}"
            "</style></head><body><main>");
  html += F("<h1>Dyno <span>logs</span></h1>"
            "<div class='sub'>");
  html += "Storage: ";
  html += logFsName;
  if (logFs != nullptr) {
    html += " · ";
    html += String((unsigned long)((logUsedBytes() + 1023) / 1024));
    html += " / ";
    html += String((unsigned long)((logTotalBytes() + 1023) / 1024));
    html += " KB used";
  }
  html += F("</div>");

  html += F("<div class='card'><div class='row'><div><div class='name'>");
  html += currentLogPath[0] != '\0' ? currentLogPath : "no active run";
  html += F("</div><div class='meta'>active run</div></div>");
  html += F("<a class='btn' href='/download?active=1'>Download CSV</a></div></div>");

  html += F("<div class='card'><h2 style='margin:0 0 6px;font-size:16px'>Saved runs</h2>");

  bool any = false;
  if (logFs != nullptr) {
    File root = logFs->open("/");
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String name = file.name();
        if (!name.startsWith("/")) name = "/" + name;
        if (name.endsWith(".csv")) {
          any = true;
          String base = baseName(name);
          html += F("<div class='row'><div><div class='name'>");
          html += base;
          html += F("</div><div class='meta'>");
          html += String((unsigned long)file.size());
          html += " bytes";
          if (name == currentLogPath) html += " · active";
          html += F("</div></div><a class='btn secondary' href='/download?file=");
          html += base;
          html += F("'>CSV</a></div>");
        }
        file = root.openNextFile();
      }
      root.close();
    }
  }

  if (!any) {
    html += F("<p class='empty'>No CSV runs saved yet.</p>");
  }

  html += F("</div>");
  html += F("<a class='btn secondary' href='/'>&larr; Back to dashboard</a>");
  html += F("</main></body></html>");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", html);
}

void handleConfig() {
  if (!server.hasArg("mass") || !server.hasArg("radius") ||
      !server.hasArg("ppr") || !server.hasArg("avgWindow")) {
    server.send(400, "application/json", "{\"error\":\"missing field\"}");
    return;
  }

  float mass = server.arg("mass").toFloat();
  float radius = server.arg("radius").toFloat();
  float friction = server.hasArg("friction") ? server.arg("friction").toFloat()
                                             : settings.friction;
  long ppr = server.arg("ppr").toInt();
  long avgWindow = server.arg("avgWindow").toInt();

  if (!(mass > 0.001f && mass <= 1000.0f)) {
    server.send(400, "application/json", "{\"error\":\"mass must be > 0\"}");
    return;
  }
  if (!(radius > 0.001f && radius <= 10.0f)) {
    server.send(400, "application/json", "{\"error\":\"radius must be > 0\"}");
    return;
  }
  if (!(friction >= 0.0f && friction <= 100.0f)) {
    server.send(400, "application/json", "{\"error\":\"friction must be >= 0\"}");
    return;
  }
  if (ppr < 1 || ppr > 1000) {
    server.send(400, "application/json", "{\"error\":\"PPR must be 1..1000\"}");
    return;
  }
  if (avgWindow < 1 || avgWindow > 64) {
    server.send(400, "application/json", "{\"error\":\"avgWindow must be 1..64\"}");
    return;
  }

  settings.massKg = mass;
  settings.radiusM = radius;
  settings.inertia = 0.5f * mass * radius * radius;
  settings.friction = friction;
  settings.ppr = (uint16_t)ppr;
  settings.avgWindow = (uint8_t)avgWindow;
  saveSettings();
  clearSpeedHistory();

  char reply[64];
  snprintf(reply, sizeof(reply), "{\"ok\":true,\"inertia\":%.5f}",
           settings.inertia);
  server.send(200, "application/json", reply);
}

void handleCalibrate() {
  if (calibActive) {
    cancelCoastDown();
    server.send(200, "application/json", "{\"ok\":true,\"active\":0}");
    return;
  }
  startCoastDown();
  server.send(200, "application/json", "{\"ok\":true,\"active\":1}");
}

void handleReset() {
  resetStats();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStart() {
  resetStats();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  stopRun();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleFormat() {
  if (!formatLogFs()) {
    server.send(500, "application/json", "{\"error\":\"format failed\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  loadSettings();
  runStartMs = millis();

  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, SENSOR_EDGE);

  mountLogFs();

  bleReady = NimBLEDevice::init("Dyno-ESP32");
  if (bleReady) startBmsScan(5000);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool ok = WiFi.softAP(AP_SSID);  // no password: open access point
  IPAddress ip = WiFi.softAPIP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/config", HTTP_POST, handleConfig);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/format", HTTP_POST, handleFormat);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/download", HTTP_GET, handleDownload);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println();
  Serial.println("Dyno ESP32");
  Serial.print("AP: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(ip);
  Serial.print("AP start: ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("Pulses/rev: ");
  Serial.println(settings.ppr);
  Serial.print("Roller: ");
  Serial.print(settings.massKg, 2);
  Serial.print(" kg, r=");
  Serial.print(settings.radiusM, 3);
  Serial.print(" m, avg window=");
  Serial.println(settings.avgWindow);
  Serial.print("Inertia: ");
  Serial.print(settings.inertia, 5);
  Serial.println(" kg*m^2");
  Serial.print("Friction: ");
  Serial.print(settings.friction, 4);
  Serial.println(" N*m");
  Serial.print("CSV logging: ");
  if (loggingReady) {
    Serial.print(currentLogPath);
    Serial.print(" on ");
    Serial.println(logFsName);
  } else {
    Serial.println("unavailable");
  }
  Serial.print("BLE init: ");
  Serial.println(bleReady ? "OK" : "FAILED");
  Serial.print("BMS MAC: ");
  Serial.println(BMS_MAC_ADDRESS);
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if ((uint32_t)(now - lastControlMs) >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updateTelemetry();
  }

  processBleRingBuffer();
  serviceBms(now);

  streamSerialTelemetry(now);

  if (runActive && loggingReady && (uint32_t)(now - lastLogMs) >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    appendLogRecord();
  }
}