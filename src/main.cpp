#define DEBUG_MODE 1  // Set to 0 for flight, 1 for bench testing

#include <Arduino.h>
#include <Globals.h>
#include <Logging.h>
#include <PhysicsConstants.h>
#include <PinDefinitions.h>
#include <SD.h>
#include <SPI.h>
#include <StateMachine.h>
#include <Wire.h>
#include <control/ControlStruct.h>
#include <sensor_drivers/Adxl.h>
#include <sensor_drivers/BNO.h>
#include <sensor_drivers/Lps22.h>
#include <Atmosphere.h>
#include <AirbrakeController.h>
#include <CdLookup.h>
#include <RocketConfig.h>
#include <UKF1D.h>
#include <SensorWeighting.h>

#define USE_BNO080 0
#define USE_TELEMETRY 0

// ── Operating Modes ──────────────────────────────────────────────
// FLIGHT:        Real sensors, full state machine, actual launch
// SHITL:         Simulated sensors from serial, local controller, servos active
// SHITL_DEMO:    Same as SHITL but physical servos are NOT driven (demo/bench)
// TEST:          Real sensors, servo testing, no state machine
// SENSOR_MOVING: Real sensors, UKF runs, no state machine, no servo motion.
//                Logs CSV to SD in the same format as FLIGHT so ground tests
//                (carrying/rotating/shaking the rocket) produce a file that
//                parses with the same tooling.
enum class TeensyMode { FLIGHT, SHITL, SHITL_DEMO, TEST, SENSOR_MOVING };
static TeensyMode currentMode = TeensyMode::FLIGHT;

static inline bool isSimMode(TeensyMode m) {
  return m == TeensyMode::SHITL || m == TeensyMode::SHITL_DEMO;
}

// driveServos: command the physical servos by extension % [SERVO_MIN_PCT..SERVO_MAX_PCT].
// BrakeState.pct mirrors the same command in user-facing airbrake-% [0..100]
// (0 at SERVO_MIN_PCT, 100 at SERVO_MAX_PCT) so the logged Airbrake_pct column
// reads the way operators expect — 0 when retracted, not 20.
void driveServos(float pct) {
  pct = constrain(pct, RocketConfig::SERVO_MIN_PCT, RocketConfig::SERVO_MAX_PCT);
  BrakeState.pct = RocketConfig::servoPctToAirbrakePct(pct);
  if (currentMode == TeensyMode::SHITL_DEMO) return;
  airbrake_servo_1.setExtension(pct);
  airbrake_servo_2.setExtension(pct);
}

static const char* modeToString(TeensyMode m) {
  switch (m) {
    case TeensyMode::FLIGHT:        return "FLIGHT";
    case TeensyMode::SHITL:         return "SHITL";
    case TeensyMode::SHITL_DEMO:    return "SHITL_DEMO";
    case TeensyMode::TEST:          return "TEST";
    case TeensyMode::SENSOR_MOVING: return "SENSOR_MOVING";
  }
  return "UNKNOWN";
}

// Audible confirmation the Teensy booted in FLIGHT mode. 5 on/off cycles of
// 500 ms each = 5 s total, using tone() so it works with both passive and
// active buzzers.
static void flightModeBuzzerConfirm() {
  const int BEEP_FREQ_HZ = 2000;
  const int BEEP_HALF_PERIOD_MS = 500;
  const int BEEP_CYCLES = 5;
  for (int i = 0; i < BEEP_CYCLES; i++) {
    tone(PinDefs.BUZZER, BEEP_FREQ_HZ);
    delay(BEEP_HALF_PERIOD_MS);
    noTone(PinDefs.BUZZER);
    digitalWrite(PinDefs.BUZZER, LOW);
    delay(BEEP_HALF_PERIOD_MS);
  }
}

// Audible confirmation the USB cable was unplugged while in FLIGHT mode and
// the Teensy is now running on internal battery power. 8 cycles × 250 ms on /
// 250 ms off = 4 s total. The tighter cadence (vs the 5-beep flight-mode
// confirm) makes it unambiguous by ear which event just fired.
static void internalPowerBuzzerConfirm() {
  const int BEEP_FREQ_HZ = 2000;
  const int BEEP_HALF_PERIOD_MS = 250;
  const int BEEP_CYCLES = 8;
  for (int i = 0; i < BEEP_CYCLES; i++) {
    tone(PinDefs.BUZZER, BEEP_FREQ_HZ);
    delay(BEEP_HALF_PERIOD_MS);
    noTone(PinDefs.BUZZER);
    digitalWrite(PinDefs.BUZZER, LOW);
    delay(BEEP_HALF_PERIOD_MS);
  }
}

// ── Hardware ─────────────────────────────────────────────────────
Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
BNO bno080;

Bilda airbrake_servo_1;
Bilda airbrake_servo_2;
Igniter primaryIgniter = Igniter(PinDefs.IGNITER_0, PinDefs.IGNITER_SENSE_0);
Igniter backupIgniter = Igniter(PinDefs.IGNITER_1, PinDefs.IGNITER_SENSE_1);

Logging logging(DEBUG_MODE, true, PinDefs.SD_CS);
StatusIndicator statusIndicator =
    StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);

AirbrakeController airbrakeController;
UKF1D altitudeFilter;

// ── Shared State ─────────────────────────────────────────────────
SensorData_t sensors;
BrakeState_t BrakeState;
FlightState_t FlightState;
I2CControl_t I2CControl;
States state = States::BOOT;
int failed_sensors = 0;

unsigned long last_loop_time = 0;

static struct {
  float pressure = 0;
  float temperature = 0;
  // Accel-Z reading at rest (gravity on the thrust axis). Defaults to ideal 1g;
  // ZERO re-measures it so residual sensor bias doesn't integrate into phantom
  // velocity drift.
  float accel_z_bias = 1.0f;
} RefCalibration;

// ── Sensor Helpers ───────────────────────────────────────────────

// Samples real sensors over ~1 second to capture the at-rest baseline.
// Pressure/temperature become the altitude reference; accel-Z bias subtracts
// the resting-gravity reading on the thrust axis so the filter doesn't drift
// when the rocket is actually stationary.
static void calibrateSensors(Lps22 &lps, Adxl *lg_accel = nullptr) {
  delay(100);
  const int N = 40;  // 40 samples × 25 ms = 1 s
  int valid = 0;
  float p_sum = 0.0f, t_sum = 0.0f, a_sum = 0.0f;
  for (int i = 0; i < N; i++) {
    float p = 0.0f, t = 0.0f;
    lps.readPressure(&p);
    lps.readTemperature(&t);
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (lg_accel) lg_accel->readAccelerometer(&ax, &ay, &az);
    if (p > 0.0f) {
      p_sum += p;
      t_sum += t;
      a_sum += az;
      valid++;
    }
    delay(25);
  }
  if (valid > 0) {
    RefCalibration.pressure = p_sum / (float)valid;
    RefCalibration.temperature = t_sum / (float)valid + CELSIUS_TO_KELVIN;
    if (lg_accel) RefCalibration.accel_z_bias = a_sum / (float)valid;
  }
}

template <typename T>
static bool initSensor(T &sensor, const char *name, int maxAttempts = 10) {
  int attempts = 0;
  while (!sensor.begin()) {
    logging.log(name);
    Serial.print(F("Waiting for "));
    Serial.print(name);
    Serial.println(F("..."));
    delay(1000);
    if (++attempts >= maxAttempts) {
      failed_sensors++;
      return false;
    }
  }
  Serial.print(name);
  Serial.println(F(" initialized"));
  return true;
}

static float altitudeDelta(float p, float T) {
  if (p <= 0.0f) return NAN;
  float p_ref = RefCalibration.pressure;
  float T_ref = RefCalibration.temperature;
  float Tbar = 0.5f * (T_ref + T);
  return (Rd * Tbar / g0) * log(p_ref / p);
}

// ── Altitude Filter ─────────────────────────────────────────────

static float filterAltitude(float raw_alt) {
  float dt = LOOP_INTERVAL_MS / 1000.0f;

  if (!altitudeFilter.isInitialized()) {
    altitudeFilter.init(raw_alt, 0.0f, 0.0f);
    FlightState.velocity = 0.0f;
    return raw_alt;
  }

  altitudeFilter.predict(dt);

  // Weight from the filter's own velocity — avoids circular dependency through
  // FlightState.velocity (which is itself derived from the filter).
  SensorWeights w = SensorWeighting::getWeights(
      static_cast<uint8_t>(state), altitudeFilter.velocity(), 343.0f);

  // Blend both accelerometer channels via inverse-variance weighting and do a
  // single UKF update — mathematically equivalent to two sequential updates
  // (no predict between them) but half the compute cost. Subtract the measured
  // at-rest bias (from ZERO calibration) instead of an ideal 1g, so residual
  // sensor bias doesn't accumulate into phantom velocity drift.
  float bias = RefCalibration.accel_z_bias;
  float a_low  = (sensors.accel_z         - bias) * g0;
  float a_high = (sensors.accel_z_high_g  - bias) * g0;
  float inv_R_low  = 1.0f / w.R_accel_low;
  float inv_R_high = 1.0f / w.R_accel_high;
  float inv_R_sum  = inv_R_low + inv_R_high;
  float accel_blended = (a_low * inv_R_low + a_high * inv_R_high) / inv_R_sum;
  float R_blended     = 1.0f / inv_R_sum;
  altitudeFilter.updateAccel(accel_blended, R_blended);

  altitudeFilter.updateBaro(raw_alt, w.R_baro);

  FlightState.velocity = altitudeFilter.velocity();

  return altitudeFilter.altitude();
}

// ── Mode Negotiation ─────────────────────────────────────────────
// Protocol: Teensy sends READY, dashboard responds with MODE,<mode>
// Fallback: Teensy sends DATAREQUEST, legacy scripts respond with CSV

// Exact-match MODE,<name> against the line (no null-terminator fuzz, no
// prefix-order gotchas — SHITL vs SHITL_DEMO is unambiguous).
static bool parseMode(const char *line, TeensyMode *out) {
  if (strcmp(line, "MODE,FLIGHT")        == 0) { *out = TeensyMode::FLIGHT;        return true; }
  if (strcmp(line, "MODE,SHITL")         == 0) { *out = TeensyMode::SHITL;         return true; }
  if (strcmp(line, "MODE,SHITL_DEMO")    == 0) { *out = TeensyMode::SHITL_DEMO;    return true; }
  if (strcmp(line, "MODE,TEST")          == 0) { *out = TeensyMode::TEST;          return true; }
  if (strcmp(line, "MODE,SENSOR_MOVING") == 0) { *out = TeensyMode::SENSOR_MOVING; return true; }
  return false;
}

static void ackMode(TeensyMode m) {
  Serial.print("MODE_ACK,");
  Serial.println(modeToString(m));
}

static TeensyMode negotiateMode() {
  while (Serial.available()) Serial.read();  // flush stale data

  // Phase 1: New protocol — send READY, wait for MODE command
  Serial.println("READY");
  Serial.setTimeout(2000);

  char buf[128];
  int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  if (len > 0) {
    TeensyMode m;
    if (parseMode(buf, &m)) { ackMode(m); return m; }

    // Legacy: CSV response to READY means old SHITL script
    int fieldCount = 1;
    for (int i = 0; i < len; i++) if (buf[i] == ',') fieldCount++;
    if (fieldCount >= 6) return TeensyMode::SHITL;
  }

  // Phase 2: Legacy — send DATAREQUEST, check for CSV response
  while (Serial.available()) Serial.read();
  Serial.println("DATAREQUEST");
  Serial.setTimeout(2000);

  len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  if (len > 0) {
    int fieldCount = 1;
    for (int i = 0; i < len; i++) if (buf[i] == ',') fieldCount++;
    if (fieldCount >= 6) return TeensyMode::SHITL;
  }

  return TeensyMode::FLIGHT;
}

// ── SHITL calibration state (declared here for switchToSHITL) ────
static bool simCalibrated = false;
static int simCalibrationCount = 0;
static const int SIM_CAL_SAMPLES = 20;  // ~1s at 50ms loop interval

// Ground truth from the dashboard. When the SHITL dashboard uses the 9-field
// extended CSV (time, ax, ay, az, pressure, temp, alt_m, vel_ms, phase_code),
// the Teensy runs its controller on those exact values instead of UKF +
// state-machine-derived estimates. That makes SHITL_DEMO byte-for-byte
// identical to the pure-Python demo in controller behavior. The 6-field
// legacy protocol still works and falls back to UKF + state machine.
struct SimGroundTruth_t {
  float sim_time_s = 0.0f;
  float alt_m = 0.0f;
  float vel_ms = 0.0f;
  int phase_code = 0;   // 0=PREFLIGHT, 1=BURN, 2=COAST, 3=DESCENT
  bool valid = false;   // true when the last frame carried ground truth
};
static SimGroundTruth_t SimTruth;

static States phaseCodeToState(int phase_code) {
  switch (phase_code) {
    case 1: return States::IGNITION;  // motor burn
    case 2: return States::ASCENT;    // coast
    case 3: return States::DESCENT;   // post-apogee
    case 0: default: return States::IDLE;  // preflight
  }
}

// ── Serial Command Handling ──────────────────────────────────────

static void enterSHITL(TeensyMode target) {
  currentMode = target;
  Serial.setTimeout(25);  // SHITL data arrives within a few ms; keep tight
  BrakeState.hasCheckedForHorizontal = true;
  simCalibrated = false;
  simCalibrationCount = 0;
  RefCalibration.pressure = 0;
  RefCalibration.temperature = 0;
  RefCalibration.accel_z_bias = 0.0f;  // zeroed before accumulation
  altitudeFilter = UKF1D();
  FlightState.velocity = 0.0f;
  SimTruth.valid = false;  // new dashboard session → re-detect protocol
  airbrakeController.reset();
  state = States::IDLE;
  logging.setDebug(true);
  if (target == TeensyMode::SHITL_DEMO) {
    // Park servos once on entry; driveServos() suppresses motion afterwards.
    airbrake_servo_1.setExtension(RocketConfig::SERVO_MIN_PCT);
    airbrake_servo_2.setExtension(RocketConfig::SERVO_MIN_PCT);
    BrakeState.pct = 0.0f;  // 0% airbrake (servos parked at SERVO_MIN_PCT)
  }
  ackMode(target);
  Serial.print(F("Mode: "));
  Serial.println(modeToString(currentMode));
}

static void handleSerialCommand(const char *line) {
  TeensyMode m;
  if (parseMode(line, &m)) {
    if (isSimMode(m)) {
      enterSHITL(m);
    } else {
      // Beep the buzzer on a *transition* into FLIGHT (e.g. dashboard
      // promoting the Teensy from SHITL to live). Only fires from ground
      // states so we never burn 5 s of the main loop mid-flight.
      bool entering_flight =
          (m == TeensyMode::FLIGHT) && (currentMode != TeensyMode::FLIGHT);
      currentMode = m;
      ackMode(m);
      if (entering_flight && state != States::SENSOR_ERROR &&
          (state == States::IDLE || state == States::AIRBRAKE_TEST ||
           state == States::BOOT || state == States::LANDED)) {
        Serial.println(F("FLIGHT mode -- buzzer confirm"));
        flightModeBuzzerConfirm();
      }
    }
    return;
  }
  if (strncmp(line, "SERVO,", 6) == 0) {
    float pct = strtof(line + 6, nullptr);
    pct = constrain(pct, RocketConfig::SERVO_MIN_PCT, RocketConfig::SERVO_MAX_PCT);
    driveServos(pct);
  } else if (strcmp(line, "ZERO") == 0) {
    // Press ZERO only when the rocket is stationary — samples for ~1s to
    // capture the true at-rest pressure, temperature, and accel bias. The
    // accel-bias correction kills the phantom 5-7 m/s velocity drift.
    Serial.println(F("Zeroing sensors (~1s sample)..."));
    RefCalibration.pressure = 0;
    RefCalibration.temperature = 0;
    calibrateSensors(lps22, &adxl345);
    altitudeFilter = UKF1D();
    FlightState.velocity = 0.0f;
    Serial.print(F("Zeroed: p_ref="));
    Serial.print(RefCalibration.pressure, 2);
    Serial.print(F(" mbar, accel_z_bias="));
    Serial.print(RefCalibration.accel_z_bias, 4);
    Serial.println(F(" g"));
  } else if (strcmp(line, "RESET") == 0) {
    Serial.println(F("RESET_ACK"));
    Serial.flush();
    delay(10);
    SCB_AIRCR = 0x05FA0004;  // ARM software reset — same as power cycle
  }
}

// Non-blocking serial line reader for FLIGHT and TEST modes
static char _cmdBuf[64];
static int _cmdPos = 0;

static void checkSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (_cmdPos > 0) {
        _cmdBuf[_cmdPos] = '\0';
        handleSerialCommand(_cmdBuf);
        _cmdPos = 0;
      }
      continue;
    }
    if (_cmdPos < (int)sizeof(_cmdBuf) - 1) {
      _cmdBuf[_cmdPos++] = c;
    }
  }
}

// ── Read Real Sensors ────────────────────────────────────────────

static float readRealSensors() {
  adxl345.readAccelerometer(&sensors.accel_x, &sensors.accel_y, &sensors.accel_z);
  adxl375.readAccelerometer(&sensors.accel_x_high_g, &sensors.accel_y_high_g, &sensors.accel_z_high_g);
  lps22.readPressure(&sensors.pressure);
  lps22.readTemperature(&sensors.temperature);

#if USE_BNO080
  if (bno080.dataAvailable()) {
    bno080.getLinearAccelerometer(&sensors.bno_x, &sensors.bno_y, &sensors.bno_z);
    bno080.getRotationVector(&sensors.bno_i, &sensors.bno_j, &sensors.bno_k, &sensors.bno_real);
  }
#endif

  return altitudeDelta(sensors.pressure, sensors.temperature + CELSIUS_TO_KELVIN);
}

// ── Read Simulated Sensors (SHITL mode) ──────────────────────────

static bool isDashboardCommand(const char *line) {
  return strncmp(line, "SERVO,", 6) == 0 || strcmp(line, "ZERO") == 0 ||
         strcmp(line, "RESET") == 0 || strncmp(line, "MODE,", 5) == 0;
}

static float readSimulatedSensors() {
  Serial.println("DATAREQUEST");

  // Extended protocol is 9 fields (legacy is 6):
  //   time_s, ax, ay, az, pressure_mbar, temp_c, alt_m, vel_ms, phase_code
  // Bigger buffer so a 9-field line with float formatting never gets split.
  char buf[192];
  float values[9];
  int gotCount = 0;

  // Worst case: 3 reads × 25ms = 75ms. Commands cap at 2 so sensor data can't
  // be starved by command spam.
  int commands_handled = 0;
  for (int attempts = 0; attempts < 3; attempts++) {
    int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    if (len == 0) break;
    buf[len] = '\0';

    if (isDashboardCommand(buf)) {
      handleSerialCommand(buf);
      if (++commands_handled >= 2) break;
      continue;
    }

    int index = 0;
    char *tok = strtok(buf, ",");
    while (tok && index < 9) {
      values[index++] = strtof(tok, nullptr);
      tok = strtok(nullptr, ",");
    }

    if (index >= 6 && values[4] > 0.0f) {
      gotCount = index;
      break;
    }
  }

  if (gotCount < 6) return NAN;

  sensors.accel_x = values[1];
  sensors.accel_y = values[2];
  sensors.accel_z = values[3];
  sensors.pressure = values[4];
  sensors.temperature = values[5];
  sensors.accel_x_high_g = sensors.accel_x;
  sensors.accel_y_high_g = sensors.accel_y;
  sensors.accel_z_high_g = sensors.accel_z;

  // Extended protocol: dashboard provided ground truth so the controller runs
  // on the same inputs as the pure-Python demo. Skip calibration + UKF path.
  if (gotCount >= 9) {
    SimTruth.sim_time_s = values[0];
    SimTruth.alt_m      = values[6];
    SimTruth.vel_ms     = values[7];
    SimTruth.phase_code = (int)values[8];
    SimTruth.valid      = true;
    return SimTruth.alt_m;
  }

  // Legacy 6-field path: calibrate ground reference, then derive altitude
  // from pressure via hypsometric formula.
  SimTruth.valid = false;
  if (!simCalibrated) {
    RefCalibration.pressure += sensors.pressure;
    RefCalibration.temperature += sensors.temperature + CELSIUS_TO_KELVIN;
    RefCalibration.accel_z_bias += sensors.accel_z;
    simCalibrationCount++;
    if (simCalibrationCount >= SIM_CAL_SAMPLES) {
      RefCalibration.pressure /= (float)SIM_CAL_SAMPLES;
      RefCalibration.temperature /= (float)SIM_CAL_SAMPLES;
      RefCalibration.accel_z_bias /= (float)SIM_CAL_SAMPLES;
      simCalibrated = true;
      Serial.print(F("SHITL calibrated: p_ref="));
      Serial.print(RefCalibration.pressure, 2);
      Serial.print(F(" mbar, accel_z_bias="));
      Serial.print(RefCalibration.accel_z_bias, 4);
      Serial.println(F(" g"));
    }
    return NAN;
  }

  return altitudeDelta(sensors.pressure, sensors.temperature + CELSIUS_TO_KELVIN);
}

// ── I2C Control Packet (FLIGHT mode only) ────────────────────────

void sendControlPacket(float altitude) {
  if (currentMode != TeensyMode::FLIGHT) return;
  if (!USE_CONTROL || I2CControl.fallback || (millis() - I2CControl.lastSend < CONTROL_INTERVAL_MS)) return;

  ControlPacket controlPacket;
  controlPacket.time_ms = millis();
  controlPacket.pressure = sensors.pressure;
  controlPacket.temperature = sensors.temperature;
  controlPacket.accel_z_low_g = sensors.accel_z;
  controlPacket.accel_z_high_g = sensors.accel_z_high_g;
  controlPacket.baro_altitude = altitude;
  controlPacket.flight_state = static_cast<uint8_t>(state);
  controlPacket.crc = computeControlCRC((uint8_t *)&controlPacket, sizeof(controlPacket) - 1);

  Wire.beginTransmission(CTRL_TEENSY_ADDR);
  Wire.write((uint8_t *)&controlPacket, sizeof(controlPacket));
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    I2CControl.failCount++;
    if (I2CControl.failCount >= I2C_FAIL_THRESHOLD) {
      I2CControl.fallback = true;
      Serial.println(F("Control Teensy I2C failed, switching to fallback"));
    }
  } else {
    I2CControl.failCount = 0;
    delayMicroseconds(200);
    uint8_t bytesRead = Wire.requestFrom(CTRL_TEENSY_ADDR, (uint8_t)sizeof(CommandPacket));
    if (bytesRead == sizeof(CommandPacket)) {
      CommandPacket cmdPkt;
      uint8_t *cmdBuf = (uint8_t *)&cmdPkt;
      for (size_t i = 0; i < sizeof(CommandPacket); i++) {
        cmdBuf[i] = Wire.read();
      }
      uint8_t cmdCrc = computeControlCRC(cmdBuf, sizeof(CommandPacket) - 1);
      if (cmdPkt.crc == cmdCrc) {
        sensors.potentiometer_value = cmdPkt.potentiometer_value;
        I2CControl.cmd_servo_1 = cmdPkt.servo_angle_1;
        I2CControl.cmd_servo_2 = cmdPkt.servo_angle_2;
        I2CControl.predicted_apogee = cmdPkt.predicted_apogee;
        I2CControl.cd_add_cmd = cmdPkt.cd_add_cmd;
      }
    } else {
      while (Wire.available()) Wire.read();
    }
  }

  I2CControl.lastSend = millis();
}

// ── Local Controller (SHITL mode) ────────────────────────────────

static void runLocalController(float altitude) {
  float vel = FlightState.velocity;
  float alt_msl = altitude + RocketConfig::LAUNCH_SITE_ALT_MSL_M;
  float sos = Atmosphere::speedOfSound(alt_msl);
  // When running on dashboard ground truth, use the sim clock so
  // POST_LAUNCH_DELAY_S etc. line up with Python's sim_time exactly.
  float time_s = SimTruth.valid ? SimTruth.sim_time_s : millis() / 1000.0f;
  float dt = LOOP_INTERVAL_MS / 1000.0f;

  float cd_add = airbrakeController.update(
      altitude, vel, sensors.accel_z * g0,
      static_cast<uint8_t>(state), sos, time_s, dt);

  float mach = fabsf(vel) / sos;
  ServoAngles angles = CdLookup::cdToServoAngles(cd_add, mach);

  I2CControl.cmd_servo_1 = angles.angle_1;
  I2CControl.cmd_servo_2 = angles.angle_2;
  I2CControl.predicted_apogee = airbrakeController.predictedApogee();
  I2CControl.cd_add_cmd = cd_add;
  I2CControl.target_cd_raw = airbrakeController.lastTargetCdRaw();
  I2CControl.apo_no_brakes = airbrakeController.lastApoNoBrakes();
  I2CControl.apo_max_brakes = airbrakeController.lastApoMaxBrakes();

  driveServos(angles.angle_1);
}

// ── Fallback Sweep (open-loop, FLIGHT mode only) ────────────────
// Last-resort sweep if I2C control Teensy is unavailable

static void runFallbackSweep() {
  if (millis() - FlightState.ignition_time < FALLBACK_IGNITION_DELAY_MS &&
      millis() - FlightState.motor_burnout_time < FALLBACK_BURNOUT_DELAY_MS) {
    return;
  }
  // BrakeState.pct and AIRBRAKE_MIN/MAX both live in airbrake-% [0..100].
  // Convert to servo-% only at the driveServos boundary.
  if (millis() - BrakeState.last_update <
      (BrakeState.pct <= AIRBRAKE_MIN ? FALLBACK_SWEEP_PAUSE_MS : FALLBACK_SWEEP_INTERVAL_MS)) {
    return;
  }
  BrakeState.last_update = millis();
  float airbrake_pct = BrakeState.pct + FALLBACK_SWEEP_STEP * BrakeState.direction;
  if (airbrake_pct >= AIRBRAKE_MAX) {
    airbrake_pct = AIRBRAKE_MAX;
    BrakeState.direction = -2;
  } else if (airbrake_pct <= AIRBRAKE_MIN) {
    airbrake_pct = AIRBRAKE_MIN;
    BrakeState.direction = 1;
  }
  driveServos(RocketConfig::airbrakePctToServoPct(airbrake_pct));
}

// ── Mode-Specific Loop Functions ─────────────────────────────────

static bool isOnPad() {
  return state == States::IDLE || state == States::AIRBRAKE_TEST || state == States::SENSOR_ERROR;
}

static void loopFlight() {
  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  float altitude = isOnPad() ? 0.0f : filterAltitude(raw_alt);

  checkSerialCommands();
  updateStateMachine(altitude);

  if (state == States::ASCENT) {
    if (USE_CONTROL && !I2CControl.fallback) {
      sendControlPacket(altitude);
      driveServos(I2CControl.cmd_servo_1);
    } else {
      runFallbackSweep();
    }
  }

  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state);
}

static void loopSHITL() {
  statusIndicator.rainbow();

  float raw_alt = readSimulatedSensors();
  if (isnan(raw_alt)) return;

  float altitude;
  if (SimTruth.valid) {
    // Ground-truth path: controller runs on the same inputs as the
    // pure-Python demo, so SHITL_DEMO and SHITL produce identical commands
    // to the non-Teensy demo. SHITL still drives servos; SHITL_DEMO parks.
    state = phaseCodeToState(SimTruth.phase_code);
    altitude = SimTruth.alt_m;
    FlightState.velocity = SimTruth.vel_ms;
    FlightState.max_altitude = fmaxf(FlightState.max_altitude, altitude);
  } else {
    // Legacy 6-field protocol: UKF + state machine derive alt/vel/state
    // from sensor data.
    altitude = isOnPad() ? 0.0f : filterAltitude(raw_alt);
    updateStateMachine(altitude);
  }

  // Call the controller every tick — it internally retracts in PREFLIGHT/BURN.
  // Matches how the Python demo invokes AirbrakeController.update every step.
  runLocalController(altitude);

  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state);
}

static void loopTest() {
  // Explicit LED every tick so mode switches out of SHITL (rainbow PWM) don't
  // leave the LED frozen on the last PWM frame.
  statusIndicator.solid(StatusIndicator::BLUE);

  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  // Filter unconditionally — TEST mode exists to expose UKF behavior for tuning.
  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state);
}

// SENSOR_MOVING: real sensors + UKF, no state machine, no servo motion.
// Intended for picking up / moving / shaking the rocket on the ground to
// watch the state estimate (altitude, velocity, acceleration) and capture
// a file to SD in the same CSV format as FLIGHT. Servos stay parked at
// SERVO_MIN_PCT (set during setup()); this loop never calls driveServos.
static void loopSensorMoving() {
  // Solid ORANGE distinguishes SENSOR_MOVING from TEST (BLUE) and IDLE (GREEN).
  // Driven every tick so switching out of SHITL (rainbow PWM) updates cleanly.
  statusIndicator.solid(StatusIndicator::ORANGE);

  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state);
}

// ── Setup ────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(PinDefs.ARM, INPUT_PULLUP);
  pinMode(PinDefs.IGNITER_0, OUTPUT);
  pinMode(PinDefs.IGNITER_1, OUTPUT);
  pinMode(PinDefs.BUZZER, OUTPUT);
  digitalWrite(PinDefs.BUZZER, LOW);  // silent until flight-mode confirm beep

  // Wait for USB serial with timeout -- flight continues without USB host
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) {
    statusIndicator.solid(StatusIndicator::RED);
  }
  statusIndicator.solid(StatusIndicator::RED);

  // Negotiate operating mode with dashboard (if connected)
  if (Serial) {
    currentMode = negotiateMode();
  }

  Serial.print(F("Mode: "));
  Serial.println(modeToString(currentMode));

  // I2C bus
  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();
  Wire.setClock(100000);

  delay(4000);  // allow sensors to power up

  // SD card — retry for ~10 s, then continue so a missing card doesn't
  // strand the Teensy on the pad with a RED LED. Flight still works; only
  // the SD log is lost.
  bool sd_ok = false;
  for (int i = 0; i < 10 && !sd_ok; i++) {
    sd_ok = logging.begin();
    if (!sd_ok) {
      statusIndicator.solid(StatusIndicator::RED);
      Serial.println(F("Waiting for SD card..."));
      delay(1000);
    }
  }
  if (sd_ok) {
    Serial.println(F("SD card initialized"));
  } else {
    Serial.println(F("SD card init failed -- continuing without log"));
  }

  // Initialize real sensors for FLIGHT and TEST modes
  if (!isSimMode(currentMode)) {
    initSensor(adxl345, "ADXL345");
    initSensor(adxl375, "ADXL375");
    initSensor(lps22, "LPS22");
#if USE_BNO080
    initSensor(bno080, "BNO080");
#endif
    calibrateSensors(lps22, &adxl345);
  }

  // SHITL-specific setup (both SHITL and SHITL_DEMO)
  if (isSimMode(currentMode)) {
    Serial.setTimeout(100);  // fast timeout for SHITL data reads
    BrakeState.hasCheckedForHorizontal = true;  // skip horizontal detect
  }

  // Enable serial telemetry when USB host is connected (for dashboard monitoring)
  if (Serial) {
    logging.setDebug(true);
  }

  // Log header
  logging.log(
      "Time,Xg,Yg,Zg,Xg_high,Yg_high,Zg_high,Pressure,Temperature,Altitude,"
      "BNO_X,BNO_Y,BNO_Z,BNO_I,BNO_J,BNO_K,BNO_Real,State,"
      "Airbrake_pct,Airbrake_dir,Predicted_Apogee,Cd_Add_Cmd,"
      "I2C_Fallback,I2C_FailCount,Potentiometer,Velocity,"
      "Target_Cd_Raw,Apo_No_Brakes,Apo_Max_Brakes");
  logging.flush();

  // Servo init LAST -- SD.begin() internally calls SPI.begin() which claims pin 10
  airbrake_servo_1.begin(PinDefs.SERVO);
  airbrake_servo_2.begin(PinDefs.SERVO_2);
  // Park at mechanical minimum instead of the Bilda default (0%).
  airbrake_servo_1.setExtension(RocketConfig::SERVO_MIN_PCT);
  airbrake_servo_2.setExtension(RocketConfig::SERVO_MIN_PCT);
  BrakeState.pct = 0.0f;  // 0% airbrake (servos parked at SERVO_MIN_PCT)

  // Set initial state
  if (failed_sensors > 0 && !isSimMode(currentMode)) {
    statusIndicator.solid(StatusIndicator::WHITE);
    Serial.println(F("Setup failed -- sensor error"));
    state = States::SENSOR_ERROR;
  } else {
    statusIndicator.solid(StatusIndicator::GREEN);
    Serial.println(F("Setup complete"));
    state = States::IDLE;
  }

  // FLIGHT mode confirmation: 5 on/off buzzer cycles over 5 s. Skip on
  // SENSOR_ERROR so the WHITE LED + silence unambiguously flags the fault.
  if (currentMode == TeensyMode::FLIGHT && state != States::SENSOR_ERROR) {
    Serial.println(F("FLIGHT mode -- buzzer confirm"));
    flightModeBuzzerConfirm();
  }
}

// ── Main Loop ────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();
  if (now - last_loop_time < LOOP_INTERVAL_MS) return;
  last_loop_time = now;

  // USB-unplug watcher (FLIGHT mode only). When the laptop cable is pulled
  // before launch, announce "on internal power" with 8 short beeps so the
  // operator has an audible confirmation that the Teensy is still running on
  // battery and the state handlers are still refreshing the LED. Only fires
  // from ground states so we never burn ~4 s of the main loop mid-flight if
  // the connector happens to rip loose under motor g-load.
  static bool usbWasConnected = false;
  bool usbNow = (bool)Serial;
  if (currentMode == TeensyMode::FLIGHT && usbWasConnected && !usbNow &&
      (state == States::IDLE || state == States::AIRBRAKE_TEST ||
       state == States::BOOT || state == States::LANDED)) {
    internalPowerBuzzerConfirm();
  }
  usbWasConnected = usbNow;

  switch (currentMode) {
    case TeensyMode::FLIGHT:        loopFlight();       break;
    case TeensyMode::SHITL:
    case TeensyMode::SHITL_DEMO:    loopSHITL();        break;
    case TeensyMode::TEST:          loopTest();         break;
    case TeensyMode::SENSOR_MOVING: loopSensorMoving(); break;
  }
}
