#define DEBUG_MODE 1  // Set to 0 for flight, 1 for bench testing

#include <Arduino.h>
#include <CommLink.h>
#include <Globals.h>
#include <Logging.h>
#include <PhysicsConstants.h>
#include <PinDefinitions.h>
#include <SD.h>
#include <SPI.h>
#include <StateMachine.h>
#include <WifiPins.h>
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
// FLIGHT:        Real sensors, full state machine, actual launch.
// SHITL:         Simulated sensors over serial, local controller, servos active.
// SHITL_DEMO:    SHITL with physical servos suppressed (bench demo).
// TEST:          Real sensors, manual servo testing, no state machine.
// SENSOR_MOVING: Real sensors + UKF + SD log, no state machine, no servo motion.
enum class TeensyMode { FLIGHT, SHITL, SHITL_DEMO, TEST, SENSOR_MOVING };
static TeensyMode currentMode = TeensyMode::FLIGHT;

// Sticky across link drops; cleared only by DISARM, mode switch, or reboot.
static bool armed = false;

static inline bool isSimMode(TeensyMode m) {
  return m == TeensyMode::SHITL || m == TeensyMode::SHITL_DEMO;
}

// Keep this board from driving pyro outputs in sim or no-pyro configurations.
bool shouldInhibitPyros() { return !PYROS_INSTALLED || isSimMode(currentMode); }

bool isArmed() { return armed; }

// Drive both servos and mirror the airbrake percent into telemetry state.
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

// Audible status codes used before the laptop/dashboard may be connected.
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

static void wifiBootBuzzerConfirm() {
  const int BEEP_FREQ_HZ = 2500;
  const int BEEP_ON_MS = 80;
  const int BEEP_OFF_MS = 80;
  const int BEEP_CYCLES = 3;
  for (int i = 0; i < BEEP_CYCLES; i++) {
    tone(PinDefs.BUZZER, BEEP_FREQ_HZ);
    delay(BEEP_ON_MS);
    noTone(PinDefs.BUZZER);
    digitalWrite(PinDefs.BUZZER, LOW);
    delay(BEEP_OFF_MS);
  }
}

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

// Pad reference used for AGL altitude and accel bias removal.
static struct {
  float pressure = 0;
  float temperature = 0;
  float accel_z_bias = 0.0f;
  float accel_z_high_g_bias = 0.0f;
} RefCalibration;

// ── Sensor Helpers ───────────────────────────────────────────────

// Capture the pad baseline. At least one valid pressure sample is required.
static bool calibrateSensors(Lps22 &lps, Adxl *lg_accel = nullptr,
                             Adxl *hg_accel = nullptr) {
  delay(100);
  const int N = 40;  // 40 samples at 25 ms = 1 s
  int valid = 0;
  float p_sum = 0.0f, t_sum = 0.0f;
  float a_low_sum = 0.0f, a_high_sum = 0.0f;
  for (int i = 0; i < N; i++) {
    float p = 0.0f, t = 0.0f;
    lps.readPressure(&p);
    lps.readTemperature(&t);
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float ax_hg = 0.0f, ay_hg = 0.0f, az_hg = 0.0f;
    if (lg_accel) lg_accel->readAccelerometer(&ax, &ay, &az);
    if (hg_accel) hg_accel->readAccelerometer(&ax_hg, &ay_hg, &az_hg);
    if (p > 0.0f) {
      p_sum += p;
      t_sum += t;
      a_low_sum += az;
      a_high_sum += az_hg;
      valid++;
    }
    delay(25);
  }
  if (valid <= 0) return false;
  RefCalibration.pressure = p_sum / (float)valid;
  RefCalibration.temperature = t_sum / (float)valid + CELSIUS_TO_KELVIN;
  if (lg_accel) RefCalibration.accel_z_bias = a_low_sum / (float)valid;
  if (hg_accel) RefCalibration.accel_z_high_g_bias = a_high_sum / (float)valid;
  return true;
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

// AGL altitude from pressure, referenced to the pad pressure sample.
static float altitudeDelta(float p, float /*T*/) {
  if (p <= 0.0f) return NAN;
  float p_ref = RefCalibration.pressure;     // mbar at launch site
  float T_ref = RefCalibration.temperature;  // K at launch site
  const float L = 0.0065f;
  const float exponent = L * Rd / g0;
  return (T_ref / L) * (1.0f - powf(p / p_ref, exponent));
}

// ── Altitude Filter ─────────────────────────────────────────────

// Accel-integrated velocity used while the motor thrust ramp dominates.
static float kinematic_vel = 0.0f;

// Fuse accel + baro into altitude and velocity. Baro is ignored only when the
// sample is invalid or the vehicle is above the configured Mach lockout speed.
static float filterAltitude(float raw_alt) {
  static States last_state = States::BOOT;
  static float last_filtered_alt = 0.0f;
  static bool have_filtered_alt = false;
  float dt = LOOP_INTERVAL_MS / 1000.0f;
  bool raw_alt_valid = !isnan(raw_alt);

  // Remove pad bias and convert g to m/s^2.
  float a_low  = (sensors.accel_z        - RefCalibration.accel_z_bias)        * g0;
  float a_high = (sensors.accel_z_high_g - RefCalibration.accel_z_high_g_bias) * g0;

  // First call after construction, mode switch, or ZERO.
  if (!altitudeFilter.isInitialized()) {
    float init_alt = raw_alt_valid ? raw_alt : 0.0f;
    altitudeFilter.init(init_alt, 0.0f, a_low);
    FlightState.velocity = 0.0f;
    kinematic_vel = 0.0f;
    last_state = state;
    last_filtered_alt = init_alt;
    have_filtered_alt = true;
    return init_alt;
  }

  // Blend low-g and high-g accelerometers using phase-dependent noise weights.
  SensorWeights w = SensorWeighting::getWeights(
      static_cast<uint8_t>(state), kinematic_vel, 343.0f);
  float inv_R_low  = 1.0f / w.R_accel_low;
  float inv_R_high = 1.0f / w.R_accel_high;
  float inv_R_sum  = inv_R_low + inv_R_high;
  float accel_blended = (a_low * inv_R_low + a_high * inv_R_high) / inv_R_sum;
  float R_blended     = 1.0f / inv_R_sum;

  kinematic_vel += accel_blended * dt;

  // Above Mach lockout, pressure can be shock-corrupted. Keep predicting from
  // accel and resume baro updates after the vehicle slows down.
  bool high_speed_flight = state == States::IGNITION || state == States::ASCENT;
  bool baro_locked_out = high_speed_flight && (fabsf(kinematic_vel) > MACH_LOCKOUT_VELOCITY);
  bool use_baro = raw_alt_valid && !baro_locked_out;

  // Reseed on state changes so coast starts with the burn-integrated velocity.
  if (state != last_state) {
    float seed_alt =
        use_baro ? raw_alt : (have_filtered_alt ? last_filtered_alt : altitudeFilter.altitude());
    altitudeFilter.init(seed_alt, kinematic_vel, a_low);
    FlightState.velocity = kinematic_vel;
    last_state = state;
    last_filtered_alt = seed_alt;
    have_filtered_alt = true;
    return seed_alt;
  }

  altitudeFilter.predict(dt);
  altitudeFilter.updateAccel(accel_blended, R_blended);
  if (use_baro) {
    altitudeFilter.updateBaro(raw_alt, w.R_baro);
  }

  // During coast, baro-corrected UKF velocity is better than pure integration.
  if (state == States::ASCENT) {
    FlightState.velocity = altitudeFilter.velocity();
  } else {
    FlightState.velocity = kinematic_vel;
  }

  last_filtered_alt = altitudeFilter.altitude();
  have_filtered_alt = true;
  return last_filtered_alt;
}

// ── Mode Negotiation ─────────────────────────────────────────────
// Boot negotiation supports both the current MODE protocol and old CSV SHITL.

// Exact matches avoid MODE,SHITL matching MODE,SHITL_DEMO.
static bool parseMode(const char *line, TeensyMode *out) {
  if (strcmp(line, "MODE,FLIGHT")        == 0) { *out = TeensyMode::FLIGHT;        return true; }
  if (strcmp(line, "MODE,SHITL")         == 0) { *out = TeensyMode::SHITL;         return true; }
  if (strcmp(line, "MODE,SHITL_DEMO")    == 0) { *out = TeensyMode::SHITL_DEMO;    return true; }
  if (strcmp(line, "MODE,TEST")          == 0) { *out = TeensyMode::TEST;          return true; }
  if (strcmp(line, "MODE,SENSOR_MOVING") == 0) { *out = TeensyMode::SENSOR_MOVING; return true; }
  return false;
}

static void ackMode(TeensyMode m) {
  comm.print("MODE_ACK,");
  comm.println(modeToString(m));
}

static TeensyMode negotiateMode() {
  while (comm.available()) comm.read();

  comm.println("READY");
  comm.setTimeout(2000);

  char buf[128];
  int len = comm.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  if (len > 0) {
    TeensyMode m;
    if (parseMode(buf, &m)) { ackMode(m); return m; }

    int fieldCount = 1;
    for (int i = 0; i < len; i++) if (buf[i] == ',') fieldCount++;
    if (fieldCount >= 6) return TeensyMode::SHITL;
  }

  while (comm.available()) comm.read();
  comm.println("DATAREQUEST");
  comm.setTimeout(2000);

  len = comm.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  if (len > 0) {
    int fieldCount = 1;
    for (int i = 0; i < len; i++) if (buf[i] == ',') fieldCount++;
    if (fieldCount >= 6) return TeensyMode::SHITL;
  }

  return TeensyMode::FLIGHT;
}

// ── SHITL calibration state ──────────────────────────────────────
// SHITL calibration accepts only at-rest frames so launch data cannot poison bias.
static bool simCalibrated = false;
static int simCalibrationCount = 0;        // good (at-rest) samples accumulated
static int simCalibrationTotalCount = 0;   // total samples seen
static const int SIM_CAL_SAMPLES = 20;     // good samples needed (~1 s)
static const int SIM_CAL_TIMEOUT = 200;    // total samples before forcing (~10 s)
static const float CAL_REST_TOLERANCE_G = 0.3f;

// Optional 9-field SHITL frames carry ground-truth alt/vel/phase.
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

// Reset flight state without dropping WiFi or closing the current log file.
static void restartStateMachine() {
  FlightState = FlightState_t();
  BrakeState = BrakeState_t();
  I2CControl = I2CControl_t();
  airbrakeController.reset();
  altitudeFilter = UKF1D();
  kinematic_vel = 0.0f;
  resetStateMachineCounters();
  armed = false;

  airbrake_servo_1.setExtension(RocketConfig::SERVO_MIN_PCT);
  airbrake_servo_2.setExtension(RocketConfig::SERVO_MIN_PCT);
  BrakeState.pct = 0.0f;

  // Stop any local pyro output before re-entering idle/test code.
  if (!shouldInhibitPyros()) primaryIgniter.stop();

  char marker[48];
  snprintf(marker, sizeof(marker), "# SOFT_RESET at millis=%lu", millis());
  logging.log(marker);
  logging.flush();

  // Real-sensor modes need a fresh pad baseline after reset.
  RefCalibration.pressure = 0.0f;
  RefCalibration.temperature = 0.0f;
  RefCalibration.accel_z_bias = 0.0f;
  RefCalibration.accel_z_high_g_bias = 0.0f;

  if (isSimMode(currentMode)) {
    simCalibrated = false;
    simCalibrationCount = 0;
    simCalibrationTotalCount = 0;
    BrakeState.hasCheckedForHorizontal = true;
    state = States::IDLE;
  } else {
    bool cal_ok = calibrateSensors(lps22, &adxl345, &adxl375);
    BrakeState.hasCheckedForHorizontal = false;
    if (!cal_ok) {
      failed_sensors++;
      state = States::SENSOR_ERROR;
    } else {
      state = (failed_sensors > 0) ? States::SENSOR_ERROR : States::IDLE;
    }
  }
}

// ── Serial Command Handling ──────────────────────────────────────

// Enter a SHITL mode with clean filter/controller state.
static void enterSHITL(TeensyMode target) {
  currentMode = target;
  comm.setTimeout(25);  // SHITL frames arrive within a few ms
  BrakeState.hasCheckedForHorizontal = true;
  simCalibrated = false;
  simCalibrationCount = 0;
  simCalibrationTotalCount = 0;
  RefCalibration.pressure = 0;
  RefCalibration.temperature = 0;
  RefCalibration.accel_z_bias = 0.0f;
  RefCalibration.accel_z_high_g_bias = 0.0f;
  altitudeFilter = UKF1D();
  FlightState.velocity = 0.0f;
  // Keep FlightState's fallback state in sync with airbrakeController.reset()
  // below — otherwise a SHITL session entered after a flight that latched a
  // lower tier would have decision_made=true with the controller already
  // reset to primary, and the gate would never re-fire because the decision
  // is already recorded.
  FlightState.fallback_decision_made = false;
  FlightState.fallback_tier = 1;
  FlightState.fallback_apo_at_decision_m = 0.0f;
  SimTruth.valid = false;
  airbrakeController.reset();
  state = States::IDLE;
  armed = false;
  logging.setDebug(true);
  if (target == TeensyMode::SHITL_DEMO) {
    airbrake_servo_1.setExtension(RocketConfig::SERVO_MIN_PCT);
    airbrake_servo_2.setExtension(RocketConfig::SERVO_MIN_PCT);
    BrakeState.pct = 0.0f;
  }
  ackMode(target);
  comm.print(F("Mode: "));
  comm.println(modeToString(currentMode));
}

// Dashboard command dispatcher.
// Commands:
//   MODE,<name>     mode switch (also clears sticky arm)
//   SERVO,<pct>     manual servo command (clamped)
//   ARM / DISARM    sticky arm flag (gates apogee pyro)
//   BEEP,ON|OFF     hold-to-beep buzzer
//   ZERO            re-sample at-rest baseline (rocket must be stationary)
//   RESET           hardware software-reset (drops WiFi for ~15 s)
//   SOFT_RESET      restart state machine without rebooting
//   GET,MODE        echo the active mode
//   CHECK,SENSORS|CALIB|SD     pre-launch checklist queries
//   LIST,SD / READ,SD,<name>   browse/download log files over the link
static void handleSerialCommand(const char *line) {
  TeensyMode m;
  if (parseMode(line, &m)) {
    if (isSimMode(m)) {
      enterSHITL(m);
    } else {
      // Never spend 5 s beeping in flight.
      bool entering_flight =
          (m == TeensyMode::FLIGHT) && (currentMode != TeensyMode::FLIGHT);
      currentMode = m;
      armed = false;
      ackMode(m);
      if (entering_flight && state != States::SENSOR_ERROR &&
          (state == States::IDLE || state == States::AIRBRAKE_TEST ||
           state == States::BOOT || state == States::LANDED)) {
        comm.println(F("FLIGHT mode -- buzzer confirm"));
        flightModeBuzzerConfirm();
      }
    }
    return;
  }
  if (strncmp(line, "SERVO,", 6) == 0) {
    float pct = strtof(line + 6, nullptr);
    pct = constrain(pct, RocketConfig::SERVO_MIN_PCT, RocketConfig::SERVO_MAX_PCT);
    driveServos(pct);
  } else if (strcmp(line, "ARM") == 0) {
    armed = true;
    comm.println(F("ARM_ACK"));
  } else if (strcmp(line, "DISARM") == 0) {
    armed = false;
    comm.println(F("DISARM_ACK"));
  } else if (strcmp(line, "BEEP,ON") == 0) {
    // Hold-to-beep from the dashboard.
    tone(PinDefs.BUZZER, 2000);
  } else if (strcmp(line, "BEEP,OFF") == 0) {
    noTone(PinDefs.BUZZER);
    digitalWrite(PinDefs.BUZZER, LOW);
  } else if (strcmp(line, "ZERO") == 0) {
    // Operator must keep the rocket still for this sample window.
    comm.println(F("Zeroing sensors (~1s sample)..."));
    RefCalibration.pressure = 0;
    RefCalibration.temperature = 0;
    bool ok = calibrateSensors(lps22, &adxl345, &adxl375);
    altitudeFilter = UKF1D();
    FlightState.velocity = 0.0f;
    if (!ok) {
      comm.println(F("ZERO failed -- no valid baro samples"));
    } else {
      comm.print(F("Zeroed: p_ref="));
      comm.print(RefCalibration.pressure, 2);
      comm.print(F(" mbar, az_bias="));
      comm.print(RefCalibration.accel_z_bias, 4);
      comm.print(F(" g, az_hg_bias="));
      comm.print(RefCalibration.accel_z_high_g_bias, 4);
      comm.println(F(" g"));
    }
  } else if (strcmp(line, "RESET") == 0) {
    comm.println(F("RESET_ACK"));
    comm.flush();
    delay(10);
    SCB_AIRCR = 0x05FA0004;  // Cortex-M software reset
  } else if (strcmp(line, "GET,MODE") == 0) {
    comm.print(F("Mode: "));
    comm.println(modeToString(currentMode));
  } else if (strcmp(line, "SOFT_RESET") == 0) {
    comm.println(F("SOFT_RESET_BEGIN"));
    restartStateMachine();
    comm.print(F("SOFT_RESET_ACK,state="));
    comm.print(stateToString(state));
    comm.print(F(",calibrated="));
    comm.print(RefCalibration.pressure > 0.0f ? 1 : 0);
    comm.print(F(",failed="));
    comm.print(failed_sensors);
    comm.println();
  } else if (strcmp(line, "CHECK,SENSORS") == 0) {
    // Mean + range make stuck sensors obvious during preflight checks.
    constexpr int N = 10;
    float p_min=1e9f,  p_max=-1e9f,  p_sum=0;
    float T_min=1e9f,  T_max=-1e9f,  T_sum=0;
    float ax_min=1e9f, ax_max=-1e9f, ax_sum=0;
    float ay_min=1e9f, ay_max=-1e9f, ay_sum=0;
    float az_min=1e9f, az_max=-1e9f, az_sum=0;
    float axh_min=1e9f, axh_max=-1e9f, axh_sum=0;
    float ayh_min=1e9f, ayh_max=-1e9f, ayh_sum=0;
    float azh_min=1e9f, azh_max=-1e9f, azh_sum=0;
    int samples = 0;

    auto acc = [](float v, float &mn, float &mx, float &sm) {
      if (v < mn) mn = v;
      if (v > mx) mx = v;
      sm += v;
    };

    if (isSimMode(currentMode)) {
      samples = 1;
      p_min = p_max = p_sum = sensors.pressure;
      T_min = T_max = T_sum = sensors.temperature;
      ax_min = ax_max = ax_sum = sensors.accel_x;
      ay_min = ay_max = ay_sum = sensors.accel_y;
      az_min = az_max = az_sum = sensors.accel_z;
      axh_min = axh_max = axh_sum = sensors.accel_x_high_g;
      ayh_min = ayh_max = ayh_sum = sensors.accel_y_high_g;
      azh_min = azh_max = azh_sum = sensors.accel_z_high_g;
    } else {
      for (int i = 0; i < N; i++) {
        float p=0, T=0, ax=0, ay=0, az=0, axh=0, ayh=0, azh=0;
        lps22.readPressure(&p);
        lps22.readTemperature(&T);
        adxl345.readAccelerometer(&ax, &ay, &az);
        adxl375.readAccelerometer(&axh, &ayh, &azh);
        acc(p,   p_min,   p_max,   p_sum);
        acc(T,   T_min,   T_max,   T_sum);
        acc(ax,  ax_min,  ax_max,  ax_sum);
        acc(ay,  ay_min,  ay_max,  ay_sum);
        acc(az,  az_min,  az_max,  az_sum);
        acc(axh, axh_min, axh_max, axh_sum);
        acc(ayh, ayh_min, ayh_max, ayh_sum);
        acc(azh, azh_min, azh_max, azh_sum);
        samples++;
        delay(30);
      }
    }

    float inv = samples > 0 ? 1.0f / (float)samples : 0.0f;
    auto emit_ch = [&](const __FlashStringHelper* k_mean,
                       const __FlashStringHelper* k_range,
                       float sum, float mn, float mx, int dp) {
      comm.print(k_mean); comm.print(sum * inv, dp);
      comm.print(k_range); comm.print(mx - mn, dp + 1);
    };
    comm.print(F("CHECK_RESULT,SENSORS,n=")); comm.print(samples);
    emit_ch(F(",p_mean="),    F(",p_range="),    p_sum,   p_min,   p_max,   2);
    emit_ch(F(",T_mean="),    F(",T_range="),    T_sum,   T_min,   T_max,   2);
    emit_ch(F(",ax_mean="),   F(",ax_range="),   ax_sum,  ax_min,  ax_max,  3);
    emit_ch(F(",ay_mean="),   F(",ay_range="),   ay_sum,  ay_min,  ay_max,  3);
    emit_ch(F(",az_mean="),   F(",az_range="),   az_sum,  az_min,  az_max,  3);
    emit_ch(F(",axh_mean="),  F(",axh_range="),  axh_sum, axh_min, axh_max, 3);
    emit_ch(F(",ayh_mean="),  F(",ayh_range="),  ayh_sum, ayh_min, ayh_max, 3);
    emit_ch(F(",azh_mean="),  F(",azh_range="),  azh_sum, azh_min, azh_max, 3);
    comm.print(F(",failed=")); comm.print(failed_sensors);
    comm.println();
  } else if (strcmp(line, "CHECK,CALIB") == 0) {
    bool calibrated = (RefCalibration.pressure > 0.0f);
    comm.print(F("CHECK_RESULT,CALIB,calibrated="));
    comm.print(calibrated ? 1 : 0);
    comm.print(F(",p_ref="));
    comm.print(RefCalibration.pressure, 2);
    comm.print(F(",T_ref="));
    comm.print(RefCalibration.temperature, 2);
    comm.print(F(",az_bias="));
    comm.print(RefCalibration.accel_z_bias, 4);
    comm.print(F(",az_hg_bias="));
    comm.print(RefCalibration.accel_z_high_g_bias, 4);
    comm.println();
  } else if (strcmp(line, "CHECK,SD") == 0) {
    bool ok = logging.selfTest();
    comm.print(F("CHECK_RESULT,SD,ok="));
    comm.print(ok ? 1 : 0);
    comm.print(F(",file="));
    comm.println(logging.fileName());
  } else if (strcmp(line, "LIST,SD") == 0) {
    comm.println(F("SD_LIST_BEGIN"));
    File root = SD.open("/");
    int count = 0;
    if (root) {
      while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
          const char* name = entry.name();
          if (name[0] != '.') {
            comm.print(name);
            comm.print(",");
            comm.print((unsigned long)entry.size());
            comm.println();
            count++;
          }
        }
        entry.close();
      }
      root.close();
    }
    comm.print(F("SD_LIST_END,"));
    comm.print(count);
    comm.println();
  } else if (strncmp(line, "READ,SD,", 8) == 0) {
    // Flush first so downloads include the latest telemetry rows.
    const char* fname = line + 8;
    logging.flush();
    File f = SD.open(fname, FILE_READ);
    if (!f) {
      comm.print(F("SD_FILE_ERR,not_found,"));
      comm.println(fname);
    } else {
      uint32_t size = f.size();
      comm.print(F("SD_FILE_BEGIN,"));
      comm.print(fname);
      comm.print(",");
      comm.print(size);
      comm.println();

      char outbuf[300];
      static const char prefix[] = "SD_LINE,";
      constexpr size_t prefix_len = sizeof(prefix) - 1;  // 8
      memcpy(outbuf, prefix, prefix_len);
      while (f.available() && comm.connected()) {
        size_t n = f.readBytesUntil('\n', outbuf + prefix_len,
                                    sizeof(outbuf) - prefix_len - 1);
        while (n > 0 && (outbuf[prefix_len + n - 1] == '\r' ||
                         outbuf[prefix_len + n - 1] == '\n')) n--;
        outbuf[prefix_len + n] = '\0';
        comm.println(outbuf);
        comm.poll();
      }
      f.close();
      comm.print(F("SD_FILE_END,"));
      comm.println(fname);
    }
  }
}

// Non-blocking command reader for non-SHITL telemetry modes.
static char _cmdBuf[64];
static int _cmdPos = 0;

static void checkSerialCommands() {
  while (comm.available()) {
    int cr = comm.read();
    if (cr < 0) break;
    char c = (char)cr;
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

// Read hardware sensors and return pressure-derived AGL altitude.
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

// SHITL interleaves dashboard commands with CSV sensor frames.
static bool isDashboardCommand(const char *line) {
  return strncmp(line, "SERVO,", 6) == 0 || strcmp(line, "ZERO") == 0 ||
         strcmp(line, "RESET") == 0 || strcmp(line, "SOFT_RESET") == 0 ||
         strncmp(line, "MODE,", 5) == 0 || strcmp(line, "GET,MODE") == 0 ||
         strncmp(line, "BEEP,", 5) == 0 || strcmp(line, "ARM") == 0 ||
         strcmp(line, "DISARM") == 0 || strncmp(line, "CHECK,", 6) == 0 ||
         strncmp(line, "LIST,", 5) == 0 || strncmp(line, "READ,", 5) == 0;
}

// Pull one CSV frame from SHITL. 9-field frames include ground truth; 6-field
// frames use the firmware estimator and state machine.
static float readSimulatedSensors() {
  comm.println("DATAREQUEST");

  char buf[192];
  float values[9];
  int gotCount = 0;

  int commands_handled = 0;
  for (int attempts = 0; attempts < 3; attempts++) {
    int len = comm.readBytesUntil('\n', buf, sizeof(buf) - 1);
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

  // Extended SHITL frame: 9 fields carry the high-g (ADXL375) channel
  // separately so the simulator can model the ADXL345 saturating at ±16 g
  // independently of the high-g sensor's wider range. 6-field frames fall
  // back to copying low-g into high-g (matches what shitl.py / older
  // simulators emitted).
  if (gotCount >= 9) {
    sensors.accel_x_high_g = values[6];
    sensors.accel_y_high_g = values[7];
    sensors.accel_z_high_g = values[8];
  } else {
    sensors.accel_x_high_g = sensors.accel_x;
    sensors.accel_y_high_g = sensors.accel_y;
    sensors.accel_z_high_g = sensors.accel_z;
  }

  SimTruth.valid = false;
  if (!simCalibrated) {
    simCalibrationTotalCount++;
    bool at_rest = fabsf(sensors.accel_z - 1.0f) < CAL_REST_TOLERANCE_G &&
                   fabsf(sensors.accel_x) < CAL_REST_TOLERANCE_G &&
                   fabsf(sensors.accel_y) < CAL_REST_TOLERANCE_G;
    if (at_rest) {
      RefCalibration.pressure += sensors.pressure;
      RefCalibration.temperature += sensors.temperature + CELSIUS_TO_KELVIN;
      RefCalibration.accel_z_bias += sensors.accel_z;
      RefCalibration.accel_z_high_g_bias += sensors.accel_z_high_g;
      simCalibrationCount++;
    }
    bool got_enough = simCalibrationCount >= SIM_CAL_SAMPLES;
    bool timed_out = simCalibrationTotalCount >= SIM_CAL_TIMEOUT;
    if (got_enough || (timed_out && simCalibrationCount > 0)) {
      RefCalibration.pressure /= (float)simCalibrationCount;
      RefCalibration.temperature /= (float)simCalibrationCount;
      RefCalibration.accel_z_bias /= (float)simCalibrationCount;
      RefCalibration.accel_z_high_g_bias /= (float)simCalibrationCount;
      simCalibrated = true;
      comm.print(F("SHITL cal "));
      comm.print(timed_out ? F("FORCED (partial): ") : F("OK: "));
      comm.print(simCalibrationCount);
      comm.print(F("/"));
      comm.print(simCalibrationTotalCount);
      comm.print(F(" good, p_ref="));
      comm.print(RefCalibration.pressure, 2);
      comm.print(F(" mbar, accel_z_bias="));
      comm.print(RefCalibration.accel_z_bias, 4);
      comm.println(F(" g"));
    } else if (timed_out) {
      comm.println(F("SHITL cal FAILED: no at-rest samples after 10s — switch to TEST mode and retry"));
      simCalibrationTotalCount = 0;
    }
    return NAN;
  }

  return altitudeDelta(sensors.pressure, sensors.temperature + CELSIUS_TO_KELVIN);
}

// Track post-flight summary values for logs and late dashboard connects.
static void updateFlightMaxima(float altitude) {
  if (altitude > FlightState.max_altitude) FlightState.max_altitude = altitude;
  float v = fabsf(FlightState.velocity);
  if (v > FlightState.max_velocity) FlightState.max_velocity = v;
  float az = fmaxf(fabsf(sensors.accel_z), fabsf(sensors.accel_z_high_g));
  if (az > FlightState.max_accel_g) FlightState.max_accel_g = az;
}

// ── I2C Control Packet (FLIGHT mode only) ────────────────────────

// Optional I2C path for the older two-Teensy architecture.
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

// Local airbrake guidance path used by current FLIGHT and SHITL modes.
static void runLocalController(float altitude) {
  float vel = FlightState.velocity;
  float alt_msl = altitude + RocketConfig::LAUNCH_SITE_ALT_MSL_M;
  float sos = Atmosphere::speedOfSound(alt_msl);
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
  I2CControl.active_target_alt = airbrakeController.targetAltitude();

  driveServos(angles.angle_1);
}

// ── Fallback Target Evaluator ────────────────────────────────────
//
// Hedge against motor underperformance. The controller flies as normal
// (targeting primary 30k) until the rocket reaches FALLBACK_ARM_ALT_AGL_M
// (20k ft AGL), at which point a single 3-way tier check fires based on
// the live `apo_no_brakes from current state`:
//
//   tier 1 (GO):  apo_no_brakes ≥ primary − margin
//                 → keep primary target, controller continues to 30k
//   tier 2:       fallback − margin ≤ apo_no_brakes < primary − margin
//                 → lower target to FALLBACK_TARGET_ALT_AGL_M (28.5k)
//   tier 3:       apo_no_brakes < fallback − margin
//                 → lower target to FALLBACK_DEEP_TARGET_ALT_AGL_M (26k)
//                   so brakes still have headroom on a deeply-underperforming
//                   flight that wouldn't reach 28.5k naturally either
//
// Why a live check at 20k is correct (and an early snapshot is unnecessary):
// the controller's predictor only commands brakes when its forward sim with
// cd_add=0 (apo_no_brakes from current state) lands above the target — that
// is the explicit early-return in ApogeePredictor::predict(). Therefore on
// any controlled flight that is actively braking toward primary, the live
// apo_no_brakes is, by construction, at or above the target. A flight on
// track for 30k cannot read below the tier-1 threshold at this gate. The
// fallback tiers only engage on genuine underperformance, where apo_no_brakes
// was < primary from burnout onward and the controller never deployed brakes
// in the first place.
//
// Gates:
//   1. Already decided → return (one-shot, no re-evaluation)
//   2. Not in ASCENT → only checked during coast
//   3. Velocity ≤ 0 → past apogee, irrelevant
//   4. Altitude < 20k AGL → not at the check point yet
//   5. Controller not in CTRL_ACTIVE → lastApoNoBrakes() is stale (set only
//      by the predict-and-search branch); wait for next tick
//   6. apo_no_brakes < altitude → impossible at CTRL_ACTIVE; defensive skip
//
// Margin (200 m / ~656 ft) absorbs predictor noise at each boundary so a
// borderline flight does not get bumped to a lower tier on noise. Decision
// is logged either way (one marker line per outcome).
static void evaluateFallbackTarget(float altitude) {
  if (FlightState.fallback_decision_made) return;
  if (state != States::ASCENT) return;
  if (FlightState.velocity <= 0.0f) return;
  if (altitude < RocketConfig::FALLBACK_ARM_ALT_AGL_M) return;
  if (airbrakeController.controllerState() != AirbrakeController::CTRL_ACTIVE) {
    // Controller still in CTRL_RETRACTED (e.g. supersonic lockout, post-
    // launch delay) or CTRL_FULL. lastApoNoBrakes is not fresh — try again
    // next tick. Decision stays pending.
    return;
  }

  float apo_no_brakes = airbrakeController.lastApoNoBrakes();
  // Defensive: at CTRL_ACTIVE the predictor's apo_no_brakes is always set
  // and should always satisfy apo >= altitude (we're below target, ascending,
  // subsonic). If the value is bogus, refuse to commit and let the next
  // tick re-evaluate.
  if (apo_no_brakes < altitude) return;

  // Decision time. Latch it so we never re-evaluate.
  FlightState.fallback_decision_made = true;
  FlightState.fallback_apo_at_decision_m = apo_no_brakes;

  const float threshold_primary =
      RocketConfig::TARGET_ALT_AGL_M - RocketConfig::FALLBACK_TRIGGER_MARGIN_M;
  const float threshold_fallback =
      RocketConfig::FALLBACK_TARGET_ALT_AGL_M - RocketConfig::FALLBACK_TRIGGER_MARGIN_M;

  // Pick a tier. tier=1 keeps the controller at primary; tier=2 and tier=3
  // call setTargetAltitude with the next-lower target so the controller's
  // predictor and over-target gate both honour the new ceiling.
  int tier;
  float new_target;
  if (apo_no_brakes >= threshold_primary) {
    tier = 1;
    new_target = RocketConfig::TARGET_ALT_AGL_M;
    // No setTargetAltitude call — controller already at primary by default.
  } else if (apo_no_brakes >= threshold_fallback) {
    tier = 2;
    new_target = RocketConfig::FALLBACK_TARGET_ALT_AGL_M;
    airbrakeController.setTargetAltitude(new_target);
  } else {
    tier = 3;
    new_target = RocketConfig::FALLBACK_DEEP_TARGET_ALT_AGL_M;
    airbrakeController.setTargetAltitude(new_target);
  }
  FlightState.fallback_tier = tier;

  char marker[200];
  snprintf(marker, sizeof(marker),
           "# FALLBACK_DECISION at millis=%lu alt=%.1fm "
           "apo_no_brakes=%.1fm tier=%d target=%.1fm "
           "thresholds={primary=%.1fm,fallback=%.1fm}",
           millis(), (double)altitude, (double)apo_no_brakes, tier,
           (double)new_target,
           (double)threshold_primary, (double)threshold_fallback);
  logging.log(marker);
  logging.flush();
}

// ── Fallback Sweep (open-loop, FLIGHT mode only) ────────────────

// Legacy open-loop fallback. Not used while USE_CONTROL is false.
static void runFallbackSweep() {
  if (BrakeState.fallback_sweep_count >= FALLBACK_SWEEP_COUNT) return;

  if (millis() - FlightState.ignition_time < FALLBACK_IGNITION_DELAY_MS &&
      millis() - FlightState.motor_burnout_time < FALLBACK_BURNOUT_DELAY_MS) {
    return;
  }
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
    BrakeState.fallback_sweep_count++;
  }
  driveServos(RocketConfig::airbrakePctToServoPct(airbrake_pct));
}

// ── Mode-Specific Loop Functions ─────────────────────────────────

// Pin displayed altitude to zero before liftoff.
static bool isOnPad() {
  return state == States::IDLE || state == States::AIRBRAKE_TEST || state == States::SENSOR_ERROR;
}

// Real flight path: sensors, estimator, state machine, then airbrake guidance.
static void loopFlight() {
  float raw_alt = readRealSensors();

  // Keep the estimator warm on the pad, but do not display baro noise as altitude.
  float filtered_alt = filterAltitude(raw_alt);
  float altitude = isOnPad() ? 0.0f : filtered_alt;

  checkSerialCommands();
  updateStateMachine(altitude);

  if (state == States::ASCENT) {
    if (USE_CONTROL && !I2CControl.fallback) {
      sendControlPacket(altitude);
      driveServos(I2CControl.cmd_servo_1);
    } else {
      runLocalController(altitude);
      // Run AFTER the controller so lastApoNoBrakes() reflects this tick's
      // prediction. Skipped automatically when USE_CONTROL is on (the I2C
      // path doesn't expose a no-brakes apogee; lower-altitude target is
      // the local-controller flight's contingency).
      evaluateFallbackTarget(altitude);
    }
  }

  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// SHITL loop: consume simulator frames and run the same controller path.
static void loopSHITL() {
  statusIndicator.rainbow();

  float raw_alt = readSimulatedSensors();
  if (isnan(raw_alt)) return;

  float altitude;
  if (SimTruth.valid) {
    state = phaseCodeToState(SimTruth.phase_code);
    altitude = SimTruth.alt_m;
    FlightState.velocity = SimTruth.vel_ms;
  } else {
    altitude = filterAltitude(raw_alt);
    updateStateMachine(altitude);
  }

  runLocalController(altitude);
  // Same fallback evaluator FLIGHT uses, so SHITL can reproduce / verify the
  // latch behaviour against simulated underperformance traces.
  evaluateFallbackTarget(altitude);

  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// TEST mode keeps real sensors and manual commands live without flight state.
static void loopTest() {
  statusIndicator.solid(StatusIndicator::BLUE);

  float raw_alt = readRealSensors();

  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// Real-sensor estimator test with no state transitions or servo motion.
static void loopSensorMoving() {
  statusIndicator.solid(StatusIndicator::ORANGE);

  float raw_alt = readRealSensors();

  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// ── Setup ────────────────────────────────────────────────────────
//
// Boot takes up to ~22 s with SD retries and the flight-mode confirm tone.

void setup() {
  comm.beginSerial(115200);

  pinMode(PinDefs.ARM, INPUT_PULLUP);
  pinMode(PinDefs.IGNITER_0, OUTPUT);
  pinMode(PinDefs.IGNITER_1, OUTPUT);
  pinMode(PinDefs.BUZZER, OUTPUT);
  digitalWrite(PinDefs.BUZZER, LOW);

  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) {
    statusIndicator.solid(StatusIndicator::RED);
  }
  statusIndicator.solid(StatusIndicator::RED);

  // Bring up WiFi before mode negotiation so a wireless dashboard can answer READY.
  if (comm.beginWiFi(WifiAP::SSID, WifiAP::PASS, WifiAP::PORT)) {
    wifiBootBuzzerConfirm();
  }

  if (comm.connected()) {
    currentMode = negotiateMode();
  }

  comm.print(F("Mode: "));
  comm.println(modeToString(currentMode));

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();
  Wire.setClock(100000);

  comm.pollDelay(4000);  // sensor power-up settle

  // Fly without SD if the card never comes up.
  bool sd_ok = false;
  for (int i = 0; i < 10 && !sd_ok; i++) {
    sd_ok = logging.begin();
    if (!sd_ok) {
      statusIndicator.solid(StatusIndicator::RED);
      Serial.println(F("Waiting for SD card..."));
      comm.pollDelay(1000);
    }
  }
  if (sd_ok) {
    Serial.println(F("SD card initialized"));
  } else {
    Serial.println(F("SD card init failed -- continuing without log"));
  }

  if (!isSimMode(currentMode)) {
    initSensor(adxl345, "ADXL345");
    initSensor(adxl375, "ADXL375");
    initSensor(lps22, "LPS22");
#if USE_BNO080
    initSensor(bno080, "BNO080");
#endif
    // Do not fly real-sensor modes without a valid pad baseline.
    if (!calibrateSensors(lps22, &adxl345, &adxl375)) {
      Serial.println(F("Sensor calibration failed -- no valid baro samples"));
      failed_sensors++;
    }
  }

  if (isSimMode(currentMode)) {
    comm.setTimeout(100);
    BrakeState.hasCheckedForHorizontal = true;
  }

  if (comm.connected()) {
    logging.setDebug(true);
  }

  // Telemetry schema is append-only for dashboard/log compatibility.
  logging.log(
      "Time,Xg,Yg,Zg,Xg_high,Yg_high,Zg_high,Pressure,Temperature,Altitude,"
      "BNO_X,BNO_Y,BNO_Z,BNO_I,BNO_J,BNO_K,BNO_Real,State,"
      "Airbrake_pct,Airbrake_dir,Predicted_Apogee,Cd_Add_Cmd,"
      "I2C_Fallback,I2C_FailCount,Potentiometer,Velocity,"
      "Target_Cd_Raw,Apo_No_Brakes,Apo_Max_Brakes,Armed,"
      "Max_Altitude,Max_Velocity,Max_Accel_g,Ignition_Time_ms,Apogee_Time_ms,"
      "Active_Target_Alt,Fallback_Tier,Fallback_Apo_At_Decision_M");
  logging.flush();

  // Attach servos after SD/SPI setup to avoid pin ownership conflicts.
  airbrake_servo_1.begin(PinDefs.SERVO);
  airbrake_servo_2.begin(PinDefs.SERVO_2);
  airbrake_servo_1.setExtension(RocketConfig::SERVO_MIN_PCT);
  airbrake_servo_2.setExtension(RocketConfig::SERVO_MIN_PCT);
  BrakeState.pct = 0.0f;

  if (failed_sensors > 0 && !isSimMode(currentMode)) {
    statusIndicator.solid(StatusIndicator::WHITE);
    Serial.println(F("Setup failed -- sensor error"));
    state = States::SENSOR_ERROR;
  } else {
    statusIndicator.solid(StatusIndicator::GREEN);
    Serial.println(F("Setup complete"));
    state = States::IDLE;
  }

  // Silence plus white LED means setup fault.
  if (currentMode == TeensyMode::FLIGHT && state != States::SENSOR_ERROR) {
    Serial.println(F("FLIGHT mode -- buzzer confirm"));
    flightModeBuzzerConfirm();
  }
}

// ── Main Loop ────────────────────────────────────────────────────
//
// Main 50 ms scheduler.

void loop() {
  unsigned long now = millis();
  if (now - last_loop_time < LOOP_INTERVAL_MS) return;
  last_loop_time = now;

  comm.poll();

  static bool commWasConnected = false;
  bool commNow = comm.connected();
  if (commNow && !commWasConnected) logging.setDebug(true);
  commWasConnected = commNow;

  // Audible "on battery" cue, ground states only.
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
