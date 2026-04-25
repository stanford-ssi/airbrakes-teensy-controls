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

// Sticky arm flag: ARM → true, DISARM / mode switch / power cycle → false.
// Never auto-cleared on WiFi loss (rocket leaves AP range mid-flight).
static bool armed = false;

static inline bool isSimMode(TeensyMode m) {
  return m == TeensyMode::SHITL || m == TeensyMode::SHITL_DEMO;
}

// Block real-pyro outputs in SHITL/SHITL_DEMO so simulated ignition profiles
// can't drive IGNITER_0 HIGH on the bench, AND on flights where no apogee
// charge is physically installed (PYROS_INSTALLED=false in Config.h).
// shouldInhibitPyros() == true means handleIdle skips primaryIgniter.arm(),
// so even if the operator ARMs and APOGEE fires, Igniter::fire() finds
// armed=false and never calls digitalWrite(igniterPin, HIGH).
bool shouldInhibitPyros() { return !PYROS_INSTALLED || isSimMode(currentMode); }

// Read-only accessor for the sticky arm flag (StateMachine gates the pyro on it).
bool isArmed() { return armed; }

// Drives both servos to the given extension % and mirrors it into BrakeState.pct
// (in airbrake-%, 0 = retracted) for telemetry. Suppresses physical motion in
// SHITL_DEMO.
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

// Boot tones. Three distinct cadences so the operator can tell by ear which
// event just fired:
//   FLIGHT mode entered:  5 × 500 ms @ 2000 Hz   (5 s total, slow)
//   WiFi AP came up:      3 × 80 ms  @ 2500 Hz   (chirpy, higher pitch)
//   USB unplugged:        8 × 250 ms @ 2000 Hz   (4 s total, tighter cadence)
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

// At-rest reference captured by calibrateSensors(). Per-chip accel-Z bias is
// required because ADXL345 and ADXL375 have independent zero offsets, and the
// IGNITION-phase weighting puts most weight on the high-g chip — a shared
// bias would integrate into kinematic_vel during burn.
static struct {
  float pressure = 0;
  float temperature = 0;
  float accel_z_bias = 0.0f;
  float accel_z_high_g_bias = 0.0f;
} RefCalibration;

// ── Sensor Helpers ───────────────────────────────────────────────

// Samples LPS22 + ADXL chips over ~1 s and stores the at-rest baseline in
// RefCalibration. Pressure/temperature anchor the AGL altitude; per-chip Z bias
// removes the resting-gravity + offset so accel integrates to zero at rest.
// Returns false if no valid baro sample landed — setup() flags failed_sensors
// in that case so we never fly on a zero-defaulted bias.
static bool calibrateSensors(Lps22 &lps, Adxl *lg_accel = nullptr,
                             Adxl *hg_accel = nullptr) {
  delay(100);
  const int N = 40;  // 40 samples × 25 ms = 1 s
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

// AGL altitude from pressure via the ISA troposphere inverse, referenced to
// RefCalibration.pressure (the at-rest baseline captured at launch site).
// Exact inside the constant-lapse-rate regime (0–11 km), matches the Python sim.
static float altitudeDelta(float p, float /*T*/) {
  if (p <= 0.0f) return NAN;
  float p_ref = RefCalibration.pressure;     // mbar at launch site
  float T_ref = RefCalibration.temperature;  // K at launch site
  // L*Rd/g ≈ 0.190263 (dimensionless). T_ref/L gives the scale constant
  // 284.05/0.0065 ≈ 43700 m — the height of the troposphere if T_ref held.
  const float L = 0.0065f;
  const float exponent = L * Rd / g0;
  return (T_ref / L) * (1.0f - powf(p / p_ref, exponent));
}

// ── Altitude Filter ─────────────────────────────────────────────

// Open-loop velocity from integrating blended accel. Used during burn because
// the UKF's constant-accel process model can't track the thrust ramp.
static float kinematic_vel = 0.0f;

// Runs the UKF + kinematic integrator on the latest sensor frame and returns
// the filtered AGL altitude. Sets FlightState.velocity using the dual-source
// rule: kinematic during burn (UKF lags the thrust ramp), UKF during coast
// (baro updates bound the accel-bias drift). On every state transition the
// UKF is reseeded with kinematic_vel so coast begins with burn velocity intact.
// See memory/project_velocity_estimator for full rationale.
static float filterAltitude(float raw_alt) {
  static States last_state = States::BOOT;
  float dt = LOOP_INTERVAL_MS / 1000.0f;

  // Proper accel → coordinate accel (m/s²) for the UKF's constant-accel model.
  float a_low  = (sensors.accel_z        - RefCalibration.accel_z_bias)        * g0;
  float a_high = (sensors.accel_z_high_g - RefCalibration.accel_z_high_g_bias) * g0;

  // First call after construction or after a UKF reset (enterSHITL / ZERO).
  if (!altitudeFilter.isInitialized()) {
    altitudeFilter.init(raw_alt, 0.0f, a_low);
    FlightState.velocity = 0.0f;
    kinematic_vel = 0.0f;
    last_state = state;
    return raw_alt;
  }

  // Inverse-variance blend of the two accel chips — same value the UKF
  // updateAccel() will consume. Weights use kinematic_vel (not
  // FlightState.velocity) to avoid feeding the filter's own output back in.
  SensorWeights w = SensorWeighting::getWeights(
      static_cast<uint8_t>(state), kinematic_vel, 343.0f);
  float inv_R_low  = 1.0f / w.R_accel_low;
  float inv_R_high = 1.0f / w.R_accel_high;
  float inv_R_sum  = inv_R_low + inv_R_high;
  float accel_blended = (a_low * inv_R_low + a_high * inv_R_high) / inv_R_sum;
  float R_blended     = 1.0f / inv_R_sum;

  // Integrate every tick — skipping a tick at the IGNITION transition loses
  // ~7 m/s of burn velocity.
  kinematic_vel += accel_blended * dt;

  // State change → reseed UKF with kinematic velocity so the next regime
  // doesn't inherit a stale steady-state Kalman gain.
  if (state != last_state) {
    altitudeFilter.init(raw_alt, kinematic_vel, a_low);
    FlightState.velocity = kinematic_vel;
    last_state = state;
    return raw_alt;
  }

  altitudeFilter.predict(dt);
  altitudeFilter.updateAccel(accel_blended, R_blended);
  altitudeFilter.updateBaro(raw_alt, w.R_baro);

  // Velocity source: UKF only during coast; kinematic everywhere else.
  if (state == States::ASCENT) {
    FlightState.velocity = altitudeFilter.velocity();
  } else {
    FlightState.velocity = kinematic_vel;
  }

  return altitudeFilter.altitude();
}

// ── Mode Negotiation ─────────────────────────────────────────────
// New-style protocol: Teensy sends READY → dashboard responds MODE,<name>.
// Legacy fallback:    Teensy sends DATAREQUEST → old SHITL scripts answer CSV.

// Exact-match MODE,<name> parser. Avoids prefix-match ambiguity between
// MODE,SHITL and MODE,SHITL_DEMO.
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
  while (comm.available()) comm.read();  // flush stale data

  // Phase 1: New protocol — send READY, wait for MODE command
  comm.println("READY");
  comm.setTimeout(2000);

  char buf[128];
  int len = comm.readBytesUntil('\n', buf, sizeof(buf) - 1);
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
// Outlier-reject cal: only at-rest frames (Z near gravity) feed the bias.
// If the operator hits LAUNCH before cal finishes, burn frames are rejected
// instead of pulling the bias 1-3 g high. After SIM_CAL_TIMEOUT we force-
// complete with whatever good samples landed; if zero, cal stays NaN and
// the controller refuses to run.
static bool simCalibrated = false;
static int simCalibrationCount = 0;        // good (at-rest) samples accumulated
static int simCalibrationTotalCount = 0;   // total samples seen
static const int SIM_CAL_SAMPLES = 20;     // good samples needed (~1 s)
static const int SIM_CAL_TIMEOUT = 200;    // total samples before forcing (~10 s)
static const float CAL_REST_TOLERANCE_G = 0.3f;

// Optional dashboard-supplied ground truth. The 9-field CSV protocol (time,
// ax/ay/az, pressure, temp, alt, vel, phase_code) lets SHITL run the controller
// on the exact same inputs as the pure-Python demo. The 6-field legacy
// protocol falls back to UKF + state machine.
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

// Resets flight/state/filter/calibration without rebooting the MCU. Keeps the
// WiFi AP + TCP client alive (hardware RESET drops them for ~15 s) and the SD
// log file open. Drops a "# SOFT_RESET" marker line into the log so post-
// flight tooling can split pre- and post-reset segments.
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

  // If we soft-reset mid-APOGEE the pyro may already be firing.
  if (!shouldInhibitPyros()) primaryIgniter.stop();

  char marker[48];
  snprintf(marker, sizeof(marker), "# SOFT_RESET at millis=%lu", millis());
  logging.log(marker);
  logging.flush();

  // Re-calibrate. SIM modes wait for fresh dashboard frames; real-sensor modes
  // re-sample for ~1 s here and go to SENSOR_ERROR if calibration fails so we
  // never fly on an implicit zero bias.
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

// Switches the firmware into SHITL or SHITL_DEMO. Resets the filter, sim
// calibration counters, and controller; clears sticky arm; parks servos once
// for SHITL_DEMO (driveServos suppresses motion afterwards).
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
  SimTruth.valid = false;  // re-detect 6-vs-9-field protocol on new session
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

// Dispatch one already-newline-stripped command line from the dashboard.
// Recognised commands:
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
      // Beep on transitions *into* FLIGHT, and only from ground states so we
      // never burn 5 s of the main loop mid-flight.
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
    // Hold-to-beep. Dashboard sends ON on mousedown, OFF on mouseup; if the
    // connection drops mid-hold the next reconnect's OFF still wins.
    tone(PinDefs.BUZZER, 2000);
  } else if (strcmp(line, "BEEP,OFF") == 0) {
    noTone(PinDefs.BUZZER);
    digitalWrite(PinDefs.BUZZER, LOW);
  } else if (strcmp(line, "ZERO") == 0) {
    // Re-capture at-rest baseline. Operator must hold the rocket stationary
    // for the full ~1 s sample window or bias gets contaminated.
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
    // Sample each chip 10× over 300 ms in real-sensor modes and report
    // mean + range per channel. range > 0 proves the chip is updating; a
    // stuck ADXL reading 0.000 g or a frozen LPS22 will show range=0.
    // SIM modes emit a single snapshot.
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
      // One snapshot from whatever the dashboard last fed us.
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
    // p_ref stays 0.0 until calibrateSensors lands at least one good sample.
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
    // Marker write + flush. Catches a card that opened OK at boot but has
    // since been pulled or filled up.
    bool ok = logging.selfTest();
    comm.print(F("CHECK_RESULT,SD,ok="));
    comm.print(ok ? 1 : 0);
    comm.print(F(",file="));
    comm.println(logging.fileName());
  } else if (strcmp(line, "LIST,SD") == 0) {
    // BEGIN/END markers let the dashboard demux this from telemetry.
    // Hidden files (".DS_Store" etc) are filtered out.
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
    // Streams the named file as SD_LINE,<content> rows wrapped in
    // SD_FILE_BEGIN/END. Flush the live log first so a download of the
    // current LOG###.TXT sees pending writes.
    //
    // Per-line shape is built in one buffer ("SD_LINE,<content>") and
    // emitted with a single comm.println — that becomes one TCP write
    // (body + CRLF combined). Previously the prefix and body were two
    // separate writes, which on long downloads filled NINA-FW's TCP send
    // buffer faster than the dashboard could drain it; the second write
    // would return 0 and (until _tcpWriteAll's retry was added) silently
    // drop the chunk, halting the download partway through.
    //
    // We also bail out of the read loop early if the link drops, instead
    // of churning the SD card just to throw bytes at a closed socket.
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
        // Pump WiFi accept / STATUS so the AP doesn't appear dead on a
        // multi-second download. _tcpWriteAll already yields per partial
        // write, so no extra delay here.
        comm.poll();
      }
      f.close();
      comm.print(F("SD_FILE_END,"));
      comm.println(fname);
    }
  }
}

// Non-blocking line reader. Accumulates bytes from comm into _cmdBuf until
// CR/LF, then dispatches to handleSerialCommand. Used in modes where the
// dashboard sends commands but does NOT push CSV sensor frames.
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

// Polls every sensor on the I2C bus, fills the global SensorData_t, and
// returns the AGL altitude derived from the latest pressure reading.
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

// Distinguishes dashboard control lines from CSV sensor frames in SHITL.
// Must list every command the operator can send during SHITL or those
// commands silently no-op against the sensor-row parser.
static bool isDashboardCommand(const char *line) {
  return strncmp(line, "SERVO,", 6) == 0 || strcmp(line, "ZERO") == 0 ||
         strcmp(line, "RESET") == 0 || strcmp(line, "SOFT_RESET") == 0 ||
         strncmp(line, "MODE,", 5) == 0 || strcmp(line, "GET,MODE") == 0 ||
         strncmp(line, "BEEP,", 5) == 0 || strcmp(line, "ARM") == 0 ||
         strcmp(line, "DISARM") == 0 || strncmp(line, "CHECK,", 6) == 0 ||
         strncmp(line, "LIST,", 5) == 0 || strncmp(line, "READ,", 5) == 0;
}

// Pulls one CSV sensor frame from the dashboard. Supports both the 9-field
// extended protocol (time, ax, ay, az, p, T, alt, vel, phase_code → ground
// truth available, controller skips UKF) and the 6-field legacy protocol
// (sensors only → UKF + state machine derive altitude/velocity/state).
// Inline command lines are dispatched as they arrive (capped at 2 per call
// so command spam can't starve sensor data).
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
  sensors.accel_x_high_g = sensors.accel_x;
  sensors.accel_y_high_g = sensors.accel_y;
  sensors.accel_z_high_g = sensors.accel_z;

  // 9-field protocol: ground truth available, skip calibration + UKF.
  if (gotCount >= 9) {
    SimTruth.sim_time_s = values[0];
    SimTruth.alt_m      = values[6];
    SimTruth.vel_ms     = values[7];
    SimTruth.phase_code = (int)values[8];
    SimTruth.valid      = true;
    return SimTruth.alt_m;
  }

  // 6-field protocol: accumulate at-rest samples to calibrate, then return
  // altitude derived from pressure.
  SimTruth.valid = false;
  if (!simCalibrated) {
    simCalibrationTotalCount++;
    // Reject burn-phase frames so an early LAUNCH doesn't poison the bias.
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
      // 10 s elapsed with zero at-rest samples → refuse to fly on a bad bias.
      comm.println(F("SHITL cal FAILED: no at-rest samples after 10s — switch to TEST mode and retry"));
      simCalibrationTotalCount = 0;  // re-arm the timeout for another window
    }
    return NAN;
  }

  return altitudeDelta(sensors.pressure, sensors.temperature + CELSIUS_TO_KELVIN);
}

// Updates max_altitude / max_velocity / max_accel_g every loop so a dashboard
// reconnecting after landing sees the peak values immediately, AND so that
// max_altitude starts climbing as soon as the firmware leaves the pad-pinned
// 0-altitude regime (i.e. the moment IGNITION fires) instead of waiting for
// ASCENT. While on the pad, callers pass altitude=0 so the max stays at 0
// regardless of baro noise.
static void updateFlightMaxima(float altitude) {
  if (altitude > FlightState.max_altitude) FlightState.max_altitude = altitude;
  float v = fabsf(FlightState.velocity);
  if (v > FlightState.max_velocity) FlightState.max_velocity = v;
  float az = fmaxf(fabsf(sensors.accel_z), fabsf(sensors.accel_z_high_g));
  if (az > FlightState.max_accel_g) FlightState.max_accel_g = az;
}

// ── I2C Control Packet (FLIGHT mode only) ────────────────────────

// Sends the latest sensor frame to the control Teensy at CTRL_TEENSY_ADDR
// (which runs the apogee predictor + servo controller) and reads back the
// commanded servo angles + diagnostics. After I2C_FAIL_THRESHOLD consecutive
// transmission failures we latch into fallback mode and runFallbackSweep
// takes over with an open-loop sweep.
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

// Runs the on-Teensy AirbrakeController + Cd-to-servo-angle lookup in SHITL.
// When the dashboard provides ground truth, uses the sim clock so the
// post-launch delay, apogee timer, etc. line up with Python's sim_time.
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

  driveServos(angles.angle_1);
}

// ── Fallback Sweep (open-loop, FLIGHT mode only) ────────────────

// Open-loop airbrake sweep for the case where the control Teensy is
// unreachable. Holds off until well past burnout, then steps the brakes
// AIRBRAKE_MIN ↔ AIRBRAKE_MAX on a fixed cadence. NOT apogee-targeted.
// Bails out after FALLBACK_SWEEP_COUNT full cycles so we don't keep poking
// a possibly-faulty actuator all the way to apogee — by then the brakes
// are parked at AIRBRAKE_MIN (we increment when arriving there).
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

// "On the pad" states where altitude must read exactly 0 — keeps the dashboard
// chart and the predictor's pre-launch math stable before liftoff.
static bool isOnPad() {
  return state == States::IDLE || state == States::AIRBRAKE_TEST || state == States::SENSOR_ERROR;
}

// FLIGHT loop: real sensors → state machine → during ASCENT, send sensor
// frame to the control Teensy and apply the returned servo command. If I2C
// to the control Teensy is down (bench testing without the second board, or
// the control Teensy fails mid-flight), run the same AirbrakeController
// locally — same code path SHITL exercises every day — so we still get
// apogee-targeted guidance instead of a dumb open-loop sweep.
//
// Why not the old runFallbackSweep(): it's a fixed-cadence MIN↔MAX pattern
// that ignores velocity, altitude, and predicted apogee, so on a bench
// flight (or any flight where I2C drops within 500 ms of liftoff) the
// rocket has no real guidance through coast. The local controller path
// has the full predictor + Cd→servo pipeline and runs on every input the
// airbrakes Teensy already has (baro, accel, velocity, atmosphere, state).
// runFallbackSweep is kept compiled but no longer reachable from FLIGHT —
// retain it for now in case we ever want to wire it back in as a deeper
// "controller-also-broken" fallback.
static void loopFlight() {
  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  // Always run the filter so kinematic_vel (and the UKF) stay live during
  // IDLE — the dashboard wants to show what the firmware thinks the current
  // velocity is even on the pad. After ZERO calibration on a stationary
  // rocket, kinematic_vel reads ~0 m/s; pick it up and the integrated accel
  // shows real motion. Display altitude is still pinned to 0 on the pad so
  // the chart doesn't render baro noise as motion.
  float filtered_alt = filterAltitude(raw_alt);
  float altitude = isOnPad() ? 0.0f : filtered_alt;

  checkSerialCommands();
  updateStateMachine(altitude);

  if (state == States::ASCENT) {
    if (USE_CONTROL && !I2CControl.fallback) {
      sendControlPacket(altitude);
      driveServos(I2CControl.cmd_servo_1);
    } else {
      // Same controller SHITL runs. Bench flights without the control
      // Teensy now exercise the real guidance instead of the sweep.
      runLocalController(altitude);
    }
  }

  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// SHITL loop: pull a sensor frame from the dashboard, then either trust the
// supplied ground truth (9-field) or derive everything via UKF + state machine
// (6-field). Local controller + servo apply on every tick — controller
// internally retracts during PREFLIGHT/BURN.
static void loopSHITL() {
  statusIndicator.rainbow();

  float raw_alt = readSimulatedSensors();
  if (isnan(raw_alt)) return;

  float altitude;
  if (SimTruth.valid) {
    state = phaseCodeToState(SimTruth.phase_code);
    altitude = SimTruth.alt_m;
    FlightState.velocity = SimTruth.vel_ms;
    // updateFlightMaxima below picks this up — no need to track here too.
  } else {
    // Run UKF + state machine every tick — including pre-launch — so velocity
    // is already tracking the burn by the time the predictor needs it.
    altitude = filterAltitude(raw_alt);
    updateStateMachine(altitude);
  }

  runLocalController(altitude);

  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// TEST loop: real sensors + UKF, no state machine. Used for manual servo
// commands and filter-tuning bench runs.
static void loopTest() {
  // Drive the LED every tick so leaving SHITL (rainbow PWM) clears cleanly.
  statusIndicator.solid(StatusIndicator::BLUE);

  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// SENSOR_MOVING loop: real sensors + UKF, no state machine, no servo motion.
// For ground tests where the rocket is moved/shaken to watch the estimator.
// Logs CSV in the same format as FLIGHT so the same tooling parses it.
static void loopSensorMoving() {
  statusIndicator.solid(StatusIndicator::ORANGE);

  float raw_alt = readRealSensors();
  if (isnan(raw_alt)) return;

  float altitude = filterAltitude(raw_alt);

  checkSerialCommands();
  updateFlightMaxima(altitude);
  logging.logTelemetry(altitude, FlightState.velocity, sensors, BrakeState, I2CControl, state, armed);
}

// ── Setup ────────────────────────────────────────────────────────
//
// Boot sequence:
//   1. Serial + GPIO init.
//   2. Wait up to 3 s for USB host (flight continues without it).
//   3. Bring up the AirLift WiFi AP (transparent fallback to USB-only).
//   4. Negotiate mode with the dashboard (or default to FLIGHT).
//   5. I2C + sensor init + at-rest calibration. Failures → SENSOR_ERROR.
//   6. SD card open, log header written.
//   7. Servos parked at SERVO_MIN_PCT.
//   8. FLIGHT-mode buzzer confirm (skipped on SENSOR_ERROR).
//
// Worst-case setup time on the pad is ~22 s (3 s serial + AirLift + 4 s
// sensor settle + up to 10 s SD retry + 5 s buzzer). Plan the timeline.

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

  // AirLift comes up BEFORE mode negotiation so a WiFi dashboard can
  // participate in the READY/MODE handshake. Three short beeps confirm
  // the AP is listening; absence of beeps = USB-only mode. SSID/PASS/PORT
  // are defined in WifiPins.h alongside the AirLift control pin map so
  // CommLink and the wifi_bringup smoke test stay in lockstep.
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

  // pollDelay() instead of delay() so the AirLift TCP backlog drains during
  // long setup waits — without it, a dashboard that opens TCP while we're
  // still in setup sits unbound until loop() starts (which can be 15+ s).
  comm.pollDelay(4000);  // sensor power-up settle

  // Retry SD for ~10 s so a slow card insertion doesn't strand the Teensy.
  // After that, continue without a log rather than locking on RED.
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
    // No silent fly-with-zero-bias: failure here promotes to SENSOR_ERROR
    // below. Without this, an uncancelled 1 g of resting accel integrates
    // into ~9.8 m/s² of phantom upward velocity per tick.
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

  // CSV column header. Schema is append-only — old dashboards parsing the
  // first 29 fields keep working; new fields read from column 30 onward.
  logging.log(
      "Time,Xg,Yg,Zg,Xg_high,Yg_high,Zg_high,Pressure,Temperature,Altitude,"
      "BNO_X,BNO_Y,BNO_Z,BNO_I,BNO_J,BNO_K,BNO_Real,State,"
      "Airbrake_pct,Airbrake_dir,Predicted_Apogee,Cd_Add_Cmd,"
      "I2C_Fallback,I2C_FailCount,Potentiometer,Velocity,"
      "Target_Cd_Raw,Apo_No_Brakes,Apo_Max_Brakes,Armed,"
      "Max_Altitude,Max_Velocity,Max_Accel_g,Ignition_Time_ms,Apogee_Time_ms");
  logging.flush();

  // Servos attach LAST: SD.begin() calls SPI.begin() which claims pin 10,
  // and the Servo library has its own pin-claim that must run after.
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

  // FLIGHT confirm beep. Skip on SENSOR_ERROR so the WHITE LED + silence
  // is unambiguous for fault diagnosis on the pad.
  if (currentMode == TeensyMode::FLIGHT && state != States::SENSOR_ERROR) {
    Serial.println(F("FLIGHT mode -- buzzer confirm"));
    flightModeBuzzerConfirm();
  }
}

// ── Main Loop ────────────────────────────────────────────────────
//
// 50 ms tick (LOOP_INTERVAL_MS). Each tick:
//   1. Pump WiFi accept / client-alive bookkeeping.
//   2. Re-enable telemetry streaming if a transport came online post-boot.
//   3. Watch for USB unplug (announce "on battery" with the 8-beep cadence).
//   4. Dispatch to the active mode's loop function.

void loop() {
  unsigned long now = millis();
  if (now - last_loop_time < LOOP_INTERVAL_MS) return;
  last_loop_time = now;

  comm.poll();

  static bool commWasConnected = false;
  bool commNow = comm.connected();
  if (commNow && !commWasConnected) logging.setDebug(true);
  commWasConnected = commNow;

  // FLIGHT-only USB-unplug watcher, gated to ground states so we never burn
  // ~4 s of the main loop mid-flight if the connector tears loose under load.
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
