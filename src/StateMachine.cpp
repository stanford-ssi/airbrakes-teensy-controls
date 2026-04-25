// State machine for one flight, driven once per main loop tick.
//
// Transitions: BOOT → IDLE → (AIRBRAKE_TEST | IGNITION) → ASCENT → APOGEE
//                                                              → DESCENT → LANDED
//              SENSOR_ERROR is a terminal sink set during setup() / SOFT_RESET
//              when calibration fails.
//
// Per-state LED color (set every tick by the handler):
//   SENSOR_ERROR  WHITE     IGNITION  ORANGE
//   IDLE          GREEN     ASCENT    RED
//   AIRBRAKE_TEST BLUE      APOGEE    RED
//   DESCENT       BLUE      LANDED    GREEN
//
// Pyro safety: handleIdle only HW-arms the igniter when isArmed() &&
// !shouldInhibitPyros(). handleApogee re-checks isArmed() before firing so a
// mid-flight DISARM still aborts the pyro.

#include <Arduino.h>
#include <Globals.h>
#include <PhysicsConstants.h>
#include <RocketConfig.h>

static States handleSensorError() {
  statusIndicator.solid(StatusIndicator::WHITE);
  return States::SENSOR_ERROR;
}

// IDLE: waiting on the pad. On the first tick we check orientation — if the
// rocket is horizontal we sweep the airbrakes as a ground test. Otherwise we
// poll for ignition (sustained accel_z above threshold).
static States handleIdle() {
  statusIndicator.solid(StatusIndicator::GREEN);

  if (sensors.accel_z > IGNITION_ACCEL_THRESHOLD) {
    FlightState.ignition_time = millis();
    FlightState.velocity = 0.0f;
    // Two gates required to HW-arm the apogee igniter:
    //   shouldInhibitPyros — never energize a real pyro from a SHITL sim.
    //   isArmed            — operator must have explicitly armed via dashboard.
    if (!shouldInhibitPyros() && isArmed()) {
      primaryIgniter.arm();
    }
    return States::IGNITION;
  }

  if (!BrakeState.hasCheckedForHorizontal) {
    // Z is the thrust axis: |Z| dominates when vertical.
    if (abs(sensors.accel_x) > abs(sensors.accel_z) ||
        abs(sensors.accel_y) > abs(sensors.accel_z)) {
      BrakeState.direction = 1;
      BrakeState.last_update = 0;
      BrakeState.pct = AIRBRAKE_MIN;
      return States::AIRBRAKE_TEST;
    }
    BrakeState.hasCheckedForHorizontal = true;
  }

  return States::IDLE;
}

// AIRBRAKE_TEST: ground demo, ramps the brakes between AIRBRAKE_MIN/MAX.
// Terminal — no exit transition. Rocket must boot vertical to ever reach
// IDLE→IGNITION; this state is for bench/horizontal testing only.
static States handleAirbrakeTest() {
  statusIndicator.solid(StatusIndicator::BLUE);

  if (millis() - BrakeState.last_update >=
      (BrakeState.pct <= AIRBRAKE_MIN ? TEST_SWEEP_PAUSE_MS : TEST_SWEEP_INTERVAL_MS)) {
    BrakeState.last_update = millis();
    float airbrake_pct = BrakeState.pct + TEST_SWEEP_STEP * BrakeState.direction;
    if (airbrake_pct >= AIRBRAKE_MAX) {
      airbrake_pct = AIRBRAKE_MAX;
      BrakeState.direction = -1;
    } else if (airbrake_pct <= AIRBRAKE_MIN) {
      airbrake_pct = AIRBRAKE_MIN;
      BrakeState.direction = 1;
    }
    driveServos(RocketConfig::airbrakePctToServoPct(airbrake_pct));
  }

  return States::AIRBRAKE_TEST;
}

// IGNITION: motor burning. Exit on burnout — accel_z < 0 for 3 consecutive
// samples (~150 ms). The 3-sample debounce prevents a single noisy reading
// or thrust dip from tripping the UKF reseed + sensor-weighting change
// mid-burn.
static int burnoutConfirmCount = 0;
static const int BURNOUT_CONFIRM_THRESHOLD = 3;

static States handleIgnition() {
  statusIndicator.solid(StatusIndicator::ORANGE);

  if (sensors.accel_z < 0) {
    burnoutConfirmCount++;
  } else {
    burnoutConfirmCount = 0;
  }
  if (burnoutConfirmCount >= BURNOUT_CONFIRM_THRESHOLD) {
    burnoutConfirmCount = 0;
    FlightState.motor_burnout_time = millis();
    return States::ASCENT;
  }

  return States::IGNITION;
}

// ASCENT: coast phase. Exit on apogee — altitude decreasing for 5 consecutive
// samples (~250 ms), or APOGEE_TIMEOUT_MS hard timeout. Mach lockout
// suppresses baro detection above MACH_LOCKOUT_VELOCITY (transonic shock
// noise on the LPS22). Decreasing-altitude check is also gated by
// APOGEE_MIN_ALTITUDE so a low-altitude motor failure can't fire the pyro.
static int apogeeConfirmCount = 0;
static const int APOGEE_CONFIRM_THRESHOLD = 5;

static States handleAscent(float altitude) {
  statusIndicator.solid(StatusIndicator::RED);

  // max_altitude is now tracked centrally in updateFlightMaxima (called
  // every tick by the loop functions), so it climbs from the moment
  // altitude leaves the pad-pinned 0 regime — i.e. as soon as IGNITION
  // fires, not only once we reach ASCENT.

  bool machLockout = FlightState.velocity > MACH_LOCKOUT_VELOCITY;
  bool altDecreasing = !machLockout && (altitude < FlightState.prev_altitude) && (altitude > APOGEE_MIN_ALTITUDE);
  bool timedOut = millis() - FlightState.ignition_time > APOGEE_TIMEOUT_MS;
  FlightState.prev_altitude = altitude;

  if (altDecreasing) {
    apogeeConfirmCount++;
  } else {
    apogeeConfirmCount = 0;
  }

  if (apogeeConfirmCount >= APOGEE_CONFIRM_THRESHOLD || timedOut) {
    driveServos(RocketConfig::SERVO_MIN_PCT);  // retract before pyro fires
    FlightState.fire_time = millis();
    return States::APOGEE;
  }

  return States::ASCENT;
}

// APOGEE: drive the apogee charge for IGNITER_FIRE_DURATION_MS, then move on.
// fire() is gated on isArmed() — defense in depth for handleIdle's gate, and
// allows a mid-flight DISARM to abort the pyro.
static States handleApogee() {
  statusIndicator.solid(StatusIndicator::RED);

  if (isArmed()) primaryIgniter.fire();

  if (millis() - FlightState.fire_time > IGNITER_FIRE_DURATION_MS) {
    primaryIgniter.stop();
    return States::DESCENT;
  }

  return States::APOGEE;
}

// DESCENT: under the recovery system. Exit when altitude drops below the
// landing threshold (close enough to ground level that we're definitely down).
static States handleDescent(float altitude) {
  statusIndicator.solid(StatusIndicator::BLUE);

  if (altitude < LANDING_ALTITUDE_THRESHOLD) {
    return States::LANDED;
  }

  return States::DESCENT;
}

// LANDED: terminal. LED green, no further transitions.
static States handleLanded() {
  statusIndicator.solid(StatusIndicator::GREEN);
  return States::LANDED;
}

// Clears file-static debounce counters. Called by SOFT_RESET so a reset issued
// mid-flight doesn't carry stale burnout/apogee tallies into the next attempt.
void resetStateMachineCounters() {
  burnoutConfirmCount = 0;
  apogeeConfirmCount = 0;
}

void updateStateMachine(float altitude) {
  switch (state) {
    case States::SENSOR_ERROR:
      state = handleSensorError();
      break;
    case States::IDLE:
      state = handleIdle();
      break;
    case States::AIRBRAKE_TEST:
      state = handleAirbrakeTest();
      break;
    case States::IGNITION:
      state = handleIgnition();
      break;
    case States::ASCENT:
      state = handleAscent(altitude);
      break;
    case States::APOGEE:
      state = handleApogee();
      break;
    case States::DESCENT:
      state = handleDescent(altitude);
      break;
    case States::LANDED:
      state = handleLanded();
      break;
    default:
      state = States::IDLE;
      break;
  }
}
