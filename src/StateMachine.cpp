// One-flight state machine. Called once per 50 ms loop tick.

#include <Arduino.h>
#include <Globals.h>
#include <PhysicsConstants.h>
#include <RocketConfig.h>

static States handleSensorError() {
  statusIndicator.solid(StatusIndicator::WHITE);
  return States::SENSOR_ERROR;
}

// IDLE waits for launch.
static States handleIdle() {
  statusIndicator.solid(StatusIndicator::GREEN);

  if (sensors.accel_z > IGNITION_ACCEL_THRESHOLD) {
    FlightState.ignition_time = millis();
    FlightState.velocity = 0.0f;
    if (!shouldInhibitPyros() && isArmed()) {
      primaryIgniter.arm();
    }
    return States::IGNITION;
  }

  return States::IDLE;
}

// Bench-only sweep. Terminal until reset or mode change.
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

// Burnout is three consecutive negative thrust-axis accel samples.
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

// Apogee is five falling-altitude samples, or the hard timeout.
static int apogeeConfirmCount = 0;
static const int APOGEE_CONFIRM_THRESHOLD = 5;

static States handleAscent(float altitude) {
  statusIndicator.solid(StatusIndicator::RED);

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

// Local pyro path. For the current launch, another board owns pyro events.
static States handleApogee() {
  statusIndicator.solid(StatusIndicator::RED);

  if (isArmed()) primaryIgniter.fire();

  if (millis() - FlightState.fire_time > IGNITER_FIRE_DURATION_MS) {
    primaryIgniter.stop();
    return States::DESCENT;
  }

  return States::APOGEE;
}

// Landed when filtered altitude is back near pad level.
static States handleDescent(float altitude) {
  statusIndicator.solid(StatusIndicator::BLUE);

  if (altitude < LANDING_ALTITUDE_THRESHOLD) {
    return States::LANDED;
  }

  return States::DESCENT;
}

static States handleLanded() {
  statusIndicator.solid(StatusIndicator::GREEN);
  return States::LANDED;
}

// Reset debounce counters for SOFT_RESET.
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
