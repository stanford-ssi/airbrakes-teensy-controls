#include <Arduino.h>
#include <Globals.h>
#include <PhysicsConstants.h>
#include <RocketConfig.h>

static States handleSensorError() {
  statusIndicator.solid(StatusIndicator::WHITE);
  return States::SENSOR_ERROR;
}

static States handleIdle() {
  statusIndicator.solid(StatusIndicator::GREEN);

  if (sensors.accel_z > IGNITION_ACCEL_THRESHOLD) {
    FlightState.ignition_time = millis();
    FlightState.velocity = 0.0f;
    primaryIgniter.arm();
    return States::IGNITION;
  }

  if (!BrakeState.hasCheckedForHorizontal) {
    // Z is the thrust axis, so the rocket is vertical when |Z| dominates.
    // If X or Y dominates instead, it's lying on its side — do an airbrake
    // sweep as a ground test instead of arming ignition detection.
    if (abs(sensors.accel_x) > abs(sensors.accel_z) ||
        abs(sensors.accel_y) > abs(sensors.accel_z)) {
      BrakeState.direction = 1;
      BrakeState.last_update = 0;
      BrakeState.pct = AIRBRAKE_MIN;  // airbrake-% (0 = retracted)
      return States::AIRBRAKE_TEST;
    }
    BrakeState.hasCheckedForHorizontal = true;
  }

  return States::IDLE;
}

static States handleAirbrakeTest() {
  statusIndicator.solid(StatusIndicator::BLUE);

  // BrakeState.pct and AIRBRAKE_MIN/MAX both live in airbrake-% [0..100].
  // Convert to servo-% only at the driveServos boundary.
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

static States handleIgnition() {
  statusIndicator.solid(StatusIndicator::ORANGE);

  // Z is the thrust axis. Burnout = net accel crosses negative (drag+gravity
  // decelerating the rocket instead of thrust accelerating it).
  if (sensors.accel_z < 0) {
    FlightState.motor_burnout_time = millis();
    return States::ASCENT;
  }

  return States::IGNITION;
}

static int apogeeConfirmCount = 0;
static const int APOGEE_CONFIRM_THRESHOLD = 5;  // ~250ms of monotonic altitude decrease

static States handleAscent(float altitude) {
  statusIndicator.solid(StatusIndicator::RED);

  FlightState.max_altitude = max(FlightState.max_altitude, altitude);

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
    driveServos(RocketConfig::SERVO_MIN_PCT);
    FlightState.fire_time = millis();
    return States::APOGEE;
  }

  return States::ASCENT;
}

static States handleApogee() {
  statusIndicator.solid(StatusIndicator::RED);

  primaryIgniter.fire();

  if (millis() - FlightState.fire_time > IGNITER_FIRE_DURATION_MS) {
    primaryIgniter.stop();
    return States::DESCENT;
  }

  return States::APOGEE;
}

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
