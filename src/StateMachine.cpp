#include <Arduino.h>
#include <Globals.h>

static States handleSensorError() {
  statusIndicator.solid(StatusIndicator::WHITE);
  return States::SENSOR_ERROR;
}

static States handleIdle() {
  statusIndicator.solid(StatusIndicator::GREEN);

  if (sensors.accel_y > IGNITION_ACCEL_THRESHOLD) {
    FlightState.ignition_time = millis();
    FlightState.velocity = 0.0f;
    primaryIgniter.arm();
    return States::IGNITION;
  }

  if (!BrakeState.hasCheckedForHorizontal) {
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

static States handleAirbrakeTest() {
  statusIndicator.solid(StatusIndicator::BLUE);

  if (millis() - BrakeState.last_update >=
      (BrakeState.pct <= AIRBRAKE_MIN ? TEST_SWEEP_PAUSE_MS : TEST_SWEEP_INTERVAL_MS)) {
    BrakeState.last_update = millis();
    BrakeState.pct += TEST_SWEEP_STEP * BrakeState.direction;
    if (BrakeState.pct >= AIRBRAKE_MAX) {
      BrakeState.pct = AIRBRAKE_MAX;
      BrakeState.direction = -1;
    } else if (BrakeState.pct <= AIRBRAKE_MIN) {
      BrakeState.pct = AIRBRAKE_MIN;
      BrakeState.direction = 1;
    }
    airbrake_servo_1.setExtension(BrakeState.pct);
    airbrake_servo_2.setExtension(BrakeState.pct);
  }

  return States::AIRBRAKE_TEST;
}

static States handleIgnition() {
  statusIndicator.solid(StatusIndicator::ORANGE);

  FlightState.velocity += (sensors.accel_y_high_g - 1.0f) * 9.81f * (LOOP_INTERVAL_MS / 1000.0f);

  if (sensors.accel_y < 0) {
    FlightState.motor_burnout_time = millis();
    return States::ASCENT;
  }

  return States::IGNITION;
}

static int apogeeConfirmCount = 0;
static const int APOGEE_CONFIRM_THRESHOLD = 5;  // require 5 consecutive altitude decreases (~250ms)

static States handleAscent(float altitude) {
  statusIndicator.solid(StatusIndicator::RED);

  FlightState.max_altitude = max(FlightState.max_altitude, altitude);

  if (!USE_CONTROL || I2CControl.fallback) {
    if (millis() - FlightState.ignition_time > FALLBACK_IGNITION_DELAY_MS ||
        millis() - FlightState.motor_burnout_time > FALLBACK_BURNOUT_DELAY_MS) {
      if (millis() - BrakeState.last_update >=
          (BrakeState.pct <= AIRBRAKE_MIN ? FALLBACK_SWEEP_PAUSE_MS : FALLBACK_SWEEP_INTERVAL_MS)) {
        BrakeState.last_update = millis();
        BrakeState.pct += FALLBACK_SWEEP_STEP * BrakeState.direction;
        if (BrakeState.pct >= AIRBRAKE_MAX) {
          BrakeState.pct = AIRBRAKE_MAX;
          BrakeState.direction = -2;
        } else if (BrakeState.pct <= AIRBRAKE_MIN) {
          BrakeState.pct = AIRBRAKE_MIN;
          BrakeState.direction = 1;
        }
        airbrake_servo_1.setExtension(BrakeState.pct);
        airbrake_servo_2.setExtension(BrakeState.pct);
      }
    }
  } else {
    sendControlPacket(altitude);
    airbrake_servo_1.setExtension(I2CControl.cmd_servo_1);
    airbrake_servo_2.setExtension(I2CControl.cmd_servo_2);
    BrakeState.pct = I2CControl.cmd_servo_1;
  }

  FlightState.velocity += (sensors.accel_y_high_g - 1.0f) * 9.81f * (LOOP_INTERVAL_MS / 1000.0f);

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
    airbrake_servo_1.setExtension(AIRBRAKE_MIN);
    airbrake_servo_2.setExtension(AIRBRAKE_MIN);
    BrakeState.pct = AIRBRAKE_MIN;
    BrakeState.direction = 1;
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
