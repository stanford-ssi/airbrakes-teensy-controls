#pragma once

#include <Arduino.h>
#include <Config.h>
#include <States.h>
#include <peripherals/Bilda.h>
#include <peripherals/Igniter.h>
#include <peripherals/StatusIndicator.h>

// Latest sensor frame, written by readRealSensors() / readSimulatedSensors()
// every loop tick. accel_*_high_g come from the ADXL375 (±200 g, dominates
// during motor burn); accel_x/y/z come from the ADXL345 (±16 g, dominates
// at low g). pressure/temperature drive the AGL altitude derivation.
struct SensorData_t {
  float accel_x, accel_y, accel_z;
  float accel_x_high_g, accel_y_high_g, accel_z_high_g;
  float pressure, temperature;
  float bno_x, bno_y, bno_z;
  float bno_i, bno_j, bno_k, bno_real;
  uint16_t potentiometer_value;
};

// Brake actuator state (commanded position + sweep bookkeeping).
//   pct: airbrake-% [0..100], 0 = retracted. driveServos() also updates this.
//   direction / last_update: AIRBRAKE_TEST and fallback-sweep step state.
//   fallback_sweep_count: completed 0→max→0 cycles in runFallbackSweep().
//     Capped at FALLBACK_SWEEP_COUNT — beyond that we hold retracted rather
//     than sweeping forever, since the same I2C failure that kicked us into
//     fallback could be a brake-actuator fault we don't want to keep poking.
//   hasCheckedForHorizontal: latched after the first IDLE tick to avoid
//     re-running the orientation check after the rocket leaves the pad.
struct BrakeState_t {
  float pct = 0.0f;
  int direction = 1;
  unsigned long last_update = 0;
  int fallback_sweep_count = 0;
  bool hasCheckedForHorizontal = false;
};

// Per-flight scalar state. Times are millis() snapshots; max_* are running
// peaks surfaced in telemetry so a dashboard reconnecting after landing
// sees the apogee and peak g/velocity without a separate query.
struct FlightState_t {
  unsigned long ignition_time = 0;
  unsigned long motor_burnout_time = 0;
  unsigned long fire_time = 0;
  float max_altitude = 0.0f;
  float prev_altitude = 0.0f;
  float velocity = 0.0f;  // m/s, signed (positive = up)
  float max_velocity = 0.0f;
  float max_accel_g = 0.0f;
};

// I2C link state to the control Teensy (apogee predictor + servo controller).
//   cmd_servo_*, predicted_apogee, cd_add_cmd: returned every tick.
//   target_cd_raw / apo_no_brakes / apo_max_brakes: predictor diagnostics.
//   fallback / failCount: tripped after I2C_FAIL_THRESHOLD consecutive
//     transmission failures; runFallbackSweep() takes over once latched.
struct I2CControl_t {
  unsigned long lastSend = 0;
  bool fallback = false;
  int failCount = 0;
  float cmd_servo_1 = 0.0f;
  float cmd_servo_2 = 0.0f;
  float predicted_apogee = 0.0f;
  float cd_add_cmd = 0.0f;
  float target_cd_raw = 0.0f;
  float apo_no_brakes = 0.0f;
  float apo_max_brakes = 0.0f;
};

extern SensorData_t sensors;
extern BrakeState_t BrakeState;
extern FlightState_t FlightState;
extern I2CControl_t I2CControl;
extern States state;
extern StatusIndicator statusIndicator;
extern Bilda airbrake_servo_1;
extern Bilda airbrake_servo_2;
extern Igniter primaryIgniter;

void sendControlPacket(float altitude);

// Drives both servos to `pct`. Suppresses physical motion in SHITL_DEMO but
// still updates BrakeState.pct so telemetry/charts read correctly.
void driveServos(float pct);

// True in SHITL / SHITL_DEMO. The state machine checks this before HW-arming
// the apogee igniter so a simulated accel/altitude profile can't fire a real
// pyro on the bench.
bool shouldInhibitPyros();

// True only after an explicit ARM command from the dashboard. Sticky: only
// DISARM, a mode switch, or a power cycle clears it. The state machine gates
// the apogee pyro on this. See memory/feedback_arm_is_sticky.
bool isArmed();
