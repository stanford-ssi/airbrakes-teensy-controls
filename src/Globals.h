#pragma once

#include <Arduino.h>
#include <Config.h>
#include <States.h>
#include <peripherals/Bilda.h>
#include <peripherals/Igniter.h>
#include <peripherals/StatusIndicator.h>

// Latest sensor frame from real hardware or SHITL.
struct SensorData_t {
  float accel_x, accel_y, accel_z;
  float accel_x_high_g, accel_y_high_g, accel_z_high_g;
  float pressure, temperature;
  float bno_x, bno_y, bno_z;
  float bno_i, bno_j, bno_k, bno_real;
  uint16_t potentiometer_value;
};

// Brake command and sweep bookkeeping.
struct BrakeState_t {
  float pct = 0.0f;
  int direction = 1;
  unsigned long last_update = 0;
  int fallback_sweep_count = 0;
  bool hasCheckedForHorizontal = false;
};

// Per-flight timing, estimator, and peak telemetry values.
struct FlightState_t {
  unsigned long ignition_time = 0;
  unsigned long motor_burnout_time = 0;
  unsigned long fire_time = 0;
  float max_altitude = 0.0f;
  float prev_altitude = 0.0f;
  float velocity = 0.0f;  // m/s, signed (positive = up)
  float max_velocity = 0.0f;
  float max_accel_g = 0.0f;
  // Fallback-target ladder state. The flight loop runs `as normal` (controller
  // targeting primary 30k) until the rocket reaches FALLBACK_ARM_ALT_AGL_M
  // (20k ft AGL), at which point evaluateFallbackTarget() picks one of three
  // tiers based on the live `apo_no_brakes from current state`. See main.cpp
  // for the safety reasoning.
  //
  //   fallback_decision_made: set once the gate fires. Prevents re-eval.
  //   fallback_tier: 1 = primary 30k (no change), 2 = 28.5k, 3 = 26k.
  //                  Defaults to 1 so pre-decision telemetry shows primary.
  //   fallback_apo_at_decision_m: apo_no_brakes value the decision used.
  bool fallback_decision_made = false;
  int fallback_tier = 1;
  float fallback_apo_at_decision_m = 0.0f;
};

// Optional two-Teensy controller state and diagnostics.
struct I2CControl_t {
  unsigned long lastSend = 0;
  bool fallback = false;          // I2C link fallback (unrelated to target fallback)
  int failCount = 0;
  float cmd_servo_1 = 0.0f;
  float cmd_servo_2 = 0.0f;
  float predicted_apogee = 0.0f;
  float cd_add_cmd = 0.0f;
  float target_cd_raw = 0.0f;
  float apo_no_brakes = 0.0f;
  float apo_max_brakes = 0.0f;
  // Mirror of AirbrakeController::targetAltitude() so the dashboard / SD log
  // can see when the fallback-target latch fires (value drops from primary
  // 9144 m to fallback 8686.8 m). Defaults to primary so pre-launch frames
  // show the right value.
  float active_target_alt = 9144.0f;
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

// Drive both servos, or telemetry only in SHITL_DEMO.
void driveServos(float pct);

// True when local pyro outputs must stay inhibited.
bool shouldInhibitPyros();

// Sticky dashboard ARM state.
bool isArmed();
