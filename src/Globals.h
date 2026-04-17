#pragma once

#include <Arduino.h>
#include <Config.h>
#include <States.h>
#include <peripherals/Bilda.h>
#include <peripherals/Igniter.h>
#include <peripherals/StatusIndicator.h>

struct SensorData_t {
  float accel_x, accel_y, accel_z;
  float accel_x_high_g, accel_y_high_g, accel_z_high_g;
  float pressure, temperature;
  float bno_x, bno_y, bno_z;
  float bno_i, bno_j, bno_k, bno_real;
  uint16_t potentiometer_value;
};

struct BrakeState_t {
  float pct = 0.0f;
  int direction = 1;
  unsigned long last_update = 0;
  bool hasCheckedForHorizontal = false;
};

struct FlightState_t {
  unsigned long ignition_time = 0;
  unsigned long motor_burnout_time = 0;
  unsigned long fire_time = 0;
  float max_altitude = 0.0f;
  float prev_altitude = 0.0f;
  float velocity = 0.0f;  // m/s, integrated from ignition
};

struct I2CControl_t {
  unsigned long lastSend = 0;
  bool fallback = false;
  int failCount = 0;
  float cmd_servo_1 = 0.0f;
  float cmd_servo_2 = 0.0f;
  float predicted_apogee = 0.0f;
  float cd_add_cmd = 0.0f;
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

// Physical servo motion is suppressed in SHITL_DEMO; BrakeState.pct is always updated.
void driveServos(float pct);
