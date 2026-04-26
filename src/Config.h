#pragma once

// Optional second Teensy controller.
#define CTRL_TEENSY_ADDR 0x42
// False for the current single-Teensy setup: control runs locally.
const bool USE_CONTROL = false;
const unsigned long CONTROL_INTERVAL_MS = 50;
const int I2C_FAIL_THRESHOLD = 10;

// False for flights where another board owns pyro events.
const bool PYROS_INSTALLED = false;

// Loop timing
const unsigned long LOOP_INTERVAL_MS = 50;

// Airbrake deployment percent: 0 = retracted, 100 = fully extended.
const float AIRBRAKE_MIN = 0.0;
const float AIRBRAKE_MAX = 50.0;

// State machine thresholds
const float IGNITION_ACCEL_THRESHOLD = 3.0;     // g on Z (thrust axis), single-sample trigger
const float LANDING_ALTITUDE_THRESHOLD = 5.0;   // meters AGL
const unsigned long APOGEE_TIMEOUT_MS = 60000;  // ms after ignition
const float APOGEE_MIN_ALTITUDE = 30.48f;       // 100 ft minimum for baro apogee
const float MACH_LOCKOUT_VELOCITY = 412.0f;     // m/s, about Mach 1.2
const unsigned long IGNITER_FIRE_DURATION_MS = 2000;

// Fallback sweep (open-loop when control Teensy unavailable)
const unsigned long FALLBACK_IGNITION_DELAY_MS = 12000;  // ms after ignition before sweep starts
const unsigned long FALLBACK_BURNOUT_DELAY_MS = 8000;    // ms after burnout before sweep starts
const unsigned long FALLBACK_SWEEP_PAUSE_MS = 1500;      // pause at min position
const unsigned long FALLBACK_SWEEP_INTERVAL_MS = 1000;   // normal sweep interval
const float FALLBACK_SWEEP_STEP = 25.0;                  // percent per step
const int FALLBACK_SWEEP_COUNT = 3;                      // total 0->max->0 cycles before parking

// Airbrake test sweep
const unsigned long TEST_SWEEP_PAUSE_MS = 50;    // minimal pause at min position
const unsigned long TEST_SWEEP_INTERVAL_MS = 50;  // step every loop tick for continuous motion
const float TEST_SWEEP_STEP = 5.0;               // percent per step (smooth sweep)
