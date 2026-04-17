#pragma once

// I2C
#define CTRL_TEENSY_ADDR 0x42
const bool USE_CONTROL = true;
const unsigned long CONTROL_INTERVAL_MS = 50;
const int I2C_FAIL_THRESHOLD = 10;

// Loop timing
const unsigned long LOOP_INTERVAL_MS = 50;

// Airbrake limits (sweep/test bounds — must stay >= RocketConfig::SERVO_MIN_PCT)
const float AIRBRAKE_MIN = 20.0;
const float AIRBRAKE_MAX = 60.0;

// State machine thresholds
const float IGNITION_ACCEL_THRESHOLD = 8.0;     // g — Y-axis accel to detect ignition
const float LANDING_ALTITUDE_THRESHOLD = 5.0;   // meters AGL
const unsigned long APOGEE_TIMEOUT_MS = 60000;  // ms after ignition — hard fallback to declare apogee
const float APOGEE_MIN_ALTITUDE = 30.48f;       // 100 ft in meters — minimum AGL to declare apogee
const float MACH_LOCKOUT_VELOCITY = 250.0f;  // m/s — lock out baro apogee detection above this speed (~Mach 0.73)
const unsigned long IGNITER_FIRE_DURATION_MS = 2000;

// Fallback sweep (open-loop when control Teensy unavailable)
const unsigned long FALLBACK_IGNITION_DELAY_MS = 12000;  // ms after ignition before sweep starts
const unsigned long FALLBACK_BURNOUT_DELAY_MS = 8000;    // ms after burnout before sweep starts
const unsigned long FALLBACK_SWEEP_PAUSE_MS = 1500;      // pause at min position
const unsigned long FALLBACK_SWEEP_INTERVAL_MS = 1000;   // normal sweep interval
const float FALLBACK_SWEEP_STEP = 25.0;                  // percent per step

// Airbrake test sweep
const unsigned long TEST_SWEEP_PAUSE_MS = 50;    // minimal pause at min position
const unsigned long TEST_SWEEP_INTERVAL_MS = 50;  // step every loop tick for continuous motion
const float TEST_SWEEP_STEP = 5.0;               // percent per step (smooth sweep)
