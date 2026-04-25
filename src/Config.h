#pragma once

// I2C
#define CTRL_TEENSY_ADDR 0x42
// Single-Teensy rocket — there is no peer at CTRL_TEENSY_ADDR. With this
// false, loopFlight skips sendControlPacket entirely and runs the
// AirbrakeController locally on the airbrakes Teensy from t=0 of ASCENT.
// The I2C scaffolding (sendControlPacket / ControlPacket / CommandPacket)
// stays compiled in case a second control Teensy is ever wired back on;
// flip this back to true at that point.
const bool USE_CONTROL = false;
const unsigned long CONTROL_INTERVAL_MS = 50;
const int I2C_FAIL_THRESHOLD = 10;

// Pyro hardware presence. Set to true when an apogee charge is physically
// wired to IGNITER_0/1, false on flights with no pyros installed. When false,
// shouldInhibitPyros() returns true unconditionally and primaryIgniter.arm()
// is never called — so the Igniter::fire() path can't drive a pin HIGH even
// if the operator ARMs from the dashboard. Defense in depth on top of the
// per-call isArmed() gate already in handleApogee.
const bool PYROS_INSTALLED = false;

// Loop timing
const unsigned long LOOP_INTERVAL_MS = 50;

// Airbrake sweep/test bounds — in airbrake deployment % (0 = fully retracted,
// 100 = fully extended). Mapped onto physical servo range via
// RocketConfig::airbrakePctToServoPct() at the driveServos() boundary.
const float AIRBRAKE_MIN = 0.0;
const float AIRBRAKE_MAX = 50.0;

// State machine thresholds
const float IGNITION_ACCEL_THRESHOLD = 3.0;     // g on Z (thrust axis), single-sample trigger
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
const int FALLBACK_SWEEP_COUNT = 3;                      // total 0→max→0 cycles before parking retracted

// Airbrake test sweep
const unsigned long TEST_SWEEP_PAUSE_MS = 50;    // minimal pause at min position
const unsigned long TEST_SWEEP_INTERVAL_MS = 50;  // step every loop tick for continuous motion
const float TEST_SWEEP_STEP = 5.0;               // percent per step (smooth sweep)
