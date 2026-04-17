#pragma once

namespace RocketConfig {
    // Mass properties (V2 rocket — matches SHITL dashboard sim)
    constexpr float MASS_KG = 23.28f;

    // Aerodynamic properties (V2 rocket — flat-plate airbrake model)
    constexpr float BODY_CD = 0.364f;       // Representative body Cd (V2 Cd-vs-Mach midpoint)
    // CFD table gives cd_add up to ~1.41 (mach 0.1) ... ~1.50 (mach 0.8) at 95° deploy.
    // 1.5 lets the inverse lookup naturally saturate to ANGLE_MAX (100% servo) when commanded full.
    constexpr float MAX_CD_ADD = 1.5f;
    constexpr float MAX_CD = BODY_CD + MAX_CD_ADD;
    constexpr float REF_AREA_M2 = 0.01929f; // Reference cross-section area (V2 diameter 0.15672m)

    // Target altitude
    constexpr float TARGET_ALT_AGL_M = 9144.0f;  // 30,000 ft
    constexpr float MAX_TARGET_ALT_M = 9144.0f;  // 30,000 ft (IREC max)

    // Launch site
    constexpr float LAUNCH_SITE_ALT_MSL_M = 630.9f; // FAR (Mojave) 2070 ft MSL

    // Controller limits
    constexpr float SERVO_MIN_PCT = 20.0f;   // Minimum servo extension (%) — mechanical park position, never commanded below this
    constexpr float SERVO_MAX_PCT = 100.0f;  // Maximum servo extension (%)
    constexpr float CD_SLEW_RATE_MAX = 1.5f; // Max Cd change rate (Cd/s)

    // Physics
    constexpr float G = 9.80665f;            // Gravitational acceleration (m/s²)
    constexpr float G_TO_MS2 = G;            // Conversion: 1g = 9.80665 m/s²

    // Predictor settings
    constexpr int BINARY_SEARCH_ITERS = 12;
    constexpr float RK4_DT = 0.05f;         // Integration timestep (s)
    constexpr float MAX_SIM_TIME = 60.0f;    // Max coast simulation time (s)

    // Mach thresholds
    constexpr float MACH_SUPERSONIC = 1.0f;
    constexpr float MACH_TRANSONIC_LOW = 0.8f;
    constexpr float MACH_TRANSONIC_HIGH = 1.2f;

    // Post-launch lockout
    constexpr float POST_LAUNCH_DELAY_S = 13.0f; // seconds after ignition before controller activates

    // Apogee detection
    constexpr float APOGEE_VELOCITY_THRESHOLD = 5.0f;  // m/s
    constexpr float POST_APOGEE_RETRACT_DELAY_S = 2.0f;

    // State machine thresholds
    constexpr float IGNITION_ACCEL_THRESHOLD = 8.0f;  // g's on thrust axis
    constexpr float LANDING_ALTITUDE_THRESHOLD = 5.0f; // meters AGL
    constexpr unsigned long APOGEE_TIMEOUT_MS = 44000;  // backup apogee detection (ms after ignition)
    constexpr unsigned long IGNITER_FIRE_DURATION_MS = 2000;
    constexpr unsigned long LOOP_INTERVAL_MS = 50;

    // Airbrake test sweep
    constexpr unsigned long TEST_SWEEP_PAUSE_MS = 3000;
    constexpr unsigned long TEST_SWEEP_INTERVAL_MS = 1000;
}
