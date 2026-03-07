#pragma once

namespace RocketConfig {
    // Mass properties
    constexpr float MASS_KG = 18.9f;

    // Aerodynamic properties (from CFD LookupTableV1.csv)
    constexpr float BODY_CD = 0.53f;        // Body drag coefficient at angle=1° (~Mach 0.45)
    constexpr float MAX_CD = 2.01f;         // Maximum total Cd (angle=95°, Mach 0.8)
    constexpr float MAX_CD_ADD = MAX_CD - BODY_CD;  // Max additional Cd from airbrakes (~1.48)
    constexpr float REF_AREA_M2 = 0.008107f; // Reference cross-section area

    // Target altitude
    constexpr float TARGET_ALT_AGL_M = 6096.0f;  // 20,000 ft
    constexpr float MAX_TARGET_ALT_M = 9144.0f;  // 30,000 ft (IREC max)

    // Launch site
    constexpr float LAUNCH_SITE_ALT_MSL_M = 1400.0f; // Spaceport America

    // Controller limits
    constexpr float SERVO_MIN_PCT = 10.0f;   // Minimum servo extension (%)
    constexpr float SERVO_MAX_PCT = 60.0f;   // Maximum servo extension (%)
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

    // Apogee detection
    constexpr float APOGEE_VELOCITY_THRESHOLD = 5.0f;  // m/s
    constexpr float POST_APOGEE_RETRACT_DELAY_S = 2.0f;
}
