#pragma once

namespace RocketConfig {
    // Mass properties (V2 rocket). Apogee predictor runs during coast, so
    // MASS_KG is the dry mass. Matches Airbrakes_SHITL / new-collins-airbrakes
    // tables/V2 Mass Change.csv final value (23.28 kg).
    constexpr float MASS_KG = 23.28f;

    // Aerodynamic properties (V2 rocket — flat-plate airbrake model, matches
    // new-collins-airbrakes/RocketPy/config.py).
    constexpr float BODY_CD = 0.364f;         // Legacy midpoint Cd — use V2BodyCd table for actual lookups
    constexpr float AIRBRAKE_CD = 1.28f;      // Flat-plate drag coefficient
    constexpr float AIRBRAKE_MAX_AREA_M2 = 0.018f;  // Projected area at full (95°) deployment
    constexpr float MACH_DEPLOY_LIMIT = 1.0f; // Airbrakes held retracted at/above this Mach
    constexpr float REF_AREA_M2 = 0.019292f;  // Reference cross-section area: π × (0.15672/2)²
    // Max additive Cd from flat-plate brake at full deployment:
    // 1.28 × 0.018 / 0.019292 ≈ 1.194. Leave predictor search headroom above that.
    constexpr float MAX_CD_ADD = 1.5f;
    constexpr float MAX_CD = BODY_CD + MAX_CD_ADD;

    // Target altitude
    constexpr float TARGET_ALT_AGL_M = 9144.0f;  // 30,000 ft
    constexpr float MAX_TARGET_ALT_M = 9144.0f;  // 30,000 ft (IREC max)

    // Launch site — FAR (Mojave), matches Airbrakes_SHITL and new-collins-airbrakes.
    constexpr float LAUNCH_SITE_ALT_MSL_M = 630.9f; // 2070 ft × 0.3048

    // Controller limits
    constexpr float SERVO_MIN_PCT = 0.0f;    // Minimum servo extension (%) — mechanical park position, never commanded below this
    constexpr float SERVO_MAX_PCT = 100.0f;  // Maximum servo extension (%)
    constexpr float CD_SLEW_RATE_MAX = 1.9f; // Max Cd change rate (Cd/s)

    // Airbrake deployment percentage <-> servo extension percentage.
    // Airbrake % is the user-facing scale: 0 = fully retracted, 100 = fully
    // extended. It maps linearly onto the physical servo range
    // [SERVO_MIN_PCT, SERVO_MAX_PCT]. Keep these scales conceptually distinct
    // — code that thinks in deployment ratios (sweep bounds, dashboard gauges)
    // uses airbrake %, code that drives the actuator uses servo %.
    inline float airbrakePctToServoPct(float airbrake_pct) {
        if (airbrake_pct < 0.0f) airbrake_pct = 0.0f;
        if (airbrake_pct > 100.0f) airbrake_pct = 100.0f;
        return SERVO_MIN_PCT + (airbrake_pct / 100.0f) * (SERVO_MAX_PCT - SERVO_MIN_PCT);
    }
    inline float servoPctToAirbrakePct(float servo_pct) {
        const float span = SERVO_MAX_PCT - SERVO_MIN_PCT;
        if (span <= 0.0f) return 0.0f;
        float t = (servo_pct - SERVO_MIN_PCT) / span;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        return t * 100.0f;
    }

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
