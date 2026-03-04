#include "ApogeePredictor.h"
#include "Atmosphere.h"
#include "RocketConfig.h"
#include <math.h>

ApogeePredictor::ApogeePredictor() {}

void ApogeePredictor::derivatives(float h_agl, float v, float cd_total,
                                   float launch_site_alt_msl,
                                   float &dh, float &dv) {
    float alt_msl = h_agl + launch_site_alt_msl;
    float rho = Atmosphere::density(alt_msl);

    // Drag force: 0.5 * rho * v^2 * Cd * A
    // Always opposes velocity (hence -sign(v))
    float drag_accel = 0.5f * rho * v * fabsf(v) * cd_total *
                       RocketConfig::REF_AREA_M2 / RocketConfig::MASS_KG;

    dh = v;
    dv = -drag_accel - RocketConfig::G;
}

float ApogeePredictor::simulateCoast(float alt_agl, float vel, float cd_add,
                                      float launch_site_alt_msl) {
    float h = alt_agl;
    float v = vel;
    float cd_total = RocketConfig::BODY_CD + cd_add;
    float dt = RocketConfig::RK4_DT;
    float max_alt = h;
    float t = 0.0f;

    while (v > 0.0f && t < RocketConfig::MAX_SIM_TIME) {
        // RK4 integration
        float k1h, k1v, k2h, k2v, k3h, k3v, k4h, k4v;

        derivatives(h, v, cd_total, launch_site_alt_msl, k1h, k1v);
        derivatives(h + 0.5f * dt * k1h, v + 0.5f * dt * k1v,
                    cd_total, launch_site_alt_msl, k2h, k2v);
        derivatives(h + 0.5f * dt * k2h, v + 0.5f * dt * k2v,
                    cd_total, launch_site_alt_msl, k3h, k3v);
        derivatives(h + dt * k3h, v + dt * k3v,
                    cd_total, launch_site_alt_msl, k4h, k4v);

        h += (dt / 6.0f) * (k1h + 2.0f * k2h + 2.0f * k3h + k4h);
        v += (dt / 6.0f) * (k1v + 2.0f * k2v + 2.0f * k3v + k4v);

        if (h > max_alt) max_alt = h;
        t += dt;
    }

    return max_alt;
}

ApogeePredictor::Result ApogeePredictor::predict(float alt_agl, float vel,
                                                   float launch_site_alt_msl,
                                                   float target_alt_agl) {
    Result result;

    // If descending or very slow, no point predicting
    if (vel <= 0.0f) {
        result.predicted_apogee = alt_agl;
        result.cd_add = 0.0f;
        return result;
    }

    // Binary search for Cd_add that achieves target apogee
    float cd_low = 0.0f;
    float cd_high = RocketConfig::MAX_CD_ADD;

    // First check: can we even reach target with no brakes?
    float apogee_no_brakes = simulateCoast(alt_agl, vel, 0.0f,
                                            launch_site_alt_msl);

    if (apogee_no_brakes <= target_alt_agl) {
        // Can't reach target even without brakes — retract
        result.predicted_apogee = apogee_no_brakes;
        result.cd_add = 0.0f;
        return result;
    }

    // Check: does max braking undershoot?
    float apogee_max_brakes = simulateCoast(alt_agl, vel, cd_high,
                                             launch_site_alt_msl);

    if (apogee_max_brakes >= target_alt_agl) {
        // Even max brakes overshoot — deploy fully
        result.predicted_apogee = apogee_max_brakes;
        result.cd_add = cd_high;
        return result;
    }

    // Binary search: find Cd_add where predicted apogee == target
    // Higher Cd → lower apogee, so search is inverted
    for (int i = 0; i < RocketConfig::BINARY_SEARCH_ITERS; i++) {
        float cd_mid = 0.5f * (cd_low + cd_high);
        float apogee_mid = simulateCoast(alt_agl, vel, cd_mid,
                                          launch_site_alt_msl);

        if (apogee_mid > target_alt_agl) {
            cd_low = cd_mid;  // Need more drag
        } else {
            cd_high = cd_mid; // Need less drag
        }
    }

    result.cd_add = 0.5f * (cd_low + cd_high);
    result.predicted_apogee = simulateCoast(alt_agl, vel, result.cd_add,
                                             launch_site_alt_msl);
    return result;
}
