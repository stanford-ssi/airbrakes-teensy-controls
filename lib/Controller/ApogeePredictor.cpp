#include "ApogeePredictor.h"
#include "Atmosphere.h"
#include "CdLookup.h"
#include "RocketConfig.h"
#include <math.h>

ApogeePredictor::ApogeePredictor() {}

void ApogeePredictor::derivatives(float h_agl, float v, float cd_total,
                                   float launch_site_alt_msl,
                                   float &dh, float &dv) {
    float alt_msl = h_agl + launch_site_alt_msl;
    float rho = Atmosphere::density(alt_msl);

    // Drag acceleration opposes velocity: F_drag = 0.5 * rho * v^2 * Cd * A
    float drag_accel = 0.5f * rho * v * fabsf(v) * cd_total *
                       RocketConfig::REF_AREA_M2 / RocketConfig::MASS_KG;

    dh = v;
    dv = -drag_accel - RocketConfig::G;
}

float ApogeePredictor::simulateCoast(float alt_agl, float vel, float cd_add,
                                      float launch_site_alt_msl) {
    float h = alt_agl;
    float v = vel;
    float dt = RocketConfig::RK4_DT;
    float t = 0.0f;

    // Body Cd is Mach-dependent, recompute as the rocket decelerates
    float alt_msl = h + launch_site_alt_msl;
    float sos = Atmosphere::speedOfSound(alt_msl);
    float mach = fabsf(v) / sos;
    float cd_total = CdLookup::bodyCd(mach) + cd_add;

    int step = 0;

    while (v > 0.0f && t < RocketConfig::MAX_SIM_TIME) {
        // Refresh Cd every 5 RK4 steps to track Mach changes
        if (step % 5 == 0) {
            alt_msl = h + launch_site_alt_msl;
            sos = Atmosphere::speedOfSound(alt_msl);
            mach = fabsf(v) / sos;
            cd_total = CdLookup::bodyCd(mach) + cd_add;
        }

        // RK4 integration
        float k1h, k1v, k2h, k2v, k3h, k3v, k4h, k4v;

        derivatives(h, v, cd_total, launch_site_alt_msl, k1h, k1v);
        derivatives(h + 0.5f * dt * k1h, v + 0.5f * dt * k1v,
                    cd_total, launch_site_alt_msl, k2h, k2v);
        derivatives(h + 0.5f * dt * k2h, v + 0.5f * dt * k2v,
                    cd_total, launch_site_alt_msl, k3h, k3v);
        derivatives(h + dt * k3h, v + dt * k3v,
                    cd_total, launch_site_alt_msl, k4h, k4v);

        float h_new = h + (dt / 6.0f) * (k1h + 2.0f * k2h + 2.0f * k3h + k4h);
        float v_new = v + (dt / 6.0f) * (k1v + 2.0f * k2v + 2.0f * k3v + k4v);

        // Velocity crossed zero, interpolate to find exact apogee altitude
        if (v_new <= 0.0f) {
            float rho = Atmosphere::density(h + launch_site_alt_msl);
            float drag_accel = 0.5f * rho * v * fabsf(v) * cd_total *
                               RocketConfig::REF_AREA_M2 / RocketConfig::MASS_KG;
            float a = -drag_accel - RocketConfig::G;

            if (a != 0.0f) {
                // Solve t where v=0: t_zero = -v/a
                float t_zero = -v / a;
                if (t_zero < 0.0f) t_zero = 0.0f;
                if (t_zero > dt) t_zero = dt;
                return h + v * t_zero + 0.5f * a * t_zero * t_zero;
            }
            return h;
        }

        h = h_new;
        v = v_new;
        t += dt;
        step++;
    }

    return h;
}

ApogeePredictor::Result ApogeePredictor::predict(float alt_agl, float vel,
                                                   float launch_site_alt_msl,
                                                   float target_alt_agl) {
    Result result;

    // Already descending, nothing to predict
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
        // Can't reach target even without brakes, retract
        result.predicted_apogee = apogee_no_brakes;
        result.cd_add = 0.0f;
        return result;
    }

    // Check: does max braking undershoot?
    float apogee_max_brakes = simulateCoast(alt_agl, vel, cd_high,
                                             launch_site_alt_msl);

    if (apogee_max_brakes >= target_alt_agl) {
        // Even full brakes overshoot target, deploy max
        result.predicted_apogee = apogee_max_brakes;
        result.cd_add = cd_high;
        return result;
    }

    // Binary search: higher Cd = lower apogee, so search is inverted
    float cd_mid = 0.0f;
    float apogee_mid = 0.0f;

    for (int i = 0; i < RocketConfig::BINARY_SEARCH_ITERS; i++) {
        cd_mid = 0.5f * (cd_low + cd_high);
        apogee_mid = simulateCoast(alt_agl, vel, cd_mid, launch_site_alt_msl);

        // Converged within 1m tolerance, exit early
        if (fabsf(apogee_mid - target_alt_agl) <= 1.0f) {
            result.cd_add = cd_mid;
            result.predicted_apogee = apogee_mid;
            return result;
        }

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
