#pragma once

class ApogeePredictor {
public:
    struct Result {
        float predicted_apogee;  // meters AGL
        float cd_add;            // additional Cd to hit target
        // Diagnostics — populated every predict() call so the dashboard can
        // cross-check the Teensy's predictor against the Python port.
        float apo_no_brakes;     // apogee if cd_add = 0
        float apo_max_brakes;    // apogee if cd_add = MAX_CD_ADD
    };

    ApogeePredictor();

    // Predict apogee with current state, find Cd_add to hit target
    // alt_agl: current altitude above ground (m)
    // vel: current vertical velocity (m/s, positive up)
    // launch_site_alt_msl: launch site altitude MSL (m)
    // target_alt_agl: target apogee AGL (m)
    Result predict(float alt_agl, float vel, float launch_site_alt_msl,
                   float target_alt_agl);

    // Single forward RK4 coast sim with the given Cd_add. Returns predicted
    // apogee AGL. Used by the controller to populate a display apogee during
    // retracted phases (post-launch delay, supersonic lockout, post-apogee),
    // so the dashboard always has a live estimate instead of falling back to
    // current altitude.
    float simulateCoast(float alt_agl, float vel, float cd_add,
                        float launch_site_alt_msl);

private:
    // RK4 derivatives: dh/dt = v, dv/dt = -(drag/m) - g
    void derivatives(float h_agl, float v, float cd_total,
                     float launch_site_alt_msl,
                     float &dh, float &dv);
};
