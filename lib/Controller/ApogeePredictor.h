#pragma once

class ApogeePredictor {
public:
    struct Result {
        float predicted_apogee;  // meters AGL
        float cd_add;            // additional Cd to hit target
    };

    ApogeePredictor();

    // Predict apogee with current state, find Cd_add to hit target
    // alt_agl: current altitude above ground (m)
    // vel: current vertical velocity (m/s, positive up)
    // launch_site_alt_msl: launch site altitude MSL (m)
    // target_alt_agl: target apogee AGL (m)
    Result predict(float alt_agl, float vel, float launch_site_alt_msl,
                   float target_alt_agl);

private:
    // Simulate coast trajectory with given Cd_add, return max altitude AGL
    float simulateCoast(float alt_agl, float vel, float cd_add,
                        float launch_site_alt_msl);

    // RK4 derivatives: dh/dt = v, dv/dt = -(drag/m) - g
    void derivatives(float h_agl, float v, float cd_total,
                     float launch_site_alt_msl,
                     float &dh, float &dv);
};
