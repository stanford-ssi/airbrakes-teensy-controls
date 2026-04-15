#pragma once
#include <stdint.h>
#include "ApogeePredictor.h"

class AirbrakeController {
public:
    enum State : uint8_t {
        CTRL_IDLE = 0,      // Not active
        CTRL_ACTIVE = 1,    // Running predictor
        CTRL_RETRACTED = 2, // Forced retract
        CTRL_FULL = 3       // Forced full brakes
    };

    AirbrakeController();

    // Main update: call every loop iteration
    // Returns the commanded Cd_add
    // alt_agl: altitude above ground (m)
    // vel: vertical velocity (m/s, positive up)
    // accel: vertical acceleration (m/s²)
    // flight_state: from STM32 (States enum)
    // speed_of_sound: at current altitude (m/s)
    // time_s: current time in seconds
    // dt: time since last call (s)
    float update(float alt_agl, float vel, float accel,
                 uint8_t flight_state, float speed_of_sound,
                 float time_s, float dt);

    float predictedApogee() const { return predicted_apogee_; }
    State controllerState() const { return state_; }

private:
    ApogeePredictor predictor_;
    State state_;
    float cd_add_cmd_;          // Current commanded Cd_add
    float predicted_apogee_;
    float apogee_time_;         // Time when apogee detected (s)
    bool apogee_detected_;
    float launch_time_;         // Time when ignition first detected (s)
    bool launch_detected_;
};
