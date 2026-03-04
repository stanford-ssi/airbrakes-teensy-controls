#pragma once
#include <stdint.h>

// Dynamic measurement noise (R values) based on flight phase and velocity
// Higher R = less trust in that sensor

struct SensorWeights {
    float R_accel_low;   // ADXL345 (±16g) measurement noise variance
    float R_accel_high;  // ADXL375 (±200g) measurement noise variance
    float R_baro;        // Barometer altitude measurement noise variance
};

namespace SensorWeighting {
    // Flight states (must match STM32 States enum)
    enum FlightState : uint8_t {
        BOOT = 0,
        SENSOR_ERROR = 1,
        IDLE = 2,
        AIRBRAKE_TEST = 3,
        IGNITION = 4,
        ASCENT = 5,
        APOGEE = 6,
        DESCENT = 7,
        LANDED = 8
    };

    // Get sensor weights based on flight state and estimated velocity
    // speed_of_sound: local speed of sound at current altitude (m/s)
    SensorWeights getWeights(uint8_t flight_state, float velocity,
                             float speed_of_sound);
}
