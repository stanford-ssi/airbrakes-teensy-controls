#include "SensorWeighting.h"
#include "RocketConfig.h"
#include <math.h>

SensorWeights SensorWeighting::getWeights(uint8_t flight_state, float velocity,
                                           float speed_of_sound) {
    SensorWeights w;
    float mach = fabsf(velocity) / speed_of_sound;

    switch (flight_state) {
    case IDLE:
    case BOOT:
    case AIRBRAKE_TEST:
    case LANDED:
        // Pad/ground readings: low-g accel and baro are the clean sensors.
        w.R_accel_low  = 0.5f;
        w.R_accel_high = 100.0f;
        w.R_baro       = 1.0f;
        break;

    case IGNITION:
        // Motor burn: high-g accel carries the estimate.
        w.R_accel_low  = 1000.0f;
        w.R_accel_high = 1.0f;
        w.R_baro       = 50.0f;
        break;

    case ASCENT:
        if (mach > RocketConfig::MACH_TRANSONIC_LOW) {
            w.R_accel_low  = 2.0f;
            w.R_accel_high = 1.0f;
            w.R_baro       = 1000.0f;
        } else if (fabsf(velocity) < 20.0f) {
            w.R_accel_low  = 0.5f;
            w.R_accel_high = 50.0f;
            w.R_baro       = 0.5f;
        } else {
            w.R_accel_low  = 1.0f;
            w.R_accel_high = 3.0f;
            w.R_baro       = 2.0f;
        }
        break;

    case APOGEE:
    case DESCENT:
        w.R_accel_low  = 1.0f;
        w.R_accel_high = 100.0f;
        w.R_baro       = 0.5f;
        break;

    default:
        w.R_accel_low  = 5.0f;
        w.R_accel_high = 5.0f;
        w.R_baro       = 5.0f;
        break;
    }

    return w;
}
