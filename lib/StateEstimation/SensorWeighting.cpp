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
        // On the ground: trust low-g accel and baro, don't need high-g
        w.R_accel_low  = 0.5f;
        w.R_accel_high = 100.0f;  // Don't trust (noisy at low g)
        w.R_baro       = 1.0f;
        break;

    case IGNITION:
        // Motor burning: low-g saturated, baro unreliable at high velocity
        w.R_accel_low  = 1000.0f; // Saturated, don't trust
        w.R_accel_high = 1.0f;    // Trust high-g
        w.R_baro       = 50.0f;   // Don't trust baro during motor burn
        break;

    case ASCENT:
        if (mach > RocketConfig::MACH_TRANSONIC_LOW) {
            // High Mach: baro corrupted by shocks, lean on accelerometers
            w.R_accel_low  = 2.0f;
            w.R_accel_high = 1.0f;
            w.R_baro       = 1000.0f;
        } else if (fabsf(velocity) < 20.0f) {
            // Near apogee: low velocity, trust baro most
            w.R_accel_low  = 0.5f;
            w.R_accel_high = 50.0f;  // High-g too noisy at low accel
            w.R_baro       = 0.5f;   // Trust baro near apogee
        } else {
            // Normal subsonic coast
            w.R_accel_low  = 1.0f;
            w.R_accel_high = 3.0f;
            w.R_baro       = 2.0f;
        }
        break;

    case APOGEE:
    case DESCENT:
        // Post-apogee: trust baro, trust low-g accel
        w.R_accel_low  = 1.0f;
        w.R_accel_high = 100.0f;
        w.R_baro       = 0.5f;
        break;

    default:
        // Safe defaults
        w.R_accel_low  = 5.0f;
        w.R_accel_high = 5.0f;
        w.R_baro       = 5.0f;
        break;
    }

    return w;
}
