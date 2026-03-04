#include "Atmosphere.h"
#include <math.h>

// US Standard Atmosphere 1976 constants (troposphere, 0-11 km)
static constexpr float T0 = 288.15f;    // Sea-level temperature (K)
static constexpr float P0 = 101325.0f;  // Sea-level pressure (Pa)
static constexpr float RHO0 = 1.225f;   // Sea-level density (kg/m³)
static constexpr float L = 0.0065f;     // Temperature lapse rate (K/m)
static constexpr float g = 9.80665f;    // Gravitational acceleration (m/s²)
static constexpr float R = 287.05f;     // Specific gas constant for dry air (J/(kg·K))
static constexpr float GAMMA = 1.4f;    // Ratio of specific heats

float Atmosphere::temperature(float altitude_msl) {
    return T0 - L * altitude_msl;
}

float Atmosphere::pressure(float altitude_msl) {
    float T = temperature(altitude_msl);
    return P0 * powf(T / T0, g / (L * R));
}

float Atmosphere::density(float altitude_msl) {
    float T = temperature(altitude_msl);
    float P = pressure(altitude_msl);
    return P / (R * T);
}

float Atmosphere::speedOfSound(float altitude_msl) {
    float T = temperature(altitude_msl);
    return sqrtf(GAMMA * R * T);
}
