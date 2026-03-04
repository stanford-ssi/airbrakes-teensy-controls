#pragma once

// US Standard Atmosphere 1976 (troposphere only, valid to ~11km)
namespace Atmosphere {
    // Returns air density (kg/m³) at given altitude MSL (meters)
    float density(float altitude_msl);

    // Returns speed of sound (m/s) at given altitude MSL (meters)
    float speedOfSound(float altitude_msl);

    // Returns temperature (K) at given altitude MSL (meters)
    float temperature(float altitude_msl);

    // Returns pressure (Pa) at given altitude MSL (meters)
    float pressure(float altitude_msl);
}
