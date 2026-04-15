#pragma once

struct ServoAngles {
    float angle_1;  // Servo 1 percentage (0-100)
    float angle_2;  // Servo 2 percentage (0-100)
};

namespace CdLookup {
    // Look up total Cd for a given airbrake angle (degrees) and Mach number
    // Uses 2D bilinear interpolation on CFD table
    float lookupCd(float angle_deg, float mach);

    // Get body Cd (airbrakes retracted, angle=1°) at given Mach
    float bodyCd(float mach);

    // Convert additional Cd command to servo angles at given Mach
    // cd_add: additional Cd above body Cd
    // mach: current Mach number
    // Returns both servo angles (mirrored for now)
    ServoAngles cdToServoAngles(float cd_add, float mach);

    // Convert servo angle percentage to airbrake angle in degrees
    float servoPctToAngleDeg(float servo_pct);

    // Convert airbrake angle in degrees to servo percentage
    float angleDegToServoPct(float angle_deg);
}
