#pragma once

struct ServoAngles {
    float angle_1;  // Servo 1 percentage (0-100)
    float angle_2;  // Servo 2 percentage (0-100)
};

namespace CdLookup {
    // Convert additional Cd command to servo angles
    // cd_add: 0.0 to MAX_CD_ADD
    // Returns both servo angles (mirrored for now)
    ServoAngles cdToServoAngles(float cd_add);

    // Convert servo angle to Cd_add (inverse lookup)
    float servoAngleToCd(float angle_pct);
}
