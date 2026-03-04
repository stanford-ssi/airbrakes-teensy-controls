#include "CdLookup.h"
#include "RocketConfig.h"

// Placeholder lookup table: linear mapping
// Cd_add 0.0 → servo 10% (retracted)
// Cd_add MAX_CD_ADD (1.0) → servo 60% (max deployed)
//
// When real wind tunnel / CFD data is available, replace these arrays
// with the actual Cd vs. servo angle relationship.

static constexpr int TABLE_SIZE = 5;

static constexpr float cd_table[TABLE_SIZE] = {
    0.00f, 0.25f, 0.50f, 0.75f, 1.00f
};

static constexpr float angle_table[TABLE_SIZE] = {
    10.0f, 22.5f, 35.0f, 47.5f, 60.0f
};

static float interpolate(const float *x_table, const float *y_table,
                          int size, float x) {
    // Clamp to table bounds
    if (x <= x_table[0]) return y_table[0];
    if (x >= x_table[size - 1]) return y_table[size - 1];

    // Find bracketing interval
    for (int i = 0; i < size - 1; i++) {
        if (x >= x_table[i] && x <= x_table[i + 1]) {
            float t = (x - x_table[i]) / (x_table[i + 1] - x_table[i]);
            return y_table[i] + t * (y_table[i + 1] - y_table[i]);
        }
    }

    return y_table[size - 1];
}

ServoAngles CdLookup::cdToServoAngles(float cd_add) {
    // Clamp input
    if (cd_add < 0.0f) cd_add = 0.0f;
    if (cd_add > RocketConfig::MAX_CD_ADD) cd_add = RocketConfig::MAX_CD_ADD;

    float angle = interpolate(cd_table, angle_table, TABLE_SIZE, cd_add);

    ServoAngles result;
    result.angle_1 = angle;
    result.angle_2 = angle; // Mirrored for now
    return result;
}

float CdLookup::servoAngleToCd(float angle_pct) {
    if (angle_pct < angle_table[0]) angle_pct = angle_table[0];
    if (angle_pct > angle_table[TABLE_SIZE - 1])
        angle_pct = angle_table[TABLE_SIZE - 1];

    return interpolate(angle_table, cd_table, TABLE_SIZE, angle_pct);
}
