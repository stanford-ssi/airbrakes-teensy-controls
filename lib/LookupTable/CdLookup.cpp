#include "CdLookup.h"
#include "RocketConfig.h"
#include "V2BodyCd.h"
#include <math.h>

// Drag model — matches Airbrakes_SHITL / new-collins-airbrakes RocketPy:
//
//   total_cd(angle, mach) = body_cd(mach) + airbrake_cd_add(angle)     (mach <  1.0)
//                         = body_cd(mach)                              (mach >= 1.0)
//
//   body_cd(mach) : linear interpolation on V2BodyCd::MACH/CD (REF CSV data).
//   airbrake_cd_add(angle_deg) : flat-plate drag, linear in projected area.
//       = AIRBRAKE_CD × area(angle) / REF_AREA_M2
//       = AIRBRAKE_CD × AIRBRAKE_MAX_AREA_M2 × (angle/95°) / REF_AREA_M2
//
// ANGLE_MIN=1° ↔ SERVO_MIN_PCT (retracted), ANGLE_MAX=95° ↔ SERVO_MAX_PCT
// (full deploy). Above MACH_DEPLOY_LIMIT the brakes produce zero additional Cd
// because they are not permitted to deploy at supersonic speeds.

static constexpr float ANGLE_MIN = 1.0f;
static constexpr float ANGLE_MAX = 95.0f;

// Linear lookup on the sorted REF body-Cd table. Matches numpy.interp
// behaviour (clamps to edge values outside the Mach range).
static float bodyCdLerp(float mach) {
    const float* M = V2BodyCd::MACH;
    const float* C = V2BodyCd::CD;
    const int N = V2BodyCd::N;

    if (mach <= M[0])      return C[0];
    if (mach >= M[N - 1])  return C[N - 1];

    int lo = 0;
    int hi = N - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (M[mid] <= mach) lo = mid;
        else                hi = mid;
    }
    float denom = M[hi] - M[lo];
    if (denom <= 0.0f) return C[lo];
    float t = (mach - M[lo]) / denom;
    return C[lo] + t * (C[hi] - C[lo]);
}

float CdLookup::bodyCd(float mach) {
    return bodyCdLerp(mach);
}

// Flat-plate airbrake additive Cd (referenced to body area), as a function of
// deployment angle. Zero at/above MACH_DEPLOY_LIMIT so the predictor sees the
// same gating the Python controller uses.
static float airbrakeCdAdd(float angle_deg, float mach) {
    if (mach >= RocketConfig::MACH_DEPLOY_LIMIT) return 0.0f;
    if (angle_deg <= ANGLE_MIN) return 0.0f;
    if (angle_deg > ANGLE_MAX) angle_deg = ANGLE_MAX;
    // Projected area scales linearly from 0 at ANGLE_MIN to AIRBRAKE_MAX_AREA_M2
    // at ANGLE_MAX (the Python model rounds ANGLE_MIN down to zero deployment
    // since servo_pct > SERVO_MIN_PCT is the deploy gate).
    float t = (angle_deg - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float area = t * RocketConfig::AIRBRAKE_MAX_AREA_M2;
    return RocketConfig::AIRBRAKE_CD * area / RocketConfig::REF_AREA_M2;
}

float CdLookup::lookupCd(float angle_deg, float mach) {
    return bodyCdLerp(mach) + airbrakeCdAdd(angle_deg, mach);
}

// Inverse of the airbrake drag model. cd_add is the controller's desired
// additive drag coefficient (referenced to body area). With the flat-plate
// model this inverts analytically — no search needed.
static float inverseAirbrakeAngleDeg(float cd_add) {
    if (cd_add <= 0.0f) return ANGLE_MIN;
    // cd_add = AIRBRAKE_CD × (AIRBRAKE_MAX_AREA × t) / REF_AREA  where t = (angle-1)/94
    // t = cd_add × REF_AREA / (AIRBRAKE_CD × AIRBRAKE_MAX_AREA)
    float t = (cd_add * RocketConfig::REF_AREA_M2) /
              (RocketConfig::AIRBRAKE_CD * RocketConfig::AIRBRAKE_MAX_AREA_M2);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return ANGLE_MIN + t * (ANGLE_MAX - ANGLE_MIN);
}

float CdLookup::angleDegToServoPct(float angle_deg) {
    // Linear mapping: ANGLE_MIN -> SERVO_MIN_PCT, ANGLE_MAX -> SERVO_MAX_PCT
    float t = (angle_deg - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return RocketConfig::SERVO_MIN_PCT + t * (RocketConfig::SERVO_MAX_PCT - RocketConfig::SERVO_MIN_PCT);
}

float CdLookup::servoPctToAngleDeg(float servo_pct) {
    const float span = RocketConfig::SERVO_MAX_PCT - RocketConfig::SERVO_MIN_PCT;
    if (span <= 0.0f) return ANGLE_MIN;
    float t = (servo_pct - RocketConfig::SERVO_MIN_PCT) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return ANGLE_MIN + t * (ANGLE_MAX - ANGLE_MIN);
}

ServoAngles CdLookup::cdToServoAngles(float cd_add, float mach) {
    ServoAngles result;

    // Supersonic or negative command → retract (matches Python controller
    // behaviour at shitl_dashboard.py:464).
    if (cd_add <= 0.0f || mach >= RocketConfig::MACH_DEPLOY_LIMIT) {
        result.angle_1 = RocketConfig::SERVO_MIN_PCT;
        result.angle_2 = RocketConfig::SERVO_MIN_PCT;
        return result;
    }

    float angle_deg = inverseAirbrakeAngleDeg(cd_add);
    float servo_pct = angleDegToServoPct(angle_deg);
    if (servo_pct < RocketConfig::SERVO_MIN_PCT) servo_pct = RocketConfig::SERVO_MIN_PCT;

    result.angle_1 = servo_pct;
    result.angle_2 = servo_pct;  // Mirrored
    return result;
}
