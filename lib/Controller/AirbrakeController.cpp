#include "AirbrakeController.h"
#include "RocketConfig.h"
#include <math.h>

AirbrakeController::AirbrakeController()
    : state_(CTRL_IDLE), cd_add_cmd_(0.0f), predicted_apogee_(0.0f),
      apogee_time_(0.0f), apogee_detected_(false) {}

float AirbrakeController::update(float alt_agl, float vel, float accel,
                                  uint8_t flight_state, float speed_of_sound,
                                  float time_s, float dt) {
    float mach = fabsf(vel) / speed_of_sound;
    float target_cd = 0.0f;

    // Rule 1: Motor still burning → retract
    if (flight_state <= 4) { // BOOT..IGNITION
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        predicted_apogee_ = 0.0f;
        return cd_add_cmd_;
    }

    // Rule 2: Supersonic → retract (structural safety)
    if (mach > RocketConfig::MACH_SUPERSONIC) {
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        return cd_add_cmd_;
    }

    // Rule 3: Descending — retract after delay
    if (vel < -RocketConfig::APOGEE_VELOCITY_THRESHOLD) {
        if (!apogee_detected_) {
            apogee_detected_ = true;
            apogee_time_ = time_s;
        }
        if (time_s - apogee_time_ > RocketConfig::POST_APOGEE_RETRACT_DELAY_S) {
            state_ = CTRL_RETRACTED;
            cd_add_cmd_ = 0.0f;
            return cd_add_cmd_;
        }
    }

    // Rule 4: Already above target and ascending → full brakes
    if (alt_agl > RocketConfig::TARGET_ALT_AGL_M && vel > 0.0f) {
        state_ = CTRL_FULL;
        cd_add_cmd_ = RocketConfig::MAX_CD_ADD;
        predicted_apogee_ = alt_agl; // Already past target
        return cd_add_cmd_;
    }

    // Rule 5: Normal coast — run predictor
    ApogeePredictor::Result result = predictor_.predict(
        alt_agl, vel, RocketConfig::LAUNCH_SITE_ALT_MSL_M,
        RocketConfig::TARGET_ALT_AGL_M);

    predicted_apogee_ = result.predicted_apogee;
    target_cd = result.cd_add;

    // Apply slew rate limit
    float max_change = RocketConfig::CD_SLEW_RATE_MAX * dt;
    float delta = target_cd - cd_add_cmd_;
    if (delta > max_change) delta = max_change;
    if (delta < -max_change) delta = -max_change;
    cd_add_cmd_ += delta;

    // Clamp
    if (cd_add_cmd_ < 0.0f) cd_add_cmd_ = 0.0f;
    if (cd_add_cmd_ > RocketConfig::MAX_CD_ADD) cd_add_cmd_ = RocketConfig::MAX_CD_ADD;

    state_ = CTRL_ACTIVE;
    return cd_add_cmd_;
}
