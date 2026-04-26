#include "AirbrakeController.h"
#include "RocketConfig.h"
#include <math.h>

AirbrakeController::AirbrakeController()
    : state_(CTRL_IDLE), cd_add_cmd_(0.0f), predicted_apogee_(0.0f),
      apogee_time_(0.0f), apogee_detected_(false),
      launch_time_(0.0f), launch_detected_(false) {}

float AirbrakeController::update(float alt_agl, float vel, float accel,
                                  uint8_t flight_state, float speed_of_sound,
                                  float time_s, float dt) {
    float mach = fabsf(vel) / speed_of_sound;
    float target_cd = 0.0f;

    // Track launch time from first ignition
    if (flight_state >= 4 && !launch_detected_) {
        launch_detected_ = true;
        launch_time_ = time_s;
    }

    // Apogee detection runs before early returns so it's never skipped
    if (vel < -RocketConfig::APOGEE_VELOCITY_THRESHOLD && !apogee_detected_) {
        apogee_detected_ = true;
        apogee_time_ = time_s;
    }

    // Helper: run a single forward coast sim using the brakes' *current*
    // commanded Cd. Keeps predicted_apogee_ live during retracted phases so
    // the dashboard shows the same no-brakes/live prediction the Python demo
    // does, instead of falling back to current altitude.
    auto displayApogee = [&]() -> float {
        if (vel <= 0.0f) return alt_agl;
        return predictor_.simulateCoast(alt_agl, vel, cd_add_cmd_,
                                         RocketConfig::LAUNCH_SITE_ALT_MSL_M);
    };

    // Motor still burning, keep brakes retracted
    if (flight_state <= 4) { // BOOT..IGNITION
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        predicted_apogee_ = displayApogee();
        return cd_add_cmd_;
    }

    // Post-launch delay, stay retracted through supersonic
    if (launch_detected_ &&
        (time_s - launch_time_) < RocketConfig::POST_LAUNCH_DELAY_S) {
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        predicted_apogee_ = displayApogee();
        return cd_add_cmd_;
    }

    // Post-apogee recovery, retract after delay
    if (apogee_detected_ &&
        (time_s - apogee_time_) > RocketConfig::POST_APOGEE_RETRACT_DELAY_S) {
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        predicted_apogee_ = displayApogee();
        return cd_add_cmd_;
    }

    // Supersonic lockout, structural risk if deployed
    if (mach > RocketConfig::MACH_SUPERSONIC) {
        state_ = CTRL_RETRACTED;
        cd_add_cmd_ = 0.0f;
        predicted_apogee_ = displayApogee();
        return cd_add_cmd_;
    }

    // Above target and still ascending, deploy full brakes immediately.
    // Honours target_alt_ so the fallback-target latch dumps brakes the
    // moment we cross the lowered target, not the original primary.
    if (alt_agl > target_alt_ && vel > 0.0f) {
        state_ = CTRL_FULL;
        cd_add_cmd_ = RocketConfig::MAX_CD_ADD;
        predicted_apogee_ = alt_agl;
        return cd_add_cmd_;
    }

    // Descending, command full brakes (slew-limited below)
    if (vel <= 0.0f) {
        state_ = CTRL_FULL;
        predicted_apogee_ = alt_agl;
        target_cd = RocketConfig::MAX_CD_ADD;
    } else {
        // Subsonic coast, run apogee predictor to find required Cd_add.
        // Uses target_alt_ rather than the constant so the fallback-target
        // latch retargets the search without touching the controller body.
        ApogeePredictor::Result result = predictor_.predict(
            alt_agl, vel, RocketConfig::LAUNCH_SITE_ALT_MSL_M,
            target_alt_);

        predicted_apogee_ = result.predicted_apogee;
        target_cd = result.cd_add;
        last_target_cd_raw_ = result.cd_add;
        last_apo_no_ = result.apo_no_brakes;
        last_apo_max_ = result.apo_max_brakes;
        state_ = CTRL_ACTIVE;
    }

    // Apply slew rate limit
    float max_change = RocketConfig::CD_SLEW_RATE_MAX * dt;
    float delta = target_cd - cd_add_cmd_;
    if (delta > max_change) delta = max_change;
    if (delta < -max_change) delta = -max_change;
    cd_add_cmd_ += delta;

    // Clamp
    if (cd_add_cmd_ < 0.0f) cd_add_cmd_ = 0.0f;
    if (cd_add_cmd_ > RocketConfig::MAX_CD_ADD) cd_add_cmd_ = RocketConfig::MAX_CD_ADD;

    return cd_add_cmd_;
}
