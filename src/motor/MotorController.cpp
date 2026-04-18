#include "motor/MotorController.hpp"
#include <cstdio>
#include <algorithm>

MotorController::MotorController(PCA9685& pwm)
    : pwm_(pwm) {}

void MotorController::init() {
    close_angle_ = CLOSE_SERVO_OPEN;
    open_angle_  = OPEN_SERVO_OPEN;
    state_       = HandState::OPEN;
    applyAngles();
    printf("MotorController: initialised — hand OPEN\n");
}

void MotorController::update(float ratio) {
    if (ratio > ENGAGE_RATIO) {
        // ── CLOSING ───────────────────────────────────────────────────────────
        state_ = HandState::CLOSING;
        float speed = speedFromRatio(ratio);

        close_angle_ = std::min(close_angle_ + speed, CLOSE_SERVO_CLOSED);
        open_angle_  = std::max(open_angle_  - speed, OPEN_SERVO_CLOSED);

        applyAngles();
        printf("CLOSING | ratio: %.2fx | speed: %.2f°/s | close: %.1f° | open: %.1f°\n",
               ratio, speed, close_angle_, open_angle_);

    } else if (ratio < RELEASE_RATIO) {
        // ── OPENING ───────────────────────────────────────────────────────────
        state_ = HandState::OPENING;
        float speed = speedFromRatio(ENGAGE_RATIO);  // open at steady speed

        close_angle_ = std::max(close_angle_ - speed, CLOSE_SERVO_OPEN);
        open_angle_  = std::min(open_angle_  + speed, OPEN_SERVO_OPEN);

        applyAngles();
        printf("OPENING  | ratio: %.2fx | speed: %.2f°/s | close: %.1f° | open: %.1f°\n",
               ratio, speed, close_angle_, open_angle_);

    } else {
        // ── HOLDING (hysteresis band 1.2× – 1.5×) ────────────────────────────
        state_ = HandState::HOLDING;
    }
}

float MotorController::speedFromRatio(float ratio) const {
    // Linear scale: 0 speed at ENGAGE_RATIO, MAX_SPEED at MAX_RATIO
    float t = (ratio - ENGAGE_RATIO) / (MAX_RATIO - ENGAGE_RATIO);
    t = std::max(0.0f, std::min(t, 1.0f));  // clamp 0–1
    return t * MAX_SPEED_DEG;
}

void MotorController::applyAngles() {
    pwm_.setAngle(CLOSE_CHANNEL, close_angle_);
    pwm_.setAngle(OPEN_CHANNEL,  open_angle_);
}
