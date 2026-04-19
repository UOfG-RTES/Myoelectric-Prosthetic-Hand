#pragma once

#include "motor/PCA9685.hpp"
#include <functional>

/**
 * @brief Velocity-based two-servo hand controller driven by EMG ratio.
 *
 * Two servos operate antagonistically:
 *   - Close servo (ch 0): pulls fingers → hand closes
 *   - Open  servo (ch 2): releases fingers → hand opens
 *
 * ── Control logic ────────────────────────────────────────────────────────────
 *  ratio > ENGAGE_RATIO  : CLOSING — speed ∝ (ratio − ENGAGE_RATIO)
 *  ratio < RELEASE_RATIO : OPENING — move back to open position
 *  between the two       : HOLD    — hysteresis band, no movement
 *
 * ── Velocity ─────────────────────────────────────────────────────────────────
 *  Speed is proportional to how far the ratio exceeds the engage threshold.
 *  At MAX_RATIO the servo moves at MAX_SPEED_DEG deg/sample.
 *  At 860 SPS, MAX_SPEED_DEG = 0.3 → ~258°/s → full 180° travel in ~0.7 s.
 * ────────────────────────────────────────────────────────────────────────────
 */
class MotorController {
public:
    // ── Tuning constants ──────────────────────────────────────────────────────
    static constexpr float ENGAGE_RATIO    = 1.5f;  ///< ratio to start closing
    static constexpr float RELEASE_RATIO   = 1.2f;  ///< ratio to start opening
    static constexpr float MAX_RATIO       = 6.0f;  ///< ratio at which max speed is reached
    static constexpr float MAX_SPEED_DEG   = 0.3f;  ///< degrees moved per sample at max ratio

    // ── Servo angle limits (tune after physical calibration) ──────────────────
    static constexpr float CLOSE_SERVO_OPEN   = 0.0f;   ///< close servo angle → hand open
    static constexpr float CLOSE_SERVO_CLOSED = 180.0f; ///< close servo angle → hand closed
    static constexpr float OPEN_SERVO_OPEN    = 180.0f; ///< open servo angle → hand open
    static constexpr float OPEN_SERVO_CLOSED  = 0.0f;   ///< open servo angle → hand closed

    // ── Channel assignments ───────────────────────────────────────────────────
    static constexpr int CLOSE_CHANNEL = 0;
    static constexpr int OPEN_CHANNEL  = 2;

    enum class HandState { OPEN, CLOSING, HOLDING, OPENING };

    explicit MotorController(PCA9685& pwm);

    /** Move both servos to open position and reset state. */
    void init();

    /**
     * @brief Update motor positions based on current EMG ratio.
     *        Call this once per EMG sample after EMA classification.
     * @param ratio  Current EMA / baseline_rms ratio
     */
    void update(float ratio);

    HandState state() const { return state_; }

private:
    PCA9685&  pwm_;
    HandState state_          = HandState::OPEN;
    float     close_angle_    = CLOSE_SERVO_OPEN;
    float     open_angle_     = OPEN_SERVO_OPEN;

    void applyAngles();
    float speedFromRatio(float ratio) const;
};
