#pragma once

#include "sensors/EMGSensors.hpp"

#include <fstream>
#include <vector>
#include <string>
#include <chrono>

/**
 * @brief Calibrates a per-user EMG baseline, then classifies each sample.
 *
 * Receives EMGSample values via onSample(), which is registered as the
 * EMGSensors callback. Produces two hand commands with hysteresis:
 *   - open  : ratio < FLEX_RELEASE  (hand opens)
 *   - close : ratio > FLEX_ENGAGE   (hand closes)
 *   - hold  : FLEX_RELEASE ≤ ratio ≤ FLEX_ENGAGE  (hysteresis band, no change)
 *
 * Hysteresis band prevents chattering at the threshold boundary.
 *
 * ── Latency budget (at 860 SPS) ─────────────────────────────────────────────
 *  • ADS1115 single-shot conversion : ~1.16 ms  (1/860 s)
 *  • I²C read overhead              : ~0.10 ms
 *  • EMA update                     : <0.01 ms (single multiply-add)
 *  • Callback + classification      : <0.01 ms
 *  Total end-to-end (muscle → state): ~1.27 ms
 *  This is well under the ~50 ms threshold at which humans perceive delay.
 * ────────────────────────────────────────────────────────────────────────────
 */
class EMGLogger {
public:
    // ── Tuning constants ──────────────────────────────────────────────────────
    static constexpr int   CALIBRATION_SAMPLES = 500;   ///< ~0.6 s at 860 SPS
    static constexpr float EMA_ALPHA           = 0.05f; ///< EMA smoothing factor (0<α≤1). Higher = faster response, more noise.
    static constexpr float FLEX_ENGAGE         = 1.5f;  ///< open → close : ratio must exceed this
    static constexpr float FLEX_RELEASE        = 1.2f;  ///< close → open : ratio must drop below this

    /**
     * @param filename  Path to CSV output file.
     *                  Columns: timestamp_ms, emg_voltage_V, rms_V,
     *                           baseline_rms_V, state
     * @throws std::runtime_error if the file cannot be opened.
     */
    explicit EMGLogger(const std::string& filename);
    ~EMGLogger();

    // Non-copyable — owns an open file handle and mutable state.
    EMGLogger(const EMGLogger&)            = delete;
    EMGLogger& operator=(const EMGLogger&) = delete;

    /**
     * @brief Process one EMG sample.
     *
     * Designed to be registered directly as the EMGSensors callback:
     * @code
     *   emg.registerCallback([&](const EMGSample& s){ logger.onSample(s); });
     * @endcode
     */
    void onSample(const EMGSample& s);

private:
    std::ofstream  file_;
    std::chrono::steady_clock::time_point start_time_;

    bool calibration_start_ = false;
    bool calibrated_        = false;
    bool hand_closed_       = false; ///< Current hand state — drives hysteresis logic
    std::vector<float> calibration_buf_;
    float baseline_rms_ = 0.0f;
    float ema_sq_       = 0.0f; ///< EMA of squared voltage — sqrt gives envelope
};
