#pragma once

#include "sensors/EMGSensors.hpp"

#include <fstream>
#include <vector>
#include <deque>
#include <string>
#include <chrono>

/**
 * @brief Calibrates a per-user EMG baseline, then classifies each sample.
 *
 * Receives EMGSample values via onSample(), which is registered as the
 * EMGSensors callback.  Classification produces three states:
 *   - rest       : RMS ≤ FLEX_MULTIPLIER  × baseline
 *   - flex       : RMS ≤ STRONG_MULTIPLIER × baseline
 *   - strong_flex: RMS >  STRONG_MULTIPLIER × baseline
 *
 * ── Latency budget (at 860 SPS) ─────────────────────────────────────────────
 *  • ADS1115 single-shot conversion : ~1.16 ms  (1/860 s)
 *  • I²C read overhead              : ~0.10 ms
 *  • RMS window (20 samples)        : ~23 ms of signal history
 *  • Callback + classification      : <0.01 ms
 *  Total end-to-end (muscle → state): ~24 ms
 *  This is well under the ~50 ms threshold at which humans perceive delay.
 * ────────────────────────────────────────────────────────────────────────────
 */
class EMGLogger {
public:
    // ── Tuning constants ──────────────────────────────────────────────────────
    static constexpr int   CALIBRATION_SAMPLES = 500;  ///< ~0.6 s at 860 SPS
    static constexpr int   RMS_WINDOW          = 50;   ///< Rolling RMS window
    static constexpr float FLEX_MULTIPLIER     = 1.5f; ///< Rest → Flex boundary
    static constexpr float STRONG_MULTIPLIER   = 3.0f; ///< Flex → Strong boundary

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

    bool               calibration_start_ = false;
    bool               calibrated_        = false;
    std::vector<float> calibration_buf_;
    float              baseline_rms_      = 0.0f;
    std::deque<float>  rms_window_;

    static float computeRMS(const std::deque<float>& buf);
};
