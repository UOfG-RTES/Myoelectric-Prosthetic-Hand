#include "EMGLogger.hpp"

#include <cstdio>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <chrono>

EMGLogger::EMGLogger(const std::string& filename) {
    file_.open(filename);
    if (!file_.is_open())
        throw std::runtime_error("Cannot open log file: " + filename);

    file_ << "timestamp_ms,emg_voltage_V,rms_V,baseline_rms_V,state\n";
    start_time_ = std::chrono::steady_clock::now();

    // Give the user time to relax before sampling starts.
    printf("\n=== RELAX YOUR MUSCLE NOW ===\n");
    for (int i = 3; i > 0; --i) {
        printf("Calibration starts in %d...\n", i);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    printf("Calibrating — keep relaxed...\n");
    calibration_start_ = true;
}

EMGLogger::~EMGLogger() {
    if (file_.is_open()) file_.close();
}

void EMGLogger::onSample(const EMGSample& s) {
    if (!calibration_start_) return;

    float emg = s.ch2;

    rms_window_.push_back(emg);
    if (static_cast<int>(rms_window_.size()) > RMS_WINDOW)
        rms_window_.pop_front();

    float rms = computeRMS(rms_window_);

    // ── Calibration phase ─────────────────────────────────────────────────────
    if (!calibrated_) {
        calibration_buf_.push_back(rms);
        int n = static_cast<int>(calibration_buf_.size());

        if (n % 100 == 0)
            printf("  Calibrating... %d/%d (current RMS: %.4fV)\n",
                   n, CALIBRATION_SAMPLES, rms);

        if (n >= CALIBRATION_SAMPLES) {
            // Sort and use the bottom 80% to discard accidental movement twitches.
            auto sorted = calibration_buf_;
            std::sort(sorted.begin(), sorted.end());
            int keep = static_cast<int>(sorted.size() * 0.8f);
            baseline_rms_ = std::accumulate(sorted.begin(),
                                            sorted.begin() + keep, 0.0f)
                            / static_cast<float>(keep);
            calibrated_ = true;

            printf("\nBaseline RMS: %.4fV\n", baseline_rms_);
            printf("Flex threshold:   %.4fV (%.1fx baseline)\n",
                   baseline_rms_ * FLEX_MULTIPLIER, FLEX_MULTIPLIER);
            printf("Strong threshold: %.4fV (%.1fx baseline)\n\n",
                   baseline_rms_ * STRONG_MULTIPLIER, STRONG_MULTIPLIER);
            printf("Now flex and relax!\n\n");
        }
        return;
    }

    // ── Detection phase ───────────────────────────────────────────────────────
    float ratio = rms / baseline_rms_;

    const char* state;
    if (ratio > STRONG_MULTIPLIER)
        state = "strong_flex";
    else if (ratio > FLEX_MULTIPLIER)
        state = "flex";
    else
        state = "rest";

    auto now = std::chrono::steady_clock::now();
    long ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - start_time_).count();

    file_ << ms << "," << emg << "," << rms << ","
          << baseline_rms_ << "," << state << "\n";

    // printf("%6ld ms  RMS: %.4fV  ratio: %.2fx  [%s]\n",ms, rms, ratio, state);
    printf("RMS: %.4f\n", rms);
}

float EMGLogger::computeRMS(const std::deque<float>& buf) {
    if (buf.empty()) return 0.0f;
    float sum = 0.0f;
    for (float v : buf) sum += v * v;
    return std::sqrt(sum / static_cast<float>(buf.size()));
}
