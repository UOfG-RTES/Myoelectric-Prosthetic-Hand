#include "EMGLogger.hpp"

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <chrono>

EMGLogger::EMGLogger(const std::string& filename) {
    file_.open(filename);
    if (!file_.is_open())
        throw std::runtime_error("Cannot open log file: " + filename);

    file_ << "timestamp_ms,raw_adc,emg_voltage_V,rms_V,ratio,state\n";
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

    // EMA of squared voltage — decays immediately when signal drops (no sticky states)
    ema_sq_   = EMA_ALPHA * (emg * emg) + (1.0f - EMA_ALPHA) * ema_sq_;
    float rms = std::sqrt(ema_sq_);

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

            printf("\nBaseline EMA : %.4fV\n", baseline_rms_);
            printf("ENGAGE  (open  → close) : ratio > %.1fx  =  EMA > %.4fV\n",
                   FLEX_ENGAGE,   baseline_rms_ * FLEX_ENGAGE);
            printf("RELEASE (close → open)  : ratio < %.1fx  =  EMA < %.4fV\n",
                   FLEX_RELEASE,  baseline_rms_ * FLEX_RELEASE);
            printf("HOLD    (no change)     : %.1fx ≤ ratio ≤ %.1fx\n\n",
                   FLEX_RELEASE, FLEX_ENGAGE);
            printf("Now flex and relax!\n\n");
        }
        return;
    }

    // ── Detection phase — two-state hysteresis ────────────────────────────────
    float ratio = rms / baseline_rms_;

    if (!hand_closed_ && ratio > FLEX_ENGAGE)
        hand_closed_ = true;          // open → close
    else if (hand_closed_ && ratio < FLEX_RELEASE)
        hand_closed_ = false;         // close → open
    // else: ratio in hysteresis band — hold current state, no change

    const char* state = hand_closed_ ? "close" : "open";

    auto now = std::chrono::steady_clock::now();
    long ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - start_time_).count();

    file_ << ms << "," << s.raw_ch2 << "," << emg << ","
          << rms << "," << ratio << "," << state << "\n";

    printf("%6ld ms | RAW: %6d | Voltage: %.4fV | EMA: %.4fV | Ratio: %.2fx | [%-5s]\n",
           ms, s.raw_ch2, emg, rms, ratio, state);
}

