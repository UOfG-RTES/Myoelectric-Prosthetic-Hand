#include "sensors/EMGSensors.hpp"

#include <cstdio>
#include <csignal>
#include <atomic>
#include <fstream>
#include <chrono>
#include <string>
#include <numeric>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <thread>

class EMGLogger {
public:
    static constexpr int   CALIBRATION_SAMPLES = 500;  // ~0.6s at 860SPS
    static constexpr int   RMS_WINDOW          = 50;
    static constexpr float FLEX_MULTIPLIER     = 1.5f;
    static constexpr float STRONG_MULTIPLIER   = 3.0f;

    explicit EMGLogger(const std::string& filename) {
        file_.open(filename);
        if (!file_.is_open())
            throw std::runtime_error("Cannot open log file: " + filename);
        file_ << "timestamp_ms,emg_voltage_V,rms_V,baseline_rms_V,state\n";
        start_time_ = std::chrono::steady_clock::now();

        // Give the user 3 seconds to relax before sampling starts
        printf("\n=== RELAX YOUR MUSCLE NOW ===\n");
        for (int i = 3; i > 0; --i) {
            printf("Calibration starts in %d...\n", i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        printf("Calibrating — keep relaxed...\n");
        calibration_start_ = true;
    }

    ~EMGLogger() {
        if (file_.is_open()) file_.close();
    }

    void onSample(const EMGSample& s) {
        if (!calibration_start_) return;

        float emg = s.ch2;

        rms_window_.push_back(emg);
        if ((int)rms_window_.size() > RMS_WINDOW)
            rms_window_.pop_front();

        float rms = computeRMS(rms_window_);

        // --- Calibration phase ---
        if (!calibrated_) {
            calibration_buf_.push_back(rms);
            int n = calibration_buf_.size();
            // Print progress every 100 samples
            if (n % 100 == 0)
                printf("  Calibrating... %d/%d (current RMS: %.4fV)\n",
                       n, CALIBRATION_SAMPLES, rms);

            if (n >= CALIBRATION_SAMPLES) {
                // Use the MINIMUM average to avoid capturing any accidental movement
                // Sort and take bottom 80% to discard any twitches
                auto sorted = calibration_buf_;
                std::sort(sorted.begin(), sorted.end());
                int keep = (int)(sorted.size() * 0.8f);
                baseline_rms_ = std::accumulate(sorted.begin(),
                                                sorted.begin() + keep, 0.0f)
                                 / keep;
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

        // --- Detection ---
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

        printf("%6ld ms  RMS: %.4fV  ratio: %.2fx  [%s]\n",
               ms, rms, ratio, state);
    }

private:
    std::ofstream  file_;
    std::chrono::steady_clock::time_point start_time_;

    bool               calibration_start_ = false;
    bool               calibrated_        = false;
    std::vector<float> calibration_buf_;
    float              baseline_rms_      = 0.0f;
    std::deque<float>  rms_window_;

    static float computeRMS(const std::deque<float>& buf) {
        if (buf.empty()) return 0.0f;
        float sum = 0.0f;
        for (float v : buf) sum += v * v;
        return std::sqrt(sum / buf.size());
    }
};

static std::atomic<bool> keep_running{true};
static void signalHandler(int) { keep_running = false; }

int main() {
    std::signal(SIGINT, signalHandler);

    EMGLogger logger("emg_log.csv");

    EMGSettings cfg;
    cfg.i2c_bus     = 1;
    cfg.i2c_address = 0x48;
    cfg.sample_rate = 860;
    cfg.pga_gain    = 4.096f;

    EMGSensors emg(cfg);
    emg.registerCallback([&](const EMGSample& s) {
        logger.onSample(s);
    });

    try {
        emg.start();
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    while (keep_running) pause();

    printf("\nStopping — CSV saved.\n");
    emg.stop();
    return 0;
}