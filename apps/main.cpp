#include "sensors/EMGSensors.hpp"
#include "EMGLogger.hpp"
#include "motor/PCA9685.hpp"
#include "motor/MotorController.hpp"

#include <cstdio>
#include <csignal>
#include <atomic>

// ── Signal handling ───────────────────────────────────────────────────────────
// POSIX signal handlers must be plain C function pointers — they cannot capture
// any context, so a file-scoped atomic is the only standards-compliant way to
// communicate a shutdown request from the handler to the main thread.
namespace {
    std::atomic<bool> keep_running{true};
    void on_signal(int) { keep_running = false; }
}

int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ── Motor control ─────────────────────────────────────────────────────────
    // Initialised before EMGLogger so the hand reaches its open position during
    // the calibration countdown, not mid-session.
    PCA9685 pwm(1, 0x40);
    try {
        pwm.init();
    } catch (const std::exception& e) {
        fprintf(stderr, "PCA9685 init failed: %s\n", e.what());
        return 1;
    }

    MotorController motor(pwm);
    motor.init();

    // ── EMG pipeline ──────────────────────────────────────────────────────────
    EMGLogger logger("emg_log.csv");

    // Wire motor controller into logger — called with ratio after each sample.
    // Runs on the EMGSensors worker thread; PCA9685 I2C writes are fast (<0.1 ms)
    // and will not stall acquisition at 860 SPS.
    logger.registerMotorCallback([&](float ratio) {
        motor.update(ratio);
    });

    EMGSettings cfg;
    cfg.i2c_bus     = 1;
    cfg.i2c_address = 0x48;
    cfg.sample_rate = 860;
    cfg.pga_gain    = 0.512f;

    EMGSensors emg(cfg);
    emg.registerCallback([&](const EMGSample& s) {
        logger.onSample(s);
    });

    try {
        emg.start();
    } catch (const std::exception& e) {
        fprintf(stderr, "EMGSensors start failed: %s\n", e.what());
        return 1;
    }

    while (keep_running) pause();

    printf("\nStopping — CSV saved.\n");
    emg.stop();
    return 0;
}
