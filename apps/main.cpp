#include "sensors/EMGSensors.hpp"
#include "EMGLogger.hpp"

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

    EMGLogger logger("emg_log.csv");

    // ── Why a callback? ───────────────────────────────────────────────────────
    // EMGSensors delivers samples via std::function callback rather than
    // exposing a queue or shared buffer because:
    //   1. Decoupling: the driver has no knowledge of or dependency on the
    //      consumer (EMGLogger).  New consumers (e.g. a plotter) can be
    //      registered without changing the driver.
    //   2. Zero-copy: the sample is handed directly from the worker thread
    //      to the consumer in-place, avoiding a queue allocation on every
    //      sample (~860 allocations/second at max rate).
    //   3. Simplicity: no mutex-protected queue or condition variable is
    //      needed inside EMGSensors, keeping its implementation minimal.
    // The trade-off is that the callback runs on the worker thread, so the
    // consumer (EMGLogger::onSample) must be thread-safe or fast enough not
    // to stall acquisition.
    // ─────────────────────────────────────────────────────────────────────────
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
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    while (keep_running) pause();

    printf("\nStopping — CSV saved.\n");
    emg.stop();
    return 0;
}
