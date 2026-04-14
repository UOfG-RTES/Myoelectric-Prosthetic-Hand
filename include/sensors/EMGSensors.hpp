#pragma once

#include <functional>
#include <thread>
#include <atomic>
#include <cstdint>
#include <stdexcept>

extern "C" {
#include <linux/i2c-dev.h>
}

/**
 * @brief Physical voltage readings from all 4 EMG channels (in Volts).
 */
struct EMGSample {
    float ch0 = 0.f;
    float ch1 = 0.f;
    float ch2 = 0.f;
    float ch3 = 0.f;
};

/**
 * @brief ADS1115 configuration — defined outside EMGSensors so that
 *        EMGSensors(Settings = Settings()) compiles without forward-decl issues.
 */
struct EMGSettings {
    int   i2c_bus     = 1;      ///< I2C bus number (usually 1 on Raspberry Pi)
    int   i2c_address = 0x48;   ///< ADS1115 I2C address (ADDR low = 0x48)
    int   sample_rate = 128;    ///< SPS: 8,16,32,64,128,250,475,860
    float pga_gain    = 4.096f; ///< Full-scale range in volts
};

/**
 * @brief ADS1115 EMG sensor driver.
 *
 * Wraps I²C access to the ADS1115 16-bit ADC and delivers samples to the
 * caller via a registered std::function callback.
 *
 * ── Why a callback, not a queue? ────────────────────────────────────────────
 * A callback (std::function) was chosen over an internal queue or shared
 * buffer for three reasons:
 *   1. Decoupling — the driver is independent of any particular consumer.
 *      Swapping or adding consumers (e.g. logger, plotter, motor controller)
 *      requires no change to this class.
 *   2. Zero-copy delivery — the sample is passed directly from the worker
 *      thread to the consumer on every call, with no heap allocation.
 *      At 860 SPS this avoids ~860 allocations/second.
 *   3. Minimal internal state — no mutex or condition variable is needed
 *      inside the driver, which reduces lock contention and complexity.
 * Trade-off: the callback executes on the worker thread.  Consumers must
 * either be thread-safe or complete quickly enough not to stall sampling.
 *
 * ── Latency budget ──────────────────────────────────────────────────────────
 *  • ADS1115 conversion time  : 1/SPS  (1.16 ms at 860 SPS)
 *  • I²C register write+read  : ~0.10 ms per transaction
 *  • Callback invocation      : <0.01 ms
 *  One-sample pipeline latency: ~1.26 ms
 *  After a 20-sample RMS window: ~24 ms end-to-end (muscle → classification)
 * ────────────────────────────────────────────────────────────────────────────
 *
 * Timing is established by blocking on the ADS1115 OS (operational status)
 * bit — no sleep() or busy-polling.
 */
class EMGSensors {
public:
    using SampleCallback = std::function<void(const EMGSample&)>;

    explicit EMGSensors(EMGSettings settings = EMGSettings());
    ~EMGSensors();

    EMGSensors(const EMGSensors&)            = delete;
    EMGSensors& operator=(const EMGSensors&) = delete;

    /** Register the callback that receives each new EMGSample. */
    void registerCallback(SampleCallback cb);

    /** Open I2C device and start the worker thread. */
    void start();

    /** Signal the worker to stop and join the thread. */
    void stop();

private:
    static constexpr uint8_t  REG_CONVERSION = 0x00;
    static constexpr uint8_t  REG_CONFIG     = 0x01;

    static constexpr uint16_t MUX_CH[4] = {
        0x4000, // AIN0 vs GND
        0x5000, // AIN1 vs GND
        0x6000, // AIN2 vs GND
        0x7000, // AIN3 vs GND
    };

    EMGSettings       settings_;
    SampleCallback    callback_;
    std::thread       worker_thread_;
    std::atomic<bool> running_{false};
    int               fd_i2c_ = -1;

    void    startConversion(int channel);
    int16_t readConversion(int channel);
    float   toVolts(int16_t raw) const;
    uint16_t drRateConfig() const;
    void    worker();
};