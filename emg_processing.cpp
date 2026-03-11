#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <pigpio.h>

#include "emg_processor.h"
#include "mcp3008.h"

// GPIO & Channels
static constexpr int GPIO_LED = 25;
static constexpr int EMG_CH_FLEX = 0;
static constexpr int EMG_CH_EXT = 1;

// Sampling Constants
static constexpr int SAMPLE_RATE_HZ = 1000;
static constexpr int PERIOD_US = 1'000'000 / SAMPLE_RATE_HZ;

// Shutdown flag
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Data structures for pipeline
struct RawEMG
{
    uint16_t flex;
    uint16_t ext;
};

struct ProcessedEMG
{
    double flex_rms;
    double ext_rms;
};

// Queue between Acquisition and Processing
std::queue<RawEMG> raw_queue;
std::mutex raw_mutex;
std::condition_variable raw_cv;

// THREAD 1: EMG Acquisition (Producer)
void acquisition_thread(MCP3008 &adc)
{
    while (g_running)
    {
        auto start = std::chrono::steady_clock::now();

        RawEMG sample;
        sample.flex = adc.read(EMG_CH_FLEX);
        sample.ext = adc.read(EMG_CH_EXT);

        {
            std::lock_guard<std::mutex> lock(raw_mutex);
            raw_queue.push(sample);
        }
        raw_cv.notify_one();

        // Maintain 1kHz timing
        auto elapsed = std::chrono::steady_clock::now() - start;
        int used_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        int sleep_us = PERIOD_US - used_us;
        if (sleep_us > 0)
            gpioDelay(sleep_us);
    }
}

// THREAD 2: Signal Processing (Consumer)
void processing_thread(EMGProcessor &flex_emg, EMGProcessor &ext_emg)
{
    int print_tick = 0;
    const int PRINT_EVERY = SAMPLE_RATE_HZ / 10; // Print at 10Hz

    while (g_running)
    {
        RawEMG sample;

        {
            std::unique_lock<std::mutex> lock(raw_mutex);
            raw_cv.wait(lock, []
                        { return !raw_queue.empty() || !g_running; });

            if (!g_running)
                return;

            sample = raw_queue.front();
            raw_queue.pop();
        }

        // Process the raw ADC values into RMS
        double flex_rms = flex_emg.update(sample.flex);
        double ext_rms = ext_emg.update(sample.ext);

        // Visualization of processed data
        if (++print_tick >= PRINT_EVERY)
        {
            print_tick = 0;
            std::cout << std::fixed << std::setprecision(1)
                      << "Flex RMS: " << std::setw(6) << flex_rms << " mV | "
                      << "Ext RMS: " << std::setw(6) << ext_rms << " mV\r" << std::flush;
        }
    }
}

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (gpioInitialise() < 0)
    {
        std::cerr << "[FATAL] gpioInitialise() failed\n";
        return 1;
    }

    gpioSetMode(GPIO_LED, PI_OUTPUT);

    std::cout << "=== EMG Acquisition & Processing Pipeline ===\n";

    MCP3008 adc;
    EMGProcessor flex_emg, ext_emg;

    std::cout << "Calibrating baseline (2s)... Keep muscles relaxed.\n";
    for (int i = 0; i < 2000; ++i)
    {
        flex_emg.update(adc.read(EMG_CH_FLEX));
        ext_emg.update(adc.read(EMG_CH_EXT));
        gpioDelay(1000);
    }

    gpioWrite(GPIO_LED, 1);
    std::cout << "Starting threads...\n";

    std::thread t_acq(acquisition_thread, std::ref(adc));
    std::thread t_proc(processing_thread, std::ref(flex_emg), std::ref(ext_emg));

    t_acq.join();
    t_proc.join();

    std::cout << "\nCleaning up...\n";
    gpioWrite(GPIO_LED, 0);
    gpioTerminate();

    return 0;
}