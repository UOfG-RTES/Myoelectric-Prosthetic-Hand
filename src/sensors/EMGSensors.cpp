#include "sensors/EMGSensors.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <string>
#include <cstring>
#include <cerrno>

static constexpr uint16_t OS_START     = 0x8000;
static constexpr uint16_t PGA_4096     = 0x0200;
static constexpr uint16_t PGA_0512     = 0x0800;
static constexpr uint16_t MODE_SINGLE  = 0x0100;
static constexpr uint16_t COMP_DISABLE = 0x0003;

static constexpr uint16_t DR_860SPS = 0x00E0;
static constexpr uint16_t DR_475SPS = 0x00C0;
static constexpr uint16_t DR_250SPS = 0x00A0;
static constexpr uint16_t DR_128SPS = 0x0080;
static constexpr uint16_t DR_64SPS  = 0x0060;
static constexpr uint16_t DR_32SPS  = 0x0040;
static constexpr uint16_t DR_16SPS  = 0x0020;
static constexpr uint16_t DR_8SPS   = 0x0000;

EMGSensors::EMGSensors(EMGSettings settings)
    : settings_(settings) {}

EMGSensors::~EMGSensors() {
    if (running_) stop();
    if (fd_i2c_ >= 0) close(fd_i2c_);
}

void EMGSensors::registerCallback(SampleCallback cb) {
    callback_ = std::move(cb);
}

void EMGSensors::start() {
    std::string device = "/dev/i2c-" + std::to_string(settings_.i2c_bus);
    fd_i2c_ = open(device.c_str(), O_RDWR);
    if (fd_i2c_ < 0)
        throw std::runtime_error("Failed to open " + device + ": " + std::strerror(errno));

    if (ioctl(fd_i2c_, I2C_SLAVE, settings_.i2c_address) < 0) {
        close(fd_i2c_); fd_i2c_ = -1;
        throw std::runtime_error(std::string("Failed to set I2C address: ") + std::strerror(errno));
    }
    running_ = true;
    worker_thread_ = std::thread(&EMGSensors::worker, this);
}

void EMGSensors::stop() {
    running_ = false;
    if (worker_thread_.joinable())
        worker_thread_.join();
    if (fd_i2c_ >= 0) { close(fd_i2c_); fd_i2c_ = -1; }
}

uint16_t EMGSensors::drRateConfig() const {
    switch (settings_.sample_rate) {
        case 8:   return DR_8SPS;
        case 16:  return DR_16SPS;
        case 32:  return DR_32SPS;
        case 64:  return DR_64SPS;
        case 128: return DR_128SPS;
        case 250: return DR_250SPS;
        case 475: return DR_475SPS;
        case 860: return DR_860SPS;
        default:  return DR_128SPS;
    }
}

void EMGSensors::startConversion(int channel) {
    uint16_t config = OS_START | MUX_CH[channel] | PGA_0512 | MODE_SINGLE | drRateConfig() | COMP_DISABLE;
    uint8_t buf[3] = {
        REG_CONFIG,
        static_cast<uint8_t>((config >> 8) & 0xFF),
        static_cast<uint8_t>(config & 0xFF)
    };
    write(fd_i2c_, buf, 3);
}

int16_t EMGSensors::readConversion(int /*channel*/) {
    // Blocking I2C reads provide timing — no sleep() required
    uint8_t reg = REG_CONFIG;
    uint8_t buf[2];
    do {
        write(fd_i2c_, &reg, 1);
        read(fd_i2c_, buf, 2);
    } while (!(buf[0] & 0x80));  // wait for OS bit = conversion complete

    reg = REG_CONVERSION;
    write(fd_i2c_, &reg, 1);
    read(fd_i2c_, buf, 2);
    return static_cast<int16_t>((buf[0] << 8) | buf[1]);
}

float EMGSensors::toVolts(int16_t raw) const {
    return (static_cast<float>(raw) / 32767.0f) * settings_.pga_gain;
}

void EMGSensors::worker() {
    while (running_) {
          EMGSample sample{};

          startConversion(2);
          int16_t raw  = readConversion(2);
          sample.raw_ch2 = raw;
          sample.ch2     = toVolts(raw);

          if (callback_ && running_)
              callback_(sample);
      }
}