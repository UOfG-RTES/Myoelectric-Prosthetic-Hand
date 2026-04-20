#include "motor/PCA9685.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <string>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <thread>
#include <chrono>

PCA9685::PCA9685(int i2c_bus, int i2c_address)
    : i2c_bus_(i2c_bus), i2c_address_(i2c_address) {}

PCA9685::~PCA9685() {
    if (fd_i2c_ >= 0) close(fd_i2c_);
}

void PCA9685::init() {
    std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
    fd_i2c_ = open(device.c_str(), O_RDWR);
    if (fd_i2c_ < 0)
        throw std::runtime_error("PCA9685: failed to open " + device
                                 + ": " + std::strerror(errno));

    if (ioctl(fd_i2c_, I2C_SLAVE, i2c_address_) < 0) {
        close(fd_i2c_); fd_i2c_ = -1;
        throw std::runtime_error(
            std::string("PCA9685: failed to set I2C address: ") + std::strerror(errno));
    }

    writeRegister(REG_MODE1, 0x00);  // reset
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    setFrequency(50.0f);

    writeRegister(REG_MODE2, 0x04);  // totem-pole outputs, change on STOP

    printf("PCA9685 initialised at 50 Hz on I2C bus %d, address 0x%02X\n",
           i2c_bus_, i2c_address_);
}

void PCA9685::setFrequency(float hz) {
    uint8_t prescale = static_cast<uint8_t>(
        std::round(25000000.0f / (4096.0f * hz)) - 1);

    uint8_t mode1 = readRegister(REG_MODE1);
    writeRegister(REG_MODE1, (mode1 & 0x7F) | 0x10);  // SLEEP bit — required to change prescaler
    writeRegister(REG_PRE_SCALE, prescale);
    writeRegister(REG_MODE1, mode1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    writeRegister(REG_MODE1, mode1 | 0xA0);            // AUTO_INCREMENT + RESTART
}

void PCA9685::setPWM(int channel, uint16_t off) {
    if (fd_i2c_ < 0) {
        fprintf(stderr, "PCA9685::setPWM: device not initialised\n");
        return;
    }
    if (channel < 0 || channel > 15) {
        fprintf(stderr, "PCA9685::setPWM: channel %d out of range\n", channel);
        return;
    }

    uint8_t base = static_cast<uint8_t>(REG_LED0_ON_L + 4 * channel);
    uint8_t buf[5] = {
        base,
        0x00, 0x00,
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>((off >> 8) & 0x0F)
    };

    if (write(fd_i2c_, buf, 5) != 5)
        fprintf(stderr, "PCA9685::setPWM: I2C write failed: %s\n", std::strerror(errno));
}

void PCA9685::setAngle(int channel, float angle_deg) {
    angle_deg = std::max(0.0f, std::min(angle_deg, 180.0f));
    uint16_t counts = static_cast<uint16_t>(
        SERVO_MIN + (angle_deg / 180.0f) * (SERVO_MAX - SERVO_MIN));
    setPWM(channel, counts);
}

void PCA9685::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    if (write(fd_i2c_, buf, 2) != 2)
        fprintf(stderr, "PCA9685::writeRegister 0x%02X failed: %s\n",
                reg, std::strerror(errno));
}

uint8_t PCA9685::readRegister(uint8_t reg) {
    if (write(fd_i2c_, &reg, 1) != 1) {
        fprintf(stderr, "PCA9685::readRegister write failed: %s\n", std::strerror(errno));
        return 0;
    }
    uint8_t value = 0;
    if (read(fd_i2c_, &value, 1) != 1)
        fprintf(stderr, "PCA9685::readRegister read failed: %s\n", std::strerror(errno));
    return value;
}
