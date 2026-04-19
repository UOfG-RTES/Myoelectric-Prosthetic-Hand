#pragma once

#include <cstdint>

extern "C" {
#include <linux/i2c-dev.h>
}

/**
 * @brief Driver for the PCA9685 16-channel 12-bit PWM controller.
 *
 * Communicates over I²C. Designed for servo control at 50 Hz.
 *
 * ── PWM counts for servo control at 50 Hz ───────────────────────────────────
 *  Period        = 20 ms = 4096 counts
 *  1 ms pulse    = 205 counts  →  0°
 *  2 ms pulse    = 410 counts  → 180°
 * ────────────────────────────────────────────────────────────────────────────
 */
class PCA9685 {
public:
    // Servo pulse width in counts (at 50 Hz, 12-bit)
    static constexpr uint16_t SERVO_MIN = 205;  ///< 1 ms pulse → 0°
    static constexpr uint16_t SERVO_MAX = 410;  ///< 2 ms pulse → 180°

    /**
     * @param i2c_bus     I²C bus number (1 on Raspberry Pi)
     * @param i2c_address PCA9685 address (default 0x40)
     */
    explicit PCA9685(int i2c_bus = 1, int i2c_address = 0x40);
    ~PCA9685();

    PCA9685(const PCA9685&)            = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    /** Open I²C device, reset PCA9685, set PWM frequency to 50 Hz. */
    void init();

    /**
     * @brief Set a servo to a given angle.
     * @param channel  PCA9685 channel (0–15)
     * @param angle_deg  Angle in degrees (0–180)
     */
    void setAngle(int channel, float angle_deg);

    /**
     * @brief Set raw PWM counts for a channel.
     * @param channel  PCA9685 channel (0–15)
     * @param off      OFF count (0–4095) — ON is always 0
     */
    void setPWM(int channel, uint16_t off);

private:
    static constexpr uint8_t REG_MODE1    = 0x00;
    static constexpr uint8_t REG_MODE2    = 0x01;
    static constexpr uint8_t REG_PRE_SCALE = 0xFE;
    static constexpr uint8_t REG_LED0_ON_L = 0x06; ///< Base register — each channel adds 4

    int i2c_bus_;
    int i2c_address_;
    int fd_i2c_ = -1;

    void     writeRegister(uint8_t reg, uint8_t value);
    uint8_t  readRegister(uint8_t reg);
    void     setFrequency(float hz);
};
