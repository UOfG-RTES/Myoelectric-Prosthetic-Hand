# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo ./main
sudo ./servo_calibration   # interactive servo angle tuner
```

`sudo` is required for I2C hardware access. Use `-DCMAKE_BUILD_TYPE=Debug` for debug builds.

**Dependencies:** `cmake >= 3.16`, `g++` (C++17), `libi2c-dev`, `pthread`. I2C must be enabled on the Raspberry Pi (`raspi-config`).

There is no test suite and no linter configured.

## Architecture

This is a real-time EMG-based prosthetic hand controller targeting Raspberry Pi 5. It reads forearm muscle signals from an ADS1115 16-bit I2C ADC, classifies muscle activity, and drives servo motors via a PCA9685 PWM controller.

### Data flow

```
ADS1115 (I2C) → EMGSensors (worker thread) --callback--> EMGLogger::onSample()
                                                               ├─ compute EMA-RMS, ratio
                                                               ├─ invoke MotorCallback → MotorController::update()
                                                               │       └─ PCA9685 (I2C) → servos
                                                               └─ write emg_log.csv
```

All stages run on the single EMGSensors worker thread via synchronous callbacks — no queues, no shared buffers. Keep callback logic fast; it directly affects sampling latency.

### Key components

**[EMGSensors](include/sensors/EMGSensors.hpp) / [src/sensors/EMGSensors.cpp](src/sensors/EMGSensors.cpp)**
- ADS1115 I2C driver; spins a dedicated worker thread for continuous sampling (configurable 8–860 SPS via `ADS1115_Config`)
- Delivers samples via `std::function` callback — zero-copy, decoupled from consumers
- Uses blocking I2C reads for hardware-accurate timing (no `sleep`-based polling)

**[EMGLogger](include/EMGLogger.hpp) / [src/EMGLogger.cpp](src/EMGLogger.cpp)**

- Receives raw voltage samples; runs a 3-second calibration phase to establish per-user baseline EMA-RMS
- Two-state hysteresis classification (`open`/`close`) against the baseline ratio
- Invokes a registered `MotorCallback(float ratio) -> std::string` after each classification; caches the returned motor state string
- Throttles console output to every `PRINT_EVERY=50` samples (~17 Hz at 860 SPS)
- Writes `emg_log.csv`: `timestamp_ms, raw_adc, emg_voltage_V, ema_rms_V, ratio, emg_state, motor_state`

**[PCA9685](include/motor/PCA9685.hpp) / [src/motor/PCA9685.cpp](src/motor/PCA9685.cpp)**

- Low-level I2C driver for the PCA9685 16-channel 12-bit PWM controller at address `0x40`
- 50 Hz servo period; pulse range `SERVO_MIN=102` (0°) to `SERVO_MAX=512` (180°) in 4096-count ticks
- Each I2C write is <0.1 ms — safe to call from the 860 SPS worker thread

**[MotorController](include/motor/MotorController.hpp) / [src/motor/MotorController.cpp](src/motor/MotorController.cpp)**

- Antagonistic two-servo hand controller: channel 1 (close servo, pulls fingers) and channel 15 (open servo, releases fingers)
- Velocity-based control: `update(ratio)` moves both servos by a speed proportional to how far the EMG ratio exceeds the engage threshold, called every sample
- Three-state hysteresis: `CLOSING` (ratio > `ENGAGE_RATIO=1.5`), `HOLDING` (1.2–1.5, no movement), `OPENING` (ratio < `RELEASE_RATIO=1.2`, fixed max speed)
- Speed scaling: linear from 0°/sample at `ENGAGE_RATIO` to `MAX_SPEED_DEG=1.0°/sample` at `MAX_RATIO=2.5`; at 860 SPS full 180° travel ≈ 0.7 s
- Servo angle limits (`CLOSE_SERVO_OPEN/CLOSED`, `OPEN_SERVO_OPEN/CLOSED`) are compile-time constants in the header — use `servo_calibration` to find correct values, then update them

**[apps/main.cpp](apps/main.cpp)**

- Initialization order matters: PCA9685 and MotorController must initialize before EMGLogger so the hand reaches open position during the calibration countdown
- Registers the motor callback: `logger.registerMotorCallback([&](float ratio){ return motor.update(ratio); })`
- Installs `SIGINT`/`SIGTERM` handlers via `std::atomic<bool>` for safe shutdown

**[apps/servo_calibration.cpp](apps/servo_calibration.cpp)**

- Interactive CLI (`<channel> <angle>` or `q`) to physically find correct open/closed angles for each servo
- Prints a summary of the four constants to update in `MotorController.hpp` on exit
- Run this before deploying to a new hardware setup

### Build targets

| Target | Sources | Links |
| --- | --- | --- |
| `emgsensors` (lib) | `src/sensors/`, `src/EMGLogger.cpp` | `pthread`, `i2c` |
| `motorcontrol` (lib) | `src/motor/` | `i2c` |
| `main` | `apps/main.cpp` | `emgsensors`, `motorcontrol` |
| `servo_calibration` | `apps/servo_calibration.cpp` | `motorcontrol` |

### Threading model

- **Main thread**: configures hardware, blocks waiting for shutdown signal
- **Worker thread** (inside `EMGSensors`): runs the I2C read loop, invokes the callback synchronously on each sample — all downstream logic (EMGLogger, MotorController, PCA9685 writes) executes here

### Hardware

- Raspberry Pi 5, I2C bus 1
- ADS1115 at address `0x48` — EMG ADC, MyoWare 2.0 sensor on channel `AIN2`
- PCA9685 at address `0x40` — servo PWM controller
- 5× servo motors (finger actuation); current software drives 2 in an antagonistic pair

## Current development state

- EMG acquisition, EMA-RMS calibration, classification, and CSV logging: complete
- PCA9685 driver and MotorController (antagonistic velocity control): complete, on `feature/pipeline-class`
- Servo angle constants in `MotorController.hpp` are placeholders (0°/180°) — run `servo_calibration` on physical hardware to tune
- Multi-channel EMG and per-finger independent control: not yet implemented
