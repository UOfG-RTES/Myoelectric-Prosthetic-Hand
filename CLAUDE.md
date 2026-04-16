# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo ./main
```

`sudo` is required for I2C hardware access. Use `-DCMAKE_BUILD_TYPE=Debug` for debug builds.

**Dependencies:** `cmake >= 3.16`, `g++` (C++17), `libi2c-dev`, `pthread`. I2C must be enabled on the Raspberry Pi (`raspi-config`).

There is no test suite and no linter configured.

## Architecture

This is a real-time EMG-based prosthetic hand controller targeting Raspberry Pi 5. It reads forearm muscle signals from an ADS1115 16-bit I2C ADC, classifies muscle activity, and (in future work) drives servo motors.

### Data flow

```
ADS1115 (I2C) → EMGSensors (worker thread) --callback--> EMGLogger → emg_log.csv
```

### Key components

**[EMGSensors](include/sensors/EMGSensors.hpp) / [EMGSensors.cpp](src/sensors/EMGSensors.cpp)**
- ADS1115 I2C driver; spins a dedicated worker thread for continuous sampling (configurable 8–860 SPS via `ADS1115_Config`)
- Delivers samples via `std::function` callback — zero-copy, decoupled from consumers
- Uses blocking I2C reads for hardware-accurate timing (no `sleep`-based polling)
- Documented latency budget: ~1.26 ms single-sample, ~24 ms end-to-end with RMS windowing

**[EMGLogger](include/EMGLogger.hpp) / [src/EMGLogger.cpp](src/EMGLogger.cpp)**
- Receives raw voltage samples from the EMGSensors callback
- 6-second calibration phase (~500 samples) to establish a per-user baseline RMS
- Rolling 50-sample RMS window for classification:
  - `rest`: ratio ≤ 1.5× baseline
  - `flex`: 1.5× < ratio ≤ 3.0×
  - `strong_flex`: ratio > 3.0×
- Writes `emg_log.csv`: `timestamp_ms, emg_voltage_V, rms_V, baseline_rms_V, state`

**[apps/main.cpp](apps/main.cpp)**
- Wires `EMGSensors` → `EMGLogger` callback, configures I2C bus 1, address `0x48`, 860 SPS, ±4.096 V range
- Installs `SIGINT`/`SIGTERM` handlers using an `std::atomic<bool>` for safe shutdown

### Threading model

- **Main thread**: configures hardware, blocks waiting for shutdown signal
- **Worker thread** (inside `EMGSensors`): runs the I2C read loop, invokes the callback synchronously on each sample

Keep EMGLogger callback logic fast — it runs on the sensor worker thread and directly affects sampling latency.

### Hardware

- Raspberry Pi 5, ADS1115 on I2C bus 1 at address `0x48`
- MyoWare 2.0 EMG sensor connected to ADS1115 channel 2 (`AIN2`)
- 5× servo motors (finger actuation) — motor control not yet implemented

## Current development state

- EMG acquisition, calibration, classification, and CSV logging: complete
- Motor control integration: in progress (feature branch `feature/pipeline-class`)
- No multi-channel support yet
