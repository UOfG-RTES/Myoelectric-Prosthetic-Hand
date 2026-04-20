# EMG-Controlled Prosthetic Hand — Real-Time Embedded System

> **ENG 5220 — Real-Time Embedded Programming | University of Glasgow**  
> A real-time C++ application on Raspberry Pi 5 that reads forearm muscle (EMG) signals via a MyoWare 2.0 sensor, classifies muscle activity using an Exponential Moving Average (EMA) filter with hysteresis, and drives servo motors to actuate a prosthetic hand.


## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Electrode Placement](#electrode-placement)
- [System Architecture](#system-architecture)
- [Data Flow](#data-flow)
- [Real-Time Design](#real-time-design)
- [Signal Processing & Classification](#signal-processing--classification)
- [Motor Control](#motor-control)
- [Hardware Debugging Log](#hardware-debugging-log)
- [EMG Data & Results](#emg-data--results)
- [Repository Structure](#repository-structure)
- [Build Instructions](#build-instructions)
- [Running the Application](#running-the-application)
- [Servo Calibration](#servo-calibration)
- [Dependencies](#dependencies)
- [Development State](#development-state)
- [Decisions Log](#decisions-log)
- [Team & Contributions](#team--contributions)
- [License](#license)


## Overview

This project implements a **real-time myoelectric (EMG) prosthetic hand controller** on a Raspberry Pi 5. A MyoWare 2.0 surface EMG sensor picks up forearm muscle electrical activity via skin-surface electrodes. An ADS1115 16-bit ADC samples the RECT output at **860 samples per second**. A dedicated C++ worker thread applies an **Exponential Moving Average (EMA)** filter with a **two-state hysteresis** classifier to distinguish rest from voluntary muscle contractions, and drives two servo motors on a PCA9685 PWM controller to actuate the prosthetic fingers.

**Why real-time?** Prosthetic control demands tight latency — a lag of more than ~100 ms between muscle contraction and mechanical response feels unnatural and makes the device hard to use. This system achieves a documented end-to-end latency of **~24 ms** from muscle activation to EMA classification.

## Hardware

| Component | Details |
|---|---|
| **Compute** | Raspberry Pi 5 |
| **EMG Sensor** | MyoWare 2.0 — RECT pin in use (see [Hardware Debugging Log](#hardware-debugging-log)) |
| **ADC** | ADS1115 — 16-bit, I²C bus 1, address `0x48` |
| **ADC Channel** | AIN2 |
| **ADC PGA Range** | ±0.512 V (8× resolution improvement over default ±4.096 V) |
| **ADC Sample Rate** | 860 SPS |
| **Servo Driver** | PCA9685 — 16-channel 12-bit PWM, I²C bus 1, address `0x40` |
| **Servo: Close** | Channel 0 — pulls fingers closed |
| **Servo: Open** | Channel 2 — releases fingers open |
| **OS** | Raspberry Pi OS (Linux) |

### I²C Device Map

```
sudo i2cdetect -y 1

     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
40: 40 -- -- -- -- -- -- -- 48 -- -- -- -- -- -- --
70: 70 -- -- -- -- -- -- --
```

| Address | Device | Notes |
|---|---|---|
| `0x40` | PCA9685 | Servo PWM driver |
| `0x48` | ADS1115 | EMG ADC |
| `0x70` | PCA9685 broadcast | Normal — ignore |

### Wiring Overview

```
MyoWare 2.0 RECT pin ──► ADS1115 AIN2
ADS1115 SDA / SCL ────► Raspberry Pi 5 GPIO 2 / 3 (I²C bus 1)
PCA9685 SDA / SCL ────► Raspberry Pi 5 GPIO 2 / 3 (I²C bus 1)
PCA9685 CH0 ───────────► Close servo (pulls fingers)
PCA9685 CH2 ───────────► Open servo  (releases fingers)
PCA9685 V+  ───────────► 5 V servo power rail (separate from 3.3 V logic VCC)
```

> **Important:** Enable I²C on the Pi before use — `sudo raspi-config` → Interface Options → I2C → Enable → reboot.

---

## Electrode Placement

The MyoWare 2.0 sensor is placed on the **forearm flexor muscle group** (flexor digitorum superficialis), approximately one-third of the way down the forearm from the elbow. The two differential electrodes straddle the muscle belly; the reference electrode is placed on a bony prominence near the wrist.

| State | Photo |
|---|---|
| **Fist clenched (flex)** | ![MyoWare sensor — flex](sensor_flex.png) |
| **Hand relaxed (rest)** | ![MyoWare sensor — rest](sensor_rest.png) |

Correct electrode placement is critical — a shift of even 10–20 mm significantly changes baseline RMS and threshold ratios. The 6-second calibration phase at startup compensates for placement variation between sessions.

---

## System Architecture

The codebase is divided into two independent libraries (`emgsensors`, `motorcontrol`) and two executables (`main`, `servo_calibration`), following the **Single Responsibility** and **Dependency Inversion** SOLID principles. The two subsystems communicate exclusively via callbacks — neither library holds a reference to the other.

### Key Components

#### `EMGSensors` — ADS1115 I²C Driver
- Spins a **dedicated worker thread** for continuous I²C sampling at up to 860 SPS
- Configurable sample rate via `ADS1115_Config` (8–860 SPS)
- Delivers samples via a `std::function` **callback** — zero-copy, fully decoupled from consumers
- Uses **blocking I²C reads** — no `sleep()`-based polling; timing is hardware-accurate
- Latency: ~1.26 ms per single conversion

#### `EMGLogger` — EMA Classifier, Logger & Motor Callback Hub
- 6-second per-user **calibration phase** at startup to establish a personal baseline EMA
- **EMA filter** (`α = 0.05`) on every post-calibration sample
- **Two-state hysteresis** classifier: engage at ratio > 1.5×, release at ratio < 1.2×
- `registerMotorCallback(std::function<void(float)>)` — fires on every classified sample, passing raw ratio to `MotorController`
- Writes `emg_log.csv` for offline analysis

> The `EMGLogger` callback runs on the sensor worker thread. Keep its logic fast to avoid sampling jitter.

#### `PCA9685` — PWM Driver
- I²C driver for the PCA9685 16-channel PWM controller
- Configured at 50 Hz, prescaler 121, servo range 205–410 counts (0°–180°)
- All `write()` / `read()` calls check return values and log to stderr on failure

#### `MotorController` — Hand Actuation
- Receives ratio via `update(float ratio)` from `EMGLogger`'s motor callback
- **Velocity-based control**: closing speed ∝ `(ratio − 1.5) / (6.0 − 1.5)`
- `HandState` enum: `OPEN`, `CLOSING`, `HOLDING`, `OPENING`
- Hysteresis band (1.2×–1.5×) prevents state chattering

---

## Data Flow

```
MyoWare 2.0 (RECT pin)
       │  analog voltage, 0–0.5 V
       ▼
ADS1115 @ 860 SPS, ±0.512 V PGA
       │  I²C blocking read, ~1.26 ms/sample
       ▼
EMGSensors worker thread
       │  std::function callback, zero-copy
       ▼
EMGLogger::onSample()
       ├─ EMA filter  (α = 0.05, ~23 ms time constant)
       ├─ Hysteresis classifier  (engage 1.5×, release 1.2×)
       ├─ emg_log.csv write
       └─ motor_cb_(ratio)
              │
              ▼
       MotorController::update()
              │
              ▼
       PCA9685::setAngle()  (~0.1 ms I²C write)
              │
              ▼
       Servo motors — CH0 (close), CH2 (open)
```

All processing runs on the **single sensor worker thread**. The PCA9685 I²C write takes < 0.1 ms — well within the 1.16 ms sample interval at 860 SPS.

---

## Real-Time Design

| Design decision | Rationale |
|---|---|
| **Dedicated worker thread** in `EMGSensors` | I²C reads block; isolating them prevents main-thread stalls |
| **`std::function` callbacks** for sensor and motor | Zero-copy, decoupled — no shared queues or polling |
| **Blocking I²C reads** (no `sleep`) | Hardware-accurate timing; avoids OS scheduler jitter |
| **EMA filter** (vs rolling RMS) | Single multiply-add, < 0.01 ms — vs ~58 ms rectangular window |
| **Motor I²C on worker thread** | PCA9685 writes < 0.1 ms at 860 SPS — no separate thread needed |
| **`std::atomic<bool>` shutdown flag** | Safe `SIGINT`/`SIGTERM` handling without mutexes |
| **PCA9685 init before `EMGLogger`** | Hand reaches open position during calibration, not mid-session |

### Latency Budget

| Stage | Latency |
|---|---|
| ADS1115 single conversion @ 860 SPS | ~1.26 ms |
| I²C read + callback dispatch | < 0.1 ms |
| EMA computation | < 0.01 ms |
| **EMG acquisition → classification** | **~24 ms** (EMA time constant) |
| PCA9685 I²C write | < 0.1 ms |
| **End-to-end: muscle → servo command** | **~24 ms** |

---

## Signal Processing & Classification

### Why RECT (not ENV or RAW)

The MyoWare 2.0 ENV envelope detector stage was **damaged** on the hardware used (see [Hardware Debugging Log](#hardware-debugging-log)). The RECT pin outputs rectified raw EMG without hardware smoothing; the software EMA replaces the missing filter:

```
RECT + software EMA  ≡  ENV (hardware-smoothed envelope)
```

### EMA Filter

```
ema_sq = α × v² + (1−α) × ema_sq
EMA    = √ema_sq
```

`α = 0.05` gives a time constant of ~23 ms at 860 SPS.

**EMA vs Rolling RMS:**

| | Rolling RMS (50 samples) | EMA (α = 0.05) |
|---|---|---|
| Decay on relax | Slow — 50 samples flush | Immediate exponential |
| Sticky states | Yes | No |
| Complexity | Deque + loop | Single multiply-add |
| Latency contribution | ~58 ms | < 0.01 ms |

### Two-State Hysteresis Classifier

| Transition | Ratio | Action |
|---|---|---|
| Rest → CLOSING | ratio > **1.5×** | Begin closing fingers |
| CLOSING/HOLDING | 1.2× ≤ ratio ≤ 1.5× | Hold current position |
| Any → OPENING | ratio < **1.2×** | Return to open |

**Data-driven threshold justification:**

| Metric | Value |
|---|---|
| Rest EMA max (noise ceiling) | 0.01448 V → **1.121× baseline** |
| Release threshold (1.2×) | Above noise ceiling — prevents chattering |
| Engage threshold (1.5×) | Clear gap above noise — requires intentional flex |

---

## Motor Control

Closing speed is proportional to contraction intensity:

```
speed = MAX_SPEED_DEG × (ratio − 1.5) / (6.0 − 1.5)
```

| Constant | Value | Meaning |
|---|---|---|
| `ENGAGE_RATIO` | 1.5× | Start closing |
| `RELEASE_RATIO` | 1.2× | Start opening |
| `MAX_RATIO` | 6.0× | Full speed (strong flex avg = 5.96×) |
| `MAX_SPEED_DEG` | 0.3°/sample | ~258°/s — full travel in ~0.7 s |

### PCA9685 Configuration

| Parameter | Value |
|---|---|
| I²C address | `0x40` |
| Servo frequency | 50 Hz |
| Prescaler | `round(25,000,000 / (4096 × 50)) − 1 = 121` |
| Servo min (0°) | 205 counts (1 ms pulse) |
| Servo max (180°) | 410 counts (2 ms pulse) |

---

## Hardware Debugging Log

### MyoWare 2.0 Pin Test Results

| Pin | Supply | Expected | Measured | Result |
|---|---|---|---|---|
| ENV | 3.3 V | ~1.65 V | 0.33 V |  Damaged |
| ENV | 5.0 V | ~2.50 V | 0.74 V |  Damaged |
| SIG/RAW | 5.0 V | ~2.50 V bias | ~2.50 V | Bias OK, no flex response |
| E+ pad | 5.0 V | — | ~2.50 V constant | Input only — not signal output |
| RECT | 3.3 V | ~0 V at rest | 0.002–0.0024 V | Working |
| RECT | 5.0 V | ~0 V at rest | ~0.004 V | Working |

**Active signal path:**
```
Forearm electrodes → MyoWare instrumentation amp → RECT pad
  → AIN2 → ADS1115 (±0.512 V PGA, 860 SPS) → EMGSensors → EMGLogger
```

### PGA Gain Fix

Initially configured at ±4.096 V — the RECT signal used only ~5% of ADC range. Switching to ±0.512 V gave **8× better voltage resolution** (0.0156 mV/bit vs 0.125 mV/bit). A register bug was caught: `0x0A00` (±0.256 V) was initially used instead of the correct `0x0800` (±0.512 V).

### PCA9685 Bring-Up Issues

| Issue | Root Cause | Resolution |
|---|---|---|
| Both I²C devices disappeared | Bad connection on shared SDA/SCL breadboard rail | Isolated ADS1115 directly to Pi; reseated wires |
| PCA9685 address reported as `0x48` | Confusion with ADS1115 address | Confirmed: PCA9685 default = `0x40` |
| PCA9685 VCC at 5 V | Wiring error | Corrected to 3.3 V logic VCC |

---

## EMG Data & Results

Five recording sessions were captured across the development and debugging phases. All plots below are generated directly from the CSV files in `data/`.

---

### Cross-Session Summary

![Cross-session summary](plot_summary.png)

| File | Samples | Duration | States |
|---|---|---|---|
| `emg_log.csv` | 23,911 | ~65 s | rest, flex, strong_flex |
| `emg_log_1.csv` | 35,447 | ~97 s | rest, flex, strong_flex |
| `emg_log_rest.csv` | 11,756 | ~32 s | rest only |
| `emg_log_flex.csv` | 9,754 | ~27 s | rest, flex, strong_flex |
| `emg_log_final.csv` | 42,701 | ~117 s | rest, flex, strong_flex |

---

### Rest Baseline Session (`emg_log_rest.csv`)

Pure resting recording used to validate that the noise floor stays well below the 1.2× release threshold. EMA ratio maximum observed: **1.121×** — confirming a comfortable margin before the 1.2× release threshold triggers.

![Rest baseline plots](plot_rest.png)

---

### Flex Test Session (`emg_log_flex.csv`)

Targeted flexing session capturing the transition between all three states. Shows the EMA tracking contraction onset and decay in real time.

![Flex session plots](plot_flex.png)

---

### Early Session — Before PGA Fix (`emg_log.csv`)

Recorded with the ADS1115 at ±4.096 V (default) — only ~5% of ADC range used. Raw ADC counts and the resulting compressed voltage range are visible. Classification still functioned via ratios, but absolute voltage values were unreliable for analysis.

![Early session plots](plot_emg_log.png)

---

### Extended Session — All Three States (`emg_log_1.csv`)

Longer session with sustained transitions across rest, flex, and strong flex. Includes RMS baseline tracking.

![Extended session plots](plot_emg_log1.png)

---

### Final Validated Session (`emg_log_final.csv`)

The primary validation dataset — 117 seconds of natural use. Classification boundaries align exactly with the data.

![Final session plots](plot_final.png)

**Classification performance (baseline = 0.0533 V):**

| State | Samples | % | Avg EMA | Avg Ratio |
|---|---|---|---|---|
| Rest | 31,790 | 74.5% | 0.0531 V | 1.00× |
| Flex | 7,775 | 18.2% | 0.1139 V | 2.14× |
| Strong flex | 3,136 | 7.3% | 0.3175 V | 5.96× |

Threshold boundaries are exact: min of flex band = 0.0799 V = 1.5 × 0.0533 V ✓  
Min of strong_flex band = 0.1599 V = 3.0 × 0.0533 V ✓

**CSV format (current schema):**
```
timestamp_ms, emg_voltage_V, rms_V, baseline_rms_V, state
4333, 0.052127, 0.053055, 0.053285, rest
```

---

## Repository Structure

```
RTES_EMG/
├── apps/
│   ├── main.cpp                # Main application — wires EMG → classify → motor
│   └── servo_calibration.cpp  # Interactive servo angle calibration utility
├── include/
│   ├── EMGLogger.hpp
│   ├── sensors/EMGSensors.hpp
│   └── motor/
│       ├── PCA9685.hpp
│       └── MotorController.hpp
├── src/
│   ├── EMGLogger.cpp
│   ├── sensors/EMGSensors.cpp
│   └── motor/
│       ├── PCA9685.cpp
│       └── MotorController.cpp
├── data/
│   ├── emg_log.csv
│   ├── emg_log_1.csv
│   ├── emg_log_rest.csv
│   ├── emg_log_flex.csv
│   └── emg_log_final.csv
├── docs/
│   ├── sensor_flex.png
│   ├── sensor_rest.png
│   ├── plot_rest.png
│   ├── plot_flex.png
│   ├── plot_final.png
│   ├── plot_emg_log.png
│   ├── plot_emg_log1.png
│   └── plot_summary.png
├── CMakeLists.txt
├── CLAUDE.md
├── .gitignore
├── LICENSE
└── README.md
```

---

## Build Instructions

### Prerequisites

```bash
sudo apt update
sudo apt install cmake g++ libi2c-dev

# Enable I²C
sudo raspi-config
# → Interface Options → I2C → Enable → reboot

# Confirm both devices are detected
sudo i2cdetect -y 1
# Expect: 0x40 (PCA9685) and 0x48 (ADS1115)
```

### Build

```bash
git clone https://github.com/<your-org>/RTES_EMG.git
cd RTES_EMG
mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

**Requirements:** `cmake >= 3.16`, `g++` (C++17), `libi2c-dev`, `pthread`.

---

## Running the Application

```bash
sudo ./main
```

**Startup sequence:**
1. I²C bus 1 opens; ADS1115 and PCA9685 initialised
2. Both servos move to their open positions
3. **6-second calibration phase** — keep forearm relaxed and still
4. Real-time EMA classification begins; `emg_log.csv` written continuously
5. `Ctrl+C` triggers clean shutdown via `SIGINT`

**Terminal output:**
```
  4352 ms | RAW:  12841 | Voltage: 0.2012V | EMA: 0.1139V | Ratio: 2.14x | [flex]
```

---

## Servo Calibration

```bash
sudo ./servo_calibration
```

```
ch angle> 0 45      # move close servo (ch 0) to 45°
ch angle> 2 120     # move open servo (ch 2) to 120°
ch angle> q         # quit
```

Record the fully-open and fully-closed angles for each servo, then update the four constants in `include/motor/MotorController.hpp` and rebuild.

---

## Dependencies

| Library | Purpose | Install |
|---|---|---|
| `libi2c-dev` | I²C bus access | `sudo apt install libi2c-dev` |
| `pthread` | POSIX threading | Included with GCC |
| `cmake >= 3.16` | Build system | `sudo apt install cmake` |
| `g++` (C++17) | Compiler | `sudo apt install g++` |

---

## Development State

| Feature | Status |
|---|---|
| ADS1115 I²C driver (`EMGSensors`) |  Complete |
| EMG calibration (6-second baseline) |  Complete |
| EMA filter (α = 0.05) |  Complete |
| Two-state hysteresis classifier |  Complete |
| CSV data logging |  Complete |
| PCA9685 PWM driver |  Complete |
| `MotorController` (velocity-based) |  Complete |
| Full pipeline integration in `main.cpp` |  Complete |
| Servo angle calibration utility |  Complete |
| Physical servo angle tuning |  Complete |
| Real-time plotting (ImGui + ImPlot) |  Complete |
| Multi-channel EMG support |  Complete |

**Active branch:** `feature/pipeline-class`

---

## Decisions Log

| Decision | Reason |
| RECT pin instead of ENV | ENV envelope detector stage damaged on board |
| PGA ±0.512 V | 8× better ADC resolution; covers RECT range with headroom |
| EMA α = 0.05 instead of rolling RMS | Immediate exponential decay, no sticky states, single multiply-add |
| Two states (rest / flex) over three | Simpler, more reliable motor control |
| Velocity control over position control | Proportional to contraction effort; more natural feel |
| MAX_SPEED_DEG = 0.3°/sample | ~0.7 s full travel at 860 SPS |
| MAX_RATIO = 6.0× | Data-driven: strong flex avg ratio = 5.96× |
| Engage 1.5×, release 1.2× | Rest noise ceiling = 1.121×; hysteresis gap prevents chattering |
| Motor callback on worker thread | PCA9685 writes < 0.1 ms — no separate thread needed |
| `registerMotorCallback` over direct pointer | Keeps `EMGLogger` fully decoupled |
| PCA9685 init before `EMGLogger` | Hand opens during calibration countdown |
| Standalone `servo_calibration` binary | Tune angles without recompiling main |
| Separate `motorcontrol` CMake library | Motor code independently compilable and testable |

---

## Team & Contributions


| Member | Role |
|---|---|
| Prathamesh Bagal | EMG sensor driver & I²C communication (`EMGSensors`) |
| Karthik Mandigiri | Signal classification & EMA filter (`EMGLogger`) |
| Suvrit Srivastava | Motor control — PCA9685 driver & `MotorController` |
| Pavan Akash | CMake build system, git management & testing |
| Aryan Fredrick | Hardware debugging, data collection & documentation |


instagram - @rtes2026_team21

---

## License

This project is licensed under the [MIT License](LICENSE).

---

> **University of Glasgow — School of Engineering**  
> ENG 5220 Real-Time Embedded Programming | 2025–2026
