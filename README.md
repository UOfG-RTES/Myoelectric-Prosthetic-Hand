# Real-Time Myoelectric Prosthetic Hand — Raspberry Pi / C++

## Project Description

This project presents the design and implementation of a **real-time myoelectric prosthetic hand control system** using a **Raspberry Pi** and **C++**. The system acquires electromyographic (EMG) signals from forearm muscles, digitizes them through an **ADS1115 analog-to-digital converter**, performs calibration and signal processing, classifies muscle activity, and uses the resulting control decisions to drive a multi-finger prosthetic hand.

The project combines:

* embedded signal acquisition
* EMG-based user calibration
* baseline-driven gesture classification
* motor actuation for a 3D-printed prosthetic hand
* practical experimentation with sensor placement and gain tuning

The overall goal is to create a robust and low-latency control pipeline that can adapt to different users through an initial calibration stage.

---

## System Overview

```text
Forearm muscle activity
        ↓
MyoWare 2.0 EMG Sensor
        ↓
ADS1115 ADC
        ↓
Raspberry Pi 5
        ↓
Calibration + baseline estimation
        ↓
Noise filtering
        ↓
Signal classification
(Rest / Flex / Strong Flex)
        ↓
Motor control logic
        ↓
3D-printed prosthetic hand
```

---

## Current Project Status

At the current stage of the project, the following components have been completed:

* Raspberry Pi setup and remote configuration using SSH
* circuit design for connecting the **MyoWare 2.0 EMG sensor**, **ADS1115**, and **Raspberry Pi 5**
* motor connection circuit for the prosthetic hand
* 3D printing of the prosthetic hand structure
* initial EMG signal acquisition and testing
* repeated testing with different users and multiple sensor placement positions
* gain adjustment experiments to improve signal quality
* implementation of a calibration-first acquisition strategy

During testing, obtaining a clean and usable EMG signal was initially difficult. Multiple rounds of sensor repositioning, user trials, and gain adjustment were carried out before a reliable signal acquisition setup was achieved. Based on these observations, the system design was refined so that the first few seconds of runtime are dedicated to user-specific calibration.

---

## Implemented Sensor Architecture

The current EMG acquisition module is implemented as an **ADS1115-based I²C driver** in C++. It opens the Raspberry Pi I²C device, configures the ADS1115 address, and launches a worker thread for continuous sample acquisition. The driver supports multiple sampling rates and converts raw ADC readings into physical voltages. The current implementation reads **channel 2 only**, which was chosen to improve acquisition speed compared with reading all four channels. 

The corresponding configuration interface defines:

* I²C bus number
* ADS1115 device address
* supported sample rates from 8 to 860 samples per second
* programmable full-scale voltage range

These configuration details are exposed through `EMGSettings`, while the `EMGSensors` class handles sampling and callback-based delivery of new EMG data. 

---

## Hardware Required

| Component                              |       Qty | Notes                                                    |
| -------------------------------------- | --------: | -------------------------------------------------------- |
| Raspberry Pi 5                         |         1 | Running Raspberry Pi OS                                  |
| ADS1115 16-bit ADC                     |         1 | I²C analog-to-digital conversion for EMG acquisition     |
| MyoWare 2.0 EMG Sensor               | 1 | Current software implementation reads one active channel |
| 3D-printed prosthetic hand             |         1 | Mechanical hand structure                                |
| Servo motors                           |         5 | One per finger                                           |
| External power supply                  |         1 | Required for stable servo operation                      |
| Jumper wires / breadboard / connectors | as needed | For sensor and motor wiring                              |
| Status LED + resistor                  |  optional | For state indication used during integration              |

---

## Wiring

### ADS1115 → Raspberry Pi (I²C)

```text
ADS1115        Raspberry Pi Header
VDD     ────── 5V
GND     ────── GND
SCL     ────── SCL
SDA     ────── SDA
ADDR    ────── GND  (for address 0x48)
```

### EMG Sensor → ADS1115

```text
EMG Output   ────── ADS1115 AIN2
EMG GND      ────── Common GND
EMG VCC      ────── Sensor supply
```

> Note: The present implementation reads **ADS1115 channel 2** as the active EMG input path. 

### Motor Connections

The motor connection circuit has been completed. Each finger is intended to be controlled through its corresponding actuator channel, integrated with the 3D-printed hand mechanism.

---

## Software Design

The software is structured around modular acquisition and processing stages:

### 1. EMG Acquisition

The Raspberry Pi communicates with the ADS1115 over I²C. The driver starts a worker thread that continuously performs conversions and forwards sampled voltages through a callback mechanism. Blocking I²C reads are used for timing rather than explicit `sleep()`-based polling. 

### 2. Initial Calibration

At startup, the system performs an initial **6-second calibration phase**. During this interval, EMG data is collected from the user while the sensor is attached to the forearm. This stage is used to estimate the **resting baseline** for that specific user.

This design was adopted because EMG amplitudes vary significantly depending on:

* sensor placement
* skin contact quality
* individual muscle characteristics
* gain configuration
* noise conditions

### 3. Baseline Estimation

After calibration, the system establishes a baseline corresponding to the user’s resting EMG level. This baseline is then used as a reference for later classification.

### 4. Signal Classification

The intended classification states are:

* **Rest**
* **Flex**
* **Strong Flex**

These states are determined relative to the calibrated baseline rather than by fixed universal thresholds. This improves adaptability across different users.

### 5. Noise Filtering

After baseline calibration, the next processing stage is noise filtering. This is included to improve robustness against electrical interference, motion artefacts, and inconsistent contact quality during acquisition.

### 6. Motor Control

Once muscle activity has been classified, the resulting control output can be mapped to motor actions for the prosthetic hand. The actuation circuit is already completed, enabling integration with the motor-control stage.

---

## Calibration Strategy

A user-independent thresholding approach did not perform reliably during testing. As a result, the project uses a **user-specific calibration window** at the beginning of operation.

### Calibration Procedure

1. Attach the EMG sensor to the user’s forearm.
2. Start the system.
3. For the first **6 seconds**, collect raw EMG data.
4. Estimate the baseline resting level.
5. Use the baseline to distinguish:

   * rest
   * flex
   * strong flex

### Rationale

This approach was chosen after testing different placements and multiple users, where raw signal amplitudes differed substantially. A baseline-based method provides a more stable starting point for classification.

### Note on Calibration Window Tuning

The calibration window size was also experimentally varied during development. However, those alternative configurations did not perform as reliably as the original working setup, so the current version retains the original successful implementation.

---

## Testing and Experimental Observations

Testing was carried out iteratively and involved multiple individuals. Key observations include:

* Signal quality was strongly dependent on electrode placement.
* Early trials produced weak or inconsistent EMG readings.
* Repeated adjustments to placement and gain improved signal acquisition.
* Different users exhibited different resting and activation amplitudes.
* A fixed threshold system was less reliable than a calibrated baseline approach.

These findings directly informed the present system design.

---

## Build and Run

```bash
# Install build tools
sudo apt update
sudo apt install -y cmake build-essential g++ i2c-tools

# Enable I2C on the Raspberry Pi
sudo raspi-config
# Interface Options -> I2C -> Enable

# Build the project
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

Run the program:

```bash
sudo ./prosthetic_hand
```

> Root access may be required depending on device and GPIO/I²C permissions.

---

## Relevant Source File

The current EMG acquisition implementation is based on the following uploaded source file:

* `EMGSensors.cpp` — ADS1115 I²C acquisition, worker-threaded sampling, voltage conversion, and single-channel EMG reading on channel 2. 

---

## Future Work

The next major steps are:

* complete end-to-end integration of EMG classification with motor actuation
* validate control reliability across more users
* refine filtering for better noise immunity
* expand from single-channel input to multi-channel acquisition if needed
* improve gesture-to-motion mapping for more natural hand movement
* quantitatively evaluate response latency and classification robustness

---

## Conclusion

This project has progressed from hardware setup and circuit design to practical EMG acquisition and calibration-driven control design. A major outcome of testing was the recognition that reliable operation depends on **user-specific calibration**, careful **sensor placement**, and appropriate **gain adjustment**. The current system reflects those insights and forms a solid basis for further integration of signal processing, classification, and prosthetic actuation.