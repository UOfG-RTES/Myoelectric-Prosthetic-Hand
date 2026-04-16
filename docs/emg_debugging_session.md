# EMG Sensor Debugging & Calibration Session

This document records all hardware experiments, findings, code changes, and results from the MyoWare 2.0 EMG sensor debugging and signal validation session.

---

## Hardware Setup

| Component | Detail |
|---|---|
| Microcontroller | Raspberry Pi 5 |
| ADC | ADS1115 16-bit I2C, address `0x48`, I2C bus 1 |
| EMG Sensor | MyoWare 2.0 |
| ADC Channel | AIN2 |
| Power supply tested | 3.3 V and 5 V to MyoWare |

### MyoWare 2.0 Output Pins

| Pin | Description |
|---|---|
| ENV | Rectified + smoothed envelope output (hardware low-pass filter) |
| SIG / RAW | Amplified raw EMG, AC-coupled, biased at Vcc/2 |
| RECT | Rectified raw EMG — absolute value of AC signal, no smoothing |
| E+, E−, REF | Electrode snap inputs (inputs to the instrumentation amp, not outputs) |

---

## Initial Problem

The ENV pin was producing constant noisy output (`0.6–1.3 V hash`) that did not respond to muscle flexing at all. Suspected the envelope detector stage on the MyoWare board was damaged.

---

## Experiment 1 — Testing ENV Pin at Different Supply Voltages

**Goal:** Confirm whether the ENV output is behaving correctly.

**Method:** Read AIN2 (connected to ENV) with the ADS1115 configured at ±4.096 V PGA.

**Expected behaviour:** At rest, ENV should output approximately Vcc/2 and rise clearly during flex.

| Supply Voltage | Expected Resting RMS | Actual Resting RMS | Result |
|---|---|---|---|
| 3.3 V | ~1.65 V | **0.33 V** | FAIL |
| 5.0 V | ~2.50 V | **0.74 V** | FAIL |

**Finding:** ENV output was ~5× lower than expected and completely unresponsive to flexing. The values scaled loosely with supply voltage, suggesting a partial leakage path rather than a clean output.

**Conclusion:** The ENV envelope detector stage on this MyoWare board is **damaged/fried**. Most likely cause: accidental application of >3.3 V to the output pin, or a short.

---

## Experiment 2 — Testing RAW / SIG Pin

**Goal:** Determine if the front-end instrumentation amplifier is still intact.

**Method:** Move the wire from ENV → SIG pad on the MyoWare. Keep the other end on AIN2. No code changes required.

**Expected behaviour:** At rest, SIG should show a stable Vcc/2 DC bias (1.65 V at 3.3 V supply, 2.5 V at 5 V supply) with small AC fluctuations during flex.

| Supply | Expected Resting RMS | Actual Resting RMS | Result |
|---|---|---|---|
| 5.0 V | ~2.50 V | **~2.50 V** | PASS |

**Finding:** SIG showed a clean 2.5 V bias at rest — confirming the instrumentation amplifier is healthy.

**Problem observed:** The 2.5 V reading did not change during flexing. Suspected electrode placement issue.

---

## Experiment 3 — RAW Pin Not Responding to Flex

**Observed output (SIG/RAW connected to AIN2 at 5 V):**

```
RAW ADC:   4821  Voltage: 0.6026V
20613 ms  RMS: 0.5823V  ratio: 1.00x  [rest]
```

**Note:** 0.6 V is far below the expected 2.5 V resting bias.

**Hypothesis 1 — Electrode placement:** Ruled out once RECT was confirmed working (see Experiment 4). Electrodes and skin contact were fine.

**Hypothesis 2 — RAW output partially damaged:** The RAW pad itself may have a faulty bias circuit or damaged trace. Since RECT is derived internally from the same amplified signal, RECT can still function correctly even if the RAW pad output is affected.

---

## Experiment 4 — Testing RECT Pin

**Goal:** Validate RECT as a replacement for the broken ENV.

**Method:** Move wire from SIG → RECT pad. Keep on AIN2. No code changes.

**Expected behaviour:** RECT outputs `|EMG − Vcc/2|` — always positive, near 0 V at rest, rising proportionally with muscle activation. This is the same as ENV but without the hardware smoothing capacitor. The software 50-sample RMS window replaces the hardware low-pass filter.

### RECT Resting Values (before PGA fix)

| Supply | Resting RMS | Notes |
|---|---|---|
| 3.3 V | 0.0020 – 0.0024 V | Near noise floor — correct |
| 5.0 V | 0.0040 V | Near noise floor — correct |

**Why values differ between supplies:** The MyoWare's internal op-amps produce slightly more noise at higher supply voltages. This is expected physical behaviour.

**Result:** RECT gave correct rest / flex / strong_flex classification. Electrode placement was confirmed fine. The ENV and RAW pad issues are hardware-only faults on this board.

---

## Experiment 5 — Spare E Pad Test

**Observation:** When the wire was accidentally connected to one of the electrode snap pads (E+/E−) instead of RECT, the output was a constant ~2.5 V (at 5 V supply) regardless of flexing.

**Why:** The E pads are the **input** to the instrumentation amplifier, not outputs. They are biased at approximately Vcc/2 through internal resistors. The actual EMG signal at electrode level is 1–5 mV peak — completely buried under the 2.5 V DC bias and invisible before amplification.

**Lesson:** The E pads will always read ~Vcc/2 and will never show flex response. They are not signal outputs.

---

## Experiment 6 — PGA Gain Analysis

**Problem identified:** The ADS1115 was configured at ±4.096 V (lowest gain, widest range). The RECT signal only spans approximately 0–0.5 V. This means only ~5% of the ADC range was being used, wasting 95% of the available 16-bit resolution.

| PGA Setting | Full-scale Range | Resolution per bit | % of range used by RECT |
|---|---|---|---|
| ±4.096 V (original) | 4.096 V | 0.125 mV | ~5% |
| ±0.512 V (new) | 0.512 V | 0.0156 mV | ~40% |

**Improvement:** 8× better ADC resolution with the new PGA setting.

### RECT Values Before PGA Fix (±4.096 V)

| State | RMS Range |
|---|---|
| Rest | ~0.004 V |
| Flex | 0.01 – 0.02 V |
| Strong flex | 0.10 – 0.20 V |

**Classification ratios were borderline:**
- Flex: 2.5×–5× (threshold: 1.5×) — barely above threshold
- Strong flex: 25×–50× — technically fine but absolute values too small for reliable RMS

---

## Code Changes

### Change 1 — PGA Register Constant (EMGSensors.cpp)

Added the correct register value for ±0.512 V PGA and switched `startConversion()` to use it.

```cpp
// Added
static constexpr uint16_t PGA_0512 = 0x0800;  // ±0.512 V

// Changed in startConversion()
uint16_t config = OS_START | MUX_CH[channel] | PGA_0512  // was PGA_4096
                | MODE_SINGLE | drRateConfig() | COMP_DISABLE;
```

**Note — bug caught during session:** The first attempt used `0x0A00` for `PGA_0512`, which is actually **±0.256 V** in the ADS1115 register map, not ±0.512 V. This caused all reported voltages to be 2× the actual value (hardware clipping at 0.256 V but `toVolts()` dividing by 0.512). Corrected to `0x0800`.

ADS1115 PGA register reference:

| Register bits | Hex | Range |
|---|---|---|
| 000 | 0x0000 | ±6.144 V |
| 001 | 0x0200 | ±4.096 V |
| 010 | 0x0400 | ±2.048 V |
| 011 | 0x0600 | ±1.024 V |
| 100 | **0x0800** | **±0.512 V** ← correct |
| 101 | 0x0A00 | ±0.256 V ← initially used by mistake |

### Change 2 — pga_gain Default (EMGSensors.hpp)

Updated the voltage conversion factor to match the new hardware range:

```cpp
// Before
float pga_gain = 4.096f;

// After
float pga_gain = 0.512f;
```

This is used in `toVolts()`:
```cpp
float EMGSensors::toVolts(int16_t raw) const {
    return (static_cast<float>(raw) / 32767.0f) * settings_.pga_gain;
}
```

Both values must match. Mismatching them causes voltage readings to be scaled incorrectly (though classification still works since it uses ratios).

### Change 3 — RMS Window (EMGLogger.hpp)

Pre-existing uncommitted change included in the same commit:

```cpp
// Before
static constexpr int RMS_WINDOW = 20;

// After
static constexpr int RMS_WINDOW = 50;
```

Larger window gives smoother RMS values at the cost of slightly more averaging latency.

### Change 4 — .gitignore

Added `*.csv` to prevent test log files from being tracked:

```
build/
.CLAUDE.md
*.csv
```

---

## Results After PGA Fix

CSV analysed: `emg_log_final.csv`

**Baseline RMS:** 0.0533 V

| State | Samples | Avg RMS | Min RMS | Max RMS | Avg Ratio |
|---|---|---|---|---|---|
| Rest | 31,790 | 0.0531 V | 0.0361 V | 0.0799 V | 1.00× |
| Flex | 7,775 | 0.1139 V | 0.0799 V | 0.1598 V | 2.14× |
| Strong flex | 3,136 | 0.3175 V | 0.1599 V | 1.0535 V | 5.96× |

**Classification thresholds:**
- Flex threshold: 1.5 × 0.0533 = **0.0799 V** — min of flex band aligns exactly ✓
- Strong flex threshold: 3.0 × 0.0533 = **0.1599 V** — min of strong_flex band aligns exactly ✓

**Assessment:** Ratios are healthy. The boundaries are clean with no overlap between states. Strong flex has a wide range (3×–20× baseline), giving good headroom for motor control mapping.

---

## Summary of Hardware Findings

| Pin | Status | Reason |
|---|---|---|
| ENV | Damaged | Envelope detector circuit fried — likely overvoltage or short |
| SIG / RAW | Partially damaged | Resting bias incorrect (~0.6 V instead of Vcc/2); internal signal path intact |
| RECT | Working | Correctly outputs rectified EMG signal |
| E pads | N/A | Electrode inputs — always read ~Vcc/2, not signal outputs |

**Final signal path in use:**

```
Electrodes → MyoWare instrumentation amp → RECT pad → AIN2 → ADS1115 (±0.512 V PGA) → EMGSensors → EMGLogger
```

Software RMS window (50 samples) replaces the broken hardware envelope detector.

---

## Git

- **Branch:** `feature/pipeline-class`
- **Commit:** `fix: switch ADS1115 PGA to 0.512V range and increase RMS window to 50`
- **Files changed:** `include/EMGLogger.hpp`, `include/sensors/EMGSensors.hpp`, `src/sensors/EMGSensors.cpp`, `.gitignore`

---

## Next Steps

- Implement real-time plotting using **ImGui + ImPlot** (C++, OpenGL via GLFW)
- Planned architecture: `EMGPlotter` class registered as a second callback alongside `EMGLogger`
- Useful for visual noise analysis and documentation screenshots
- Prerequisite: confirm display server (`echo $XDG_SESSION_TYPE` → `x11` or `wayland`)
