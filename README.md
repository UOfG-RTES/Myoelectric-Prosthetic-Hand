# Real-Time Myoelectric Prosthetic Hand — Raspberry Pi / C++

## Hardware Required
| Component | Qty | Notes |
|---|---|---|
| Raspberry Pi 3/4/Zero 2W | 1 | with Raspberry Pi OS |
| MCP3008 10-bit SPI ADC | 1 | |
| MyoWare / DIY EMG module | 2 | Flexor CH0, Extensor CH1 |
| Servo motor (MG996R or similar) | 5 | One per finger |
| 5 V / 3 A power supply | 1 | Servos draw heavy current |
| Status LED + 330Ω resistor | 1 | GPIO 25 |

## Wiring

### MCP3008 → Raspberry Pi SPI0
```
MCP3008        RPi Header
VDD  ──────── 3.3 V (pin 1)
VREF ──────── 3.3 V (pin 1)
AGND ──────── GND   (pin 6)
CLK  ──────── SCLK  (pin 23 / GPIO 11)
DOUT ──────── MISO  (pin 21 / GPIO 9)
DIN  ──────── MOSI  (pin 19 / GPIO 10)
CS/SHDN ───── CE0   (pin 24 / GPIO 8)
DGND ──────── GND
CH0  ──────── Flexor EMG out
CH1  ──────── Extensor EMG out
```

### Servo GPIO pins
```
Finger   GPIO
Thumb    17
Index    27
Middle   22
Ring     23
Little   24
```

## Build & Run

```bash
# 1. Install dependencies
sudo apt update
sudo apt install -y libpigpio-dev cmake build-essential

# 2. Enable SPI
sudo raspi-config  # → Interface Options → SPI → Enable

# 3. Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# 4. Run (pigpio requires root or pigpiod daemon)
sudo ./prosthetic_hand
```

## Calibration / Tuning

Edit thresholds in `gesture_classifier.h`:
- `FLEX_THRESHOLD` — minimum flexor RMS (mV) to trigger grip
- `EXT_THRESHOLD`  — minimum extensor RMS (mV) to trigger release
- `CO_THRESHOLD`   — co-contraction level to cycle gestures
- `CO_WINDOW_MS`   — how long to hold co-contraction for gesture switch

After connecting electrodes, observe telemetry output:
```
flex= 45.2 mV  ext= 22.1 mV  grip=0.00  [Rest]  state=0
```
Set `FLEX_THRESHOLD` to ~50–70% of your max voluntary contraction RMS.

## Grip Patterns

| Pattern | Thumb | Index | Middle | Ring | Little |
|---|---|---|---|---|---|
| Rest | 0% | 0% | 0% | 0% | 0% |
| Power Grip | 90% | 100% | 100% | 100% | 100% |
| Pinch | 80% | 90% | 0% | 0% | 0% |
| Tripod | 80% | 90% | 90% | 0% | 0% |
| Lateral | 50% | 100% | 100% | 100% | 100% |
| Point | 0% | 0% | 100% | 100% | 100% |

Cycle through patterns via 300 ms co-contraction.
