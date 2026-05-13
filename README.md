# ESP32-S3 Compression Therapy System

## Project Overview

This project implements a **closed-loop compression therapy system** running on an ESP32-S3 DevKitC-1. Three DC motors deliver programmable sine-wave compression cycles with force feedback, BLE control from a phone app, PPG heart-rate monitoring via a MAX30105 sensor, and SD card session logging.

The system supports two operating modes:
- **BLE Session Mode** — A phone app connects and selects a compression level (1–4). The outer Force PID loop adjusts each motor's travel amplitude in real time to hit the target force setpoint.
- **Standalone Mode** — Runs continuously with force control or a fixed default amplitude (no BLE required).

---

## System Architecture

### Control Loop (Cascaded PID)

```
BLE Level (1-4)
    │
    ▼
Force Setpoint (N)
    │
    ▼
Force PID (50 Hz, outer loop)   ←── Force Sensor (ADC, per motor)
    │  outputs: amplitude (revolutions)
    ▼
Sine Trajectory Generator
    │  outputs: target position (encoder counts)
    ▼
Position PID (500 Hz, inner loop)  ←── Quadrature Encoder (PCNT)
    │  outputs: PWM duty cycle
    ▼
BTS7960 H-Bridge → DC Motor
```

The outer Force PID measures peak force per sequence and adjusts the next sequence's amplitude. The inner Position PID runs at 500 Hz and tracks the sine-wave target.

### Module Structure

```
include/
├── config.h          - All pin definitions and tunable parameters
├── BTS7960.h         - H-bridge motor driver interface
├── Encoder.h         - PCNT quadrature encoder interface
├── PIDController.h   - Generic PID controller
├── LimitSensors.h    - Interrupt-based limit switch interface
├── MotionControl.h   - Sine trajectory + 4 motion patterns
├── ForceControl.h    - Outer force PID loop (cascaded control)
└── PPGModule.h       - MAX30105 PPG sensor + BLE + SD card

src/
├── main.cpp          - Setup, main loop, mode selection
├── BTS7960.cpp       - LEDC PWM motor driver implementation
├── Encoder.cpp       - PCNT hardware encoder counting
├── PIDController.cpp - PID computation with anti-windup
├── LimitSensors.cpp  - GPIO interrupt limit sensor handling
├── MotionControl.cpp - Trajectory generation and 4 patterns
├── ForceControl.cpp  - Force sensor ADC read + amplitude PID
└── PPGModule.cpp     - BLE server, MAX30105, SD card logging
```

---

## Pin Map (ESP32-S3 DevKitC-1)

All pins are defined in `include/config.h`.

### Motor Drivers (BTS7960 H-Bridge)

| Motor | RPWM (Forward) | LPWM (Reverse) | R_EN / L_EN |
|-------|---------------|----------------|-------------|
| M1 | GPIO 14 | GPIO 15 | Hardwired HIGH |
| M2 | GPIO 16 | GPIO 21 | Hardwired HIGH |
| M3 | GPIO 40 | GPIO 41 | Hardwired HIGH |

> R_EN and L_EN are hardwired to 5V (`M1_REN = -1` in config). Connect BTS7960 GND to ESP32 GND.

### LEDC PWM Channel Allocation

| Motor | RPWM Channel | LPWM Channel |
|-------|-------------|-------------|
| M1 | CH 0 | CH 1 |
| M2 | CH 2 | CH 3 |
| M3 | CH 4 | CH 5 |

### Quadrature Encoders (PCNT hardware)

| Motor | Channel A | Channel B |
|-------|----------|----------|
| M1 | GPIO 42 | GPIO 2 |
| M2 | GPIO 1 | GPIO 3 |
| M3 | GPIO 47 | GPIO 48 |

### Limit Sensors (one per motor, active LOW / INPUT_PULLUP)

| Motor | Sensor Pin |
|-------|-----------|
| M1 | GPIO 7 |
| M2 | GPIO 8 |
| M3 | GPIO 9 |

### Force Sensors (analog, 12-bit ADC)

| Motor | ADC Pin |
|-------|--------|
| M1 | GPIO 4 |
| M2 | GPIO 5 |
| M3 | GPIO 6 |

Force conversion: `Force (N) = (ADC / 4095) × 3.3V × 11.25`

### PPG Sensor — MAX30105 (I2C)

| Signal | Pin |
|--------|-----|
| SDA | GPIO 17 |
| SCL | GPIO 18 |

### SD Card (SPI)

| Signal | Pin |
|--------|-----|
| CS | GPIO 10 |
| MOSI | GPIO 11 |
| SCK | GPIO 12 |
| MISO | GPIO 13 |

---

## Operating Modes

### Mode 1: BLE Session Mode

1. Phone app connects over BLE and writes a **compression level (1–4)** to the Level characteristic.
2. The level maps to a **force setpoint (N)**:

   | Level | Description | Force Setpoint |
   |-------|-------------|---------------|
   | 0 | Off / Stop | 0.0 N |
   | 1 | Light | 8.0 N |
   | 2 | Medium-Light | 13.0 N |
   | 3 | Medium | 18.0 N |
   | 4 | Firm | 25.0 N |

3. The Force PID adjusts motor amplitude each sequence to reach the target force.
4. PPG data is streamed over BLE at 50 Hz and logged to SD card.
5. Sending level `0` or disconnecting ends the session.
6. Sending `99` fetches session history from SD card over BLE.

**To enable/disable BLE session mode:** It is always active. The system returns to Standalone Mode automatically when no BLE session is running.

### Mode 2: Standalone Mode (no BLE)

Controlled by the `useForceControl` flag in `main.cpp`:

```cpp
bool useForceControl = true;   // Amplitude driven by force sensor PID
// or
bool useForceControl = false;  // Fixed amplitude (DEFAULT_AMPLITUDE_REV = 5.0 rev)
```

- When `useForceControl = true`: Force PID adjusts amplitude per motor each cycle.
- When `useForceControl = false`: All motors use a fixed `5.0` revolution amplitude.

---

## Motion Patterns

Select the active pattern in `main.cpp`:

```cpp
MotionControl::MotionPattern currentPattern = MotionControl::PATTERN_SEQUENTIAL;
```

| Value | Enum | Description |
|-------|------|-------------|
| 1 | `PATTERN_SEQUENTIAL` | One motor at a time (default) |
| 2 | `PATTERN_PHASE_OFFSET` | M1 & M3 simultaneous, M2 with delayed start |
| 3 | `PATTERN_CASCADING` | Sequential half-cycles with hold states at peak and zero |
| 4 | `PATTERN_SEQUENTIAL_THEN_PARALLEL` | Sequential ascent (0→peak), simultaneous descent (peak→0) |

### Motor State Machine (used in all patterns)

```
IDLE → MOVING_FIRST_HALF → HOLD_AT_HALF → MOVING_SECOND_HALF → HOLD_AT_FULL
```

Each half-cycle follows a sine trajectory. Direction: motor moves to negative peak (reverse) on the first half and returns to zero on the second half.

---

## Force Control Details

Force control is a **sequence-based outer loop** (not continuous PID):

1. At the start of each sequence, peak force tracking begins.
2. The motor runs at the current amplitude.
3. After the sequence completes, the peak force measured is compared to the setpoint:
   ```
   amplitude_next = amplitude_current + (force_setpoint - peak_force) × Kp
   ```
4. Amplitude is clamped to **1.0 – 6.0 revolutions**.

Force PID parameters (`ForceControl.h`):

| Parameter | Value |
|-----------|-------|
| Kp | 0.7 |
| Ki | 0.1 |
| Kd | 0.01 |
| Sample rate | 50 Hz (20 ms) |
| Min amplitude output | 1.0 rev |
| Max amplitude output | 6.0 rev |
| Default setpoint | 17.5 N |
| Initial amplitude | 5.0 rev (all motors) |

---

## Position PID Parameters

Defined in `include/config.h`:

| Parameter | Value |
|-----------|-------|
| Kp | 2.5 |
| Ki | 0.0 |
| Kd | 0.01 |
| Sample rate | 500 Hz (2 ms) |
| Output limit | ±1023 (full PWM range) |
| Anti-windup limit | ±500 |

---

## Encoder Configuration

| Parameter | Value |
|-----------|-------|
| PPR (Pulses Per Revolution) | 2125 |
| x4 PCNT decoding | 8500 counts/rev |
| PCNT glitch filter | 100 APB cycles (~1.25 µs) |
| Angular resolution | ~0.042° per count |

---

## PWM & Timing

| Parameter | Value |
|-----------|-------|
| PWM frequency | 20 kHz (inaudible) |
| PWM resolution | 10-bit (0–1023) |
| Sine cycle duration | 5000 ms per motor |
| Pause between motors | 400 ms |
| Inter-cycle pause (BLE mode) | 1000 ms |

---

## BLE Interface

**Device name:** configured via BLE stack (NimBLE / Arduino BLE)

| Characteristic | UUID | Direction | Description |
|---------------|------|-----------|-------------|
| PPG Data | `6e400002-...e24dcca9e` | Notify → App | Raw PPG value at 50 Hz |
| Level Control | `6e400003-...e24dcca9e` | Write ← App | Compression level (0–4, or 99=history) |
| Session History | `6e400004-...e24dcca9e` | Read/Notify → App | Fetched SD session data |

Full service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`

---

## SD Card Logging

- **PPG + level log:** XOR-encrypted CSV, one file per session (`/0001.csv`, `/0002.csv`, …), written at 2 Hz.
- **Force log:** Plaintext CSV, written at 50 Hz during active sessions. Columns: `elapsed_ms, force_M1_N, force_M2_N, force_M3_N`.
- **Encryption key:** `0x5A` (XOR cipher on all non-newline characters).
- SD write task runs on Core 0; force log task runs on Core 1 at 50 Hz.

---

## Power Connections

```
┌──────────────┐         ┌──────────────┐
│  USB / 5V    │         │  12–24V 10A+ │
│  Power       │         │  DC Supply   │
└──────┬───────┘         └──────┬───────┘
       │                        │
       ├── ESP32-S3 (5V)        ├── BTS7960 VCC (motor voltage)
       ├── Encoders (3.3V)      ├── Motor Power (M+)
       ├── Force sensors (3.3V) └── ⚠ SHARE GND with ESP32 ⚠
       └── MAX30105 (3.3V)
```

- **DO NOT** power motors from the ESP32 or USB power.
- **MUST** share GND between ESP32 and all BTS7960 drivers.

---

## Quick Start

### 1. Hardware Setup
1. Connect ESP32-S3 via USB.
2. Wire all components per the pin map above.
3. Connect the motor power supply (12–24 V).
4. Verify common GND between ESP32 and BTS7960 boards.

### 2. Build & Upload (PlatformIO)
```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor
```

Board target: `esp32-s3-devkitc-1`

### 3. Expected Serial Output on Boot
```
========================================
ESP32 + BTS7960: Closed-Loop Position Control
========================================

✓ Motor drivers initialized
✓ Limit sensors initialized
✓ Encoders initialized
PID Controllers initialized:
Kp=2.50, Ki=0.00, Kd=0.01
Encoder PPR=2125, Counts/Rev=8500
PID Sample Rate: 2.0ms (500Hz)
✓ Motion control initialized
✓ Force control initialized
✓ PPG + BLE initialized

>>> FORCE CONTROL MODE: Amplitude from force sensor <<<
    Force setpoint: 17.5 N
```

---

## Configuration Reference (`include/config.h`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SINE_DURATION_MS` | 5000 ms | Duration of one sine cycle per motor |
| `PAUSE_BETWEEN_MS` | 400 ms | Pause after each motor finishes |
| `PWM_FREQ` | 20000 Hz | Motor PWM frequency |
| `PWM_RESBIT` | 10 | PWM resolution bits (0–1023) |
| `ENCODER_PPR` | 2125 | Encoder pulses per revolution |
| `PCNT_FILTER_VAL` | 100 | PCNT glitch filter (APB cycles) |
| `PID_KP / KI / KD` | 2.5 / 0.0 / 0.01 | Position PID gains |
| `PID_SAMPLE_TIME_MS` | 2.0 ms | Position PID update rate (500 Hz) |
| `LEVEL_FORCE_SETPOINT[]` | {0, 8, 13, 18, 25} N | Force targets for BLE levels 0–4 |
| `DEFAULT_AMPLITUDE_REV` | 5.0 rev | Amplitude when force control is off |
| `SD_WRITE_INTERVAL_MS` | 500 ms | PPG log write rate (2 Hz) |
| `FORCE_LOG_INTERVAL_MS` | 20 ms | Force log write rate (50 Hz) |
| `BLE_PPG_INTERVAL_MS` | 50 ms | BLE PPG notify rate (20 Hz) |
| `ENCRYPTION_KEY` | 0x5A | XOR key for SD card CSV encryption |

---

## Tuning Guide

### Position PID

1. Start: `Kp=1.0`, `Ki=0`, `Kd=0`
2. Increase `Kp` until oscillation, then reduce by 30%.
3. Add `Kd` to damp oscillations (`Kd ≈ Kp / 10`).
4. Add `Ki` only if steady-state error is unacceptable (`Ki ≈ Kp / 5`).

### Force PID (amplitude loop)

- `FORCE_PID_KP = 0.7` — amplitude step per Newton of error per sequence
- Increase if force converges slowly; decrease if amplitude oscillates between sequences.
- `MIN_REVOLUTIONS = 1.0` and `MAX_REVOLUTIONS = 6.0` clamp output.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Motor does not move | No motor power / enable pins | Verify 12–24 V supply; check R_EN/L_EN wired HIGH |
| Encoder count stays at 0 | Wrong pins or power | Verify pin map; check 3.3 V encoder supply |
| Position overshoots/oscillates | `PID_KP` too high | Reduce `PID_KP`; increase `PID_KD` |
| Force never reaches setpoint | `FORCE_PID_KP` too low | Increase `FORCE_PID_KP` |
| Amplitude saturates at max | Force sensor not reading | Check GPIO 4/5/6 wiring; verify ADC 11 dB attenuation |
| BLE not advertising | BLE init failure | Check Serial for BLE error messages |
| SD card not found | SPI wiring / no card | Verify CS=10, MOSI=11, SCK=12, MISO=13; check card format (FAT32) |
| Limit sensor not triggering | Wiring type mismatch | Sensors are INPUT_PULLUP active-LOW; verify NC/NO sensor type |
| ESP32 resets or crashes | Stack overflow / bad pin | Check forceLogTask stack (2048 words); verify all GPIO numbers |

---

## System Specifications

| Parameter | Value |
|-----------|-------|
| Microcontroller | ESP32-S3 DevKitC-1 |
| Position PID rate | 500 Hz (2 ms) |
| Force loop rate | 50 Hz (20 ms) |
| PPG BLE stream rate | 50 Hz (20 ms) |
| Force log rate | 50 Hz (20 ms) |
| PWM frequency | 20 kHz |
| PWM resolution | 10-bit (0–1023) |
| Encoder resolution | 8500 counts/rev (2125 PPR × 4) |
| Angular accuracy | ~0.042° per count |
| Force sensor range | 0–100 N (0–3.3 V ADC, 12-bit) |
| Amplitude range | 1.0–6.0 revolutions |
| BLE compression levels | 4 (8 / 13 / 18 / 25 N) |
| Max motors | 3 |

---

## License

Provided as-is for educational / capstone project purposes.

## Author

Created for Capstone Project — December 2025
