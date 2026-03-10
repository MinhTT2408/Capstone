# System Architecture & Module Workflow

## Overview

This firmware runs on an **ESP32 (esp-wrover-kit)** and implements a closed-loop compression therapy device. It combines three DC motor position controllers with a PPG sensor, BLE connectivity, and a cascaded force PID outer loop. The app (phone) connects over BLE and controls the intensity of compression. The system measures the resulting force against the patient and adjusts motor amplitude automatically.

---

## Hardware Summary

| Peripheral | GPIO(s) |
|---|---|
| Motor 1 PWM (RPWM / LPWM) | 25 / 26 |
| Motor 2 PWM (RPWM / LPWM) | 32 / 33 |
| Motor 3 PWM (RPWM / LPWM) | 12 / 13 |
| Encoder M1 (A / B) | 22 / 23 |
| Encoder M2 (A / B) | 18 / 19 |
| Encoder M3 (A / B) | 34 / 35 |
| Limit Sensor M1 / M2 / M3 | 21 / 2 / 5 |
| Force Sensor (ADC) | 4 |
| PPG Sensor I2C (SDA / SCL) | 15 / 14 |

---

## Module Map

```
┌─────────────────────────────────────────────────────────┐
│                        main.cpp                         │
│   setup() ──► initialises all modules in sequence       │
│   loop()  ──► orchestrates BLE session vs standalone    │
└──────┬────────┬──────────┬──────────┬───────────────────┘
       │        │          │          │
       ▼        ▼          ▼          ▼
  PPGModule  ForceControl  MotionControl  LimitSensors
       │        │          │    │
       │        │          │    ├── EncoderModule (PCNT)
       │        │          │    └── PIDController (×3)
       │        │          └── BTS7960 (×3, LEDC PWM)
       │        └── PIDController (force outer loop)
       └── MAX30105 (I2C), BLE stack
```

---

## Module Responsibilities

### `config.h`
Central header included by every module. Defines all GPIO pin constants, PID tuning values, PWM parameters, BLE UUIDs, and the `LEVEL_FORCE_SETPOINT[]` table.  
**No runtime code — constants only.**

---

### `PPGModule` (`include/PPGModule.h`, `src/PPGModule.cpp`)
Owns the BLE server and the MAX30105 PPG sensor.

**Initialised by:** `main.cpp → PPGModule::begin()`  
- Starts I2C on GPIO 15 (SDA) / 14 (SCL)  
- Configures MAX30105 sensor  
- Creates BLE server + 3 characteristics, starts advertising  

**Called every loop:** `main.cpp → PPGModule::update()`  
- Reads IR value from MAX30105, applies DC-removal filter, clamps to 0–1000  
- Sends PPG float over BLE notify at 50 Hz  
- Processes three RTOS-safe volatile flags set by BLE callbacks:

| Flag | Trigger | Processed action |
|---|---|---|
| `startSessionRequested` | App writes level 1–4 while session inactive | `startNewSession()` — sets `_isSessionActive = true`, clears `stopMotorRequested` |
| `endSessionRequested` | App writes level 0, or BLE disconnect | `doEndSession()` — sets `_isSessionActive = false`, sets `stopMotorRequested = true` |
| `fetchHistoryRequested` | App writes level 99 | Sends status message over History characteristic |

**Exposes to `main.cpp`:**
- `isSessionActive()` — whether a BLE session is running  
- `getCompressionLevel()` — current level (0–4)  
- `sessionJustEnded()` — one-shot flag, reset at start of next `update()` call  
- `stopMotorRequested` — `volatile bool`, read by every MotionControl inner loop for mid-cycle abort  

---

### `ForceControl` (`include/ForceControl.h`, `src/ForceControl.cpp`)
Outer PID loop. Reads the force sensor (GPIO 4 ADC) and computes the motor amplitude (in revolutions) needed to reach the desired compression force.

**Initialised by:** `main.cpp → ForceControl::begin()` then `ForceControl::setEnabled(true/false)`

**Called by `main.cpp` (both BLE session and standalone modes):**

| Call | When | Purpose |
|---|---|---|
| `setForceSetpoint(N)` | When BLE level changes | Updates desired force target from `LEVEL_FORCE_SETPOINT[level]` |
| `getInitialAmplitude()` | First sequence of a new setpoint | Returns a seed amplitude (default 5.0 rev) to start from |
| `startPeakTracking()` | Beginning of each motion sequence | Resets peak force accumulator |
| `updatePeakTracking()` | Inside MotionControl's 500 Hz PID loop | Samples force sensor, records the peak value |
| `computeNextAmplitude()` | After each pattern completes | Compares peak force to setpoint, adjusts amplitude proportionally: `amplitude += (setpoint - peakForce) × Kp` |
| `getDesiredRevolutions()` | Subsequent sequences | Returns the amplitude computed by the last `computeNextAmplitude()` call |
| `getLastMeasuredForce()` | Serial logging | Latest ADC reading in Newtons |

**Force → Amplitude logic (one cycle):**
```
Force setpoint (from BLE level)
        │
        ▼
   readForce()  ──► peak over sequence ──► peakForce
        │
   error = setpoint - peakForce
        │
   amplitudeAdjustment = error × FORCE_PID_KP (0.7)
        │
   nextAmplitude = clamp(currentAmplitude + adjustment, 1.0 – 6.0 rev)
```

---

### `MotionControl` (`include/MotionControl.h`, `src/MotionControl.cpp`)
Executes closed-loop sine-wave trajectories on the three BTS7960 motors. Owns three `PIDController` instances (one per motor, inner loop at 500 Hz / 2 ms).

**Initialised by:** `main.cpp → MotionControl::begin()`
- Initialises all three inner PID controllers with gains from `config.h`.

**Called by `main.cpp`:** one of `executePattern1()` … `executePattern4()`, passing the `amplitude` in revolutions computed by `ForceControl`.

**Inside every motion inner loop (2 ms tick):**
1. Check `PPGModule::stopMotorRequested` → if set, `motor.coast()` and return immediately
2. Check `LimitSensors::isTriggered(motorIndex)` → if triggered, coast and return
3. Call `ForceControl::updatePeakTracking()` → samples force sensor for peak tracking
4. Compute sine setpoint from elapsed time and amplitude
5. Read encoder position via `EncoderModule::getPosition(i)`
6. Run PID: `pidMotor[i].compute(currentPos, dt)` → PWM duty
7. Send duty + direction to `BTS7960::set()`

**Pattern variants:**

| Pattern | Enum | Behaviour |
|---|---|---|
| 1 | `PATTERN_SEQUENTIAL` | M1 → M2 → M3, full sine each, 400 ms pause between |
| 2 | `PATTERN_PHASE_OFFSET` | M1 & M3 start at t=0; M2 starts at t=SINE_DURATION/2 |
| 3 | `PATTERN_CASCADING` | Cascading half-cycles with hold states (state machine) |
| 4 | `PATTERN_SEQUENTIAL_THEN_PARALLEL` | Sequential ascending half-cycles, simultaneous return |

---

### `EncoderModule` (`include/Encoder.h`, `src/Encoder.cpp`)
Uses the ESP32 hardware **PCNT (Pulse Counter)** peripheral for zero-CPU quadrature decoding at X4 resolution. Maintains a 32-bit software accumulator to extend the 16-bit hardware counter.

**Called by:** `MotionControl` (inside the 500 Hz loop)
- `getPosition(i)` — returns accumulated 32-bit count
- `resetPosition(i)` — zeroes count before each sine cycle

---

### `PIDController` (`include/PIDController.h`, `src/PIDController.cpp`)
Generic reusable PID class with anti-windup integral clamping.

**Instantiated by:**
- `MotionControl` — 3 instances (position inner loop, 500 Hz)
- `ForceControl` — 1 instance (amplitude outer loop, per-sequence proportional)

**Key call:** `compute(current, dt)` → returns signed output using `error = setpoint - current`.

---

### `BTS7960` (`include/BTS7960.h`, `src/BTS7960.cpp`)
Thin wrapper around ESP32 LEDC PWM channels for one H-bridge.

**Called by:** `MotionControl` (inside every PID tick)
- `set(duty, FORWARD/REVERSE)` — drives one PWM channel high, other at duty
- `coast()` — both channels zero

---

### `LimitSensors` (`include/LimitSensors.h`, `src/LimitSensors.cpp`)
Three interrupt-driven digital inputs (one per motor). ISRs set bit-flags; no heavy work in ISR.

**Called by:**
- `MotionControl` inner loops — `isTriggered(i)` checked every 2 ms
- `main.cpp` — `checkPendingTriggers()` called between pattern cycles as a safety sweep

---

## Full Control Flow

### BLE Session Mode (app connected, level 1–4 written)

```
Phone App
  │  writes compression level 1–4 to BLE Level Characteristic
  ▼
PPGModule::LevelCallbacks::onWrite()
  │  sets startSessionRequested = true, compressionLevel = level
  ▼
PPGModule::update()  ← called every loop() iteration
  │  processes flag → startNewSession()
  │  clears stopMotorRequested
  ▼
main.cpp loop()
  │  detects PPGModule::isSessionActive() == true
  │  detects level change → ForceControl::setForceSetpoint(LEVEL_FORCE_SETPOINT[level])
  │
  ├─[seq 0]─► ForceControl::getInitialAmplitude()  → amplitude = 5.0 rev
  ├─[seq N]─► ForceControl::getDesiredRevolutions() → amplitude from last cycle's PID
  │
  ├─► ForceControl::startPeakTracking()   ← resets peak accumulator
  │
  ├─► MotionControl::executePatternX(motors, amplitude)
  │       │
  │       │  [every 2 ms inside pattern]
  │       ├─► PPGModule::stopMotorRequested?  → abort if set
  │       ├─► LimitSensors::isTriggered(i)?  → coast & abort if set
  │       ├─► ForceControl::updatePeakTracking()  ← records peak force
  │       ├─► EncoderModule::getPosition(i)
  │       ├─► PIDController::compute(pos, dt)  → PWM duty
  │       └─► BTS7960::set(duty, dir)
  │
  ├─► ForceControl::computeNextAmplitude()
  │       │  error = setpoint - peakForce
  │       │  nextAmplitude = clamp(current + error × 0.7, 1–6 rev)
  │       └─► updates desiredRevolutions for next sequence
  │
  ├─► LimitSensors::checkPendingTriggers()
  │
  └─► 1 s inter-cycle pause (PPGModule::update() polled every 10 ms during pause)
```

### Session End

```
Phone App writes level 0  (or BLE disconnect)
  ▼
PPGModule::LevelCallbacks::onWrite() / ServerCallbacks::onDisconnect()
  │  sets endSessionRequested = true, compressionLevel = 0
  ▼
PPGModule::update()
  │  processes flag → doEndSession()
  │  sets stopMotorRequested = true
  ▼
MotionControl inner loop (next 2 ms tick)
  │  checks PPGModule::stopMotorRequested == true
  │  motor.coast() → return
  ▼
main.cpp loop()  exits pattern call, returns to top of loop
  │  PPGModule::isSessionActive() == false  → falls to standalone mode
```

### Standalone Mode (no BLE session)

```
main.cpp loop()
  │  useForceControl == true?
  │
  ├─[yes]──► ForceControl::getInitialAmplitude() / getDesiredRevolutions()
  │           ForceControl::startPeakTracking()
  │           MotionControl::executePatternX(motors, amplitude)
  │           ForceControl::computeNextAmplitude()
  │
  └─[no]───► MotionControl::executePatternX(motors, DEFAULT_AMPLITUDE_REV = 5.0 rev)
```

---

## Compression Level → Force Setpoint → Amplitude

```
BLE Level  │  Force Setpoint  │  Amplitude range
───────────┼──────────────────┼──────────────────────────────────────
    1       │     8.0 N        │  Adjusted toward 8 N by PID (1–6 rev)
    2       │    13.0 N        │  Adjusted toward 13 N by PID
    3       │    18.0 N        │  Adjusted toward 18 N by PID
    4       │    25.0 N        │  Adjusted toward 25 N by PID
```

The amplitude is **not fixed per level**. Each completed motion pattern measures the peak force delivered, compares it to the setpoint, and the proportional controller adjusts the amplitude for the **next** sequence. Amplitude converges to the value that produces the desired compression force on the patient.

---

## Initialization Sequence (`setup()`)

```
Serial.begin(115200)
      ▼
BTS7960::begin()  × 3        ← LEDC PWM channels configured
      ▼
LimitSensors::begin()         ← GPIO INPUT_PULLUP + FALLING interrupts attached
      ▼
EncoderModule::begin()        ← PCNT units 0/1/2 configured, glitch filter enabled
      ▼
MotionControl::begin()        ← 3× PIDController::init() with gains from config.h
      ▼
ForceControl::begin()         ← ADC configured, force PID initialised
ForceControl::setEnabled()    ← enabled/disabled per useForceControl flag in main.cpp
      ▼
PPGModule::begin()            ← Wire.begin(15, 14), MAX30105 setup, BLE server start
```

---

## Key Tunable Parameters (`config.h`)

| Parameter | Default | Effect |
|---|---|---|
| `SINE_DURATION_MS` | 5000 ms | Duration of one full sine cycle per motor |
| `PID_KP / KI / KD` | 2.5 / 0.0 / 0.01 | Inner position PID gains |
| `FORCE_PID_KP / KI / KD` | 0.7 / 0.1 / 0.01 | Outer force PID gains |
| `MIN_REVOLUTIONS` | 1.0 rev | Floor for force-controlled amplitude |
| `MAX_REVOLUTIONS` | 6.0 rev | Ceiling for force-controlled amplitude |
| `DEFAULT_AMPLITUDE_REV` | 5.0 rev | Fixed amplitude when force control disabled and no BLE session |
| `LEVEL_FORCE_SETPOINT[]` | {0, 8, 13, 18, 25} N | Target force per BLE compression level |
| `BLE_PPG_INTERVAL_MS` | 20 ms | PPG BLE notify rate (50 Hz) |
