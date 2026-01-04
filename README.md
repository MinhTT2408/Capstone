# ESP32 Closed-Loop Motor Position Control System

## 📋 Project Overview

This project implements a **closed-loop position control system** for 3 DC motors using an ESP32 microcontroller. The system uses:
- **BTS7960 H-Bridge drivers** for bidirectional motor control
- **Quadrature encoders** for precise position feedback
- **PID controllers** for accurate position tracking
- **Limit sensors** for safety/end-stop detection

Motors follow a programmable sine-wave trajectory with real-time position correction via PID feedback control.

---

## 🎯 Key Features

✅ **Closed-loop position control** - Motors track precise position setpoints  
✅ **PID feedback** - Automatic error correction (100Hz update rate)  
✅ **Quadrature encoding** - 4x resolution for accurate position sensing  
✅ **Safety limit switches** - Immediate motor stop on sensor trigger  
✅ **Modular code architecture** - Clean separation of concerns  
✅ **Serial debugging** - Real-time position and PWM monitoring  

---

## 🏗️ System Architecture

### **Control Flow:**
```
Sine Wave Generator → Target Position → PID Controller → Motor Driver → Physical Motor
                                             ↑                              ↓
                                             └──────── Encoder Feedback ────┘
```

### **Module Structure:**
```
include/
├── config.h          - Pin definitions & tunable parameters
├── BTS7960.h         - Motor driver interface
├── Encoder.h         - Position tracking
├── PIDController.h   - Control algorithm
├── LimitSensors.h    - Safety sensors
└── MotionControl.h   - High-level motion control

src/
├── main.cpp          - Main program loop
├── BTS7960.cpp       - Motor driver implementation
├── Encoder.cpp       - Encoder ISR & position tracking
├── PIDController.cpp - PID computation
├── LimitSensors.cpp  - Interrupt-based sensor handling
└── MotionControl.cpp - Trajectory generation & control
```

---

## 🔌 Hardware Connections

### **ESP32 Pin Mapping**

#### **Motor 1 Connections**
| Component | Function | ESP32 Pin | Wire Color (suggested) |
|-----------|----------|-----------|------------------------|
| BTS7960 #1 | RPWM (Forward PWM) | GPIO 25 | Yellow |
| BTS7960 #1 | LPWM (Reverse PWM) | GPIO 26 | Orange |
| BTS7960 #1 | R_EN (Enable) | 5V or GPIO* | Red |
| BTS7960 #1 | L_EN (Enable) | 5V or GPIO* | Red |
| BTS7960 #1 | GND | GND | Black |
| Encoder M1 | Channel A | GPIO 36 (input-only) | Blue |
| Encoder M1 | Channel B | GPIO 39 (input-only) | Green |
| Encoder M1 | VCC | 5V | Red |
| Encoder M1 | GND | GND | Black |
| Limit Sensor M1 | Forward | GPIO 13 | White |
| Limit Sensor M1 | Reverse | GPIO 12 | Brown |
| Limit Sensor M1 | GND | GND | Black |

#### **Motor 2 Connections**
| Component | Function | ESP32 Pin | Wire Color (suggested) |
|-----------|----------|-----------|------------------------|
| BTS7960 #2 | RPWM | GPIO 32 | Yellow |
| BTS7960 #2 | LPWM | GPIO 33 | Orange |
| BTS7960 #2 | R_EN | 5V or GPIO* | Red |
| BTS7960 #2 | L_EN | 5V or GPIO* | Red |
| BTS7960 #2 | GND | GND | Black |
| Encoder M2 | Channel A | GPIO 34 (input-only) | Blue |
| Encoder M2 | Channel B | GPIO 35 (input-only) | Green |
| Encoder M2 | VCC | 5V | Red |
| Encoder M2 | GND | GND | Black |
| Limit Sensor M2 | Forward | GPIO 18 | White |
| Limit Sensor M2 | Reverse | GPIO 27 | Brown |
| Limit Sensor M2 | GND | GND | Black |

#### **Motor 3 Connections**
| Component | Function | ESP32 Pin | Wire Color (suggested) |
|-----------|----------|-----------|------------------------|
| BTS7960 #3 | RPWM | GPIO 16 | Yellow |
| BTS7960 #3 | LPWM | GPIO 17 | Orange |
| BTS7960 #3 | R_EN | 5V or GPIO* | Red |
| BTS7960 #3 | L_EN | 5V or GPIO* | Red |
| BTS7960 #3 | GND | GND | Black |
| Encoder M3 | Channel A | GPIO 14 | Blue |
| Encoder M3 | Channel B | GPIO 15 | Green |
| Encoder M3 | VCC | 5V | Red |
| Encoder M3 | GND | GND | Black |
| Limit Sensor M3 | Forward | GPIO 22 | White |
| Limit Sensor M3 | Reverse | GPIO 21 | Brown |
| Limit Sensor M3 | GND | GND | Black |

*Note: R_EN and L_EN can be hardwired to 5V if always enabled. Set `M1_REN = -1` in `config.h` if not using GPIO control.

---

## ⚡ Power Connections

### **Critical: Use Separate Power Supplies**

```
┌──────────────┐         ┌──────────────┐
│   5V 2A USB  │         │  12-24V 10A+ │
│   Power      │         │   DC Supply  │
└──────┬───────┘         └──────┬───────┘
       │                        │
       ├── ESP32 (5V)           ├── BTS7960 VCC (12-24V)
       ├── Encoders (5V)        ├── Motor Power (M+)
       └── Sensors (5V)         └─── ⚠️ SHARE GND with ESP32 ⚠️
```

**Important:**
- **DO NOT** power motors from ESP32/USB
- **MUST** connect GND between ESP32 and BTS7960
- BTS7960 needs **12-24V** depending on motor rating
- ESP32 needs **5V** (USB or external regulator)

---

## 🔧 Configuration

### **Adjust Motor Travel Distance**

Edit `src/main.cpp`:
```cpp
void loop() {
  // Change the 3rd parameter (amplitude in revolutions)
  MotionControl::runSineCycle(motor[0], 0, 2.0f);  // ±2 full rotations
  MotionControl::runSineCycle(motor[1], 1, 5.0f);  // ±5 full rotations
  MotionControl::runSineCycle(motor[2], 2, 0.5f);  // ±0.5 rotation (180°)
}
```

### **Adjust Motion Speed**

Edit `include/config.h`:
```cpp
static const uint32_t SINE_DURATION_MS = 10000;  // 10 seconds per cycle (slower)
static const uint32_t SINE_DURATION_MS = 5000;   // 5 seconds (faster)
```

### **Tune PID Gains**

Edit `include/config.h`:
```cpp
static const float PID_KP = 2.5f;   // Proportional: increase for faster response
static const float PID_KI = 0.8f;   // Integral: increase to eliminate steady-state error
static const float PID_KD = 0.1f;   // Derivative: increase to reduce overshoot
```

**Tuning Guide:**
1. Start with `Ki=0`, `Kd=0`, `Kp=1.0`
2. Increase `Kp` until oscillation, then reduce by 30%
3. Add `Kd` to dampen oscillations (start with `Kd = Kp/10`)
4. Add `Ki` to eliminate steady-state error (start with `Ki = Kp/5`)

### **Match Your Encoder Specification**

Edit `include/config.h`:
```cpp
static const int ENCODER_PPR = 2125;  // Change to YOUR encoder's PPR
```

Common values:
- **600 PPR** - Standard incremental encoder
- **2048 PPR** - High-resolution encoder
- **400 PPR** - Low-cost encoder

---

## 🚀 Quick Start

### **1. Hardware Setup**
1. Connect ESP32 to computer via USB
2. Wire motors, encoders, and sensors per pin table above
3. Connect motor power supply (12-24V)
4. **Verify GND connection between ESP32 and BTS7960**

### **2. Software Setup**
```bash
# Clone or open project in VS Code with PlatformIO
cd Capstone

# Build project
pio run

# Upload to ESP32
pio run --target upload

# Open Serial Monitor (115200 baud)
pio device monitor
```

### **3. Expected Serial Output**
```
========================================
ESP32 + BTS7960: Closed-Loop Position Control
========================================

✓ Motor drivers initialized
✓ Limit sensors initialized
✓ Encoders initialized
PID Controllers initialized:
Kp=2.50, Ki=0.80, Kd=0.10
Encoder PPR=2125, Counts/Rev=8500
PID Sample Rate: 10.0ms (100Hz)
✓ Motion control initialized

Enc counts: M1=0, M2=0, M3=0  | deg: M1=0.0, M2=0.0, M3=0.0

--- Motor 1: sine cycle ---
Motor 1 | PWM:  234 FWD | Current Pos:   1250 counts | Target Pos:   1800 counts
Motor 1 | PWM:  456 FWD | Current Pos:   2340 counts | Target Pos:   3200 counts
...
Motor 1: Cycle complete. Final position: 0.012 rev (102 counts)
```

---

## 🐛 Troubleshooting

### **Motor doesn't move**
- Check power supply voltage (12-24V for BTS7960)
- Verify PWM wiring (RPWM, LPWM connected)
- Check enable pins (R_EN, L_EN should be HIGH)
- Test motor directly with battery

### **Encoder counts don't change**
- Verify encoder power (5V, GND)
- Swap A and B channels if counting backward
- Check encoder spec matches `ENCODER_PPR` in config
- Use input-only pins (34-39) for encoders

### **Position oscillates/overshoots**
- Reduce `PID_KP` (too aggressive)
- Increase `PID_KD` (damping)
- Check for mechanical backlash
- Verify encoder is securely mounted

### **Limit sensor not working**
- Sensors should be normally HIGH (INPUT_PULLUP)
- Trigger = LOW (short to GND)
- Verify sensor wiring and type (NC vs NO)

### **ESP32 crashes ("Guru Meditation Error")**
- Pin numbers invalid (check config.h)
- Don't use pins 34, 35, 36, 39 for OUTPUT
- Don't use pins > 39 (don't exist)
- Verify all pins match your ESP32 board

---

## 📊 System Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Control Rate** | 100 Hz (10ms) | PID update frequency |
| **PWM Frequency** | 20 kHz | Ultrasonic, motor-silent |
| **PWM Resolution** | 10-bit (0-1023) | LEDC hardware PWM |
| **Encoder Resolution** | 4x quadrature | 8500 counts/rev @ 2125 PPR |
| **Position Accuracy** | ±1 encoder count | ~0.042° @ 2125 PPR |
| **Max Motors** | 3 | Expandable with more GPIOs |
| **Safety Response** | <10ms | Limit sensor interrupt latency |

---

## 📝 Code Functions Summary

### **Main Control Flow (`main.cpp`)**
```cpp
void loop() {
  EncoderModule::printPositions();           // Print current positions
  MotionControl::runSineCycle(motor[0], 0);  // Run Motor 1
  delay(PAUSE_BETWEEN_MS);                   // Brief pause
  MotionControl::runSineCycle(motor[1], 1);  // Run Motor 2
  delay(PAUSE_BETWEEN_MS);
  MotionControl::runSineCycle(motor[2], 2);  // Run Motor 3
  LimitSensors::checkPendingTriggers();      // Safety check
}
```

### **Key API Functions**

#### **Motor Control (`BTS7960`)**
```cpp
motor.begin(rpwmCh, lpwmCh, freq, resBit);  // Initialize driver
motor.set(duty, direction);                  // Set speed & direction (duty: 0-1023)
motor.coast();                               // Freewheel stop
motor.brake();                               // Active brake
```

#### **Encoder Reading (`EncoderModule`)**
```cpp
EncoderModule::begin();                      // Setup pins & interrupts
long pos = EncoderModule::getPosition(0);    // Get motor 0 position (counts)
EncoderModule::resetPosition(0);             // Zero encoder
float deg = EncoderModule::countsToDegrees(pos);   // Convert to degrees
float rev = EncoderModule::countsToRevolutions(pos); // Convert to revolutions
EncoderModule::printPositions();             // Debug print all encoders
```

#### **PID Control (`PIDController`)**
```cpp
pid.init(kp, ki, kd, outMin, outMax, intMin, intMax);  // Initialize
pid.reset();                                  // Zero integral, reset state
pid.setSetpoint(targetPosition);              // Set desired position
float output = pid.compute(currentPos, dt);   // Compute control output
```

#### **Limit Sensors (`LimitSensors`)**
```cpp
LimitSensors::begin();                        // Setup pins & interrupts
bool hit = LimitSensors::isTriggered(0);      // Check if motor 0 sensor triggered
LimitSensors::clearFlags(0);                  // Acknowledge trigger
LimitSensors::checkPendingTriggers();         // Handle any pending events
```

#### **Motion Control (`MotionControl`)**
```cpp
MotionControl::begin();                       // Initialize PID controllers
MotionControl::runSineCycle(motor, index, amplitude);  // Run trajectory
// amplitude = revolutions (e.g., 2.0 = ±2 full rotations)
```

---

## 📚 Further Reading

- **PID Tuning:** See `POSITION_CONTROL_GUIDE.md` for detailed tuning instructions
- **Pin Reference:** ESP32 pinout: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- **BTS7960 Datasheet:** Motor driver specifications and wiring
- **Quadrature Encoding:** How x4 decoding works

---

## 📄 License

This project is provided as-is for educational purposes.

---

## 👤 Author

Created for Capstone Project - December 2025
