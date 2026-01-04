# Motor Position Control - Variable Reference

## 🎯 **To Change Motor Position/Movement**

### **1. Change Position Amplitude (How Far Motor Moves)**

**Variable:** `amplitudeRevolutions` parameter in `MotionControl::runSineCycle()`

**Location:** `src/main.cpp` - when calling the function

**Current default:** `2.0f` revolutions (±2 full rotations)

**How to change:**

```cpp
// In main.cpp loop():
MotionControl::runSineCycle(motor[0], 0);  // Uses default 2.0 rev

// OR specify custom amplitude:
MotionControl::runSineCycle(motor[0], 0, 5.0f);  // ±5 revolutions
MotionControl::runSineCycle(motor[0], 0, 0.5f);  // ±0.5 revolutions (180°)
MotionControl::runSineCycle(motor[0], 0, 1.0f);  // ±1 revolution (360°)
```

**Effect:**
- Larger value = motor travels further
- Sine wave goes: `0 → +amplitude → 0 → -amplitude → 0`
- Example: `amplitudeRevolutions = 3.0` means motor swings ±3 full rotations from center

---

### **2. Change Movement Duration (How Fast Motor Moves)**

**Variable:** `SINE_DURATION_MS`

**Location:** `include/config.h`

**Current value:** `5000` (5 seconds)

**How to change:**

```cpp
// In config.h:
static const uint32_t SINE_DURATION_MS = 10000;  // 10 seconds (slower)
static const uint32_t SINE_DURATION_MS = 3000;   // 3 seconds (faster)
static const uint32_t SINE_DURATION_MS = 2000;   // 2 seconds (very fast)
```

**Effect:**
- Larger value = slower, smoother motion
- Smaller value = faster, more aggressive motion
- Too fast → motor can't keep up, large tracking errors

---

### **3. Change Movement Profile (Trajectory Shape)**

**Variable:** Position setpoint calculation in `MotionControl.cpp`

**Location:** `src/MotionControl.cpp` line ~58-60

**Current formula:**
```cpp
float theta = TWO_PI_F * progress;           // 0 to 2π
float sineValue = sinf(theta);               // -1 to +1
float targetPosition = sineValue * amplitudeCounts;
```

**Alternative profiles:**

**A) Square wave (move fast to position, hold, return):**
```cpp
float targetPosition;
if (progress < 0.25) {
  targetPosition = amplitudeCounts;  // Move to +position
} else if (progress < 0.5) {
  targetPosition = -amplitudeCounts; // Move to -position
} else if (progress < 0.75) {
  targetPosition = amplitudeCounts;  // Back to +position
} else {
  targetPosition = 0;                // Return to center
}
```

**B) Triangular wave (constant speed ramp):**
```cpp
float targetPosition;
if (progress < 0.5) {
  targetPosition = (2.0 * progress - 0.5) * 2.0 * amplitudeCounts; // Ramp up
} else {
  targetPosition = (1.5 - 2.0 * progress) * 2.0 * amplitudeCounts; // Ramp down
}
```

**C) Single direction sweep (0 → max → 0):**
```cpp
float sineValue = sinf(PI * progress);  // Half sine (0 to π)
float targetPosition = sineValue * amplitudeCounts;  // Always positive
```

**D) Step response (instant position change):**
```cpp
float targetPosition = (progress < 0.5) ? amplitudeCounts : -amplitudeCounts;
```

---

### **4. Change Individual Motor Positions Independently**

**Method 1: Call with different amplitudes**
```cpp
void loop() {
  MotionControl::runSineCycle(motor[0], 0, 1.0f);  // Motor 1: ±1 rev
  delay(PAUSE_BETWEEN_MS);
  
  MotionControl::runSineCycle(motor[1], 1, 3.0f);  // Motor 2: ±3 rev
  delay(PAUSE_BETWEEN_MS);
  
  MotionControl::runSineCycle(motor[2], 2, 0.5f);  // Motor 3: ±0.5 rev
  delay(PAUSE_BETWEEN_MS);
}
```

**Method 2: Create custom motion function**
```cpp
// Add to MotionControl.h:
void moveToPosition(BTS7960& motor, int motorIndex, float targetRevolutions);

// Add to MotionControl.cpp:
void moveToPosition(BTS7960& motor, int motorIndex, float targetRevolutions) {
  float targetCounts = targetRevolutions * COUNTS_PER_REV;
  
  // Simple move-to-position with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10 sec timeout
    long currentPosition = EncoderModule::getPosition(motorIndex);
    pidMotor[motorIndex].setSetpoint(targetCounts);
    float pidOutput = pidMotor[motorIndex].compute((float)currentPosition, 0.01f);
    
    Dir direction = (pidOutput >= 0) ? FORWARD : REVERSE;
    int duty = constrain(abs((int)pidOutput), 0, PWM_MAX);
    motor.set(duty, direction);
    
    // Check if reached target (within tolerance)
    if (abs(currentPosition - (long)targetCounts) < 50) {
      motor.coast();
      return;
    }
    delay(10);
  }
  motor.coast();
}

// Use in main.cpp:
MotionControl::moveToPosition(motor[0], 0, 2.5f);   // Move to +2.5 rev
MotionControl::moveToPosition(motor[0], 0, -1.0f);  // Move to -1.0 rev
MotionControl::moveToPosition(motor[0], 0, 0.0f);   // Return to zero
```

---

## 📊 **Summary Table**

| What to Change | Variable | File | Effect |
|----------------|----------|------|--------|
| **Distance traveled** | `amplitudeRevolutions` parameter | `main.cpp` | Larger = further swing |
| **Speed of motion** | `SINE_DURATION_MS` | `config.h` | Smaller = faster |
| **Motion shape** | `targetPosition` formula | `MotionControl.cpp` | Sine/Square/Triangle/etc |
| **Per-motor distance** | Pass different values to `runSineCycle()` | `main.cpp` | Independent control |
| **Absolute position** | Create `moveToPosition()` function | Custom | Move to specific position |

---

## 🔧 **Quick Examples**

**Make motor swing ±10 revolutions:**
```cpp
MotionControl::runSineCycle(motor[0], 0, 10.0f);
```

**Make motion twice as fast:**
```cpp
// In config.h:
static const uint32_t SINE_DURATION_MS = 2500;  // Was 5000
```

**Move motor to specific position (not sine wave):**
See "Method 2" above to add `moveToPosition()` function.

---

## ⚠️ **Important Notes**

1. **Encoder counts** = `revolutions × COUNTS_PER_REV` (default: 2400 counts/rev)
2. **Position wraps** if you don't reset encoder (encoder is cumulative)
3. **PID may struggle** with very fast motions (increase `SINE_DURATION_MS`)
4. **Always check** that amplitude doesn't cause mechanical collision with limit switches

---

## 🐛 **Debugging Position Issues**

**Motor doesn't move far enough:**
- Increase `amplitudeRevolutions`
- Check if PID can't keep up (increase `PID_KP` or slow down motion)

**Motor overshoots:**
- Decrease `PID_KP`, increase `PID_KD`
- Check for mechanical backlash

**Irregular motion:**
- Check encoder wiring (counts should be smooth)
- Verify `COUNTS_PER_REV` matches your encoder spec
- Look at Serial debug: Error and PWM should correlate
