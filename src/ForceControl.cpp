// ForceControl.cpp - Outer loop force PID control implementation
// Cascaded control structure: Force PID (outer) -> Position PID (inner)

#include "ForceControl.h"
#include "config.h"

namespace ForceControl {

// ===================== INTERNAL STATE =====================
static PIDController forcePID[3];
static float forceSetpoint = DEFAULT_FORCE_SETPOINT;
static float lastMeasuredForce[3] = {0.0f, 0.0f, 0.0f};
static float desiredRevolutions[3] = {0.0f, 0.0f, 0.0f};
static unsigned long lastUpdateTime = 0;
static bool forceControlEnabled = false;

// Peak force tracking for sequence-based control
static float peakForce[3] = {0.0f, 0.0f, 0.0f};
static bool isTrackingPeak[3] = {false, false, false};
static float currentAmplitude[3] = {5.0f, 5.0f, 5.0f};  // Initial amplitude per motor

// ADC calibration constants
static const float ADC_MAX = 4095.0f;  // 12-bit ADC
static const float ADC_REF_VOLTAGE = 3.3f;

// ===================== IMPLEMENTATION =====================

void begin() {
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);

  // Configure each sensor pin and PID controller
  for (int i = 0; i < 3; i++) {
    pinMode(FORCE_SENSOR_PINS[i], INPUT);
    analogSetPinAttenuation(FORCE_SENSOR_PINS[i], ADC_11db);

    forcePID[i].init(
      FORCE_PID_KP,
      FORCE_PID_KI,
      FORCE_PID_KD,
      MIN_REVOLUTIONS,
      MAX_REVOLUTIONS,
      -FORCE_PID_INTEGRAL_LIMIT,
      FORCE_PID_INTEGRAL_LIMIT
    );
    forcePID[i].setSetpoint(forceSetpoint);
  }

  // Initialize timing
  lastUpdateTime = millis();

  Serial.println("Force control initialized (3 independent sensors):");
  Serial.printf("  Sensor pins: GPIO %d, %d, %d\n",
                FORCE_SENSOR_PINS[0], FORCE_SENSOR_PINS[1], FORCE_SENSOR_PINS[2]);
  Serial.printf("  Force range: %.1f - %.1f N\n", FORCE_MIN, FORCE_MAX);
  Serial.printf("  Output range: %.1f - %.1f revolutions\n", MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  Serial.printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", FORCE_PID_KP, FORCE_PID_KI, FORCE_PID_KD);
  Serial.printf("  Sample time: %.1f ms (%.0f Hz)\n", FORCE_PID_SAMPLE_TIME_MS, 1000.0f / FORCE_PID_SAMPLE_TIME_MS);
}

int readRawSensor(int motorIndex) {
  return analogRead(FORCE_SENSOR_PINS[motorIndex]);
}

float readForce(int motorIndex) {
  // Read ADC value (0-4095)
  int rawValue = readRawSensor(motorIndex);
  
  // Convert to voltage
  float voltage = (rawValue / ADC_MAX) * ADC_REF_VOLTAGE;
  
  // Map voltage to force (linear mapping)
  float force = voltage * 10; 
  // Clamp to valid range
  force = constrain(force, FORCE_MIN, FORCE_MAX);

  return force;
}

void setForceSetpoint(float force) {
  forceSetpoint = constrain(force, FORCE_MIN, FORCE_MAX);
  for (int i = 0; i < 3; i++) {
    forcePID[i].setSetpoint(forceSetpoint);
  }
  Serial.printf("Force setpoint updated: %.2f N (all 3 motors)\n", forceSetpoint);
}

float getForceSetpoint() {
  return forceSetpoint;
}

float update(int motorIndex) {
  if (!forceControlEnabled) {
    return desiredRevolutions[motorIndex];
  }
  
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0f;  // Convert to seconds
  
  // Read current force for this motor
  lastMeasuredForce[motorIndex] = readForce(motorIndex);
  
  // Compute PID output (desired revolutions)
  desiredRevolutions[motorIndex] = forcePID[motorIndex].compute(lastMeasuredForce[motorIndex], dt);
  
  // Ensure output is within bounds
  desiredRevolutions[motorIndex] = constrain(desiredRevolutions[motorIndex], MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  
  lastUpdateTime = now;
  
  return desiredRevolutions[motorIndex];
}

float getDesiredRevolutions(int motorIndex) {
  return desiredRevolutions[motorIndex];
}

float getLastMeasuredForce(int motorIndex) {
  return lastMeasuredForce[motorIndex];
}

void reset() {
  for (int i = 0; i < 3; i++) {
    forcePID[i].reset();
    desiredRevolutions[i] = 0.0f;
    lastMeasuredForce[i] = 0.0f;
  }
  lastUpdateTime = millis();
  Serial.println("Force control reset (all 3 motors)");
}

bool isUpdateTime() {
  return (millis() - lastUpdateTime) >= FORCE_PID_SAMPLE_TIME_MS;
}

void setEnabled(bool enabled) {
  forceControlEnabled = enabled;
  if (enabled) {
    reset();  // Reset PID when enabling
    Serial.println("Force control ENABLED - outer loop active");
  } else {
    Serial.println("Force control DISABLED - using manual amplitude");
  }
}

bool isEnabled() {
  return forceControlEnabled;
}

void printStatus() {
  Serial.println("\n=== Force Control Status ===");
  Serial.printf("  Enabled: %s\n", forceControlEnabled ? "YES" : "NO");
  Serial.printf("  Force Setpoint: %.2f N\n", forceSetpoint);
  for (int i = 0; i < 3; i++) {
    Serial.printf("  Motor %d | Sensor GPIO %d | Force: %.2f N | Error: %.2f N | Output: %.3f rev | ADC: %d\n",
                  i + 1, FORCE_SENSOR_PINS[i],
                  lastMeasuredForce[i],
                  forceSetpoint - lastMeasuredForce[i],
                  desiredRevolutions[i],
                  readRawSensor(i));
  }
  Serial.println("============================\n");
}

void startPeakTracking() {
  for (int i = 0; i < 3; i++) {
    peakForce[i] = 0.0f;
    isTrackingPeak[i] = true;
  }
  Serial.println("[Force Control] Peak tracking started (all motors)");
}

void startPeakTracking(int motorIndex) {
  peakForce[motorIndex] = 0.0f;
  isTrackingPeak[motorIndex] = true;
}

void updatePeakTracking() {
  if (!forceControlEnabled) return;

  for (int i = 0; i < 3; i++) {
    if (!isTrackingPeak[i]) continue;
    float currentForce = readForce(i);
    if (currentForce > peakForce[i]) {
      peakForce[i] = currentForce;
    }
  }
}

float getPeakForce(int motorIndex) {
  isTrackingPeak[motorIndex] = false;  // Stop tracking when reading peak
  return peakForce[motorIndex];
}

float computeNextAmplitude(int motorIndex) {
  // Get the peak force from this sequence for this motor
  float measuredPeak = getPeakForce(motorIndex);
  
  // Calculate error
  float error = forceSetpoint - measuredPeak;
  
  // Proportional control for amplitude adjustment
  float amplitudeAdjustment = error * FORCE_PID_KP;
  
  // Update amplitude for this motor
  currentAmplitude[motorIndex] += amplitudeAdjustment;
  
  // Clamp to valid range
  currentAmplitude[motorIndex] = constrain(currentAmplitude[motorIndex], MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  
  // Update desiredRevolutions so getDesiredRevolutions() returns correct value
  desiredRevolutions[motorIndex] = currentAmplitude[motorIndex];
  
  Serial.printf("[Force Control M%d] Peak: %.2f N | SP: %.2f N | Error: %.2f N | Adj: %+.2f | Next amp: %.2f rev\n",
                motorIndex + 1, measuredPeak, forceSetpoint, error,
                amplitudeAdjustment, currentAmplitude[motorIndex]);
  
  return currentAmplitude[motorIndex];
}

float getInitialAmplitude(int motorIndex) {
  return currentAmplitude[motorIndex];
}

} // namespace ForceControl
