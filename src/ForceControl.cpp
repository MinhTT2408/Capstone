// ForceControl.cpp - Outer loop force PID control implementation
// Cascaded control structure: Force PID (outer) -> Position PID (inner)

#include "ForceControl.h"
#include "config.h"

namespace ForceControl {

// ===================== INTERNAL STATE =====================
static PIDController forcePID;
static float forceSetpoint = DEFAULT_FORCE_SETPOINT;
static float lastMeasuredForce = 0.0f;
static float desiredRevolutions = 0.0f;
static unsigned long lastUpdateTime = 0;
static bool forceControlEnabled = false;

// Peak force tracking for sequence-based control
static float peakForce = 0.0f;
static bool isTrackingPeak = false;
static float currentAmplitude = 5.0f;  // Initial amplitude

// ADC calibration constants
static const float ADC_MAX = 4095.0f;  // 12-bit ADC
static const float ADC_REF_VOLTAGE = 3.3f;

// ===================== IMPLEMENTATION =====================

void begin() {
  // Configure ADC for force sensor
  pinMode(FORCE_SENSOR_PIN, INPUT);
  
  // Set ADC resolution to 12 bits (0-4095)
  analogReadResolution(12);
  
  // Set ADC attenuation for full 0-3.3V range
  analogSetPinAttenuation(FORCE_SENSOR_PIN, ADC_11db);
  
  // Initialize force PID controller
  forcePID.init(
    FORCE_PID_KP,
    FORCE_PID_KI,
    FORCE_PID_KD,
    MIN_REVOLUTIONS,
    MAX_REVOLUTIONS,
    -FORCE_PID_INTEGRAL_LIMIT,
    FORCE_PID_INTEGRAL_LIMIT
  );
  
  // Set initial setpoint
  forcePID.setSetpoint(forceSetpoint);
  
  // Initialize timing
  lastUpdateTime = millis();
  
  Serial.println("Force control initialized:");
  Serial.printf("  Sensor pin: GPIO %d\n", FORCE_SENSOR_PIN);
  Serial.printf("  Force range: %.1f - %.1f N\n", FORCE_MIN, FORCE_MAX);
  Serial.printf("  Output range: %.1f - %.1f revolutions\n", MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  Serial.printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", FORCE_PID_KP, FORCE_PID_KI, FORCE_PID_KD);
  Serial.printf("  Sample time: %.1f ms (%.0f Hz)\n", FORCE_PID_SAMPLE_TIME_MS, 1000.0f / FORCE_PID_SAMPLE_TIME_MS);
}

int readRawSensor() {
  return analogRead(FORCE_SENSOR_PIN);
}

float readForce() {
  // Read ADC value (0-4095)
  int rawValue = readRawSensor();
  
  // Convert to voltage
  float voltage = (rawValue / ADC_MAX) * ADC_REF_VOLTAGE;
  
  // Map voltage to force (linear mapping)
  float force = voltage * 10; 
  // Clamp to valid range
  force = constrain(force, FORCE_MIN, FORCE_MAX);
//   Serial.printf("Debug: Raw ADC=%d, Voltage=%.2f V, Force=%.2f N\n", rawValue, voltage, force);

  return force;
}

void setForceSetpoint(float force) {
  forceSetpoint = constrain(force, FORCE_MIN, FORCE_MAX);
  forcePID.setSetpoint(forceSetpoint);
  Serial.printf("Force setpoint updated: %.2f N\n", forceSetpoint);
}

float getForceSetpoint() {
  return forceSetpoint;
}

float update() {
  if (!forceControlEnabled) {
    return desiredRevolutions;
  }
  
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0f;  // Convert to seconds
  
  // Read current force
  lastMeasuredForce = readForce();
  
  // Compute PID output (desired revolutions)
  // Note: PID computes error = setpoint - current internally
  desiredRevolutions = forcePID.compute(lastMeasuredForce, dt);
  
  // Ensure output is within bounds
  desiredRevolutions = constrain(desiredRevolutions, MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  
  lastUpdateTime = now;
  
  return desiredRevolutions;
}

float getDesiredRevolutions() {
  return desiredRevolutions;
}

float getLastMeasuredForce() {
  return lastMeasuredForce;
}

void reset() {
  forcePID.reset();
  desiredRevolutions = 0.0f;
  lastMeasuredForce = 0.0f;
  lastUpdateTime = millis();
  Serial.println("Force control reset");
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
  Serial.printf("  Measured Force: %.2f N\n", lastMeasuredForce);
  Serial.printf("  Force Error: %.2f N\n", forceSetpoint - lastMeasuredForce);
  Serial.printf("  Output (Revolutions): %.3f rev\n", desiredRevolutions);
  Serial.printf("  Raw ADC: %d\n", readRawSensor());
  Serial.println("============================\n");
}

void startPeakTracking() {
  peakForce = 0.0f;
  isTrackingPeak = true;
  Serial.println("[Force Control] Peak tracking started");
}

void updatePeakTracking() {
  if (!isTrackingPeak || !forceControlEnabled) {
    return;
  }
  
  // Read current force
  float currentForce = readForce();
  
  // Update peak if current is higher
  if (currentForce > peakForce) {
    peakForce = currentForce;
  }
}

float getPeakForce() {
  isTrackingPeak = false;  // Stop tracking when reading peak
  return peakForce;
}

float computeNextAmplitude() {
  // Get the peak force from this sequence
  float measuredPeak = getPeakForce();
  
  // Calculate error
  float error = forceSetpoint - measuredPeak;
  
  // Proportional control for amplitude adjustment
  // If force is too low, increase amplitude; if too high, decrease amplitude
  float amplitudeAdjustment = error * FORCE_PID_KP;
  
  // Update amplitude
  currentAmplitude += amplitudeAdjustment;
  
  // Clamp to valid range
  currentAmplitude = constrain(currentAmplitude, MIN_REVOLUTIONS, MAX_REVOLUTIONS);
  
  // Update desiredRevolutions so getDesiredRevolutions() returns correct value
  desiredRevolutions = currentAmplitude;
  
  Serial.printf("[Force Control] Peak Force: %.2f N | Setpoint: %.2f N | Error: %.2f N\n",
                measuredPeak, forceSetpoint, error);
  Serial.printf("[Force Control] Amplitude adjustment: %+.2f | Next amplitude: %.2f rev\n",
                amplitudeAdjustment, currentAmplitude);
  
  return currentAmplitude;
}

float getInitialAmplitude() {
  return currentAmplitude;
}

} // namespace ForceControl
