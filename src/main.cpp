// ESP32 + 3x BTS7960: Sequential sine-wave position control with PID
// Modular architecture for clean, maintainable code
// Author: GitHub Copilot
// Date: December 2025

#include <Arduino.h>
#include "config.h"
#include "BTS7960.h"
#include "Encoder.h"
#include "PIDController.h"
#include "LimitSensors.h"
#include "MotionControl.h"
#include "ForceControl.h"
#include "PPGModule.h"

// ===================== MOTOR INSTANCES =====================
BTS7960 motor[3] = {
  BTS7960(M1_RPWM, M1_LPWM, M1_REN, M1_LEN),
  BTS7960(M2_RPWM, M2_LPWM, M2_REN, M2_LEN),
  BTS7960(M3_RPWM, M3_LPWM, M3_REN, M3_LEN)
};

// ===================== PATTERN SELECTION =====================
// Change this value to select motion pattern:
// 1 = Sequential (one motor at a time)
// 2 = Phase offset (M1 & M3 simultaneous, M2 delayed start)
// 3 = Cascading half-cycle (sequential with hold states)
// 4 = Sequential-then-parallel (sequential ascent, simultaneous descent)
MotionControl::MotionPattern currentPattern = MotionControl::PATTERN_SEQUENTIAL;

// ===================== FORCE CONTROL MODE =====================
// Set to true to enable outer force PID loop (amplitude controlled by force sensor)
// Set to false to use default amplitude (DEFAULT_AMPLITUDE_REV in config.h)
bool useForceControl = true;

// ===================== SETUP =====================
void setup() {
  // Initialize serial for logging
  Serial.begin(115200);
  delay(200);

  Serial.println("\n========================================");
  Serial.println("ESP32 + BTS7960: Closed-Loop Position Control");
  Serial.println("========================================\n");

  // Initialize motor drivers
  motor[0].begin(CH_M1_R, CH_M1_L, PWM_FREQ, PWM_RESBIT);
  motor[1].begin(CH_M2_R, CH_M2_L, PWM_FREQ, PWM_RESBIT);
  motor[2].begin(CH_M3_R, CH_M3_L, PWM_FREQ, PWM_RESBIT);
  Serial.println("✓ Motor drivers initialized");

  // Initialize limit sensors
  LimitSensors::begin();
  Serial.println("✓ Limit sensors initialized");

  // Initialize encoders
  EncoderModule::begin();
  Serial.println("✓ Encoders initialized");

  // Initialize motion control (PID)
  MotionControl::begin();
  Serial.println("✓ Motion control initialized\n");

  // Initialize force control (outer PID loop)
  ForceControl::begin();
  ForceControl::setEnabled(useForceControl);
  Serial.println("✓ Force control initialized\n");

  // Initialize PPG sensor + BLE
  PPGModule::begin();
  Serial.println("✓ PPG + BLE initialized\n");

  // Print control mode
  if (useForceControl) {
    Serial.println(">>> FORCE CONTROL MODE: Amplitude from force sensor <<<");
    Serial.printf("    Force setpoint: %.1f N\n", ForceControl::getForceSetpoint());
  } else {
    Serial.println(">>> DEFAULT AMPLITUDE MODE <<<");
    Serial.printf("    Amplitude: %.1f revolutions\n", DEFAULT_AMPLITUDE_REV);
  }
  Serial.println();

  delay(500);
}

// ===================== MAIN LOOP =====================
void loop() {
  // Sequence counter for force control (first sequence uses initial amplitude)
  static int sequenceCount = 0;
  
  // Always process PPG sensor reads + BLE events every iteration
  PPGModule::update();
  
  // ---- BLE SESSION MODE ----
  // When BLE session is active, level 1-4 sets the force setpoint.
  // ForceControl PID computes amplitude (revolutions) from force error.
  if (PPGModule::isSessionActive()) {
    int level = PPGModule::getCompressionLevel();
    
    // Track level changes and update force setpoint accordingly
    static int lastBleLevel = 0;
    if (level != lastBleLevel && level >= 1 && level <= NUM_LEVELS) {
      ForceControl::setEnabled(true);
      ForceControl::setForceSetpoint(LEVEL_FORCE_SETPOINT[level]);
      sequenceCount = 0;  // Reset so initial amplitude is used with new setpoint
      lastBleLevel = level;
      Serial.printf("[BLE Session] Level %d -> Force setpoint: %.1f N\n",
                    level, LEVEL_FORCE_SETPOINT[level]);
    }
    // Reset tracking when session ends
    if (PPGModule::sessionJustEnded()) {
      lastBleLevel = 0;
    }
    
    if (level >= 1 && level <= NUM_LEVELS) {
      // Determine amplitude via ForceControl PID
      float amplitude;
      if (sequenceCount == 0) {
        amplitude = ForceControl::getInitialAmplitude();
        Serial.printf("[BLE Session] First sequence - Initial amplitude: %.2f rev\n", amplitude);
      } else {
        amplitude = ForceControl::getDesiredRevolutions();
      }
      
      // Clear stop flag before starting pattern
      PPGModule::stopMotorRequested = false;
      
      // Start peak force tracking for this sequence
      ForceControl::startPeakTracking();
      
      Serial.printf("[BLE Session] Level %d | Force SP: %.1f N | Measured: %.1f N | Amplitude: %.2f rev\n",
                    level, ForceControl::getForceSetpoint(), ForceControl::getLastMeasuredForce(), amplitude);
      
      // Execute selected motion pattern
      switch (currentPattern) {
        case MotionControl::PATTERN_SEQUENTIAL:
          MotionControl::executePattern1(motor, amplitude);
          break;
        case MotionControl::PATTERN_PHASE_OFFSET:
          MotionControl::executePattern2(motor, amplitude);
          break;
        case MotionControl::PATTERN_CASCADING:
          MotionControl::executePattern3(motor, amplitude);
          break;
        case MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL:
          MotionControl::executePattern4(motor, amplitude);
          break;
        default:
          Serial.println("ERROR: Invalid pattern!");
          delay(1000);
          break;
      }
      
      // After pattern completes, compute next amplitude from peak force vs setpoint
      float nextAmplitude = ForceControl::computeNextAmplitude();
      Serial.printf("[BLE Session] Next amplitude: %.2f rev\n", nextAmplitude);
      sequenceCount++;
      
      // Safety check
      LimitSensors::checkPendingTriggers();
      
      // Brief pause between cycles (keep processing BLE during wait)
      unsigned long waitStart = millis();
      while (millis() - waitStart < 1000) {
        PPGModule::update();
        if (PPGModule::stopMotorRequested || !PPGModule::isSessionActive()) break;
        delay(10);
      }
    }
    return;  // Stay in BLE session mode
  }
  
  // ---- STANDALONE MODE (no BLE session) ----
  // Original behavior: force control or manual button control
  
  // Determine amplitude based on control mode
  float amplitude;
  
  if (useForceControl && ForceControl::isEnabled()) {
    if (sequenceCount == 0) {
      amplitude = ForceControl::getInitialAmplitude();
      Serial.printf("[Force Control] First sequence - Initial amplitude: %.2f rev\n", amplitude);
    } else {
      amplitude = ForceControl::getDesiredRevolutions();
    }
    ForceControl::startPeakTracking();
  } else {
    amplitude = DEFAULT_AMPLITUDE_REV;
  }
  
  // Execute selected motion pattern
  switch (currentPattern) {
    case MotionControl::PATTERN_SEQUENTIAL:
      MotionControl::executePattern1(motor, amplitude);
      break;
    case MotionControl::PATTERN_PHASE_OFFSET:
      MotionControl::executePattern2(motor, amplitude);
      break;
    case MotionControl::PATTERN_CASCADING:
      MotionControl::executePattern3(motor, amplitude);
      break;
    case MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL:
      MotionControl::executePattern4(motor, amplitude);
      break;
    default:
      Serial.println("ERROR: Invalid pattern selected!");
      delay(1000);
      break;
  }
  
  // After pattern completes, compute next amplitude based on peak force
  if (useForceControl && ForceControl::isEnabled()) {
    float nextAmplitude = ForceControl::computeNextAmplitude();
    Serial.printf("[Force Control] Computed amplitude for next sequence: %.2f rev\n\n", nextAmplitude);
    sequenceCount++;
  }
  
  // Safety check
  LimitSensors::checkPendingTriggers();
  
  Serial.println("\n========== Cycle complete, repeating... ==========\n");
  delay(1000);
}
