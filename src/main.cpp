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
#include "ButtonInput.h"
#include "ForceControl.h"

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
// Set to false to use manual amplitude from buttons
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

  // Initialize button input
  ButtonInput::begin();
  Serial.println("✓ Button input initialized");

  // Initialize force control (outer PID loop)
  ForceControl::begin();
  ForceControl::setEnabled(useForceControl);
  Serial.println("✓ Force control initialized\n");

  // Print control mode
  if (useForceControl) {
    Serial.println(">>> FORCE CONTROL MODE: Amplitude from force sensor <<<");
    Serial.printf("    Force setpoint: %.1f N\n", ForceControl::getForceSetpoint());
  } else {
    Serial.println(">>> MANUAL MODE: Amplitude from buttons <<<");
    Serial.printf("    Initial amplitude: %.1f revolutions\n", ButtonInput::getAmplitude());
  }
  Serial.println();

  delay(500);
}

// ===================== MAIN LOOP =====================
void loop() {
  // Sequence counter for force control (first sequence uses initial amplitude)
  static int sequenceCount = 0;
  
  // Update button input (always update for manual mode or setpoint adjustment)
  ButtonInput::update();
  
  // Determine amplitude based on control mode
  float amplitude;
  
  if (useForceControl && ForceControl::isEnabled()) {
    // SEQUENCE-BASED FORCE CONTROL:
    // - First sequence: Use initial amplitude
    // - Subsequent sequences: Use amplitude computed from previous sequence's peak force
    
    if (sequenceCount == 0) {
      // First sequence: use initial amplitude
      amplitude = ForceControl::getInitialAmplitude();
      Serial.printf("[Force Control] First sequence - Initial amplitude: %.2f rev\n", amplitude);
    } else {
      // Use amplitude computed from previous sequence
      amplitude = ForceControl::getDesiredRevolutions();
    }
    
    // Start tracking peak force for this sequence
    ForceControl::startPeakTracking();
    
  } else {
    // MANUAL MODE: Get amplitude from button input
    amplitude = ButtonInput::getAmplitude();
  }
  
  // Execute selected motion pattern (INNER LOOP uses amplitude)
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
  
  // Safety check: handle any sensor triggers that occurred during pauses
  LimitSensors::checkPendingTriggers();

  // Repeat forever
  Serial.println("\n========== Cycle complete, repeating... ==========\n");
  delay(1000);
}
