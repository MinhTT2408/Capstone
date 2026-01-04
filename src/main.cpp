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

// ===================== MOTOR INSTANCES =====================
BTS7960 motor[3] = {
  BTS7960(M1_RPWM, M1_LPWM, M1_REN, M1_LEN),
  BTS7960(M2_RPWM, M2_LPWM, M2_REN, M2_LEN),
  BTS7960(M3_RPWM, M3_LPWM, M3_REN, M3_LEN)
};

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

  delay(500);
}

// ===================== MAIN LOOP =====================
void loop() {
  // Print encoder positions before starting
  EncoderModule::printPositions();

  // Motor 1 -> Motor 2 -> Motor 3, sequentially
  Serial.println("\n--- Motor 1: sine cycle ---");
  MotionControl::runSineCycle(motor[0], 0, 1.0f); // 5 revolutions amplitude
  delay(PAUSE_BETWEEN_MS);

  Serial.println("\n--- Motor 2: sine cycle ---");
  MotionControl::runSineCycle(motor[1], 1, 5.0f);
  delay(PAUSE_BETWEEN_MS);

  Serial.println("\n--- Motor 3: sine cycle ---");
  MotionControl::runSineCycle(motor[2], 2, 1.0f);
  delay(PAUSE_BETWEEN_MS);

  // Safety check: handle any sensor triggers that occurred during pauses
  LimitSensors::checkPendingTriggers();

  // Repeat forever
  Serial.println("\n========== Cycle complete, repeating... ==========\n");
  delay(1000);
}
