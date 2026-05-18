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
// 5 = Simultaneous (all motors in phase, full sine cycle together)
MotionControl::MotionPattern currentPattern = MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL;

// ===================== FORCE CONTROL MODE =====================
// Set to true to enable outer force PID loop (amplitude controlled by force sensor)
// Set to false to use default amplitude (DEFAULT_AMPLITUDE_REV in config.h)
bool useForceControl = true;

// ===================== FORCE SENSOR LOG TASK =====================
// Runs on Core 1 at 50Hz (FORCE_LOG_INTERVAL_MS = 20ms).
// Reads all three force sensors and queues rows for sdWriteTask (Core 0) to write.
static void forceLogTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(FORCE_LOG_INTERVAL_MS));
    if (!PPGModule::isSessionActive()) continue;
    uint32_t elapsed = (uint32_t)(millis() - PPGModule::getSessionStartTime());
    PPGModule::logForceData(
      elapsed,
      ForceControl::readForce(0),
      ForceControl::readForce(1),
      ForceControl::readForce(2)
    );
  }
}

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

  // Spawn force sensor logging task (Core 1, same core as ADC reads)
  xTaskCreatePinnedToCore(
    forceLogTask,      // Task function
    "ForceLog_Task",   // Task name
    2048,              // Stack size in words
    NULL,              // Parameters
    1,                 // Priority (same as sdWriteTask)
    NULL,              // Task handle
    1                  // Core 1 — ADC reads stay on the app core
  );

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

// ===================== STATE FLAG =====================
// Flag to track if the app has ever taken control. 
bool appHasTakenOver = false;

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
    appHasTakenOver = true; // The app is now permanently in control!
    
    int level = PPGModule::getCompressionLevel();
    int type = PPGModule::getCompressionType(); // Read the Mode
    
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
      // Determine per-motor amplitudes via ForceControl
      float amplitudes[3];
      if (sequenceCount == 0) {
        for (int i = 0; i < 3; i++) amplitudes[i] = ForceControl::getInitialAmplitude(i);
        Serial.printf("[BLE Session] First sequence - Initial amplitudes: M1=%.2f M2=%.2f M3=%.2f rev\n",
                      amplitudes[0], amplitudes[1], amplitudes[2]);
      } else {
        for (int i = 0; i < 3; i++) amplitudes[i] = ForceControl::getDesiredRevolutions(i);
      }

      // Clear stop flag before starting pattern
      PPGModule::stopMotorRequested = false;

      // Start peak force tracking for all motors (Pattern 1 will reset per-motor internally)
      ForceControl::startPeakTracking();

      // --- MAP APP MODES TO ESP32 PATTERNS ---
      switch (type) {
        case 1: currentPattern = MotionControl::PATTERN_SEQUENTIAL; break;
        case 2: currentPattern = MotionControl::PATTERN_SIMULTANEOUS; break;
        case 3: currentPattern = MotionControl::PATTERN_CASCADING; break;
        case 4: currentPattern = MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL; break;
        default: currentPattern = MotionControl::PATTERN_SEQUENTIAL; break;
      }

      Serial.printf("[BLE Session] Level %d | Force SP: %.1f N | Amplitudes: M1=%.2f M2=%.2f M3=%.2f rev\n",
                    level, ForceControl::getForceSetpoint(), amplitudes[0], amplitudes[1], amplitudes[2]);

      // Execute selected motion pattern
      switch (currentPattern) {
        case MotionControl::PATTERN_SEQUENTIAL:
          MotionControl::executePattern1(motor, amplitudes);
          break;
        case MotionControl::PATTERN_PHASE_OFFSET:
          MotionControl::executePattern2(motor, amplitudes);
          break;
        case MotionControl::PATTERN_CASCADING:
          MotionControl::executePattern3(motor, amplitudes);
          break;
        case MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL:
          MotionControl::executePattern4(motor, amplitudes);
          break;
        case MotionControl::PATTERN_SIMULTANEOUS:
          MotionControl::executePattern5(motor, amplitudes);
          break;
        default:
          Serial.println("ERROR: Invalid pattern!");
          delay(1000);
          break;
      }

      // After pattern completes, compute next amplitude per motor from each sensor's peak
      for (int i = 0; i < 3; i++) {
        ForceControl::computeNextAmplitude(i);
      }
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
  if (!appHasTakenOver) {
    // Original behavior: force control or manual button control
    
    // Determine per-motor amplitudes based on control mode
    float amplitudes[3];

    if (useForceControl && ForceControl::isEnabled()) {
      if (sequenceCount == 0) {
        for (int i = 0; i < 3; i++) amplitudes[i] = ForceControl::getInitialAmplitude(i);
        Serial.printf("[Force Control] First sequence - Initial amplitudes: M1=%.2f M2=%.2f M3=%.2f rev\n",
                      amplitudes[0], amplitudes[1], amplitudes[2]);
      } else {
        for (int i = 0; i < 3; i++) amplitudes[i] = ForceControl::getDesiredRevolutions(i);
      }
      ForceControl::startPeakTracking();
    } else {
      for (int i = 0; i < 3; i++) amplitudes[i] = DEFAULT_AMPLITUDE_REV;
    }

    // Execute selected motion pattern
    switch (currentPattern) {
      case MotionControl::PATTERN_SEQUENTIAL:
        MotionControl::executePattern1(motor, amplitudes);
        break;
      case MotionControl::PATTERN_PHASE_OFFSET:
        MotionControl::executePattern2(motor, amplitudes);
        break;
      case MotionControl::PATTERN_CASCADING:
        MotionControl::executePattern3(motor, amplitudes);
        break;
      case MotionControl::PATTERN_SEQUENTIAL_THEN_PARALLEL:
        MotionControl::executePattern4(motor, amplitudes);
        break;
      case MotionControl::PATTERN_SIMULTANEOUS:
        MotionControl::executePattern5(motor, amplitudes);
        break;
      default:
        Serial.println("ERROR: Invalid pattern selected!");
        delay(1000);
        break;
    }

    // After pattern completes, compute next amplitude per motor from each sensor's peak
    if (useForceControl && ForceControl::isEnabled()) {
      for (int i = 0; i < 3; i++) {
        ForceControl::computeNextAmplitude(i);
      }
      sequenceCount++;
    }
    
    // Safety check
    LimitSensors::checkPendingTriggers();
    
    Serial.println("\n========== Cycle complete, repeating... ==========\n");
    delay(1000);
    
  } else {
    // IDLE: The app took over previously, but the user clicked "End Run". 
    // Do not run the default pattern. Coast the motors.
    motor[0].coast();
    motor[1].coast();
    motor[2].coast();
    delay(50);
  }
}