// MotionControl.cpp - High-level motion control implementation
#include "MotionControl.h"
#include "config.h"
#include "Encoder.h"
#include "LimitSensors.h"
#include "ForceControl.h"
#include "PPGModule.h"
#include <math.h>

namespace MotionControl {

// PID controllers for each motor
PIDController pidMotor[NUM_MOTORS];

void begin() {
  // Initialize PID controllers
  for (int i = 0; i < NUM_MOTORS; i++) {
    pidMotor[i].init(PID_KP, PID_KI, PID_KD, 
                     -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
                     -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  }

  Serial.println("PID Controllers initialized:");
  Serial.printf("Kp=%.2f, Ki=%.2f, Kd=%.2f\n", PID_KP, PID_KI, PID_KD);
  Serial.printf("Encoder PPR=%d, Counts/Rev=%.0f\n", ENCODER_PPR, COUNTS_PER_REV);
  Serial.printf("PID Sample Rate: %.1fms (%.0fHz)\n", PID_SAMPLE_TIME_MS, 1000.0f/PID_SAMPLE_TIME_MS);
  Serial.flush();
}

float updateMotorState(BTS7960& motor, int motorIndex, MotorState state, 
                       unsigned long phaseStartTime, uint32_t halfCycleDuration,
                       float amplitudeCounts, float holdTarget) {
  const float pidSampleTimeS = PID_SAMPLE_TIME_MS / 1000.0f;
  long currentPos = EncoderModule::getPosition(motorIndex);
  float targetPos = 0.0f;
  
  switch(state) {
    case IDLE:
      motor.coast();
      break;
      
    case MOVING_FIRST_HALF:
      {
        // First half: 0 → Negative Peak (counterclockwise first)
        unsigned long now = millis();
        float elapsed = (now - phaseStartTime) / 1000.0f;
        float progress = elapsed / (halfCycleDuration / 1000.0f);
        progress = constrain(progress, 0.0f, 1.0f);
        
        // Sine wave from 0 to negative peak (reversed direction)
        float theta = M_PI * progress; // 0 to PI for half cycle
        float sineValue = sinf(theta * 0.5f); // 0..+1
        targetPos = -sineValue * amplitudeCounts; // Negative for reverse direction
        
        pidMotor[motorIndex].setSetpoint(targetPos);
        float pidOut = pidMotor[motorIndex].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motor.set(duty, dir);
      }
      break;
      
    case MOVING_SECOND_HALF:
      {
        // Second half: Negative Peak → 0 (return from reverse direction)
        unsigned long now = millis();
        float elapsed = (now - phaseStartTime) / 1000.0f;
        float progress = elapsed / (halfCycleDuration / 1000.0f);
        progress = constrain(progress, 0.0f, 1.0f);
        
        // Sine wave from negative peak back to 0
        float theta = M_PI * progress; // 0 to PI for half cycle
        float sineValue = sinf(theta * 0.5f); // 0..+1
        targetPos = (sineValue - 1.0f) * amplitudeCounts; // Start at -amplitude, end at 0
        
        pidMotor[motorIndex].setSetpoint(targetPos);
        float pidOut = pidMotor[motorIndex].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motor.set(duty, dir);
      }
      break;
      
    case HOLD_AT_HALF:
    case HOLD_AT_FULL:
      targetPos = holdTarget;
      pidMotor[motorIndex].setSetpoint(targetPos);
      {
        float pidOut = pidMotor[motorIndex].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motor.set(duty, dir);
      }
      break;
  }
  
  return targetPos;
}

void runSineCycle(BTS7960& motor, int motorIndex, float amplitudeRevolutions) {
  const float TWO_PI_F = 2.0f * (float)M_PI;
  const uint32_t totalDurationMs = SINE_DURATION_MS;
  const float pidSampleTimeS = PID_SAMPLE_TIME_MS / 1000.0f;

  // Reset encoder and PID state at start
  EncoderModule::resetPosition(motorIndex);
  pidMotor[motorIndex].reset();

  // Calculate total position swing (in counts)
  const float amplitudeCounts = amplitudeRevolutions * COUNTS_PER_REV;

  unsigned long startTime = millis();
  unsigned long lastPidTime = startTime;
  unsigned long lastLogTime = startTime;

  Serial.printf("\n=== Motor %d: Starting closed-loop sine trajectory ===\n", motorIndex + 1);
  Serial.printf("Amplitude: %.1f rev (%.0f counts)\n", amplitudeRevolutions, amplitudeCounts);
  Serial.printf("Duration: %d ms\n", totalDurationMs);
  Serial.flush();

  int loopCount = 0;

  while (millis() - startTime < totalDurationMs) {
    unsigned long now = millis();
    
    // Check BLE stop request (session ended mid-cycle)
    if (PPGModule::stopMotorRequested) {
      Serial.printf("[Motor %d] BLE stop requested -> aborting\n", motorIndex + 1);
      motor.coast();
      return;
    }
    
    // Check limit sensors
    if (LimitSensors::isTriggered(motorIndex)) {
      Serial.printf("** Sensor triggered for motor %d -> stopping\n", motorIndex + 1);
      motor.coast();
      LimitSensors::clearFlags(motorIndex);
      return;
    }

    // PID runs at fixed rate
    if (now - lastPidTime >= PID_SAMPLE_TIME_MS) {
      loopCount++;
      
      // Update force tracking during motion
      ForceControl::updatePeakTracking();
      
      float elapsed = (now - startTime) / 1000.0f; // seconds
      float progress = elapsed / (totalDurationMs / 1000.0f); // 0..1

      // Generate setpoint from sine wave (negative first for reverse direction)
      float theta = TWO_PI_F * progress;
      float sineValue = sinf(theta * 0.5f); // 0..+1
      float targetPosition = -sineValue * amplitudeCounts; // Negative for counterclockwise first

      // Read current encoder position
      long currentPosition = EncoderModule::getPosition(motorIndex);

      // Update PID setpoint and compute control output
      pidMotor[motorIndex].setSetpoint(targetPosition);
      float pidOutput = pidMotor[motorIndex].compute((float)currentPosition, pidSampleTimeS);

      // Convert PID output to motor command
      // PID output is signed: positive = forward, negative = reverse
      Dir direction = (pidOutput >= 0.0f) ? FORWARD : REVERSE;
      int duty = (int)constrain(fabsf(pidOutput), 0.0f, (float)PWM_MAX);
      
      // Send command to motor
      motor.set(duty, direction);

      // Teleplot output (every loop for real-time plotting)
      // Serial.printf(">M%d_Target:%ld\n", motorIndex + 1, (long)targetPosition);
      // Serial.printf(">M%d_Current:%ld\n", motorIndex + 1, currentPosition);
      // Serial.printf(">M%d_Error:%ld\n", motorIndex + 1, (long)(targetPosition - currentPosition));
      // Serial.printf(">M%d_PWM:%d\n", motorIndex + 1, (direction == FORWARD) ? duty : -duty);

      // Log first 10 loops + every 500ms after
      if (loopCount <= 10 || (now - lastLogTime > 500)) {
        Serial.printf("Motor %d | PWM: %4d %s | Current Pos: %6ld counts | Target Pos: %6ld counts\n",
                      motorIndex + 1,
                      duty,
                      (direction == FORWARD) ? "FWD" : "REV",
                      currentPosition,
                      (long)targetPosition);
        Serial.flush();
        lastLogTime = now;
      }

      lastPidTime = now;
    }

    delay(1); // Small delay to prevent tight loop
  }

  // Graceful stop
  motor.coast();
  long finalPos = EncoderModule::getPosition(motorIndex);
  Serial.printf("\n=== Motor %d: Cycle complete ===\n", motorIndex + 1);
  Serial.printf("Final position: %.3f rev (%ld counts)\n", 
                EncoderModule::countsToRevolutions(finalPos), finalPos);
  Serial.printf("Total PID loops: %d (expected: %d)\n\n", 
                loopCount, (int)(totalDurationMs / PID_SAMPLE_TIME_MS));
  Serial.flush();
}

void executePattern1(BTS7960 motors[], float amplitude) {
  Serial.println("\n========== PATTERN 1: SEQUENTIAL ==========\n");
  
  // Motor 1
  Serial.printf("--- Motor 1: sine cycle (%.1f rev) ---\n", amplitude);
  EncoderModule::printPositions();
  runSineCycle(motors[0], 0, amplitude);
  delay(PAUSE_BETWEEN_MS);

  // Motor 2
  Serial.printf("\n--- Motor 2: sine cycle (%.1f rev) ---\n", amplitude);
  EncoderModule::printPositions();
  runSineCycle(motors[1], 1, amplitude);
  delay(PAUSE_BETWEEN_MS);

  // Motor 3
  Serial.printf("\n--- Motor 3: sine cycle (%.1f rev) ---\n", amplitude);
  EncoderModule::printPositions();
  runSineCycle(motors[2], 2, amplitude);
  delay(PAUSE_BETWEEN_MS);
  
  Serial.println("\n========== Pattern 1 Complete ==========\n");
}

void executePattern2(BTS7960 motors[], float amplitude) {
  Serial.println("\n========== PATTERN 2: PHASE OFFSET ==========\n");
  Serial.printf("Motor 1 & 3: Start at t=0, duration=%d ms\n", SINE_DURATION_MS);
  Serial.printf("Motor 2: Start at t=%d ms (when M1&M3 at 50%%), duration=%d ms\n", SINE_DURATION_MS/2, SINE_DURATION_MS);
  Serial.printf("Total duration: %d ms\n\n", SINE_DURATION_MS + SINE_DURATION_MS/2);
  
  EncoderModule::printPositions();
  
  // Reset all encoders and PIDs
  for (int i = 0; i < 3; i++) {
    EncoderModule::resetPosition(i);
    pidMotor[i].reset();
  }
  
  unsigned long globalStartTime = millis();
  unsigned long motor2StartTime = globalStartTime + SINE_DURATION_MS / 2;
  const uint32_t totalDuration = SINE_DURATION_MS + SINE_DURATION_MS / 2;
  const float TWO_PI_F = 2.0f * (float)M_PI;
  const float pidSampleTimeS = PID_SAMPLE_TIME_MS / 1000.0f;
  const float amplitudeCounts = amplitude * COUNTS_PER_REV;
  
  unsigned long lastPidTime = globalStartTime;
  unsigned long lastLogTime = globalStartTime;
  int loopCount = 0;
  bool motor2Started = false;
  
  while (millis() - globalStartTime < totalDuration) {
    unsigned long now = millis();
    
    // Check BLE stop request
    if (PPGModule::stopMotorRequested) {
      Serial.println("[Pattern 2] BLE stop requested -> aborting");
      for (int j = 0; j < 3; j++) motors[j].coast();
      return;
    }
    
    // Check limit sensors for all motors
    for (int i = 0; i < 3; i++) {
      if (LimitSensors::isTriggered(i)) {
        Serial.printf("** Sensor triggered for motor %d -> stopping all\n", i + 1);
        for (int j = 0; j < 3; j++) {
          motors[j].coast();
        }
        LimitSensors::clearFlags(i);
        return;
      }
    }
    
    // Check if Motor 2 should start
    if (!motor2Started && now >= motor2StartTime) {
      motor2Started = true;
      Serial.println("\n>>> Motor 2 starting (M1 & M3 at 50% progress) <<<\n");
    }
    
    // PID runs at fixed rate for all active motors
    if (now - lastPidTime >= PID_SAMPLE_TIME_MS) {
      loopCount++;
      
      // Update force tracking during motion
      ForceControl::updatePeakTracking();
      
      float globalElapsed = (now - globalStartTime) / 1000.0f;
      
      // Motor 1: Active from t=0 to SINE_DURATION_MS
      if (now - globalStartTime < SINE_DURATION_MS) {
        float elapsed = (now - globalStartTime) / 1000.0f;
        float progress = elapsed / (SINE_DURATION_MS / 1000.0f);
        float theta = TWO_PI_F * progress;
        float sineValue = sinf(theta * 0.5f);
        float targetPos = -sineValue * amplitudeCounts; // Negative for reverse direction
        long currentPos = EncoderModule::getPosition(0);
        pidMotor[0].setSetpoint(targetPos);
        float pidOut = pidMotor[0].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motors[0].set(duty, dir);
      } else {
        motors[0].coast();
      }
      
      // Motor 2: Active from motor2StartTime to (motor2StartTime + SINE_DURATION_MS)
      if (motor2Started && (now - motor2StartTime < SINE_DURATION_MS)) {
        float elapsed = (now - motor2StartTime) / 1000.0f;
        float progress = elapsed / (SINE_DURATION_MS / 1000.0f);
        float theta = TWO_PI_F * progress;
        float sineValue = sinf(theta * 0.5f);
        float targetPos = -sineValue * amplitudeCounts; // Negative for reverse direction
        long currentPos = EncoderModule::getPosition(1);
        pidMotor[1].setSetpoint(targetPos);
        float pidOut = pidMotor[1].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motors[1].set(duty, dir);
      } else if (!motor2Started) {
        motors[1].coast();
      }
      
      // Motor 3: Active from t=0 to SINE_DURATION_MS (same as Motor 1)
      if (now - globalStartTime < SINE_DURATION_MS) {
        float elapsed = (now - globalStartTime) / 1000.0f;
        float progress = elapsed / (SINE_DURATION_MS / 1000.0f);
        float theta = TWO_PI_F * progress;
        float sineValue = sinf(theta * 0.5f);
        float targetPos = -sineValue * amplitudeCounts; // Negative for reverse direction
        long currentPos = EncoderModule::getPosition(2);
        pidMotor[2].setSetpoint(targetPos);
        float pidOut = pidMotor[2].compute((float)currentPos, pidSampleTimeS);
        Dir dir = (pidOut >= 0.0f) ? FORWARD : REVERSE;
        int duty = (int)constrain(fabsf(pidOut), 0.0f, (float)PWM_MAX);
        motors[2].set(duty, dir);
      } else {
        motors[2].coast();
      }
      
      // Logging
      if (loopCount <= 5 || (now - lastLogTime > 1000)) {
        long pos1 = EncoderModule::getPosition(0);
        long pos2 = EncoderModule::getPosition(1);
        long pos3 = EncoderModule::getPosition(2);
        Serial.printf("[%.1fs] M1:%6ld | M2:%6ld | M3:%6ld counts\n",
                      globalElapsed, pos1, pos2, pos3);
        Serial.flush();
        lastLogTime = now;
      }
      
      lastPidTime = now;
    }
    
    delay(1);
  }
  
  // Stop all motors
  for (int i = 0; i < 3; i++) {
    motors[i].coast();
  }
  
  Serial.println("\n=== Final Positions ===");
  EncoderModule::printPositions();
  Serial.println("\n========== Pattern 2 Complete ==========\n");
}

void executePattern3(BTS7960 motors[], float amplitude) {
  Serial.println("\n========== PATTERN 3: CASCADING HALF-CYCLE ==========\n");
  Serial.println("Visualization:");
  Serial.println("Timeline (each step = half sine cycle duration):");
  Serial.println("");
  Serial.println("  t=0: M1:---- M2:---- M3:---- (All idle)");
  Serial.println("  t=1: M1:^^^^ M2:---- M3:---- (M1: 0→Half)");
  Serial.println("  t=2: M1:HOLD M2:^^^^ M3:---- (M1 holds at Half, M2: 0→Half)");
  Serial.println("  t=3: M1:^^^^ M2:HOLD M3:^^^^ (M1: Half→Full, M2 holds, M3: 0→Half)");
  Serial.println("  t=4: M1:HOLD M2:^^^^ M3:HOLD (M1 holds at Full, M2: Half→Full, M3 holds)");
  Serial.println("  t=5: M1:^^^^ M2:HOLD M3:^^^^ (M1: reset→Half, M2 holds, M3: Half→Full)");
  Serial.println("  [Repeats from t=2]\n");
  Serial.printf("Half-cycle duration: %d ms\n", SINE_DURATION_MS / 2);
  Serial.printf("Full pattern cycle: %d ms (5 half-cycles)\n\n", SINE_DURATION_MS / 2 * 5);
  
  const uint32_t halfCycleDuration = SINE_DURATION_MS / 2;
  const float TWO_PI_F = 2.0f * (float)M_PI;
  const float pidSampleTimeS = PID_SAMPLE_TIME_MS / 1000.0f;
  const float amplitudeCounts = amplitude * COUNTS_PER_REV;
  
  // Reset all encoders and PIDs
  for (int i = 0; i < 3; i++) {
    EncoderModule::resetPosition(i);
    pidMotor[i].reset();
  }
  
  // State machine variables
  MotorState motorStates[3] = {IDLE, IDLE, IDLE};
  float motorTargets[3] = {0.0f, 0.0f, 0.0f};
  unsigned long motorPhaseStartTime[3] = {0, 0, 0};
  
  unsigned long globalStartTime = millis();
  unsigned long lastPidTime = globalStartTime;
  unsigned long lastLogTime = globalStartTime;
  int currentTimeStep = 0;
  unsigned long stepStartTime = globalStartTime;
  
  // Run for 3 full pattern cycles (15 half-cycles)
  const int totalSteps = 15;
  
  Serial.println("=== Starting Pattern 3 ===\n");
  EncoderModule::printPositions();
  
  while (currentTimeStep < totalSteps) {
    unsigned long now = millis();
    
    // Check BLE stop request
    if (PPGModule::stopMotorRequested) {
      Serial.println("[Pattern 3] BLE stop requested -> aborting");
      for (int j = 0; j < 3; j++) motors[j].coast();
      return;
    }
    
    // Check if it's time to advance to next time step
    if (now - stepStartTime >= halfCycleDuration) {
      currentTimeStep++;
      stepStartTime = now;
      
      Serial.printf("\n>>> Time Step %d <<<\n", currentTimeStep);
      
      // Update state machine based on pattern sequence
      // The pattern follows: t1-t2-t3-t4-t5 then repeats from t2
      int cycleStep = currentTimeStep;
      if (cycleStep > 5) {
        cycleStep = 2 + ((currentTimeStep - 2) % 4); // Repeat t2-t5
      }
      
      switch(cycleStep) {
        case 1: // t=1: M1 starts (0→half)
          motorStates[0] = MOVING_FIRST_HALF;
          motorPhaseStartTime[0] = now;
          motorStates[1] = IDLE;
          motorStates[2] = IDLE;
          Serial.println("M1: 0→Half | M2: Idle | M3: Idle");
          break;
          
        case 2: // t=2: M1 holds at half, M2 starts (0→half)
          motorStates[0] = HOLD_AT_HALF;
          motorTargets[0] = -amplitudeCounts; // Hold at negative peak
          motorStates[1] = MOVING_FIRST_HALF;
          motorPhaseStartTime[1] = now;
          motorStates[2] = IDLE;
          Serial.println("M1: Hold at Half | M2: 0→Half | M3: Idle");
          break;
          
        case 3: // t=3: M1 continues (half→full = peak→0), M2 holds, M3 starts
          motorStates[0] = MOVING_SECOND_HALF; // Continue from peak down to 0
          motorPhaseStartTime[0] = now;
          motorStates[1] = HOLD_AT_HALF;
          motorTargets[1] = -amplitudeCounts; // Hold at negative peak
          motorStates[2] = MOVING_FIRST_HALF;
          motorPhaseStartTime[2] = now;
          Serial.println("M1: Half→Full (Peak→0) | M2: Hold at Half | M3: 0→Half");
          break;
          
        case 4: // t=4: M1 holds at full (0), M2 continues (half→full = peak→0), M3 holds
          motorStates[0] = HOLD_AT_FULL; // Hold at 0 position
          motorTargets[0] = 0.0f;
          motorStates[1] = MOVING_SECOND_HALF; // Continue from peak down to 0
          motorPhaseStartTime[1] = now;
          motorStates[2] = HOLD_AT_HALF;
          motorTargets[2] = -amplitudeCounts; // Hold at negative peak
          Serial.println("M1: Hold at Full (0) | M2: Half→Full (Peak→0) | M3: Hold at Half");
          break;
          
        case 5: // t=5: M1 restarts (0→half), M2 holds at full (0), M3 continues (half→full = peak→0)
          motorStates[0] = MOVING_FIRST_HALF;
          motorPhaseStartTime[0] = now;
          motorStates[1] = HOLD_AT_FULL; // Hold at 0 position
          motorTargets[1] = 0.0f;
          motorStates[2] = MOVING_SECOND_HALF; // Continue from peak down to 0
          motorPhaseStartTime[2] = now;
          Serial.println("M1: 0→Half | M2: Hold at Full (0) | M3: Half→Full (Peak→0)");
          break;
      }
    }
    
    // Check limit sensors
    for (int i = 0; i < 3; i++) {
      if (LimitSensors::isTriggered(i)) {
        Serial.printf("** Sensor triggered for motor %d -> stopping all\n", i + 1);
        for (int j = 0; j < 3; j++) {
          motors[j].coast();
        }
        LimitSensors::clearFlags(i);
        return;
      }
    }
    
    // PID control loop for all motors
    if (now - lastPidTime >= PID_SAMPLE_TIME_MS) {
      // Update force tracking during motion
      ForceControl::updatePeakTracking();
      
      for (int i = 0; i < 3; i++) {
        updateMotorState(motors[i], i, motorStates[i], motorPhaseStartTime[i], 
                        halfCycleDuration, amplitudeCounts, motorTargets[i]);
      }
      
      // Logging
      if (now - lastLogTime > 500) {
        long pos1 = EncoderModule::getPosition(0);
        long pos2 = EncoderModule::getPosition(1);
        long pos3 = EncoderModule::getPosition(2);
        float elapsed = (now - globalStartTime) / 1000.0f;
        Serial.printf("[%.1fs Step %d] M1:%6ld | M2:%6ld | M3:%6ld counts\n",
                      elapsed, currentTimeStep, pos1, pos2, pos3);
        lastLogTime = now;
      }
      
      lastPidTime = now;
    }
    
    delay(1);
  }
  
  // Stop all motors
  for (int i = 0; i < 3; i++) {
    motors[i].coast();
  }
  
  Serial.println("\n=== Final Positions ===");
  EncoderModule::printPositions();
  Serial.println("\n========== Pattern 3 Complete ==========\n");
}

void executePattern4(BTS7960 motors[], float amplitude) {
  Serial.println("\n========== PATTERN 4: SEQUENTIAL-THEN-PARALLEL ==========\n");
  Serial.println("Sequence:");
  Serial.println("  Phase 1: M1 moves 0→Peak and holds");
  Serial.println("  Phase 2: M2 moves 0→Peak and holds (M1 still holding)");
  Serial.println("  Phase 3: M3 moves 0→Peak and holds (M1 & M2 still holding)");
  Serial.println("  Phase 4: ALL motors move Peak→0 simultaneously\n");
  
  const uint32_t halfCycleDuration = SINE_DURATION_MS / 2;
  const float pidSampleTimeS = PID_SAMPLE_TIME_MS / 1000.0f;
  const float amplitudeCounts = amplitude * COUNTS_PER_REV;
  
  // Reset all encoders and PIDs
  for (int i = 0; i < 3; i++) {
    EncoderModule::resetPosition(i);
    pidMotor[i].reset();
  }
  
  EncoderModule::printPositions();
  
  // State tracking
  MotorState motorStates[3] = {IDLE, IDLE, IDLE};
  float motorTargets[3] = {0.0f, 0.0f, 0.0f};
  unsigned long motorPhaseStartTime[3] = {0, 0, 0};
  
  unsigned long globalStartTime = millis();
  unsigned long lastPidTime = globalStartTime;
  unsigned long lastLogTime = globalStartTime;
  
  int currentPhase = 0; // 0=waiting, 1=M1 moving, 2=M2 moving, 3=M3 moving, 4=all descending
  unsigned long phaseStartTime = globalStartTime;
  
  Serial.println("=== Starting Pattern 4 ===\n");
  
  while (currentPhase < 5) {
    unsigned long now = millis();
    
    // Check BLE stop request
    if (PPGModule::stopMotorRequested) {
      Serial.println("[Pattern 4] BLE stop requested -> aborting");
      for (int j = 0; j < 3; j++) motors[j].coast();
      return;
    }
    
    // Check limit sensors
    for (int i = 0; i < 3; i++) {
      if (LimitSensors::isTriggered(i)) {
        Serial.printf("** Sensor triggered for motor %d -> stopping all\n", i + 1);
        for (int j = 0; j < 3; j++) {
          motors[j].coast();
        }
        LimitSensors::clearFlags(i);
        return;
      }
    }
    
    // Phase management
    bool phaseComplete = false;
    
    if (currentPhase == 0) {
      // Start Phase 1: M1 begins first half
      currentPhase = 1;
      motorStates[0] = MOVING_FIRST_HALF;
      motorPhaseStartTime[0] = now;
      phaseStartTime = now;
      Serial.println(">>> Phase 1: M1 moving 0→Peak <<<");
    }
    else if (currentPhase >= 1 && currentPhase <= 3) {
      // Check if current motor finished its first half
      if (now - phaseStartTime >= halfCycleDuration) {
        phaseComplete = true;
        int justFinishedMotor = currentPhase - 1; // 0, 1, or 2
        motorStates[justFinishedMotor] = HOLD_AT_HALF;
        motorTargets[justFinishedMotor] = -amplitudeCounts; // Hold at negative peak
        
        if (currentPhase == 3) {
          // All motors have reached peak, start simultaneous descent
          currentPhase = 4;
          for (int i = 0; i < 3; i++) {
            motorStates[i] = MOVING_SECOND_HALF;
            motorPhaseStartTime[i] = now;
          }
          phaseStartTime = now;
          Serial.println(">>> Phase 4: ALL motors moving Peak→0 simultaneously <<<");
        } else {
          // Start next motor
          currentPhase++;
          int nextMotor = currentPhase - 1;
          motorStates[nextMotor] = MOVING_FIRST_HALF;
          motorPhaseStartTime[nextMotor] = now;
          phaseStartTime = now;
          Serial.printf(">>> Phase %d: M%d moving 0→Peak <<<\n", currentPhase, nextMotor + 1);
        }
      }
    }
    else if (currentPhase == 4) {
      // Check if all motors finished descending
      if (now - phaseStartTime >= halfCycleDuration) {
        currentPhase = 5; // Done
      }
    }
    
    // PID control loop
    if (now - lastPidTime >= PID_SAMPLE_TIME_MS) {
      // Update force tracking during motion
      ForceControl::updatePeakTracking();
      
      for (int i = 0; i < 3; i++) {
        updateMotorState(motors[i], i, motorStates[i], motorPhaseStartTime[i], 
                        halfCycleDuration, amplitudeCounts, motorTargets[i]);
      }
      
      // Logging
      if (now - lastLogTime > 500) {
        long pos1 = EncoderModule::getPosition(0);
        long pos2 = EncoderModule::getPosition(1);
        long pos3 = EncoderModule::getPosition(2);
        float elapsed = (now - globalStartTime) / 1000.0f;
        Serial.printf("[%.1fs Phase %d] M1:%6ld | M2:%6ld | M3:%6ld counts\n",
                      elapsed, currentPhase, pos1, pos2, pos3);
        lastLogTime = now;
      }
      
      lastPidTime = now;
    }
    
    delay(1);
  }
  
  // Stop all motors
  for (int i = 0; i < 3; i++) {
    motors[i].coast();
  }
  
  Serial.println("\n=== Final Positions ===");
  EncoderModule::printPositions();
  Serial.println("\n========== Pattern 4 Complete ==========\n");
}

} // namespace MotionControl
