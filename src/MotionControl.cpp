// MotionControl.cpp - High-level motion control implementation
#include "MotionControl.h"
#include "config.h"
#include "Encoder.h"
#include "LimitSensors.h"
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
      float elapsed = (now - startTime) / 1000.0f; // seconds
      float progress = elapsed / (totalDurationMs / 1000.0f); // 0..1

      // Generate setpoint from sine wave
      float theta = TWO_PI_F * progress;
      float sineValue = sinf(theta * 0.5f); // 0..+1
      float targetPosition = sineValue * amplitudeCounts; // desired encoder counts

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

} // namespace MotionControl
