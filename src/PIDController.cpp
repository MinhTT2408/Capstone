// PIDController.cpp - PID control implementation
#include "PIDController.h"

PIDController::PIDController()
  : kp(0), ki(0), kd(0), setpoint(0), integral(0), lastError(0),
    outputMin(-1023), outputMax(1023), integralMin(-500), integralMax(500),
    lastTime(0) {
}

void PIDController::init(float p, float i, float d, float outMin, float outMax, float intMin, float intMax) {
  kp = p;
  ki = i;
  kd = d;
  outputMin = outMin;
  outputMax = outMax;
  integralMin = intMin;
  integralMax = intMax;
  reset();
}

void PIDController::reset() {
  integral = 0.0f;
  lastError = 0.0f;
  lastTime = millis();
}

float PIDController::compute(float current, float dt) {
  float error = setpoint - current;

  // Proportional term
  float pTerm = kp * error;

  // Integral term with anti-windup
  integral += error * dt;
  integral = constrain(integral, integralMin, integralMax);
  float iTerm = ki * integral;

  // Derivative term
  float dTerm = 0.0f;
  if (dt > 0.0f) {
    float derivative = (error - lastError) / dt;
    dTerm = kd * derivative;
  }

  lastError = error;

  // Total output
  float output = pTerm + iTerm + dTerm;
  return constrain(output, outputMin, outputMax);
}
