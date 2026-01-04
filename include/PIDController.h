// PIDController.h - PID control interface
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
  PIDController();
  
  void init(float p, float i, float d, float outMin, float outMax, float intMin, float intMax);
  void reset();
  float compute(float current, float dt);
  
  void setSetpoint(float sp) { setpoint = sp; }
  float getSetpoint() const { return setpoint; }
  
private:
  float kp, ki, kd;
  float setpoint;
  float integral;
  float lastError;
  float outputMin, outputMax;
  float integralMin, integralMax;
  unsigned long lastTime;
};

#endif // PID_CONTROLLER_H
