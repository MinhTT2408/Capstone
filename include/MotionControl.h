// MotionControl.h - High-level motion control interface
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "BTS7960.h"
#include "PIDController.h"

namespace MotionControl {
  // Initialize motion control system (PID controllers)
  void begin();
  
  // Run one complete sine cycle with closed-loop position control
  void runSineCycle(BTS7960& motor, int motorIndex, float amplitudeRevolutions = 2.0f);
}

#endif // MOTION_CONTROL_H
