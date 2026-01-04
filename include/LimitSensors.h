// LimitSensors.h - Limit switch sensor interface
#ifndef LIMIT_SENSORS_H
#define LIMIT_SENSORS_H

#include <Arduino.h>

#define NUM_MOTORS 3

namespace LimitSensors {
  // Initialize sensor pins and attach interrupts
  void begin();
  
  // Check if any sensor for a given motor has triggered
  bool isTriggered(int motorIndex);
  
  // Clear triggered flags for a motor
  void clearFlags(int motorIndex);
  
  // Clear all flags
  void clearAllFlags();
  
  // Check and handle any pending sensor triggers (for safety outside motion)
  void checkPendingTriggers();
}

#endif // LIMIT_SENSORS_H
