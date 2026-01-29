// MotionControl.h - High-level motion control interface
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "BTS7960.h"
#include "PIDController.h"

namespace MotionControl {
  // Motor state machine for position control
  enum MotorState { 
    IDLE,                  // Motor off/coasting
    MOVING_FIRST_HALF,     // 0 → Peak (ascending)
    HOLD_AT_HALF,          // Holding at peak position
    MOVING_SECOND_HALF,    // Peak → 0 (descending)
    HOLD_AT_FULL           // Holding at 0 position (full cycle complete)
  };
  
  // Motion pattern selection
  enum MotionPattern {
    PATTERN_SEQUENTIAL = 1,                 // Motors run one after another
    PATTERN_PHASE_OFFSET = 2,               // Motors 1&3 simultaneous, Motor 2 with delayed start
    PATTERN_CASCADING = 3,                  // Cascading half-cycle movements with hold states
    PATTERN_SEQUENTIAL_THEN_PARALLEL = 4    // Sequential first-half, simultaneous second-half
  };
  
  // Initialize motion control system (PID controllers)
  void begin();
  
  // Helper function: Execute motor control based on current state
  // Returns the target position being commanded
  float updateMotorState(BTS7960& motor, int motorIndex, MotorState state, 
                         unsigned long phaseStartTime, uint32_t halfCycleDuration,
                         float amplitudeCounts, float holdTarget = 0.0f);
  
  // Run one complete sine cycle with closed-loop position control (single motor)
  void runSineCycle(BTS7960& motor, int motorIndex, float amplitudeRevolutions = 2.0f);                               
  
  // Execute motion pattern 1: Sequential (one motor at a time)
  void executePattern1(BTS7960 motors[], float amplitude);
  
  // Execute motion pattern 2: Simultaneous with phase offset                                                                                                                        
  void executePattern2(BTS7960 motors[], float amplitude);
  
  // Execute motion pattern 3: Cascading half-cycle sequential
  void executePattern3(BTS7960 motors[], float amplitude);
  
  // Execute motion pattern 4: Sequential first-half then parallel second-half
  void executePattern4(BTS7960 motors[], float amplitude);
}

#endif // MOTION_CONTROL_H
