// ButtonInput.h - Button input handler for amplitude control
#ifndef BUTTON_INPUT_H
#define BUTTON_INPUT_H

#include <Arduino.h>

namespace ButtonInput {
  // Initialize button pins
  void begin();
  
  // Update button states (call in loop)
  void update();
  
  // Get current amplitude setting (in revolutions)
  float getAmplitude();
  
  // Check if amplitude has changed since last check
  bool hasChanged();
}

#endif // BUTTON_INPUT_H
