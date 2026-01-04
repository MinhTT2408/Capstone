// Encoder.h - Quadrature encoder interface
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Number of motors/encoders
#define NUM_ENCODERS 3

namespace EncoderModule {
  // Initialize encoder pins and attach interrupts
  void begin();
  
  // Read encoder position safely (atomic read)
  long getPosition(int motorIndex);
  
  // Reset encoder position to zero
  void resetPosition(int motorIndex);
  
  // Convert encoder counts to degrees
  float countsToDegrees(long counts);
  
  // Convert encoder counts to revolutions
  float countsToRevolutions(long counts);
  
  // Print all encoder values to serial
  void printPositions();
}

#endif // ENCODER_H
