// LimitSensors.cpp - Limit switch sensor implementation
#include "LimitSensors.h"
#include "config.h"

namespace LimitSensors {

// bit flags set from ISR: bit 0..2 for sensors S_M1, S_M2, S_M3
volatile uint8_t sensorFlags = 0;

// Minimal ISRs: mark the corresponding bit when a change occurs
void IRAM_ATTR sensorISR0() { sensorFlags |= (1 << 0); } // S_M1
void IRAM_ATTR sensorISR1() { sensorFlags |= (1 << 1); } // S_M2
void IRAM_ATTR sensorISR2() { sensorFlags |= (1 << 2); } // S_M3

void begin() {
  // Configure limit sensors
  pinMode(S_M1, INPUT_PULLUP);
  pinMode(S_M2, INPUT_PULLUP);
  pinMode(S_M3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(S_M1), sensorISR0, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M2), sensorISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M3), sensorISR2, FALLING);
}

bool isTriggered(int motorIndex) {
  noInterrupts();
  uint8_t flags = sensorFlags;
  interrupts();
  
  return (flags & (1 << motorIndex)) != 0;
}

void clearFlags(int motorIndex) {
  noInterrupts();
  sensorFlags &= ~(1 << motorIndex);
  interrupts();
}

void clearAllFlags() {
  noInterrupts();
  sensorFlags = 0;
  interrupts();
}

void checkPendingTriggers() {
  noInterrupts();
  uint8_t pending = sensorFlags;
  interrupts();
  
  if (pending) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      if (pending & (1 << i)) {
        Serial.printf("Sensor triggered for motor %d outside run -> flagged\n", i + 1);
        clearFlags(i);
      }
    }
  }
}

} // namespace LimitSensors
