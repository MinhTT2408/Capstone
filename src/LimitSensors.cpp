// LimitSensors.cpp - Limit switch sensor implementation
#include "LimitSensors.h"
#include "config.h"

namespace LimitSensors {

// bit flags set from ISR: bit 0..5 for sensors S_M1_F..S_M3_R
volatile uint8_t sensorFlags = 0;

// Minimal ISRs: mark the corresponding bit when a change occurs
void IRAM_ATTR sensorISR0() { sensorFlags |= (1 << 0); } // S_M1_F
void IRAM_ATTR sensorISR1() { sensorFlags |= (1 << 1); } // S_M1_R
void IRAM_ATTR sensorISR2() { sensorFlags |= (1 << 2); } // S_M2_F
void IRAM_ATTR sensorISR3() { sensorFlags |= (1 << 3); } // S_M2_R
void IRAM_ATTR sensorISR4() { sensorFlags |= (1 << 4); } // S_M3_F
void IRAM_ATTR sensorISR5() { sensorFlags |= (1 << 5); } // S_M3_R

void begin() {
  // Configure limit sensors
  pinMode(S_M1_F, INPUT_PULLUP);
  pinMode(S_M1_R, INPUT_PULLUP);
  pinMode(S_M2_F, INPUT_PULLUP);
  pinMode(S_M2_R, INPUT_PULLUP);
  pinMode(S_M3_F, INPUT_PULLUP);
  pinMode(S_M3_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(S_M1_F), sensorISR0, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M1_R), sensorISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M2_F), sensorISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M2_R), sensorISR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M3_F), sensorISR4, FALLING);
  attachInterrupt(digitalPinToInterrupt(S_M3_R), sensorISR5, FALLING);
}

bool isTriggered(int motorIndex) {
  noInterrupts();
  uint8_t flags = sensorFlags;
  interrupts();
  
  int sensorBitBase = motorIndex * 2;
  return (flags & ((1 << sensorBitBase) | (1 << (sensorBitBase + 1)))) != 0;
}

void clearFlags(int motorIndex) {
  int sensorBitBase = motorIndex * 2;
  noInterrupts();
  sensorFlags &= ~((1 << sensorBitBase) | (1 << (sensorBitBase + 1)));
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
      uint8_t mask = (1 << (i * 2)) | (1 << (i * 2 + 1));
      if (pending & mask) {
        Serial.printf("Sensor triggered for motor %d outside run -> flagged\n", i + 1);
        clearFlags(i);
      }
    }
  }
}

} // namespace LimitSensors
