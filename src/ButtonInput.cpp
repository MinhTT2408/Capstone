// ButtonInput.cpp - Button input handler implementation
#include "ButtonInput.h"
#include "config.h"

namespace ButtonInput {

// Current amplitude setting
static float currentAmplitude = 5.0f;  // Default: 5 revolutions
static bool amplitudeChanged = false;

// Button debouncing for increase button
static unsigned long lastDebounceTimeInc = 0;
static bool lastIncButtonState = HIGH;
static bool incButtonState = HIGH;
static bool incButtonPressed = false;

// Button debouncing for decrease button
static unsigned long lastDebounceTimeDec = 0;
static bool lastDecButtonState = HIGH;
static bool decButtonState = HIGH;
static bool decButtonPressed = false;

static const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce

void begin() {
  pinMode(BTN_INCREASE_PIN, INPUT_PULLUP);
  pinMode(BTN_DECREASE_PIN, INPUT_PULLUP);
  
  Serial.println("Button input initialized:");
  Serial.printf("  Increase amplitude: GPIO%d\n", BTN_INCREASE_PIN);
  Serial.printf("  Decrease amplitude: GPIO%d\n", BTN_DECREASE_PIN);
  Serial.printf("  Default amplitude: %.1f rev\n", currentAmplitude);
}

void update() {
  unsigned long now = millis();
  
  // Read raw button states (active LOW with pullup)
  bool incReading = (digitalRead(BTN_INCREASE_PIN) == LOW);
  bool decReading = (digitalRead(BTN_DECREASE_PIN) == LOW);
  
  // --- Debounce INCREASE button ---
  if (incReading != lastIncButtonState) {
    lastDebounceTimeInc = now;  // Reset timer on state change
  }
  
  if ((now - lastDebounceTimeInc) > DEBOUNCE_DELAY) {
    // Signal has been stable for DEBOUNCE_DELAY
    if (incReading != incButtonState) {
      incButtonState = incReading;
      
      // Detect press event (transition from HIGH to LOW)
      if (incButtonState == LOW && !incButtonPressed) {
        incButtonPressed = true;
        if (currentAmplitude < 10.0f) {  // Max 10 revolutions
          currentAmplitude += 1.0f;
          amplitudeChanged = true;
          Serial.printf(">>> Amplitude increased: %.1f rev\n", currentAmplitude);
        }
      } else if (incButtonState == HIGH) {
        incButtonPressed = false;  // Button released
      }
    }
  }
  lastIncButtonState = incReading;
  
  // --- Debounce DECREASE button ---
  if (decReading != lastDecButtonState) {
    lastDebounceTimeDec = now;  // Reset timer on state change
  }
  
  if ((now - lastDebounceTimeDec) > DEBOUNCE_DELAY) {
    // Signal has been stable for DEBOUNCE_DELAY
    if (decReading != decButtonState) {
      decButtonState = decReading;
      
      // Detect press event (transition from HIGH to LOW)
      if (decButtonState == LOW && !decButtonPressed) {
        decButtonPressed = true;
        if (currentAmplitude > 1.0f) {  // Min 1 revolution
          currentAmplitude -= 1.0f;
          amplitudeChanged = true;
          Serial.printf(">>> Amplitude decreased: %.1f rev\n", currentAmplitude);
        }
      } else if (decButtonState == HIGH) {
        decButtonPressed = false;  // Button released
      }
    }
  }
  lastDecButtonState = decReading;
}

float getAmplitude() {
  return currentAmplitude;
}

bool hasChanged() {
  bool changed = amplitudeChanged;
  amplitudeChanged = false;  // Clear flag after reading
  return changed;
}

} // namespace ButtonInput
