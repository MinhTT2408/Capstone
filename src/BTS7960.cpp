// BTS7960.cpp - BTS7960 motor driver implementation
#include "BTS7960.h"
#include <esp32-hal-ledc.h>

BTS7960::BTS7960(int rpwm, int lpwm, int ren, int len)
  : rpwmPin(rpwm), lpwmPin(lpwm), renPin(ren), lenPin(len), 
    rpwmCh(-1), lpwmCh(-1), pwmMax(1023) {
}

void BTS7960::begin(int rpwmChannel, int lpwmChannel, uint32_t freq, uint8_t resBit) {
  rpwmCh = rpwmChannel;
  lpwmCh = lpwmChannel;
  pwmMax = (1 << resBit) - 1;

  // Configure LEDC channels (hardware PWM)
  ledcSetup(rpwmCh, freq, resBit);
  ledcSetup(lpwmCh, freq, resBit);

  // Attach channels to pins
  ledcAttachPin(rpwmPin, rpwmCh);
  ledcAttachPin(lpwmPin, lpwmCh);

  // Optional EN pins
  if (renPin >= 0) {
    pinMode(renPin, OUTPUT);
    digitalWrite(renPin, HIGH);  // enable
  }
  if (lenPin >= 0) {
    pinMode(lenPin, OUTPUT);
    digitalWrite(lenPin, HIGH);  // enable
  }

  // Ensure stopped
  brake();
}

void BTS7960::set(int duty, Dir dir) {
  duty = constrain(duty, 0, pwmMax);
  if (dir == FORWARD) {
    ledcWrite(rpwmCh, duty);
    ledcWrite(lpwmCh, 0);
  } else {
    ledcWrite(rpwmCh, 0);
    ledcWrite(lpwmCh, duty);
  }
}

void BTS7960::coast() {
  ledcWrite(rpwmCh, 0);
  ledcWrite(lpwmCh, 0);
}

void BTS7960::brake() {
  coast(); // safest default for these modules
}
