// BTS7960.h - BTS7960 motor driver interface
#ifndef BTS7960_H
#define BTS7960_H

#include <Arduino.h>

enum Dir { FORWARD = 0, REVERSE = 1 };

// Represents one BTS7960 H-bridge motor driver
class BTS7960 {
public:
  BTS7960(int rpwm, int lpwm, int ren = -1, int len = -1);
  
  void begin(int rpwmChannel, int lpwmChannel, uint32_t freq, uint8_t resBit);
  void set(int duty, Dir dir);
  void coast();
  void brake();

private:
  int rpwmPin;
  int lpwmPin;
  int renPin;
  int lenPin;
  int rpwmCh;
  int lpwmCh;
  int pwmMax;
};

#endif // BTS7960_H
