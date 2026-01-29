// config.h - Global configuration and pin definitions
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== USER TUNABLE PARAMETERS =====================

// Sine profile timing
static const uint32_t SINE_DURATION_MS = 5000;   // total time per motor for one sine cycle
static const uint32_t PAUSE_BETWEEN_MS = 400;    // small pause after each motor finishes

// PWM setup
static const uint32_t PWM_FREQ   = 20000;        // 20 kHz -> silent for most motors
static const uint8_t  PWM_RESBIT = 10;           // 10-bit resolution (0..1023)
static const int      PWM_MAX    = (1 << PWM_RESBIT) - 1;

// ===================== PIN MAP =====================

// Motor 1 pins
static const int M1_RPWM = 25;
static const int M1_LPWM = 26;
static const int M1_REN  = -1;    // set to -1 if hard-wired HIGH
static const int M1_LEN  = -1;    // set to -1 if hard-wired HIGH

// Motor 2 pins
static const int M2_RPWM = 32;   
static const int M2_LPWM = 33;   
static const int M2_REN  = -1;
static const int M2_LEN  = -1;

// Motor 3 pins
static const int M3_RPWM = 12; 
static const int M3_LPWM = 13;  
static const int M3_REN  = -1;
static const int M3_LEN  = -1;

// Limit sensor pins (one per motor)
static const int S_M1 = 15;
static const int S_M2 = 21;
static const int S_M3 = 14;

// Encoder pins (quadrature encoders: A and B channels)
static const int ENC_M1_A = 22; 
static const int ENC_M1_B = 23; 
static const int ENC_M2_A = 18;  
static const int ENC_M2_B = 19; 
static const int ENC_M3_A = 34; 
static const int ENC_M3_B = 35; 

// Button input pins
static const int BTN_INCREASE_PIN = 27; // Button to increase amplitude
static const int BTN_DECREASE_PIN = 4;  // Button to decrease amplitude

// ===================== ENCODER CONFIGURATION =====================
static const int ENCODER_PPR = 2125;        // Pulses Per Revolution (adjust to your encoder)
static const float COUNTS_PER_REV = ENCODER_PPR * 4.0f; // x4 PCNT decoding (counts on both edges of A and B)

// ===================== PCNT CONFIGURATION =====================
// Glitch filter value in APB clock cycles (80MHz → 12.5ns per cycle)
// Range: 0-1023. Higher = more filtering. 1000 ≈ 12.5µs filter window
static const uint16_t PCNT_FILTER_VAL = 100;  // ~1.25µs filter (good for most encoders)

// ===================== PID CONFIGURATION =====================
static const float PID_KP = 2.5f;          // Proportional gain
static const float PID_KI = 0.0f;          // Integral gain
static const float PID_KD = 0.01f;          // Derivative gain
static const float PID_SAMPLE_TIME_MS = 2.0f;  // PID update rate (2ms = 500Hz)
static const float PID_OUTPUT_LIMIT = PWM_MAX;  // Max PID output = max PWM
static const float PID_INTEGRAL_LIMIT = 500.0f; // Anti-windup limit

// ===================== LEDC CHANNEL ALLOCATION =====================
static const int CH_M1_R = 0;
static const int CH_M1_L = 1;
static const int CH_M2_R = 2;
static const int CH_M2_L = 3;
static const int CH_M3_R = 4;
static const int CH_M3_L = 5;

#endif // CONFIG_H
