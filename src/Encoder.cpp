// Encoder.cpp - Hardware PCNT-based quadrature encoder implementation
// Uses ESP32 Pulse Counter (PCNT) peripheral for accurate, low-overhead counting
// Configuration: X4 decoding (counts on both edges of A and B)

#include "Encoder.h"
#include "config.h"
#include "driver/pcnt.h"

namespace EncoderModule {

// Spinlock for thread-safe access to overflow counters
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// PCNT unit assignments for each motor encoder
static const pcnt_unit_t PCNT_UNITS[NUM_ENCODERS] = {
  PCNT_UNIT_0,  // Motor 1 encoder
  PCNT_UNIT_1,  // Motor 2 encoder
  PCNT_UNIT_2   // Motor 3 encoder
};

// Encoder pin arrays for easy iteration
static const int ENC_A_PINS[NUM_ENCODERS] = {ENC_M1_A, ENC_M2_A, ENC_M3_A};
static const int ENC_B_PINS[NUM_ENCODERS] = {ENC_M1_B, ENC_M2_B, ENC_M3_B};

// Software overflow accumulators (PCNT is 16-bit signed, we need 32-bit range)
// PCNT range is -32768 to +32767; we accumulate overflows for larger counts
static volatile int32_t overflowCounts[NUM_ENCODERS] = {0, 0, 0};

// Overflow threshold - when PCNT reaches these values, we accumulate and reset
static const int16_t PCNT_H_LIM = 30000;   // High limit before overflow accumulation
static const int16_t PCNT_L_LIM = -30000;  // Low limit before overflow accumulation

// ISR handler for PCNT overflow events (minimal overhead)
static void IRAM_ATTR pcnt_overflow_handler(void* arg) {
  int motorIndex = (int)(intptr_t)arg;
  pcnt_unit_t unit = PCNT_UNITS[motorIndex];
  
  // Check which event triggered
  uint32_t status = 0;
  pcnt_get_event_status(unit, &status);
  
  if (status & PCNT_EVT_H_LIM) {
    overflowCounts[motorIndex] += PCNT_H_LIM;
    pcnt_counter_clear(unit);
  } else if (status & PCNT_EVT_L_LIM) {
    overflowCounts[motorIndex] += PCNT_L_LIM;
    pcnt_counter_clear(unit);
  }
}

void begin() {
  Serial.println("Initializing PCNT-based encoders...");
  
  for (int i = 0; i < NUM_ENCODERS; i++) {
    pcnt_unit_t unit = PCNT_UNITS[i];
    int pinA = ENC_A_PINS[i];
    int pinB = ENC_B_PINS[i];
    
    // Configure Channel 0
    pcnt_config_t pcnt_config_0 = {
      .pulse_gpio_num = pinA,
      .ctrl_gpio_num  = pinB,
      .lctrl_mode     = PCNT_MODE_REVERSE, // Reverse counting when CTRL is low
      .hctrl_mode     = PCNT_MODE_KEEP,    // Keep counting when CTRL is high
      .pos_mode       = PCNT_COUNT_DEC,    // Decrement on positive edge
      .neg_mode       = PCNT_COUNT_INC,    // Increment on negative edge
      .counter_h_lim  = PCNT_H_LIM,
      .counter_l_lim  = PCNT_L_LIM,
      .unit           = unit,
      .channel        = PCNT_CHANNEL_0,
    };
    esp_err_t err0 = pcnt_unit_config(&pcnt_config_0);
    if (err0 != ESP_OK) {
      Serial.printf("  ERROR: PCNT config Ch0 failed for M%d (err=%d)\n", i + 1, err0);
      continue;
    }

    // Configure Channel 1 (Input/Ctrl pins swapped for X4 reading)
    // This reads the edges of Signal B relative to Signal A
    pcnt_config_t pcnt_config_1 = {
      .pulse_gpio_num = pinB, // Swap Input
      .ctrl_gpio_num  = pinA, // Swap Ctrl
      .lctrl_mode     = PCNT_MODE_REVERSE,
      .hctrl_mode     = PCNT_MODE_KEEP,
      .pos_mode       = PCNT_COUNT_INC,
      .neg_mode       = PCNT_COUNT_DEC,
      .counter_h_lim  = PCNT_H_LIM,
      .counter_l_lim  = PCNT_L_LIM,
      .unit           = unit,
      .channel        = PCNT_CHANNEL_1,
    };
    esp_err_t err1 = pcnt_unit_config(&pcnt_config_1);
    if (err1 != ESP_OK) {
      Serial.printf("  ERROR: PCNT config Ch1 failed for M%d (err=%d)\n", i + 1, err1);
      continue;
    }
    
    // Enable glitch filter to suppress noise (PCNT_FILTER_VAL APB clock cycles)
    // At 80MHz APB clock: 1000 cycles ≈ 12.5µs filter window
    pcnt_set_filter_value(unit, PCNT_FILTER_VAL);
    pcnt_filter_enable(unit);
    
    // Configure overflow event interrupts
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    
    // Install ISR service and add handler for this unit
    if (i == 0) {
      // Install ISR service only once (shared across all units)
      // Check if already installed to avoid error
      pcnt_isr_service_install(0);
    }
    pcnt_isr_handler_add(unit, pcnt_overflow_handler, (void*)(intptr_t)i);
    
    // Initialize counter to zero
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    overflowCounts[i] = 0;
    pcnt_counter_resume(unit);
    
    Serial.printf("  -> Encoder M%d: PCNT_UNIT_%d (A=GPIO%d, B=GPIO%d)\n", 
                  i + 1, i, pinA, pinB);
  }
  
  Serial.printf("  Glitch filter: %d APB cycles (~%.1f us)\n", 
                PCNT_FILTER_VAL, PCNT_FILTER_VAL * 0.0125f);
  Serial.println("  Mode: X4 decoding (counts on both edges of A and B)");
  Serial.printf("  Overflow limits: +/-%d counts (auto-accumulated)\n", PCNT_H_LIM);
}

long getPosition(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_ENCODERS) return 0;
  
  pcnt_unit_t unit = PCNT_UNITS[motorIndex];
  int16_t count = 0;
  
  // Read current PCNT value (atomic hardware read)
  pcnt_get_counter_value(unit, &count);
  
  // Combine with overflow accumulator for full 32-bit range
  // Use critical section to ensure atomic read of overflow counter
  portENTER_CRITICAL(&spinlock);
  int32_t totalCount = overflowCounts[motorIndex] + count;
  portEXIT_CRITICAL(&spinlock);
  
  return (long)totalCount;
}

void resetPosition(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_ENCODERS) return;
  
  pcnt_unit_t unit = PCNT_UNITS[motorIndex];
  
  // Pause counter, clear, and reset overflow accumulator
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  
  portENTER_CRITICAL(&spinlock);
  overflowCounts[motorIndex] = 0;
  portEXIT_CRITICAL(&spinlock);
  
  pcnt_counter_resume(unit);
}

float countsToDegrees(long counts) {
  return (counts / COUNTS_PER_REV) * 360.0f;
}

float countsToRevolutions(long counts) {
  return counts / COUNTS_PER_REV;
}

void printPositions() {
  long e0 = getPosition(0);
  long e1 = getPosition(1);
  long e2 = getPosition(2);
  Serial.printf("Enc counts: M1=%ld, M2=%ld, M3=%ld  | deg: M1=%.1f, M2=%.1f, M3=%.1f\n",
                e0, e1, e2, countsToDegrees(e0), countsToDegrees(e1), countsToDegrees(e2));
}

} // namespace EncoderModule
