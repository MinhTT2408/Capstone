// ForceControl.h - Outer loop force PID control interface
// Cascaded control: Force PID (outer) -> Position PID (inner)
#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H

#include <Arduino.h>
#include "PIDController.h"

namespace ForceControl {

// ===================== FORCE CONTROL CONFIGURATION =====================
// Force sensor configuration
static const int FORCE_SENSOR_PIN = 4;        // ADC1_CH0 (GPIO 4 = VP, input only)
static const float FORCE_SENSOR_MIN_V = 0.0f;  // Minimum sensor voltage
static const float FORCE_SENSOR_MAX_V = 3.3f;  // Maximum sensor voltage (ESP32 ADC reference)
static const float FORCE_MIN = 0.0f;           // Minimum force in Newtons (at min voltage)
static const float FORCE_MAX = 100.0f;         // Maximum force in Newtons (at max voltage)

// Force PID parameters (outer loop - typically slower than inner loop)
static const float FORCE_PID_KP = 0.7f;        // Proportional gain
static const float FORCE_PID_KI = 0.1f;        // Integral gain
static const float FORCE_PID_KD = 0.01f;       // Derivative gain
static const float FORCE_PID_SAMPLE_TIME_MS = 20.0f;  // 20ms = 50Hz (slower than position loop)

// Output limits (revolutions)
static const float MIN_REVOLUTIONS = 1.0f;     // Minimum output revolutions
static const float MAX_REVOLUTIONS = 6.0f;    // Maximum output revolutions
static const float FORCE_PID_INTEGRAL_LIMIT = 5.0f;  // Anti-windup limit

// Default setpoint
static const float DEFAULT_FORCE_SETPOINT = 17.5f;  // Default desired force in Newtons

// ===================== FUNCTION DECLARATIONS =====================

/**
 * @brief Initialize force control system
 * Sets up ADC for force sensor reading and initializes outer PID controller
 */
void begin();

/**
 * @brief Read raw force sensor value
 * @return Raw ADC value (0-4095 for 12-bit ADC)
 */
int readRawSensor();

/**
 * @brief Read force sensor and convert to Newtons
 * @return Force in Newtons
 */
float readForce();

/**
 * @brief Set the desired force setpoint
 * @param force Desired force in Newtons
 */
void setForceSetpoint(float force);

/**
 * @brief Get current force setpoint
 * @return Current force setpoint in Newtons
 */
float getForceSetpoint();

/**
 * @brief Update force PID controller and compute desired revolutions
 * Should be called at FORCE_PID_SAMPLE_TIME_MS intervals
 * @return Desired revolutions for the inner position control loop
 */
float update();

/**
 * @brief Get the last computed desired revolutions output
 * @return Desired revolutions
 */
float getDesiredRevolutions();

/**
 * @brief Get the last measured force
 * @return Last measured force in Newtons
 */
float getLastMeasuredForce();

/**
 * @brief Reset the force PID controller
 * Clears integral term and error history
 */
void reset();

/**
 * @brief Check if it's time to update the force control loop
 * @return true if enough time has passed since last update
 */
bool isUpdateTime();

/**
 * @brief Enable or disable force control mode
 * @param enabled true to enable force control, false to use manual amplitude
 */
void setEnabled(bool enabled);

/**
 * @brief Check if force control is enabled
 * @return true if force control is active
 */
bool isEnabled();

/**
 * @brief Print force control status to Serial
 */
void printStatus();

/**
 * @brief Start tracking peak force during a sequence
 * Resets the peak force value to begin fresh tracking
 */
void startPeakTracking();

/**
 * @brief Update peak force tracking with current measurement
 * Should be called periodically during sequence execution
 */
void updatePeakTracking();

/**
 * @brief Get the peak force measured during current sequence
 * @return Peak force in Newtons
 */
float getPeakForce();

/**
 * @brief Compute amplitude for next sequence based on peak force
 * Compares peak force to setpoint and adjusts amplitude
 * @return Computed amplitude in revolutions for next sequence
 */
float computeNextAmplitude();

/**
 * @brief Get initial amplitude for the first sequence
 * @return Initial amplitude in revolutions
 */
float getInitialAmplitude();

} // namespace ForceControl

#endif // FORCE_CONTROL_H
