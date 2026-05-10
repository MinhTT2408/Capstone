// ForceControl.h - Outer loop force PID control interface
// Cascaded control: Force PID (outer) -> Position PID (inner)
#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H

#include <Arduino.h>
#include "PIDController.h"

namespace ForceControl {

// ===================== FORCE CONTROL CONFIGURATION =====================
// Force sensor configuration (one sensor per motor)
static constexpr int FORCE_SENSOR_PINS[3] = {4, 5, 6};  // GPIO pins for motors 1, 2, 3
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
 * @brief Read raw force sensor value for a specific motor
 * @param motorIndex Motor index (0-2)
 * @return Raw ADC value (0-4095 for 12-bit ADC)
 */
int readRawSensor(int motorIndex);

/**
 * @brief Read force sensor and convert to Newtons for a specific motor
 * @param motorIndex Motor index (0-2)
 * @return Force in Newtons
 */
float readForce(int motorIndex);

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
 * @brief Update force PID controller for a specific motor
 * Should be called at FORCE_PID_SAMPLE_TIME_MS intervals
 * @param motorIndex Motor index (0-2)
 * @return Desired revolutions for the inner position control loop
 */
float update(int motorIndex);

/**
 * @brief Get the last computed desired revolutions for a specific motor
 * @param motorIndex Motor index (0-2)
 * @return Desired revolutions
 */
float getDesiredRevolutions(int motorIndex);

/**
 * @brief Get the last measured force for a specific motor
 * @param motorIndex Motor index (0-2)
 * @return Last measured force in Newtons
 */
float getLastMeasuredForce(int motorIndex);

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
 * @brief Start tracking peak force for all motors
 * Resets all peak force values to begin fresh tracking
 */
void startPeakTracking();

/**
 * @brief Start tracking peak force for a specific motor
 * Resets that motor's peak force value only
 * @param motorIndex Motor index (0-2)
 */
void startPeakTracking(int motorIndex);

/**
 * @brief Update peak force tracking for all motors
 * Should be called periodically during sequence execution
 */
void updatePeakTracking();

/**
 * @brief Get the peak force measured for a specific motor during current sequence
 * @param motorIndex Motor index (0-2)
 * @return Peak force in Newtons
 */
float getPeakForce(int motorIndex);

/**
 * @brief Compute amplitude for next sequence for a specific motor
 * Compares peak force to setpoint and adjusts amplitude
 * @param motorIndex Motor index (0-2)
 * @return Computed amplitude in revolutions for next sequence
 */
float computeNextAmplitude(int motorIndex);

/**
 * @brief Get initial amplitude for the first sequence for a specific motor
 * @param motorIndex Motor index (0-2)
 * @return Initial amplitude in revolutions
 */
float getInitialAmplitude(int motorIndex);

} // namespace ForceControl

#endif // FORCE_CONTROL_H
