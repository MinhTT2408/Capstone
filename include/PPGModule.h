// PPGModule.h - PPG sensor + BLE interface for compression therapy monitoring
// Handles MAX30105 sensor reading, BLE communication, and session management
#ifndef PPG_MODULE_H
#define PPG_MODULE_H

#include <Arduino.h>

namespace PPGModule {

  // ===================== LIFECYCLE =====================
  
  /**
   * @brief Initialize PPG sensor (MAX30105), BLE server, and characteristics
   * Must be called in setup() after Serial.begin()
   */
  void begin();

  /**
   * @brief Process BLE requests and read/send PPG data
   * Must be called every loop() iteration.
   * Handles: session start/end flags, history fetch, PPG read + BLE notify
   */
  void update();

  // ===================== SESSION STATE =====================
  
  /**
   * @brief Check if a BLE-initiated session is currently active
   * @return true if session is running (compression level 1-4 received)
   */
  bool isSessionActive();

  /**
   * @brief Check if a session was just started this update cycle
   * Resets to false after being read (one-shot flag)
   */
  bool sessionJustStarted();

  /**
   * @brief Check if a session was just ended this update cycle
   * Resets to false after being read (one-shot flag)
   */
  bool sessionJustEnded();

  /**
   * @brief End the current session programmatically (e.g., on BLE disconnect)
   */
  void endSession();

  // ===================== COMPRESSION LEVEL =====================
  
  /**
   * @brief Get the current compression level received from BLE (1-4, or 0 if stopped)
   */
  int getCompressionLevel();

  /**
   * @brief Get motor amplitude in revolutions based on current compression level
   * Uses LEVEL_AMPLITUDE[] mapping from config.h
   */
  // NOTE: Amplitude is now computed by ForceControl PID from force setpoint.
  // Use ForceControl::setForceSetpoint(LEVEL_FORCE_SETPOINT[getCompressionLevel()])
  // then ForceControl::getDesiredRevolutions() instead.

  // ===================== BLE STATE =====================
  
  /**
   * @brief Check if a BLE device (phone app) is currently connected
   */
  bool isBleConnected();

  // ===================== STOP REQUEST =====================
  
  /**
   * @brief Global flag: set true when BLE requests motor stop (session end)
   * Checked by MotionControl inner loops for mid-cycle abort
   */
  extern volatile bool stopMotorRequested;

  // ===================== FORCE SENSOR LOGGING =====================

  /**
   * @brief Queue a force sensor reading row for SD card write.
   * Non-blocking — drops silently if queue is full.
   * @param elapsedMs Milliseconds since session start
   * @param f1 Force sensor 1 reading in Newtons
   * @param f2 Force sensor 2 reading in Newtons
   * @param f3 Force sensor 3 reading in Newtons
   */
  void logForceData(uint32_t elapsedMs, float f1, float f2, float f3);

  /**
   * @brief Return the millis() timestamp when the current session started.
   * Used by forceLogTask in main.cpp to compute elapsed time.
   */
  unsigned long getSessionStartTime();
}

#endif // PPG_MODULE_H
