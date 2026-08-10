#ifndef ORION_LSM6DSO32_H // include guard
#define ORION_LSM6DSO32_H

#include <Arduino.h>
#include <stdint.h>

/* ============================================================================
 * OrionTracker - LSM6DSO32 6-axis IMU driver (accel + gyro, STM32duino library).
 *
 * On SPI1 (shared bus with the barometer, separate CS). No interrupt pin -> the
 * flight loop polls imuUpdate(). imuToPacket() stages the reading into the
 * SX1262 telemetry packet. Accel in g, gyro in deg/s.
 * ========================================================================== */

// Bring up the sensor (accel + gyro). Returns false if it does not respond.
bool imuInit();

// Read a fresh accel + gyro sample. Returns true on success. Poll periodically.
bool imuUpdate();

// Cached values (no I/O; valid after a successful imuUpdate()).
float imuAccX();   // g
float imuAccY();
float imuAccZ();
float imuGyroX();  // deg/s
float imuGyroY();
float imuGyroZ();

// Push the current accel + gyro into the outgoing telemetry packet.
void imuToPacket();

#endif // ORION_LSM6DSO32_H
