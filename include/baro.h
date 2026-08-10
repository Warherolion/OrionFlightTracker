#ifndef ORION_BARO_H // include guard
#define ORION_BARO_H

#include <Arduino.h>
#include <stdint.h>

/* ============================================================================
 * OrionTracker - LPS22HB barometer driver (STM32duino LPS22HB library).
 *
 * Reads pressure + temperature and derives barometric altitude, then stages the
 * result into the SX1262 telemetry packet. Altitude is MSL by default (uses a
 * sea-level reference pressure); call baroZero() on the pad to make it read
 * height above ground (AGL).
 * ========================================================================== */

// Bring up the sensor. Returns false if the LPS22HB does not respond.
bool baroInit();

// Read a fresh pressure/temperature sample and recompute altitude. Returns true
// on a successful read. Call periodically (e.g. 25-50 Hz).
bool baroUpdate();

// Cached values (no I/O; valid after a successful baroUpdate()).
float baroPressure();      // hPa
float baroTemperature();   // degrees C
float baroAltitude();      // metres (MSL, or AGL after baroZero())

// Capture the current pressure as the 0 m reference, so altitude reads ~0 at the
// launch pad and climbs as AGL. Call once when armed/on the pad.
void baroZero();

// Set the MSL reference pressure (hPa) for absolute altitude. Default 1013.25.
// Clears any AGL zero set by baroZero().
void baroSetSeaLevel(float hPa);

// Push the current altitude/pressure/temperature into the outgoing radio packet.
void baroToPacket();

#endif // ORION_BARO_H
