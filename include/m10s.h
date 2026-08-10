#ifndef ORION_M10S_H // include guard
#define ORION_M10S_H

#include <Arduino.h>
#include <stdint.h>

/* ============================================================================
 * OrionTracker - u-blox MAX-M10S GNSS driver (RX-only NMEA, via TinyGPSPlus).
 *
 * Wiring (USART1):
 *   module TX -> PA10 (USART1_RX)   <- the only line this driver needs
 *   module RX <- PA15 (USART1_TX)   <- unused; we never transmit to the module
 *   data-ready / PPS -> PB4 (EXTI interrupt, used only as a poll hint)
 *
 * The module streams NMEA on its own, so gpsUpdate() just parses the incoming
 * sentences into a cached snapshot; the getters below never do I/O, so the
 * flight loop can read them freely. gpsToPacket() hands the GPS fields the radio
 * sends to the ground station over to the SX1262 telemetry packet.
 * ========================================================================== */

// Bring up USART1 and lock onto the module's NMEA baud. Returns false if no
// NMEA is seen on RX. Call once in setup().
bool GPSInit();

// Parse any waiting NMEA. Returns true when a fresh solution was decoded.
bool gpsUpdate();

// True if the last solution is a usable fix.
bool gpsHasFix();

// True once per fresh solution; reading it clears the "new" flag.
bool gpsHasNewFix();

// True if valid NMEA has been received recently (link is alive).
bool gpsIsAlive();

// True if the data-ready interrupt (PB4) has fired since the last gpsUpdate().
bool gpsDataReady();

// --- Cached fix values (no I/O; valid after a successful gpsUpdate()) --------
double  gpsLatitude();   // degrees
double  gpsLongitude();  // degrees
double  gpsAltitude();   // metres MSL
double  gpsSpeed();      // m/s over ground
double  gpsCourse();     // degrees (heading of motion)
double  gpsHDOP();       // horizontal dilution of precision
uint8_t gpsSats();       // satellites used in the solution
uint8_t gpsFixType();    // 0 = no fix, 3 = fix

// --- Time --------------------------------------------------------------------
bool     gpsTimeValid(); // true when UTC time from the module is valid
uint32_t gpsTimestamp(); // Unix epoch seconds (0 until time is valid)

// Push the current fix into the outgoing telemetry packet (calls radioSetGPS).
void gpsToPacket();

// Print the current fix over Serial for debugging.
void gpsReport();

#endif // ORION_M10S_H
