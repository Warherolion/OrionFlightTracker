#ifndef ORION_SX1262_H // include guard
#define ORION_SX1262_H

#include <Arduino.h>
#include <stdint.h>

/* ============================================================================
 * OrionTracker - SX1262 LoRa telemetry link (flight-computer / transmit side)
 *
 * Packages IMU + barometer + GPS data into one fixed binary packet and sends
 * it to the ground station over LoRa. Transmission is NON-BLOCKING: startTx
 * hands the packet to the SX1262 which sends it in the background while the
 * flight loop keeps polling sensors, so the loop never stalls on the radio.
 *
 * Typical use (see README / main.cpp):
 *   setup():  radioInit();
 *   loop():   radioSetIMU(...);   // when a new IMU sample is ready
 *             radioSetBaro(...);  // when a new baro sample is ready
 *             radioSetGPS(...);   // when a new GPS fix is ready
 *             radioSetState(...); // when flight state changes
 *             if (telemetryDue && !radioBusy()) radioSend();  // ~10 Hz
 *
 * The ground station decodes the TelemetryPacket byte layout below. LoRa CRC
 * is enabled, so the receiver silently drops corrupted packets.
 * ========================================================================== */

// Sync/version byte at the start of every packet. Bump if the layout changes
// so the ground station can reject packets from an old firmware build.
static const uint8_t ORION_PACKET_MAGIC = 0xA5;

// Flight state codes - shared verbatim with the ground station decoder.
enum FlightState : uint8_t {
  STATE_IDLE    = 0,
  STATE_ARMED   = 1,
  STATE_BOOST   = 2,
  STATE_COAST   = 3,
  STATE_APOGEE  = 4,
  STATE_DESCENT = 5,
  STATE_LANDED  = 6
};

/* Fixed binary telemetry packet, little-endian (STM32 native, no conversion).
 * Packed so there is no padding - the ground station must use a byte-identical
 * struct to decode it. lat/lon keep u-blox native scaling (degrees * 1e7) to
 * avoid the precision loss of packing coordinates into a 32-bit float. */
struct __attribute__((packed)) TelemetryPacket {
  uint8_t  magic;        // ORION_PACKET_MAGIC
  uint16_t seq;          // incrementing packet counter (wraps at 65535)
  uint32_t t_ms;         // flight-computer millis() at send time
  uint8_t  state;        // FlightState

  // --- GPS (u-blox MAX-M10S) ---
  int32_t  lat;          // degrees * 1e7
  int32_t  lon;          // degrees * 1e7
  float    altGps;       // metres MSL
  uint32_t gpsAge;       // ms since last GPS update (0xFFFFFFFF = never)
  uint8_t  sats;         // satellites in view
  uint8_t  fixType;      // 0 = no fix, 2 = 2D, 3 = 3D

  // --- Barometer (LPS22HB) ---
  float    altBaro;      // metres (derived from pressure)
  float    pressure;     // hPa
  float    temperature;  // degrees C

  // --- IMU (LSM6DSO32) ---
  float    accX, accY, accZ;     // g
  float    gyroX, gyroY, gyroZ;  // deg/s
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise the SX1262 and LoRa parameters. Call once in setup().
// Returns false if the radio does not respond or a parameter is rejected.
bool radioInit();

// Stage the latest data from each subsystem into the outgoing packet. Call
// these whenever the corresponding sensor produces a fresh sample; they only
// copy into a buffer and are cheap, so different call rates are fine.
void radioSetGPS(int32_t lat, int32_t lon, float altMSL,
                 uint8_t sats, uint8_t fixType);
void radioSetBaro(float altBaro, float pressureHpa, float temperatureC);
void radioSetIMU(float ax, float ay, float az, float gx, float gy, float gz);
void radioSetState(uint8_t flightState);

// Finalise the current packet (stamp magic, sequence number, timestamp and
// GPS age). Called automatically by radioSend(); exposed for advanced use.
bool radioFormat();

// Non-blocking transmit of the current packet. Returns false if the radio is
// still sending the previous packet or if the transmit failed to start (in
// which case just try again next loop). Use this in flight.
bool radioSend();

// Blocking transmit - waits until the packet is fully sent. Simple for bench
// testing; do NOT use in flight (it stalls the loop for the packet airtime).
bool radioSendBlocking();

// True while a transmission is in progress. Gate radioSend() on !radioBusy().
bool radioBusy();

// Number of packets whose transmission has been started (== next seq number).
uint16_t radioPacketCount();

#endif // ORION_SX1262_H
