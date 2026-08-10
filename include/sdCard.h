#ifndef ORION_SDCARD_H // include guard
#define ORION_SDCARD_H

#include <Arduino.h>
#include <stdint.h>

/* ============================================================================
 * OrionTracker - SD card binary logger (SDIO 4-bit, STM32SD / FatFs).
 *
 * On sdInit() the card is mounted and a fresh log file is opened with an
 * incrementing 8.3 name (LOG000.BIN, LOG001.BIN, ...), so each power-up gets
 * its own file. sdWrite() appends fixed-size binary records (e.g. the packed
 * TelemetryPacket) through a RAM staging buffer to keep SD writes sector-sized.
 * Call sdFlush() periodically (and sdClose() on landing) so a power loss can
 * lose at most one flush interval of data.
 * ========================================================================== */

// Mount the SD card and open a new binary log file. Returns false if the card
// cannot be mounted or the file cannot be created. Call once in setup().
bool sdInit();

// True when the card is mounted and a log file is open for writing.
bool sdIsReady();

// Append 'len' raw bytes to the log (buffered). Returns false if not ready or a
// write failed (on write failure the logger latches to not-ready).
bool sdWrite(const void* data, uint16_t len);

// Force buffered bytes out to the card and fsync the file. Call periodically
// and before shutdown. Returns false if not ready or the sync failed.
bool sdFlush();

// Flush and close the log file (e.g. on landing / disarm).
void sdClose();

// Total payload bytes passed to sdWrite() since sdInit().
uint32_t sdBytesWritten();

#endif // ORION_SDCARD_H
