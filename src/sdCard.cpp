#include "sdCard.h"
#include <STM32SD.h>
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * SDIO uses the STM32's fixed SDMMC pins (F412: PC8-11 = D0-D3, PC12 = CK,
 * PD2 = CMD). No card-detect pin wired -> SD_DETECT_NONE.
 * ------------------------------------------------------------------------- */
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif

// RAM staging buffer: one SD sector so writes hit the card sector-aligned.
#ifndef SD_LOG_BUF
#define SD_LOG_BUF 512
#endif

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static File     logFile;
static bool     ready      = false;
static uint32_t totalBytes = 0;

static uint8_t  buf[SD_LOG_BUF];
static uint16_t bufLen = 0;

// Push the staging buffer to the file (no fsync). Latches not-ready on error.
static bool drainBuffer() {
  if (bufLen == 0) {
    return true;
  }
  size_t written = logFile.write(buf, bufLen);
  bool ok = (written == bufLen);
  bufLen = 0;
  if (!ok) {
    ready = false;
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool sdInit() {
  ready      = false;
  totalBytes = 0;
  bufLen     = 0;

  if (!SD.begin(SD_DETECT_PIN)) {
    return false;
  }

  // Pick the next unused 8.3 log name: LOG000.BIN, LOG001.BIN, ...
  char name[16];
  for (uint16_t i = 0; i < 1000; i++) {
    snprintf(name, sizeof(name), "LOG%03u.BIN", i);
    if (!SD.exists(name)) {
      break;
    }
  }

  logFile = SD.open(name, FILE_WRITE);
  if (!logFile) {
    return false;
  }

  ready = true;
  return true;
}

bool sdIsReady() {
  return ready;
}

bool sdWrite(const void* data, uint16_t len) {
  if (!ready || data == nullptr || len == 0) {
    return false;
  }

  const uint8_t* p = (const uint8_t*)data;
  totalBytes += len;

  // Copy into the staging buffer, draining to the card whenever it fills.
  while (len > 0) {
    uint16_t space = SD_LOG_BUF - bufLen;
    uint16_t n = (len < space) ? len : space;
    memcpy(buf + bufLen, p, n);
    bufLen += n;
    p      += n;
    len    -= n;
    if (bufLen == SD_LOG_BUF) {
      if (!drainBuffer()) {
        return false;
      }
    }
  }
  return true;
}

bool sdFlush() {
  if (!ready) {
    return false;
  }
  if (!drainBuffer()) {
    return false;
  }
  logFile.flush();   // FatFs sync -> physically commit to the card
  return true;
}

void sdClose() {
  if (!ready) {
    return;
  }
  drainBuffer();
  logFile.flush();
  logFile.close();
  ready = false;
}

uint32_t sdBytesWritten() {
  return totalBytes;
}
