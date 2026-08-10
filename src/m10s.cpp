#include "m10s.h"
#include "sx1262.h"                     // radioSetGPS() for gpsToPacket()
#include <TinyGPSPlus.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * RX-ONLY NMEA driver. The M10S streams NMEA on its own, so we only need the
 * module's TX -> PA10 (USART1_RX); we never transmit to it.
 *
 * PIN MAP - OrionTracker PCB wiring, M10S on USART1.
 * ------------------------------------------------------------------------- */
#ifndef GPS_RX
#define GPS_RX    PA10   // USART1_RX  <- module TX  (the line we rely on)
#endif
#ifndef GPS_TX
#define GPS_TX    PA15   // USART1_TX  -> module RX  (unused in RX-only mode)
#endif
#ifndef GPS_INT
#define GPS_INT   PB4    // data-ready / PPS from the module (EXTI4)
#endif
#ifndef GPS_BAUD
#define GPS_BAUD  38400  // MAX-M10S default UART baud
#endif

// ---------------------------------------------------------------------------
// Module + state
// ---------------------------------------------------------------------------
static HardwareSerial gpsSerial(GPS_RX, GPS_TX);   // binds to USART1
static TinyGPSPlus     gps;

static volatile bool g_irqFired = false;   // set by the PB4 interrupt
static bool          g_initOk   = false;
static bool          g_newFix   = false;   // set per fresh update, cleared by reader
static uint32_t      g_lastRxMs = 0;       // millis() of the last byte received

static struct {
  int32_t  latRaw;   // deg * 1e7
  int32_t  lonRaw;   // deg * 1e7
  double   lat, lon; // degrees
  double   altMSL;   // metres
  double   speed;    // m/s
  double   course;   // degrees
  double   hdop;
  uint8_t  sats;
  uint8_t  fixType;  // 0 = no fix, 3 = fix
  bool     fixOk;
  uint32_t epoch;    // Unix seconds
  bool     timeValid;
  uint32_t updatedMs;
} fix = {0};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void gpsISR() {
  g_irqFired = true;
}

// UTC date/time -> Unix epoch seconds (proleptic Gregorian, days-from-civil).
static uint32_t toUnix(uint16_t y, uint8_t mo, uint8_t d,
                       uint8_t hh, uint8_t mm, uint8_t ss) {
  y -= (mo <= 2);
  int32_t  era = (int32_t)(y) / 400;
  uint32_t yoe = (uint32_t)(y - era * 400);
  uint32_t doy = (153u * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int32_t  days = era * 146097 + (int32_t)doe - 719468;
  return (uint32_t)days * 86400UL + hh * 3600UL + mm * 60UL + ss;
}

// Sniff the RX line for valid NMEA ("$G...") at the given baud.
static bool detectNMEA(uint32_t ms) {
  uint32_t t0 = millis();
  char prev = 0;
  uint8_t hits = 0;
  while (millis() - t0 < ms) {
    if (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (prev == '$' && c == 'G' && ++hits >= 2) return true;
      prev = c;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool GPSInit() {
  pinMode(GPS_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPS_INT), gpsISR, RISING);

  static const uint32_t bauds[] = { GPS_BAUD, 9600, 115200, 57600 };
  const uint8_t nb = sizeof(bauds) / sizeof(bauds[0]);

  for (uint8_t i = 0; i < nb; i++) {
    gpsSerial.begin(bauds[i]);
    if (detectNMEA(800)) {
      printf("[gps] NMEA detected at %lu baud\n", (unsigned long)bauds[i]);
      g_initOk = true;
      g_lastRxMs = millis();
      return true;
    }
  }

  printf("[gps] no NMEA on RX -> check module TX->PA10, power, or baud\n");
  g_initOk = false;
  return false;
}

bool gpsUpdate() {
  if (!g_initOk) {
    return false;
  }

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    g_lastRxMs = millis();
  }

  if (!(gps.location.isUpdated() || gps.satellites.isUpdated() ||
        gps.altitude.isUpdated())) {
    return false;
  }

  if (gps.location.isValid()) {
    fix.lat     = gps.location.lat();
    fix.lon     = gps.location.lng();
    fix.latRaw  = (int32_t)(fix.lat * 1e7);
    fix.lonRaw  = (int32_t)(fix.lon * 1e7);
    fix.fixType = 3;
    fix.fixOk   = true;
  } else {
    fix.fixType = 0;
    fix.fixOk   = false;
  }

  fix.altMSL = gps.altitude.isValid()   ? gps.altitude.meters() : 0.0;
  fix.speed  = gps.speed.isValid()      ? gps.speed.mps()       : 0.0;
  fix.course = gps.course.isValid()     ? gps.course.deg()      : 0.0;
  fix.hdop   = gps.hdop.isValid()       ? gps.hdop.hdop()       : 0.0;
  fix.sats   = gps.satellites.isValid() ? gps.satellites.value(): 0;

  if (gps.date.isValid() && gps.time.isValid()) {
    fix.timeValid = true;
    fix.epoch = toUnix(gps.date.year(), gps.date.month(), gps.date.day(),
                       gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    fix.timeValid = false;
  }

  fix.updatedMs = millis();
  g_newFix   = true;
  g_irqFired = false;
  return true;
}

bool gpsHasFix()    { return fix.fixOk && fix.fixType >= 3; }
bool gpsHasNewFix() { bool n = g_newFix; g_newFix = false; return n; }
bool gpsIsAlive()   { return g_initOk && (millis() - g_lastRxMs < 2000); }
bool gpsDataReady() { return g_irqFired; }

double  gpsLatitude()  { return fix.lat; }
double  gpsLongitude() { return fix.lon; }
double  gpsAltitude()  { return fix.altMSL; }
double  gpsSpeed()     { return fix.speed; }
double  gpsCourse()    { return fix.course; }
double  gpsHDOP()      { return fix.hdop; }
uint8_t gpsSats()      { return fix.sats; }
uint8_t gpsFixType()   { return fix.fixType; }

bool     gpsTimeValid() { return fix.timeValid; }
uint32_t gpsTimestamp() { return fix.epoch; }

void gpsToPacket() {
  radioSetGPS(fix.latRaw, fix.lonRaw, (float)fix.altMSL, fix.sats, fix.fixType);
}

void gpsReport() {
  printf("[gps] fix=%u sats=%u lat=%.7f lon=%.7f alt=%.1fm "
         "spd=%.1fm/s crs=%.1f hdop=%.2f epoch=%lu%s\n",
         fix.fixType, fix.sats, fix.lat, fix.lon, fix.altMSL,
         fix.speed, fix.course, fix.hdop, (unsigned long)fix.epoch,
         fix.timeValid ? "" : " (time invalid)");
}
