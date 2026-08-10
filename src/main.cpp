#include <Arduino.h>
#include <stdio.h>
#include "sx1262.h"
#include "m10s.h"
#include "sdCard.h"
#include "baro.h"
#include "LSM6DSO32.h"

/* ============================================================================
 * OrionTracker L2 flight computer - main.
 * Parses GPS (RX-only NMEA), packages it, and transmits over the SX1262 (LoRa),
 * logging each packet to SD. Buzzer (PA8) + status LED (PB14) indicate whether
 * the GPS is still searching or has a lock.
 * ========================================================================== */

// Debug output over the native USB CDC serial port (printf -> Serial).
extern "C" int _write(int file, char *ptr, int len) {
  (void)file;
  Serial.write((const uint8_t*)ptr, len);
  return len;
}

// Blocking beep/flash used for boot and lock/loss events (brief, infrequent).
static void chirp(uint8_t count, uint16_t onMs, uint16_t offMs, bool led) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(PA8, HIGH);
    if (led) digitalWrite(PB14, HIGH);
    delay(onMs);
    digitalWrite(PA8, LOW);
    if (led) digitalWrite(PB14, LOW);
    if (i + 1 < count) delay(offMs);
  }
}


void setup() {
  // USB CDC serial for debug output.
  Serial.begin(115200);
  setvbuf(stdout, NULL, _IONBF, 0);
  while (!Serial && millis() < 3000) { }
  printf("Orion boot\n");

  // Buzzer (PA8) and status LED (PB14).
  pinMode(PA8, OUTPUT);  digitalWrite(PA8, LOW);
  pinMode(PB14, OUTPUT); digitalWrite(PB14, LOW);
  chirp(2, 80, 120, true);   // power-on indication

  // Subsystems.
  if (!radioInit()) printf("[radio] init FAILED - check SPI2 wiring\n");
  else              printf("[radio] init OK\n");

  if (!GPSInit())   printf("[gps] init FAILED - check module TX->PA10 / power\n");
  else              printf("[gps] init OK\n");

  if (!imuInit())   printf("[imu] init FAILED - check LSM6DSO32 SPI1 wiring / CS\n");
  else              printf("[imu] init OK\n");

  if (!baroInit()) {
    printf("[baro] init FAILED - check LPS22HB SPI1 wiring / CS (PC5)\n");
  } else {
    // Let the sensor settle, then zero to pad altitude so baro reads AGL.
    for (int i = 0; i < 10; i++) { baroUpdate(); delay(20); }
    baroZero();
    printf("[baro] init OK, zeroed at P0=%.1f hPa\n", baroPressure());
  }

  if (!sdInit())    printf("[sd] init FAILED - no card / SDIO wiring (logging off)\n");
  else              printf("[sd] init OK\n");
}

void loop() {
  // 1) GPS FIRST - drain the UART every iteration so no NMEA is missed. On each
  //    fresh solution, package -> transmit -> log. (GPS is the priority.)
  if (gpsUpdate()) {
    gpsToPacket();                                    // stage GPS into the packet
    radioFormat();                                    // stamp seq/time/gpsAge
    if (!radioBusy()) {
      radioSend();                                    // non-blocking LoRa transmit
    }
    sdWrite(&radioPacket(), sizeof(TelemetryPacket));  // binary log record
    gpsReport();                                       // debug line over USB
  }

  // 2) Barometer + IMU: polled on SPI1 (no interrupt pins), after GPS so they
  //    never delay NMEA parsing. Quick SPI reads can't starve the buffered UART.
  static uint32_t baroT = 0;
  if (millis() - baroT >= 40) {          // ~25 Hz
    baroT = millis();
    if (baroUpdate()) baroToPacket();
  }
  static uint32_t imuT = 0;
  if (millis() - imuT >= 10) {           // ~100 Hz
    imuT = millis();
    if (imuUpdate()) imuToPacket();
  }

  // 3) Beep + LED on lock/loss transitions.
  bool fix = gpsHasFix();
  static bool     prevFix   = false;
  static uint32_t lockBeepT = 0;     // next periodic lock-beep reference
  if (fix && !prevFix) {
    printf("[gps] LOCK acquired (sats=%u)\n", gpsSats());
    // Distinctive lock alert: one long beep, then a rapid triple.
    chirp(1, 500, 0, true);          // "daaaa"
    delay(150);
    chirp(3, 70, 70, true);          // "dot dot dot"
    lockBeepT = millis();            // start the 5 s cadence after this alert
  } else if (!fix && prevFix) {
    printf("[gps] lock lost, searching\n");
    chirp(1, 250, 0, true);          // loss: one long beep
  }
  prevFix = fix;

  // 4) Ongoing status indicator.
  if (fix) {
    digitalWrite(PB14, HIGH);        // LOCKED: LED solid on
    // While the lock holds, a short beep every 5 s.
    if (millis() - lockBeepT >= 5000) {
      lockBeepT = millis();
      digitalWrite(PA8, HIGH); delay(40); digitalWrite(PA8, LOW);
    }
  } else {
    // SEARCHING: brief LED flash (100 ms on / 900 ms off) + a short beep ~3 s.
    static uint32_t ledT = 0;
    static bool     ledOn = false;
    uint32_t now = millis();
    uint32_t interval = ledOn ? 100 : 900;
    if (now - ledT >= interval) {
      ledT = now;
      ledOn = !ledOn;
      digitalWrite(PB14, ledOn ? HIGH : LOW);
    }
    static uint32_t beepT = 0;
    if (now - beepT >= 3000) {
      beepT = now;
      digitalWrite(PA8, HIGH); delay(40); digitalWrite(PA8, LOW);
    }
  }

  // 5) Commit the SD log to the card once a second.
  static uint32_t flushT = 0;
  if (millis() - flushT >= 1000) {
    flushT = millis();
    sdFlush();
  }
}
