#include "baro.h"
#include "sx1262.h"              // radioSetBaro() for baroToPacket()
#include "spibus.h"              // shared SPI1 instance
#include <LPS22HBSensor.h>
#include <stdio.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * BUS / WIRING - OrionTracker PCB: LPS22HB on SPI1.
 *   SCK = PA5, MISO = PA6, MOSI = PA7, CS = PC5.  No interrupt pin -> polled.
 * ------------------------------------------------------------------------- */
#ifndef BARO_SCK
#define BARO_SCK   PA5
#endif
#ifndef BARO_MISO
#define BARO_MISO  PA6
#endif
#ifndef BARO_MOSI
#define BARO_MOSI  PA7
#endif
#ifndef BARO_CS
#define BARO_CS    PC5
#endif
#ifndef BARO_ODR_HZ
#define BARO_ODR_HZ  25.0f
#endif

static LPS22HBSensor baro(&sensorSPI1, BARO_CS);   // shared SPI1, CS = BARO_CS

static bool  initialised   = false;
static float g_pressure    = 0.0f;      // hPa
static float g_temperature = 0.0f;      // deg C
static float g_altitude    = 0.0f;      // m
static float g_seaLevelhPa = 1013.25f;  // MSL reference pressure
static float g_refPressure = 0.0f;      // AGL ground reference (0 = use sea level)

// International barometric formula (standard atmosphere) - altitude in metres.
static float pressureToAltitude(float p_hPa, float ref_hPa) {
  return 44330.0f * (1.0f - powf(p_hPa / ref_hPa, 0.1902949f));
}

bool baroInit() {
  sensorSPI1.begin();
  if (baro.begin() != LPS22HB_STATUS_OK) { initialised = false; return false; }

  // Real comms check: begin() only writes registers (can't fail on SPI), so
  // verify the WHO_AM_I to confirm the sensor is actually talking.
  uint8_t id = 0;
  baro.ReadID(&id);
  printf("[baro] WHO_AM_I=0x%02X (expect 0xB1)\n", id);
  if (id != 0xB1) { initialised = false; return false; }

  if (baro.Enable() != LPS22HB_STATUS_OK) { initialised = false; return false; }
  baro.SetODR(BARO_ODR_HZ);
  initialised = true;
  return true;
}

bool baroUpdate() {
  if (!initialised) {
    return false;
  }
  float p, t;
  if (baro.GetPressure(&p)    != LPS22HB_STATUS_OK) return false;
  if (baro.GetTemperature(&t) != LPS22HB_STATUS_OK) return false;

  g_pressure    = p;
  g_temperature = t;
  float ref = (g_refPressure > 0.0f) ? g_refPressure : g_seaLevelhPa;
  g_altitude = pressureToAltitude(p, ref);
  return true;
}

float baroPressure()    { return g_pressure; }
float baroTemperature() { return g_temperature; }
float baroAltitude()    { return g_altitude; }

void baroZero() {
  // Use the latest pressure as the reference so altitude reads ~0 here (AGL).
  if (g_pressure > 0.0f) {
    g_refPressure = g_pressure;
  }
}

void baroSetSeaLevel(float hPa) {
  g_seaLevelhPa = hPa;
  g_refPressure = 0.0f;   // back to absolute MSL altitude
}

void baroToPacket() {
  radioSetBaro(g_altitude, g_pressure, g_temperature);
}
