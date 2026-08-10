#ifndef ORION_SPIBUS_H // include guard
#define ORION_SPIBUS_H

#include <SPI.h>

/* Shared SPI1 instance for the on-board sensors (LPS22HB baro + LSM6DSO32 IMU).
 * One SPIClass for the peripheral avoids the flaky two-instances-per-bus setup.
 *   SCK = PA5, MISO = PA6, MOSI = PA7.  Each sensor has its own CS. */
extern SPIClass sensorSPI1;

#endif // ORION_SPIBUS_H
