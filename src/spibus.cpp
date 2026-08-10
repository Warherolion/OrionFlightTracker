#include "spibus.h"

// SPI1: MOSI = PA7, MISO = PA6, SCK = PA5. Shared by the baro and IMU drivers.
SPIClass sensorSPI1(PA7, PA6, PA5);
