#include "LSM6DSO32.h"
#include "sx1262.h"                 // radioSetIMU() for imuToPacket()
#include "spibus.h"                 // shared SPI1 instance
#include <LSM6DSO32Sensor.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * BUS / WIRING - OrionTracker PCB: LSM6DSO32 on SPI1 (shared with the baro).
 *   SCK = PA5, MISO = PA6, MOSI = PA7.  CS on its own pin (SET BELOW).
 * ------------------------------------------------------------------------- */
#ifndef IMU_SCK
#define IMU_SCK   PA5
#endif
#ifndef IMU_MISO
#define IMU_MISO  PA6
#endif
#ifndef IMU_MOSI
#define IMU_MOSI  PA7
#endif
#ifndef IMU_CS
#define IMU_CS    PC4    // <-- SET TO YOUR IMU CHIP-SELECT PIN
#endif
#ifndef IMU_ODR_HZ
#define IMU_ODR_HZ  104.0f
#endif
#ifndef IMU_ACC_FS_G
#define IMU_ACC_FS_G  32     // +/-g full scale (LSM6DSO32: 4/8/16/32); high for rockets
#endif
#ifndef IMU_GYR_FS_DPS
#define IMU_GYR_FS_DPS 2000  // +/-dps full scale
#endif

static LSM6DSO32Sensor imu(&sensorSPI1, IMU_CS);   // shared SPI1, CS = IMU_CS

static bool  initialised = false;
static float g_ax = 0, g_ay = 0, g_az = 0;   // g
static float g_gx = 0, g_gy = 0, g_gz = 0;   // deg/s

bool imuInit() {
  sensorSPI1.begin();
  if (imu.begin() != LSM6DSO32_OK) { initialised = false; return false; }

  // Real comms check: verify the WHO_AM_I (begin() only writes registers).
  uint8_t id = 0;
  imu.ReadID(&id);
  printf("[imu] WHO_AM_I=0x%02X (expect 0x6C)\n", id);
  if (id != 0x6C) { initialised = false; return false; }

  if (imu.Enable_X()   != LSM6DSO32_OK) { initialised = false; return false; }
  if (imu.Enable_G()   != LSM6DSO32_OK) { initialised = false; return false; }
  imu.Set_X_FS(IMU_ACC_FS_G);
  imu.Set_G_FS(IMU_GYR_FS_DPS);
  imu.Set_X_ODR(IMU_ODR_HZ);
  imu.Set_G_ODR(IMU_ODR_HZ);
  initialised = true;
  return true;
}

bool imuUpdate() {
  if (!initialised) {
    return false;
  }
  int32_t a[3], g[3];
  if (imu.Get_X_Axes(a) != LSM6DSO32_OK) return false;   // mg
  if (imu.Get_G_Axes(g) != LSM6DSO32_OK) return false;   // mdps

  g_ax = a[0] / 1000.0f;   // mg   -> g
  g_ay = a[1] / 1000.0f;
  g_az = a[2] / 1000.0f;
  g_gx = g[0] / 1000.0f;   // mdps -> deg/s
  g_gy = g[1] / 1000.0f;
  g_gz = g[2] / 1000.0f;
  return true;
}

float imuAccX()  { return g_ax; }
float imuAccY()  { return g_ay; }
float imuAccZ()  { return g_az; }
float imuGyroX() { return g_gx; }
float imuGyroY() { return g_gy; }
float imuGyroZ() { return g_gz; }

void imuToPacket() {
  radioSetIMU(g_ax, g_ay, g_az, g_gx, g_gy, g_gz);
}
