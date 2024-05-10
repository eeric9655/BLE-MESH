#include "Arduino.h"
#include "Wire.h"


#define MPU9250_IMU_ADDRESS 0x68
#define MPU9250_MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18

#define TEMPERATURE_OFFSET 21 // As defined in documentation

#define INTERVAL_MS_PRINT 1000

#define G_CONSTANT 9.80665
struct {
 struct {
    float x, y, z;
  } accelerometer, gyroscope, magnetometer;

  float temperature;
} normalized;

void MPU9250Setup(int sda,int scl,uint32_t frequ );
void loopMPU9250(void);
float get_gyrox();
float get_gyroy();
float get_gyroz();
float get_accelx();
float get_accely();
float get_accelz();
float get_magx();
float get_magy();
float get_magz();
float get_Temp();