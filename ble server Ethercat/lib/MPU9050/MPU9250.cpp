#include "MPU9250.h"

struct gyroscope_raw {
  int16_t x, y, z;
} gyroscope;

struct accelerometer_raw {
  int16_t x, y, z;
} accelerometer;

struct magnetometer_raw {
  int16_t x, y, z;

  struct {
    int8_t x, y, z;
  } adjustment;
} magnetometer;

struct temperature_raw {
  int16_t value;
} temperature;



TwoWire wire1= TwoWire(0);

void I2Cread(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* data)
{
  wire1.beginTransmission(address); // Set register address
  wire1.write(reg);
  wire1.endTransmission();

  wire1.requestFrom(address, bytes); //Read bytes amount
  
  uint8_t index = 0;
  while (wire1.available()) {
    data[index++] = wire1.read();
  }
}


void I2CwriteByte(uint8_t address, uint8_t reg, uint8_t data)
{
  wire1.beginTransmission(address); //Set register address
  wire1.write(reg);
  wire1.write(data);
  wire1.endTransmission();
}



bool isImuReady()
{
  uint8_t isReady; // Interruption flag

  I2Cread(MPU9250_IMU_ADDRESS, 58, 1, &isReady);

  return isReady & 0x01; // Read register and wait for the RAW_DATA_RDY_INT
}

void readRawImu()
{
  uint8_t buff[14];

  // Read output registers:
  // [59-64] Accelerometer
  // [65-66] Temperature
  // [67-72] Gyroscope
  I2Cread(MPU9250_IMU_ADDRESS, 59, 14, buff);

  // Accelerometer, create 16 bits values from 8 bits data
  accelerometer.x = (buff[0] << 8 | buff[1]);
  accelerometer.y = (buff[2] << 8 | buff[3]);
  accelerometer.z = (buff[4] << 8 | buff[5]);

  // Temperature, create 16 bits values from 8 bits data
  temperature.value = (buff[6] << 8 | buff[7]);

  // Gyroscope, create 16 bits values from 8 bits data
  gyroscope.x = (buff[8] << 8 | buff[9]);
  gyroscope.y = (buff[10] << 8 | buff[11]);
  gyroscope.z = (buff[12] << 8 | buff[13]);
}
// -------------------------------------------------
// Copyright (c) 2022 HiBit <https://www.hibit.dev>
// -------------------------------------------------

void setMagnetometerAdjustmentValues()
{
  uint8_t buff[3];

  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x1F); // Set 16-bits output & fuse ROM access mode

  delay(1000);

  I2Cread(MPU9250_MAG_ADDRESS, 0x10, 3, buff); // Read adjustments

  magnetometer.adjustment.x = buff[0]; //Adjustment for X axis
  magnetometer.adjustment.y = buff[1]; //Adjustment for Y axis
  magnetometer.adjustment.z = buff[2]; //Adjustment for Z axis

  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x10); // Power down
}

bool isMagnetometerReady()
{
  uint8_t isReady; // Interruption flag

  I2Cread(MPU9250_MAG_ADDRESS, 0x02, 1, &isReady);

  return isReady & 0x01; // Read register and wait for the DRDY
}

void readRawMagnetometer()
{
  uint8_t buff[7];

  // Read output registers:
  // [0x03-0x04] X-axis measurement
  // [0x05-0x06] Y-axis measurement
  // [0x07-0x08] Z-axis measurement
  I2Cread(MPU9250_MAG_ADDRESS, 0x03, 7, buff);

  // Magnetometer, create 16 bits values from 8 bits data
  magnetometer.x = (buff[1] << 8 | buff[0]);
  magnetometer.y = (buff[3] << 8 | buff[2]);
  magnetometer.z = (buff[5] << 8 | buff[4]);
}

// -------------------------------------------------
// Copyright (c) 2022 HiBit <https://www.hibit.dev>
// -------------------------------------------------

void normalize(gyroscope_raw gyroscope)
{
  // Sensitivity Scale Factor (MPU datasheet page 8)
  normalized.gyroscope.x = gyroscope.x / 32.8;
  normalized.gyroscope.y = gyroscope.y / 32.8;
  normalized.gyroscope.z = gyroscope.z / 32.8;
}

void normalize(accelerometer_raw accelerometer)
{
  // Sensitivity Scale Factor (MPU datasheet page 9)
  normalized.accelerometer.x = accelerometer.x * G_CONSTANT / 16384;
  normalized.accelerometer.y = accelerometer.y * G_CONSTANT / 16384;
  normalized.accelerometer.z = accelerometer.z * G_CONSTANT / 16384;
}

void normalize(temperature_raw temperature)
{
  // Sensitivity Scale Factor (MPU datasheet page 11) & formula (MPU registers page 33)
  normalized.temperature = ((temperature.value - TEMPERATURE_OFFSET) / 333.87) + TEMPERATURE_OFFSET;
}

void normalize(magnetometer_raw magnetometer)
{
  // Sensitivity Scale Factor (MPU datasheet page 10)
  // 0.6 µT/LSB (14-bit)
  // 0.15µT/LSB (16-bit)
  // Adjustment values (MPU register page 53)
  normalized.magnetometer.x = magnetometer.x * 0.15 * (((magnetometer.adjustment.x - 128) / 256) + 1);
  normalized.magnetometer.y = magnetometer.y * 0.15 * (((magnetometer.adjustment.y - 128) / 256) + 1);
  normalized.magnetometer.z = magnetometer.z * 0.15 * (((magnetometer.adjustment.z - 128) / 256) + 1);
}
////////////////////------------------        Sensor section       -----------------------------------------------

void MPU9250Setup(int sda,int scl,uint32_t frequ ){
    //wire1 = wire_;
    wire1.begin(sda,scl,frequ);
    //wire1.setPins(4,5);
    delay(2000);
    I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS); // Configure gyroscope range
    I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);        // Configure accelerometer range

    I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02); // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01); // Enable interrupt pin for raw data

    setMagnetometerAdjustmentValues();

    //Start magnetometer
    I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x12); // Request continuous magnetometer measurements in 16 bits (mode 1)
}
void loopMPU9250(void){
      if (isImuReady()) {
            readRawImu();   
            normalize(gyroscope);
            normalize(accelerometer);
            normalize(temperature);
        }

        if (isMagnetometerReady()) {
            readRawMagnetometer();
            normalize(magnetometer);
        }
  
  ///<---  Temperature 0:  gyro_x 1: gyro_y 2: gyro_z 3:  accel_x 4: accel_y 5: accel_z 6:  magnet_x 7: magnet_y 8: magnet_z 9:  --->\
    °C, GYR (°/s):, ACC (m/s^2):,  MAG (μT): 
    float data_MPU9250[10] = {normalized.temperature,  
        normalized.gyroscope.x,  normalized.gyroscope.y,  normalized.gyroscope.z,
        normalized.accelerometer.x,  normalized.accelerometer.y, normalized.accelerometer.z,
        normalized.magnetometer.x,  normalized.magnetometer.y,  normalized.magnetometer.z};
   if(abs(normalized.accelerometer.x)> 22000 || abs(normalized.accelerometer.y)> 22000|| abs(normalized.accelerometer.z) > 27000){
    Serial.println("----------------------Fall detected-----------------------------");
    }
    //for(int pp = 0;pp<10;pp++)Serial.println("ID: "+String(pp)+" Data: "+String(data_MPU9250[pp]));
}
float get_gyrox(){
  return normalized.gyroscope.x;
}
float get_gyroy(){
  return normalized.gyroscope.y;
}
float get_gyroz(){
  return normalized.gyroscope.z;
}
float get_accelx(){
  return normalized.accelerometer.x;
}
float get_accely(){
  return normalized.accelerometer.y;
}
float get_accelz(){
  return normalized.accelerometer.z;
}
float get_magx(){
  return normalized.magnetometer.x;
}
float get_magy(){
  return normalized.magnetometer.y;
}
float get_magz(){
  return normalized.magnetometer.z;
}
float get_Temp(){
  return normalized.temperature;
}