#include <Arduino.h>
#include <NimBLEDevice.h>
#include "sdkconfig.h"
#include "Wire.h"
#include "MPU9250.h"
#include "OLED.h"
#include "i2c_sensors.h"
#include "Modes_Pins.h"
#define DEFAULT_VAL "default"
unsigned int long old_time_sent = 0;
uint16_t period_Sensor = 2000;
TwoWire wire = TwoWire(0);
//////////////MPU9250
uint8_t address_sensor = 0x00;
static NimBLEServer* pServer;
#define DEVICES 9
#define CURRENT_DEVICE 10
const char* uuids_Service[DEVICES] = {"D001","D002","D003","D004","D005","D006","D007","D008","D009"};
const char* uuid_char[DEVICES] = {"DA01","DA02","DA03","DA04","DA05","DA06","DA07","DA08","DA09"};
const char* uuid_desc = "DEC1";
NimBLEService* pBLEService[DEVICES];
typedef struct data_package{
  uint8_t node;
  uint8_t id;
  int16_t data0;
  int16_t data1;
  int16_t data2;
  int16_t data3;
  int16_t data4;
  int16_t data5;
  int16_t data6;
  int16_t data7;
  int16_t data8;
} data_package;

data_package BLE_DATA;
data_package myData;
data_package sms[DEVICES]={BLE_DATA};

void I2C_scanner();
String M_sensor_data();
void get_BLE_Data();
data_package prepare_data(String data);