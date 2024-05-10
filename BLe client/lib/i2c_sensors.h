#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
void Wire_Setup(int sdaPin,int sclPin,uint32_t freq);

////////------------    BH1721 light sensor     ------------/////////
float readLux();
void setup_BH1751();

////////------------    BH1721 light sensor     ------------/////////

////////------------    HDC Temp/Hum sensor     ------------/////////

float read_Humiduty();
float read_Temperature();
void setup_HDC1080();


typedef enum {
	HDC1080_RESOLUTION_8BIT,
	HDC1080_RESOLUTION_11BIT,
	HDC1080_RESOLUTION_14BIT,
} HDC1080_MeasurementResolution;

typedef enum {
	HDC1080_TEMPERATURE		= 0x00,
	HDC1080_HUMIDITY		= 0x01,
	HDC1080_CONFIGURATION	= 0x02,
	HDC1080_MANUFACTURER_ID = 0xFE,
	HDC1080_DEVICE_ID		= 0xFF,
	HDC1080_SERIAL_ID_FIRST	= 0xFB,
	HDC1080_SERIAL_ID_MID	= 0xFC,
	HDC1080_SERIAL_ID_LAST	= 0xFD,
} HDC1080_Pointers;

typedef union {
	uint8_t rawData[6];
	struct {
		uint16_t serialFirst;
		uint16_t serialMid;
		uint16_t serialLast;
	};
} HDC1080_SerialNumber;

typedef union {
	uint8_t rawData;
	struct {
		uint8_t HumidityMeasurementResolution : 2;
		uint8_t TemperatureMeasurementResolution : 1;
		uint8_t BatteryStatus : 1;
		uint8_t ModeOfAcquisition : 1;
		uint8_t Heater : 1;
		uint8_t ReservedAgain : 1;
		uint8_t SoftwareReset : 1;
	};
} HDC1080_Registers;

class ClosedCube_HDC1080 {
public:
	ClosedCube_HDC1080();

	void begin(uint8_t address);
	uint16_t readManufacturerId(); // 0x5449 ID of Texas Instruments
	uint16_t readDeviceId(); // 0x1050 ID of the device

	HDC1080_Registers readRegister();	
	void writeRegister(HDC1080_Registers reg);

	HDC1080_SerialNumber readSerialNumber();

	void heatUp(uint8_t seconds);
	
	void setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature);

	double readTemperature();
	double readHumidity();

	double readT(); // short-cut for readTemperature
	double readH(); // short-cut for readHumidity

private:
	uint8_t _address;
	uint16_t readData(uint8_t pointer);
	
};

////////------------    HDC Temp/Hum sensor     ------------/////////


////////------------    BMP280 Temp/Pressure Barometer sensor     ------------/////////
void setup_BMP280();

float get_tempBMP280();
float get_pressureBMP280();
float get_altitudeBMP280();


////////------------    BMP280 Temp/Pressure Barometer sensor     ------------/////////