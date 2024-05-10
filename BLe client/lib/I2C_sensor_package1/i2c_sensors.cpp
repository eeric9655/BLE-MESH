#include "i2c_sensors.h"

////////------------    BH1721 light sensor     ------------/////////

TwoWire wire2=TwoWire(1);
//Driver code for the BH1721
void Wire_Setup(int sdaPin,int sclPin,uint32_t freq){
    wire2.begin(sdaPin,sclPin,freq);
}
class BH1721 {
public:
  enum measMode {
    measModeAR,
    measModeHR,
    measModeLR,
  };

  bool begin() {
    //wire2.begin(sdaPin,sclPin,freq); //SDA  SCL 
    //wire2.setPins(2,3);
    return true;
  }

  bool setMeasTime(int mt) {
    if ((mt < MT_MIN) || (mt > MT_MAX)) {
      return false;
    }
    return (sendCmd(CMD_MT_HIGH | (mt >> 5)) && sendCmd(CMD_MT_LOW | (mt & 0x10)));
  }

  bool startMeasure(enum measMode mode) {
    byte cmd;

    switch (mode) {
    case measModeAR:
      cmd = CMD_AR_MODE;
      break;
    case measModeHR:
      cmd = CMD_HR_MODE;
      break;
    case measModeLR:
      cmd = CMD_LR_MODE;
      break;
    default:
      return false;
    }
    return sendCmd(cmd);
  }

  bool readLux(float *lux) {
    if (wire2.requestFrom(I2C_ADDR, 2) != 2) {
      return false;
    }
    byte msb = wire2.read();
    byte lsb = wire2.read();
    *lux = (msb * 0x100U | lsb) / 1.2;
    return true;
  }

private:
  bool sendCmd(byte cmd) {
    wire2.beginTransmission(I2C_ADDR);
    int size = wire2.write(cmd);
    if ((wire2.endTransmission() != 0) || (size != 1)) {
      return false;
    }
    return true;
  }

  static const byte CMD_POWER_DOWN = 0x00;
  static const byte CMD_POWER_ON = 0x01;
  static const byte CMD_AR_MODE = 0x10;
  static const byte CMD_HR_MODE = 0x12;
  static const byte CMD_LR_MODE = 0x13;
  static const byte CMD_MT_HIGH = 0x40;
  static const byte CMD_MT_LOW = 0x60;

  static const int MT_MIN = 140;
  static const int MT_MAX = 1020;

  static const int I2C_ADDR = 0x23;
};

BH1721 bh1721;
void setup_BH1751(){
    //wire2 = wire3;
    //bh1721.begin(sdaPin,sclPin,freq);
    bh1721.begin();
    if (!bh1721.startMeasure(BH1721::measModeAR)) {
        Serial.println("Cannot start measurements");
    }
}
float readLux(){
    float lux=-1;
    if (bh1721.readLux(&lux)) {
        Serial.println("Luminosity: " + String(lux) + " lux");
    }
    else {
        Serial.println("Cannot measure luminosity!");
    }
    return lux;
}



////////------------    BH1721 light sensor     ------------/////////
////////------------    HDC Temp/Hum sensor     ------------/////////


ClosedCube_HDC1080::ClosedCube_HDC1080()
{
}

void ClosedCube_HDC1080::begin(uint8_t address) {
    _address = address;
	//wire5.setPins(2,3);
   // wire2.begin(2,3,100000);
	setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
}

void ClosedCube_HDC1080::setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) {
	HDC1080_Registers reg;
	reg.HumidityMeasurementResolution = 0;
	reg.TemperatureMeasurementResolution = 0;

	if (temperature == HDC1080_RESOLUTION_11BIT)
		reg.TemperatureMeasurementResolution = 0x01;

	switch (humidity)
	{
		case HDC1080_RESOLUTION_8BIT:
			reg.HumidityMeasurementResolution = 0x02;
			break;
		case HDC1080_RESOLUTION_11BIT:
			reg.HumidityMeasurementResolution = 0x01;
			break;
		default:
			break;
	}

	writeRegister(reg);
}

HDC1080_SerialNumber ClosedCube_HDC1080::readSerialNumber() {
	HDC1080_SerialNumber sernum;
	sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
	sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
	sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
	return sernum;
}

HDC1080_Registers ClosedCube_HDC1080::readRegister() {
	HDC1080_Registers reg;
	reg.rawData = (readData(HDC1080_CONFIGURATION) >> 8);
	return reg;
}

void ClosedCube_HDC1080::writeRegister(HDC1080_Registers reg) {
	wire2.beginTransmission(_address);
	wire2.write(HDC1080_CONFIGURATION);
	wire2.write(reg.rawData);
	wire2.write(0x00);
	wire2.endTransmission();
	delay(10);
}

void ClosedCube_HDC1080::heatUp(uint8_t seconds) {
	HDC1080_Registers reg = readRegister();
	reg.Heater = 1;
	reg.ModeOfAcquisition = 1;
	writeRegister(reg);

	uint8_t buf[4];
	for (int i = 1; i < (seconds*66); i++) {
		wire2.beginTransmission(_address);
		wire2.write(0x00);
		wire2.endTransmission();
		delay(20);
		wire2.requestFrom(_address, (uint8_t)4);
		wire2.readBytes(buf, (size_t)4);
	}
	reg.Heater = 0;
	reg.ModeOfAcquisition = 0;
	writeRegister(reg);
}


double ClosedCube_HDC1080::readT() {
	return readTemperature();
}

double ClosedCube_HDC1080::readTemperature() {
	uint16_t rawT = readData(HDC1080_TEMPERATURE);
	return (rawT / pow(2, 16)) * 165.0 - 40.0;
}

double ClosedCube_HDC1080::readH() {
	return readHumidity();
}

double ClosedCube_HDC1080::readHumidity() {
	uint16_t rawH = readData(HDC1080_HUMIDITY);
	return (rawH / pow(2, 16)) * 100.0;
}

uint16_t ClosedCube_HDC1080::readManufacturerId() {
	return readData(HDC1080_MANUFACTURER_ID);
}

uint16_t ClosedCube_HDC1080::readDeviceId() {
	return readData(HDC1080_DEVICE_ID);
}

uint16_t ClosedCube_HDC1080::readData(uint8_t pointer) {
	wire2.beginTransmission(_address);
	wire2.write(pointer);
	wire2.endTransmission();
	
	delay(9);
	wire2.requestFrom(_address, (uint8_t)2);

	byte msb = wire2.read();
	byte lsb = wire2.read();

	return msb << 8 | lsb;
}
ClosedCube_HDC1080 hdc1080;
void setup_HDC1080(){
    //wire5 = wire6;
    hdc1080.begin(0x40);
}
float read_Temperature(){
   return hdc1080.readTemperature();
}
float read_Humiduty(){
   return hdc1080.readHumidity();
}
void print_HDC1080_Info(){
    Serial.print("Manufacturer ID=0x");
	Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	Serial.print("Device ID=0x");
	Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
    Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}


////////------------    HDC Temp/Hum sensor     ------------------------------------------------------------------------/////////



////////----------------------------------    BMP280 Temp/Pressure Barometer sensor     ---------------------------------------------/////////
Adafruit_BMP280 bmp(&wire2);
void setup_BMP280(){

    unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
float get_tempBMP280(){
    return bmp.readTemperature();
}
float get_pressureBMP280(){
    return bmp.readPressure();
}
float get_altitudeBMP280(){
  float data_alti = bmp.readAltitude();
  //Serial.println("-------------------- ALTITUDE : ", String(data_alti));
    return data_alti;
}




////////----------------------------------    BMP280 Temp/Pressure Barometer sensor     ---------------------------------------------/////////