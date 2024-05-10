#include "HDC1080.h"
TwoWire wire5=TwoWire(1);

ClosedCube_HDC1080::ClosedCube_HDC1080()
{
}

void ClosedCube_HDC1080::begin(uint8_t address) {
    _address = address;
	//wire5.setPins(2,3);
    //wire5.begin();//2,3,100000);
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
	wire5.beginTransmission(_address);
	wire5.write(HDC1080_CONFIGURATION);
	wire5.write(reg.rawData);
	wire5.write(0x00);
	wire5.endTransmission();
	delay(10);
}

void ClosedCube_HDC1080::heatUp(uint8_t seconds) {
	HDC1080_Registers reg = readRegister();
	reg.Heater = 1;
	reg.ModeOfAcquisition = 1;
	writeRegister(reg);

	uint8_t buf[4];
	for (int i = 1; i < (seconds*66); i++) {
		wire5.beginTransmission(_address);
		wire5.write(0x00);
		wire5.endTransmission();
		delay(20);
		wire5.requestFrom(_address, (uint8_t)4);
		wire5.readBytes(buf, (size_t)4);
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
	wire5.beginTransmission(_address);
	wire5.write(pointer);
	wire5.endTransmission();
	
	delay(9);
	wire5.requestFrom(_address, (uint8_t)2);

	byte msb = wire5.read();
	byte lsb = wire5.read();

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