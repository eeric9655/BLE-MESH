#include "BH1721.h"
TwoWire wire2=TwoWire(1);
//Driver code for the BH1721
class BH1721 {
public:
  enum measMode {
    measModeAR,
    measModeHR,
    measModeLR,
  };

  bool begin(int sdaPin,int sclPin,uint32_t freq) {
    wire2.begin(sdaPin,sclPin,freq); //SDA  SCL 
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
    bh1721.begin(2,3,100000);
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