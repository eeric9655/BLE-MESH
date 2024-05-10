#include "include_files.h"

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected");
        Serial.println("Multi-connect support: start advertising");
        NimBLEDevice::startAdvertising();
    };
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.print("Client address: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    };
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };
    uint32_t onPassKeyRequest(){
        Serial.println("Server Passkey Request");
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");Serial.println(pass_key);
        return true;
    };

    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        if(!desc->sec_state.encrypted) {
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            Serial.println("Encrypt connection failed - disconnecting client");
            return;
        }
        Serial.println("Starting BLE work!");
    };
};
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onRead(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onWrite(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };
    void onNotify(NimBLECharacteristic* pCharacteristic) {
        Serial.println("Sending notification to clients");
    };
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", ";
        str += NimBLEUtils::returnCodeToString(code);
        Serial.println(str);
    };

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if(subValue == 0) {
            str += " Unsubscribed to ";
        }else if(subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if(subValue == 2) {
            str += " Subscribed to indications for ";
        } else if(subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();

        Serial.println(str);
    };
};
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor) {
        std::string dscVal = pDescriptor->getValue();
        Serial.print("Descriptor witten value:");
        Serial.println(dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor) {
        Serial.print(pDescriptor->getUUID().toString().c_str());
        Serial.println(" Descriptor read");
    };
};
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;


void setup() {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Server");
    NimBLEDevice::init("BLE MESH");
    NimBLEDevice::setPower(ESP_PWR_LVL_P21); /** +9db */
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    NimBLEService* pDeadService = pServer->createService("DEAD");
    NimBLECharacteristic* pBeefCharacteristic = pDeadService->createCharacteristic(
                                               "BEEF",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                                               NIMBLE_PROPERTY::WRITE_ENC   // only allow writing if paired / encrypted
                                              );

    pBeefCharacteristic->setValue(DEFAULT_VAL);
    pBeefCharacteristic->setCallbacks(&chrCallbacks);
    NimBLE2904* pBeef2904 = (NimBLE2904*)pBeefCharacteristic->createDescriptor("2904");
    pBeef2904->setFormat(NimBLE2904::FORMAT_UTF8);
    pBeef2904->setCallbacks(&dscCallbacks);
    
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    
    for(int i=0;i<DEVICES;i++){
        pBLEService[i] = pServer->createService(uuids_Service[i]);
        NimBLECharacteristic* pCharacteristic = pBLEService[i]->createCharacteristic(
                                               uuid_char[i],
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                              );

        pCharacteristic->setValue(DEFAULT_VAL);
        pCharacteristic->setCallbacks(&chrCallbacks);
        NimBLEDescriptor* pC01Ddsc = pCharacteristic->createDescriptor(
                                               uuid_desc,
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE|
                                               NIMBLE_PROPERTY::WRITE_ENC, // only allow writing if paired / encrypted
                                               20
                                              );
        pC01Ddsc->setValue("Send it back!");
        pC01Ddsc->setCallbacks(&dscCallbacks);
        pBLEService[i]->start();
        pAdvertising->addServiceUUID(pBLEService[i]->getUUID());
    }
    
    pDeadService->start();
    pAdvertising->addServiceUUID(pDeadService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");
    //I2C_scanner();
    Serial.println("initializing...");
    #ifdef MPU9250_GYR
        address_sensor = MPU9250;
    #elif BH1721_LIGHT
        address_sensor = BH1750;
    #elif BME280_BAROMETER
        address_sensor = BMP280;
    #elif HDC1080_TEMP
        address_sensor = HDC1080;
    #endif
    if(address_sensor==MPU9250)
        MPU9250Setup(SDA_PIN,SCL_PIN,i2C_FREQ);
    else if(address_sensor==BH1750 || address_sensor== BMP280){
        Wire_Setup(SDA_PIN,SCL_PIN,i2C_FREQ);
        if(address_sensor== BH1750)setup_BH1751();
        else  setup_BMP280();
    }        
    else if(address_sensor == HDC1080){
        Wire_Setup(SDA_PIN,SCL_PIN,i2C_FREQ);
        setup_HDC1080();
    } 
    #ifdef OLED_DISPLAY
        setupOLED();
        printOLED(0,15, "initialize DONE");
    #endif
}


void loop() {
    if(millis()-old_time_sent>period_Sensor){
        old_time_sent = millis();
        get_BLE_Data();
        Serial.println(M_sensor_data());
    }
}
void get_BLE_Data(){
    if(pServer->getConnectedCount()) {
        printOLED(10,15, "Connected :: "+String(pServer->getConnectedCount()));
        Serial.println("COnnected :: " + String(pServer->getConnectedCount()));
        for(int i=0;i<DEVICES;i++){
            pBLEService[i] = pServer->getServiceByUUID(uuids_Service[i]);
            if(pBLEService[i]) {
                NimBLECharacteristic* pChr = pBLEService[i]->getCharacteristic(uuid_char[i]);
                if(pChr) {
                    pChr->notify(true);
                    String dda = pChr->getValue();
                    //dda.toCharArray(BLE_DATA,dda.length());
                    prepare_data(pChr->getValue());
                    Serial.println(BLE_DATA.id);
                }
            }
        }
        
    }
}
data_package prepare_data(String data){
    data_package _dd;

    return _dd;
}
void I2C_scanner(){
   byte error, address;
  int nDevices = 0;
  delay(2000);
  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    wire.beginTransmission(address);
    error = wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      address_sensor = address;
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
}
String M_sensor_data(){
    String data;
    myData.id = address_sensor;
    myData.node= CURRENT_DEVICE;
    if(address_sensor==MPU9250){
        loopMPU9250();
        myData.data0 = (int16_t)(get_accelx()*10);
        myData.data1 = (int16_t)(get_accely()*10);
        myData.data2 = (int16_t)(get_accelz()*10);
        myData.data3 = (int16_t)(get_gyrox()*10);
        myData.data4 = (int16_t)(get_gyroy()*10);
        myData.data5 = (int16_t)(get_gyroz()*10);
        myData.data6 = (int16_t)(get_magx()*10);
        myData.data7 = (int16_t)(get_magy()*10);
        myData.data8 = (int16_t)(get_magz()*10);
    }
    else if(address_sensor == BH1750)
    {
        myData.data0 = (int16_t)(readLux()*10);
    }
    else if(address_sensor == HDC1080){
        myData.data0 = int16_t(read_Humiduty()*100);
        myData.data1 = int16_t(read_Temperature()*100);
    }
    else if(address_sensor == BMP280){
        myData.data0 = int16_t(get_tempBMP280()*100);
        myData.data1 = int16_t(get_pressureBMP280());
        myData.data2 = int16_t(get_altitudeBMP280());
    }else return "";
    data = String(myData.id)+SPLIT+String(myData.node)+SPLIT+String(myData.data0);
    if(myData.data1!=0)data=data+SPLIT+String(myData.data1);
    if(myData.data2!=0)data=data+SPLIT+String(myData.data2);
    if(myData.data3!=0)data=data+SPLIT+String(myData.data3);
    if(myData.data4!=0)data=data+SPLIT+String(myData.data4);
    if(myData.data5!=0)data=data+SPLIT+String(myData.data5);
    if(myData.data6!=0)data=data+SPLIT+String(myData.data6);
    if(myData.data7!=0)data=data+SPLIT+String(myData.data7);
    if(myData.data8!=0)data=data+SPLIT+String(myData.data8);
    return data;
}