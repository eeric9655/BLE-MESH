#include "includes.h"

void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
static uint32_t scanTime = 0; /** 0 = scan forever */
  NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;
NimBLEClient* pClient = nullptr;
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected");
         pClient->updateConnParams(120,120,0,60);
    };

    void onDisconnect(NimBLEClient* pClient) {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    };

    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        if(params->itvl_min < 24) { /** 1.25ms units */
            return false;
        } else if(params->itvl_max > 40) { /** 1.25ms units */
            return false;
        } else if(params->latency > 2) { /** Number of intervals allowed to skip */
            return false;
        } else if(params->supervision_timeout > 100) { /** 10ms units */
            return false;
        }

        return true;
    };

    uint32_t onPassKeyRequest(){
        Serial.println("Client Passkey Request");
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");
        Serial.println(pass_key);
        return true;
    };
    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        if(!desc->sec_state.encrypted) {
            Serial.println("Encrypt connection failed - disconnecting");
            NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
            return;
        }
    };
};
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice->toString().c_str());
        if(advertisedDevice->isAdvertisingService(NimBLEUUID("DEAD")))
        {
            Serial.println("Found Our Service");
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }
    };
};

void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    Serial.println(str.c_str());
}
void scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}
static ClientCallbacks clientCB;

int vld=0;
bool connectToServer() {
    if(NimBLEDevice::getClientListSize()) {
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
       else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }
        pClient = NimBLEDevice::createClient();
        Serial.println("New client created");
        pClient->setClientCallbacks(&clientCB, false);
        pClient->setConnectionParams(12,12,0,51);
        pClient->setConnectTimeout(5);
        if (!pClient->connect(advDevice)) {
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());
    pSvc = pClient->getService(uuids_Service[CURRENT_DEVICE]);
    if(pSvc) {     /** make sure it's not null */
        pChr = pSvc->getCharacteristic(uuid_char[CURRENT_DEVICE]);

        if(pChr) {     /** make sure it's not null */
            if(pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pChr->readValue().c_str());
            }

            pDsc = pChr->getDescriptor(NimBLEUUID(uuid_desc));
            if(pDsc) {   /** make sure it's not null */
                Serial.print("Descriptor: ");
                Serial.print(pDsc->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pDsc->readValue().c_str());
            }
            vld++;
            if(pChr->canWrite()) {
                if(pChr->writeValue("Data:"+String(vld++))) {
                    Serial.print("Wrote new value to: ");
                    Serial.println(pChr->getUUID().toString().c_str());
                }
                else {
                    pClient->disconnect();
                    return false;
                }

                if(pChr->canRead()) {
                    Serial.print("The value of: ");
                    Serial.print(pChr->getUUID().toString().c_str());
                    Serial.print(" is now: ");
                    Serial.println(pChr->readValue().c_str());
                }
            }
            if(pChr->canNotify()) {
                if(!pChr->subscribe(true, notifyCB)) {
                    pClient->disconnect();
                    return false;
                }
            }
            else if(pChr->canIndicate()) {
                if(!pChr->subscribe(false, notifyCB)) {
                    pClient->disconnect();
                    return false;
                }
            }
        }

    } else {
        Serial.println(String(uuids_Service[CURRENT_DEVICE])+"service not found.");
    }

    Serial.println("Done with this device!");
    return true;
}

void setup (){
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");
    NimBLEDevice::init("");
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    NimBLEDevice::setPower(ESP_PWR_LVL_P21);
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    pScan->setInterval(45);
    pScan->setWindow(15);
    pScan->setActiveScan(true);
    pScan->start(scanTime, scanEndedCB);       
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
    else if(address_sensor==BH1750 || address_sensor== BMP280 || address_sensor == HDC1080){
        Wire_Setup(SDA_PIN,SCL_PIN,i2C_FREQ);
        if(address_sensor== BH1750)setup_BH1751();
        else if(address_sensor == HDC1080)setup_HDC1080();
        else  setup_BMP280();
    }
    else Serial.println("Connected other Sensor");
    #ifdef OLED_DISPLAY
        setupOLED();
        printOLED(0,15, "initialize DONE");
    #endif
}


void loop (){
    if(!doConnect){
        if(millis()-old_time_sent>period_Sensor){
            old_time_sent = millis();
            vld++;
            Send_BLE(M_sensor_data());
        }
    }
    else{
        doConnect = false;
        if(connectToServer()) {
            Serial.println("Success! we should now be getting notifications, scanning for more!");
        } else {
            Serial.println("Failed to connect, starting scan");
        }
        NimBLEDevice::getScan()->start(scanTime,scanEndedCB);
    }    
}


void Send_BLE(String data){
    
    Serial.println("lena");
    if(pSvc) {     /** make sure it's not null */
        if(pChr) {
            Serial.print(pChr->getUUID().toString().c_str());
            Serial.print(" Value: ");
            String dataBLE = pChr->readValue().c_str();
            Serial.println(dataBLE);
            if(dataBLE == ""){
                Serial.println("Disconnected from Service");
                delay(2000);
                ESP.restart();
            }
            if(pChr->writeValue(data)) {
                Serial.print("Wrote new value to: ");
                Serial.println(pChr->getUUID().toString().c_str());
            }
        }
    }else {
        Serial.println(String(uuids_Service[CURRENT_DEVICE])+" not found.");
    }
}

String M_sensor_data(){
    String data;
    myData.id = address_sensor;
    myData.node= CURRENT_DEVICE;
    if(address_sensor==MPU9250){
        loopMPU9250();
        myData.data0 = (int16_t)(get_accelx()*100);
        myData.data1 = (int16_t)(get_accely()*100);
        myData.data2 = (int16_t)(get_accelz()*100);
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