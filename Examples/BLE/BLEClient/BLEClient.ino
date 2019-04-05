#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <myPushButton.h>
#include "BLEDevice.h"
#include <driver/adc.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

struct STICK_DATA {
	float batteryVoltage;
	float motorCurrent;
	bool moving;
	bool vescOnline;
};
STICK_DATA stickdata;

/* ---------------------------------------------- */
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.printf("connected! \n");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.printf("disconnected!");
  }
};

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) {
			
	memcpy(&stickdata, pData, sizeof(stickdata));
	Serial.printf("Received batteryVoltage: %.1f \n", stickdata.batteryVoltage);
}

void setup() {

    Serial.begin(9600);
    Serial.println("\nStarting Arduino BLE Client application...");
    
    bleConnectToServer();
}

long now = 0;

void loop()
{
  if ( connected && millis() - now > 2000 ) {
    now = millis();
    sendToMaster();
  }
  delay(200);
}

void sendToMaster() {
    Serial.printf("sending to master\n");
    // char buff[6];
    // ltoa(millis(), buff, 10);
    // pRemoteCharacteristic->writeValue(buff, sizeof(buff));
    pRemoteCharacteristic->writeValue("test", sizeof("test"));
}

bool bleConnectToServer() {
    BLEDevice::init("");
    pServerAddress = new BLEAddress("80:7d:3a:c5:6a:36");
    delay(200);
    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    pClient->connect(*pServerAddress);
    Serial.println("Connected to server");
    delay(500);
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic->canNotify()) {
        Serial.println("registering for notify");
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
}
