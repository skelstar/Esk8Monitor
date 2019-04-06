
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

/*--------------------------------------------------------------------------------*/

const char compile_date[] = __DATE__ " " __TIME__;
const char file_name[] = __FILE__;

//--------------------------------------------------------------
struct STICK_DATA {
	float batteryVoltage;
	float motorCurrent;
	bool moving;
	bool vescOnline;
};
STICK_DATA stickdata;

// float batteryVoltage = 0.0;
float ampHours = 0.0;
// float motorCurrent = 0.0;
// bool moving = false;

//--------------------------------------------------------------------------------

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

/**************************************************************/

bool deviceConnected = false;

class MyServerCallbacks: public BLECharacteristicCallbacks {
	// receive
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
      }
      Serial.println();
      Serial.println("*********");
    }
  }

	void onConnect(BLEServer* pServer) {
		Serial.printf("device connected\n");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
		Serial.printf("device disconnected\n");
    deviceConnected = false;
  }
};

//--------------------------------------------------------------------------------

void setup()
{
	Serial.begin(9600);
  Serial.println("Starting BLE work!");

  stickdata.batteryVoltage = 35.0;

  setupBLE();
}

//*************************************************************

long now = 0;

void loop() {

  if (millis() - now > 2000) {
    now = millis();
    sendToClient();
    if (stickdata.batteryVoltage > 44.2) {
      stickdata.batteryVoltage = 35.0;
    }
  }
}
//*************************************************************
bool controllerOnline = true;

//--------------------------------------------------------------
void sendToClient() {

  stickdata.batteryVoltage += 0.1;
  stickdata.motorCurrent += 0.2;

	uint8_t bs[sizeof(stickdata)];
	memcpy(bs, &stickdata, sizeof(stickdata));

	pCharacteristic->setValue(bs, sizeof(bs));
	// Serial.printf("notifying!: %0.1f\n", stickdata.batteryVoltage);
	pCharacteristic->notify();
}
//--------------------------------------------------------------
void setupBLE() {

    BLEDevice::init("ESP32 Board Monitor");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY
    );
	  pCharacteristic->addDescriptor(new BLE2902());

    pCharacteristic->setCallbacks(new MyServerCallbacks());
    pCharacteristic->setValue("Hello World says Neil");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.printf("Characteristic defined! Now you can read it in your phone!\n");
}

