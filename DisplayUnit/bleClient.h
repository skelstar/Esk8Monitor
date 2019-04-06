#include "BLEDevice.h"

/* ---------------------------------------------- */
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean serverConnected = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    serverConnected = true;
    Serial.printf("serverConnected! \n");
  }

  void onDisconnect(BLEClient *pclient)
  {
    serverConnected = false;
    Serial.printf("disconnected!");
  }
};

static void notifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  uint8_t *pData,
  size_t length,
  bool isNotify)
{
  memcpy(&vescdata, pData, sizeof(vescdata));
  Serial.printf("Received: %.1fAh %.1fkm \n", vescdata.ampHours, vescdata.odometer);
}

bool bleConnectToServer()
{
  BLEDevice::init("");
  pServerAddress = new BLEAddress("80:7d:3a:c5:6a:36");
  delay(200);
  BLEClient *pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(*pServerAddress);
  Serial.println("Connected to server");
  delay(500);
  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic->canNotify())
  {
    Serial.println("registering for notify");
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }
}
