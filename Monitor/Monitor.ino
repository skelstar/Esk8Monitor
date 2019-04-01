
#include <OnlineStatusLib.h>
#include <ESP8266VESC.h>
#include <VescUart.h> // https://github.com/SolidGeek/VescUart
#include "datatypes.h"
#include <debugHelper.h>
// #include <rom/rtc.h>
// #include <esp_int_wdt.h>
// #include <esp_task_wdt.h>
#include <TaskScheduler.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

/*--------------------------------------------------------------------------------*/

const char compile_date[] = __DATE__ " " __TIME__;
const char file_name[] = __FILE__;

//--------------------------------------------------------------

VescUart UART;

#define CONTROLLER_TIMEOUT 500
#define CONTROLLER_CONSECUTIVE_TIMEOUTS_ALLOWANCE 1
#define GET_VESC_DATA_AND_SEND_TO_CONTROLLER_INTERVAL 1000
#define SEND_TO_VESC_INTERVAL 200
#define GET_FROM_VESC_INTERVAL 1000

struct STICK_DATA
{
	float batteryVoltage;
	float motorCurrent;
	bool moving;
	bool vescOnline;
};
STICK_DATA stickdata;

float ampHours = 0.0;

//--------------------------------------------------------------------------------

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

//--------------------------------------------------------------------------------

char auth[] = "5db4749b3d1f4aa5846fc01dfaf2188a";

//--------------------------------------------------------------------------------
#define STARTUP 1 << 0		// 1
#define WARNING 1 << 1		// 2
#define VESC_COMMS 1 << 2 // 4
#define DEBUG 1 << 3			// 8
// #define CONTROLLER_COMMS 	1 << 4	// 16
#define HARDWARE 1 << 5 // 32
#define STATUS 1 << 6		// 64 << MAX

debugHelper debug;

//--------------------------------------------------------------
// #define 	VESC_UART_RX		16		// orange
// #define 	VESC_UART_TX		17		// green
#define VESC_UART_BAUDRATE 115200 // old: 19200

HardwareSerial VescSerial(2);

#define INBUILT_LED 2

//--------------------------------------------------------------

// portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;

//--------------------------------------------------------------

Scheduler runner;

bool ledOn;

void tGetFromVESC_callback();
Task tGetFromVESC(GET_FROM_VESC_INTERVAL, TASK_FOREVER, &tGetFromVESC_callback);
void tGetFromVESC_callback()
{

	if (getVescValues() == false)
	{
		// vesc offline
	}
	sendToStick();
}

/**************************************************************/

void vescOfflineCallback()
{
	debug.print(STATUS, "vescOfflineCallback();\n");
}
void vescOnlineCallback()
{
	debug.print(STATUS, "vescOnlineCallback();\n");
}

OnlineStatusLib vescStatus(
		vescOfflineCallback,
		vescOnlineCallback,
		1, /*offlineNumConsecutiveTimesAllowance*/
		false /*debug*/);

/**************************************************************/

bool deviceConnected = false;

class MyServerCallbacks : public BLECharacteristicCallbacks
{
	// receive data from client
	void onWrite(BLECharacteristic *pCharacteristic)
	{
		std::string value = pCharacteristic->getValue();

		if (value.length() > 0)
		{
			Serial.println("*********");
			Serial.print("New value: ");
			for (int i = 0; i < value.length(); i++)
			{
				Serial.print(value[i]);
			}
			Serial.println();
			Serial.println("*********");
		}
	}
	void onConnect(BLEServer *pServer)
	{
		Serial.printf("device connected\n");
		deviceConnected = true;
	};

	void onDisconnect(BLEServer *pServer)
	{
		Serial.printf("device disconnected\n");
		deviceConnected = false;
	}
};

//--------------------------------------------------------------------------------

void setup()
{
	Serial.begin(9600);

	Serial.println("Starting BLE work!");

	pinMode(INBUILT_LED, OUTPUT);

	VescSerial.begin(VESC_UART_BAUDRATE);
	UART.setSerialPort(&VescSerial);

	Serial.println("Ready");

	debug.init();
	debug.addOption(STARTUP, "STARTUP");
	debug.addOption(DEBUG, "DEBUG");
	debug.addOption(VESC_COMMS, "VESC_COMMS");
	debug.addOption(HARDWARE, "HARDWARE");
	debug.addOption(STATUS, "STATUS");
	debug.setFilter(STARTUP | VESC_COMMS);

	debug.print(STARTUP, "%s\n", file_name);
	debug.print(STARTUP, "%s\n", compile_date);

	setupBLE();

	/** Define which ports to use as UART */

	bool vescOnline = getVescValues();
	// debug.print(STARTUP, "%s\n", vescOnline ? "VESC Online!" : "ERROR: VESC Offline!");

	runner.startNow();
	runner.addTask(tGetFromVESC);
	tGetFromVESC.enable();
}

//*************************************************************

long now = 0;

void loop()
{

	// esp_task_wdt_feed();

	runner.execute();
}
//*************************************************************
bool controllerOnline = true;

//--------------------------------------------------------------
void sendToStick()
{

	uint8_t bs[sizeof(stickdata)];
	memcpy(bs, &stickdata, sizeof(stickdata));

	pCharacteristic->setValue(bs, sizeof(bs));
	// Serial.printf("notifying!: %0.1f\n", stickdata.batteryVoltage);
	pCharacteristic->notify();
}
//--------------------------------------------------------------
void setupBLE()
{

	BLEDevice::init("ESP32 Board Monitor");
	BLEServer *pServer = BLEDevice::createServer();
	BLEService *pService = pServer->createService(SERVICE_UUID);
	pCharacteristic = pService->createCharacteristic(
			CHARACTERISTIC_UUID,
			BLECharacteristic::PROPERTY_READ |
					BLECharacteristic::PROPERTY_WRITE |
					BLECharacteristic::PROPERTY_NOTIFY);
	pCharacteristic->addDescriptor(new BLE2902());

	pCharacteristic->setCallbacks(new MyServerCallbacks());
	pCharacteristic->setValue("Hello World says Neil");
	pService->start();
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(true);
	pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
	pAdvertising->setMinPreferred(0x12);
	BLEDevice::startAdvertising();
	Serial.printf("Characteristic defined! Now you can read it in your phone!");
}

//--------------------------------------------------------------
bool getVescValues()
{
	/*
	struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		%ld rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
	}; */

	bool success = UART.getVescValues();
	if (success)
	{

		stickdata.batteryVoltage = UART.data.inpVoltage;
		ampHours = UART.data.ampHours;
		stickdata.moving = UART.data.tachometer > 100;
		stickdata.motorCurrent = UART.data.avgMotorCurrent;
		stickdata.vescOnline = true;

		Serial.printf("inpVoltage: %.1f\n", UART.data.inpVoltage);
		Serial.printf("ampHours: %.1f\n", UART.data.ampHours);
		Serial.printf("rpm: %ul\n", UART.data.rpm);
		// bool moving = UART.data.rpm > 100;
		bool accelerating = UART.data.avgMotorCurrent > 0.2;
		Serial.printf("moving: %d accelerating: %d \n", stickdata.moving, accelerating);
		Serial.printf("motor current: %.1f\n", UART.data.avgMotorCurrent);
		Serial.printf("Odometer: %ul\n", UART.data.tachometerAbs / 42);

		// esk8.boardPacket.batteryVoltage = UART.data.inpVoltage;
		// esk8.boardPacket.odometer = UART.data.tachometerAbs/42;
		// esk8.boardPacket.areMoving = UART.data.rpm > 100;
		// Serial.printf("areMoving: %d\n", esk8.boardPacket.areMoving);

		// Serial.print("avgMotorCurrent: "); 	Serial.println(UART.data.avgMotorCurrent);
		// Serial.print("avgInputCurrent: "); 	Serial.println(UART.data.avgInputCurrent);
		// Serial.print("dutyCycleNow: "); 	Serial.println(UART.data.dutyCycleNow);
		// Serial.printf("rpm: %ld\n", UART.data.rpm);
		// Serial.print("inputVoltage: "); 	Serial.println(UART.data.inpVoltage);
		// Serial.print("ampHours: "); 		Serial.println(UART.data.ampHours);
		// Serial.print("ampHoursCharges: "); 	Serial.println(UART.data.ampHoursCharged);
		// Serial.print("tachometer: "); 		Serial.println(UART.data.tachometer;);
		// Serial.print("tachometerAbs: "); 	Serial.println(UART.data.tachometerAbs);
	}
	else
	{
		stickdata.vescOnline = false;
		stickdata.batteryVoltage = 0.0;
		stickdata.moving = false;
		stickdata.motorCurrent = 0.0;

		debug.print(VESC_COMMS, "vescOnline = false\n");
	}
	return success;
}
//--------------------------------------------------------------
// void print_reset_reason(RESET_REASON reason, int cpu)
// {
// 	debug.print(STARTUP, "Reboot reason (CPU%d): ", cpu);
// 	switch ( reason)
// 	{
// 		case 1 :  debug.print(STARTUP, "POWERON_RESET \n");break;          /**<1, Vbat power on reset*/
// 		case 3 :  debug.print(STARTUP, "SW_RESET \n");break;               /**<3, Software reset digital core*/
// 		case 4 :  debug.print(STARTUP, "OWDT_RESET \n");break;             /**<4, Legacy watch dog reset digital core*/
// 		case 5 :  debug.print(STARTUP, "DEEPSLEEP_RESET \n");break;        /**<5, Deep Sleep reset digital core*/
// 		case 6 :  debug.print(STARTUP, "SDIO_RESET \n");break;             /**<6, Reset by SLC module, reset digital core*/
// 		case 7 :  debug.print(STARTUP, "TG0WDT_SYS_RESET \n");break;       /**<7, Timer Group0 Watch dog reset digital core*/
// 		case 8 :  debug.print(STARTUP, "TG1WDT_SYS_RESET \n");break;       /**<8, Timer Group1 Watch dog reset digital core*/
// 		case 9 :  debug.print(STARTUP, "RTCWDT_SYS_RESET \n");break;       /**<9, RTC Watch dog Reset digital core*/
// 		case 10 : debug.print(STARTUP, "INTRUSION_RESET \n");break;       /**<10, Instrusion tested to reset CPU*/
// 		case 11 : debug.print(STARTUP, "TGWDT_CPU_RESET \n");break;       /**<11, Time Group reset CPU*/
// 		case 12 : debug.print(STARTUP, "SW_CPU_RESET \n");break;          /**<12, Software reset CPU*/
// 		case 13 : debug.print(STARTUP, "RTCWDT_CPU_RESET \n");break;      /**<13, RTC Watch dog Reset CPU*/
// 		case 14 : debug.print(STARTUP, "EXT_CPU_RESET \n");break;         /**<14, for APP CPU, reseted by PRO CPU*/
// 		case 15 : debug.print(STARTUP, "RTCWDT_BROWN_OUT_RESET \n");break;/**<15, Reset when the vdd voltage is not stable*/
// 		case 16 : debug.print(STARTUP, "RTCWDT_RTC_RESET \n");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
// 		default : debug.print(STARTUP, "NO_MEAN\n");
// 	}
// }