#include <OnlineStatusLib.h>
#include <debugHelper.h>
#include "vesc_comm.h";
#include <TaskScheduler.h>
#include <rom/rtc.h>

// #include <WiFi.h>
// #include <WiFiClient.h>
// #include "wificonfig.h";
// #include <ESPmDNS.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h>
// #include <WiFiManager.h> // https://github.com/tzapu/WiFiManager DEVELOPER BRANCH

// https://raw.githubusercontent.com/LilyGO/TTGO-TS/master/Image/TS%20V1.0.jpg

/*--------------------------------------------------------------------------------*/

const char compile_date[] = __DATE__ " " __TIME__;
const char file_name[] = __FILE__;

//--------------------------------------------------------------

#define MOTOR_POLE_PAIRS 7
#define WHEEL_DIAMETER_MM 97
#define MOTOR_PULLEY_TEETH 15
#define WHEEL_PULLEY_TEETH 36 // https://hobbyking.com/en_us/gear-set-with-belt.html

uint8_t vesc_packet[PACKET_MAX_LENGTH];

#define GET_FROM_VESC_INTERVAL 500

struct VESC_DATA
{
	float batteryVoltage;
	float motorCurrent;
	bool moving;
	bool vescOnline;
	float ampHours;
	int32_t tripMeters;
};
VESC_DATA vescdata;

//--------------------------------------------------------------
// #define 	VESC_UART_RX		16		// orange
// #define 	VESC_UART_TX		17		// green
#define VESC_UART_BAUDRATE 115200




bool connectedToWifi = false;

#define STORE_NAMESPACE "data"
#define STORE_TOTAL_AMP_HOURS "totalAmpHours"
// #define STORE_TRIP_AMP_HOURS		"tripAmpHours"
#define STORE_POWERED_DOWN "poweredDown"
#define STORE_LAST_VOLTAGE_READ "lastVolts"

#include "nvmstorage.h";

//--------------------------------------------------------------------------------

char auth[] = "5db4749b3d1f4aa5846fc01dfaf2188a";

//--------------------------------------------------------------------------------
#define	STARTUP 		1 << 0
#define DEBUG 			1 << 1
#define COMMUNICATION 	1 << 2
#define HARDWARE		1 << 3
// #define SOMETHING 	1 << 4
#define ONLINE_STATUS	1 << 5
#define LOGGING			1 << 6

debugHelper debug;

//--------------------------------------------------------------

// portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;

//--------------------------------------------------------------

Scheduler runner;

bool ledOn;

bool firstTime = false;
float lastVoltsRead = 0.0;
float lastStableVoltsRead = 0.0;
bool interimUpdated = false; // when not moving
bool alreadyStoreValues = false;
bool appendAmpHoursOnPowerDown = false;
long lastReport = 0;

void tGetFromVESC_callback();
Task tGetFromVESC(GET_FROM_VESC_INTERVAL, TASK_FOREVER, &tGetFromVESC_callback);
void tGetFromVESC_callback()
{

	float battVoltsOld = vescdata.batteryVoltage;

	if (getVescValues() == false)
	{
		// vesc offline
		if (millis() - lastReport > 5000)
		{
			lastReport = millis();
			// debugD("vesc offline\n");
		}
	}
	else
	{
		bool updateDisplay = battVoltsOld != vescdata.batteryVoltage;
		if (updateDisplay)
		{
		}
		if (vescPoweringDown(vescdata.batteryVoltage))
		{
			// store values (not batteryVoltage)
			if (alreadyStoreValues == false)
			{
				storeValuesOnPowerdown(vescdata);
				alreadyStoreValues = true;
				float tripAH = vescdata.ampHours;
				float totalAH = recallFloat(STORE_TOTAL_AMP_HOURS);
			}
		}
		else
		{
			if (vescdata.moving == false)
			{
				// save volts
				lastStableVoltsRead = vescdata.batteryVoltage;
			}
			else
			{
				// moving
			}
			lastVoltsRead = vescdata.batteryVoltage;
		}
	}
}

bool vescPoweringDown(float volts)
{
	return volts < 32.0;
}

/**************************************************************/

void vescOfflineCallback()
{
}

void vescOnlineCallback()
{
}

OnlineStatusLib vescStatus(
	vescOfflineCallback,
	vescOnlineCallback,
	1 /*offlineNumConsecutiveTimesAllowance*/,
	false /*debug*/);

/**************************************************************/

bool deviceConnected = false;

//--------------------------------------------------------------------------------

void setup()
{
	Serial.begin(9600);

	vesc_comm_init(VESC_UART_BAUDRATE);
	vescdata.tripMeters = 0;

	debug.init();
	debug.addOption(STARTUP, "STARTUP");
	debug.addOption(DEBUG, "DEBUG");
	debug.addOption(HARDWARE, "HARDWARE");
	debug.addOption(COMMUNICATION, "COMMUNICATION");
	debug.addOption(ONLINE_STATUS, "ONLINE_STATUS");
	debug.addOption(LOGGING, "LOGGING");
	//debug.setFilter( STARTUP | COMMUNICATION | ONLINE_STATUS | TIMING );
	debug.setFilter( STARTUP | DEBUG | COMMUNICATION );// | COMMUNICATION | HARDWARE );
	debug.print( STARTUP, "Ready!" );

	bool vescOnline = getVescValues();

	runner.startNow();
	runner.addTask(tGetFromVESC);
	tGetFromVESC.enable();

	storeUInt8(STORE_POWERED_DOWN, 0);
}

//*************************************************************

long now = 0;

void loop()
{

	runner.execute();
}
//*************************************************************
bool controllerOnline = true;

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

	bool success = vesc_comm_fetch_packet(vesc_packet) > 0;

	if (success)
	{

		// // debugD("%d (rpm) %.1f (Ah) %u (tacho) %u (tachoabs)\n",
		// 	vesc_comm_get_rpm(vesc_packet),
		// 	vesc_comm_get_amphours_discharged(vesc_packet),
		// 	vesc_comm_get_tachometer(vesc_packet),
		// 	vesc_comm_get_tachometer_abs(vesc_packet)
		// );

		vescdata.batteryVoltage = vesc_comm_get_voltage(vesc_packet);
		vescdata.moving = vesc_comm_get_rpm(vesc_packet) > 50;
		vescdata.motorCurrent = vesc_comm_get_motor_current(vesc_packet); // UART.data.avgMotorCurrent;
		vescdata.ampHours = vesc_comm_get_amphours_discharged(vesc_packet);
		vescdata.vescOnline = true;
		vescdata.tripMeters = rotations_to_meters(vesc_comm_get_tachometer(vesc_packet) / 6);
	}
	else
	{
		vescdata.vescOnline = false;
		vescdata.batteryVoltage = 0.0;
		vescdata.moving = false;
		vescdata.motorCurrent = 0.0;
	}
	return success;
}
//--------------------------------------------------------------
char *get_reset_reason(RESET_REASON reason, int cpu)
{
	switch (reason)
	{
	case 1:
		return "POWERON_RESET"; /**<1, Vbat power on reset*/
	case 3:
		return "SW_RESET"; /**<3, Software reset digital core*/
	case 4:
		return "OWDT_RESET"; /**<4, Legacy watch dog reset digital core*/
	case 5:
		return "DEEPSLEEP_RESET"; /**<5, Deep Sleep reset digital core*/
	case 6:
		return "SDIO_RESET"; /**<6, Reset by SLC module, reset digital core*/
	case 7:
		return "TG0WDT_SYS_RESET"; /**<7, Timer Group0 Watch dog reset digital core*/
	case 8:
		return "TG1WDT_SYS_RESET"; /**<8, Timer Group1 Watch dog reset digital core*/
	case 9:
		return "RTCWDT_SYS_RESET"; /**<9, RTC Watch dog Reset digital core*/
	case 10:
		return "INTRUSION_RESET"; /**<10, Instrusion tested to reset CPU*/
	case 11:
		return "TGWDT_CPU_RESET"; /**<11, Time Group reset CPU*/
	case 12:
		return "SW_CPU_RESET"; /**<12, Software reset CPU*/
	case 13:
		return "RTCWDT_CPU_RESET"; /**<13, RTC Watch dog Reset CPU*/
	case 14:
		return "EXT_CPU_RESET"; /**<14, for APP CPU, reseted by PRO CPU*/
	case 15:
		return "RTCWDT_BROWN_OUT_RESET"; /**<15, Reset when the vdd voltage is not stable*/
	case 16:
		return "RTCWDT_RTC_RESET"; /**<16, RTC Watch dog reset digital core and rtc module*/
	default:
		return "NO_MEAN";
	}
}
//--------------------------------------------------------------
int32_t rotations_to_meters(int32_t rotations)
{
	float gear_ratio = float(WHEEL_PULLEY_TEETH) / float(MOTOR_PULLEY_TEETH);
	return (rotations / MOTOR_POLE_PAIRS / gear_ratio) * WHEEL_DIAMETER_MM * PI / 1000;
}
