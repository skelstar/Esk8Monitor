#include <TaskScheduler.h>
#include <rom/rtc.h>
#include <vesc_comms.h>

// https://raw.githubusercontent.com/LilyGO/TTGO-TS/master/Image/TS%20V1.0.jpg
// https://www.aliexpress.com/item/Wemos-18650-Battery-shield-V3-RaspberryPi-Arduino-Lolin-ESP32-OLED-wemos-for-Arduino-ESP32S-WiFi-Modules/32807898232.html?spm=a2g0s.9042311.0.0.27424c4dy1K0TN
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
  float ampHours;
  float totalAmpHours;
  float odometer;
  float totalOdometer;
};
VESC_DATA vescdata;

#define STATUS_BIT_POWER_DOWN_NORMAL 0
#define STATUS_BIT_CLEARED_TRIP      1

float initial_ampHours = 0.0; // get from first packet from vesc
float initial_odometer = 0.0;
float totalAmpHours;
float totalOdometer;

//--------------------------------------------------------------
#define VESC_UART_BAUDRATE 19200

#define STORE_NAMESPACE "data"
#define STORE_TOTAL_AMP_HOURS "totalAmpHours"
#define STORE_TOTAL_ODOMETER "total_odometer"
#define STORE_POWERED_DOWN "poweredDown"
#define STORE_LAST_VOLTAGE_READ "lastVolts"


#include "nvmstorage.h"

//--------------------------------------------------------------------------------

// configure TX RX in vesc_comms.cpp
vesc_comms vesc;

bool handledFirstVescPacket = false;
float lastStableVoltsRead = 0.0;
bool alreadyStoreValues = false;
long lastReport = 0;


//--------------------------------------------------------------
bool controllerOnline = true;

void initData() {
  totalAmpHours = recallFloat( STORE_TOTAL_AMP_HOURS );
  totalOdometer = recallFloat( STORE_TOTAL_ODOMETER );
}
//--------------------------------------------------------------

int32_t rotations_to_meters(int32_t rotations)
{
  float gear_ratio = float(WHEEL_PULLEY_TEETH) / float(MOTOR_PULLEY_TEETH);
  return (rotations / MOTOR_POLE_PAIRS / gear_ratio) * WHEEL_DIAMETER_MM * PI / 1000;
}

bool getVescValues()
{
  bool success = vesc.fetch_packet(vesc_packet) > 0;

  if ( success )
  {
    vescdata.batteryVoltage = vesc.get_voltage(vesc_packet);
    vescdata.moving = vesc.get_rpm(vesc_packet) > 50;
    vescdata.motorCurrent = vesc.get_motor_current(vesc_packet);
    // amphours
    vescdata.ampHours = vesc.get_amphours_discharged(vesc_packet);
    vescdata.totalAmpHours = vescdata.ampHours + totalAmpHours - initial_ampHours;;
    // odometer
    int32_t distanceMeters = rotations_to_meters(vesc.get_tachometer(vesc_packet) / 6);
    vescdata.odometer = distanceMeters / 1000.0;
    vescdata.totalOdometer = vescdata.odometer + totalOdometer - initial_odometer;
    // vescdata.vescOnline = true;
  }
  else
  {
    // vescdata.vescOnline = false;
    vescdata.batteryVoltage = 0.0;
    vescdata.moving = false;
    vescdata.motorCurrent = 0.0;
  }
  return success;
}

//--------------------------------------------------------------

Scheduler runner;

//------------------------------------------------------------------

bool hadPoweredDownNormally() {
  bool normal = recallUInt8(STORE_POWERED_DOWN) == 1;
  storeUInt8(STORE_POWERED_DOWN, 0);
  return normal;
}

void handleIfFirstVescPacket() {
	if ( handledFirstVescPacket == false ) {
		handledFirstVescPacket = true;
    Serial.printf("handledFirstVescPacket! (%.1f) /n", vescdata.ampHours);
		// make sure ampHours == false
		if ( hadPoweredDownNormally() ) {
			initial_odometer = vescdata.odometer;
      initial_ampHours = vescdata.ampHours;
		}
	}
}

void handleBoardNotMoving()
{
  lastStableVoltsRead = vescdata.batteryVoltage;
}

void handleBoardMoving()
{
}

void handlePoweringDown()
{
  // store values (not batteryVoltage)
  if (alreadyStoreValues == false)
  {
    alreadyStoreValues = true;
    // store total amp hours, total odometer

    float updatedTotalAmpHours = totalAmpHours + vescdata.ampHours - initial_ampHours;
    storeFloat( STORE_TOTAL_AMP_HOURS, updatedTotalAmpHours );
    float updatedTotalOdometer = totalOdometer + vescdata.odometer - initial_odometer;
    storeFloat( STORE_TOTAL_ODOMETER, updatedTotalOdometer );
    storeUInt8(STORE_POWERED_DOWN, 1); // true
    Serial.printf("Powering down. Storing totalAmpHours (%.1f + %.1f)\n", updatedTotalAmpHours, vescdata.ampHours);
    handledFirstVescPacket = false;
  }
  return;
}

// gets called from ble_notify.h
void clearTripMeterAndOdometer() {
  storeFloat( STORE_TOTAL_AMP_HOURS, 0 );
  storeFloat( STORE_TOTAL_ODOMETER, 0 );
  totalAmpHours = 0;
  totalOdometer = 0;
  Serial.printf("clearTripMeterAndOdometer() \n");
}

// #include "ble_notify.h"

void vescOfflineCallback()
{
}

void vescOnlineCallback()
{
}

/**************************************************************/

float oldBattVoltage = 0.0;

void tGetFromVESC_callback();
Task tGetFromVESC(GET_FROM_VESC_INTERVAL, TASK_FOREVER, &tGetFromVESC_callback);
void tGetFromVESC_callback()
{
  float battVoltsOld = vescdata.batteryVoltage;
  bool vescOnline = getVescValues() == true;

  if (vescOnline == false)
  {
    Serial.printf("VESC not responding!\n");
    // vesc offline
    if (millis() - lastReport > 5000)
    {
      lastReport = millis();
    }
  }
  else
  {
    Serial.printf("batt volts: %.1f \n", vescdata.batteryVoltage);
		handleIfFirstVescPacket();

    //sendDataToClient();
		bool updateDisplay = battVoltsOld != vescdata.batteryVoltage;
		if ( updateDisplay ) {
      // drawBatteryTopScreen( vescdata.batteryVoltage );
      // drawAmpHoursUsed( vescdata.ampHours );
      // drawTotalAmpHours( recallFloat(STORE_TOTAL_AMP_HOURS) );
      // drawTripMeter( vescdata.odometer );
    }

    bool vescPoweringDown = vescdata.batteryVoltage < 32.0 && vescdata.batteryVoltage > 10;
    if (vescPoweringDown)
    {
      handlePoweringDown();
    }
    else if (vescdata.moving == false)
    {
      handleBoardNotMoving();
    }
    else
    {
      handleBoardMoving();
    }
  }
}
//*************************************************************

void setup()
{
  Serial.begin(115200);

  vesc.init(VESC_UART_BAUDRATE);

  //setupDisplay();

  initData();

  runner.startNow();
  runner.addTask( tGetFromVESC );
  tGetFromVESC.enable();

  // setupBLE();

  // drawBatteryTopScreen(vescdata.batteryVoltage);
}
//--------------------------------------------------------------------------------
void loop()
{
  runner.execute();
}
//*************************************************************
