#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <myPushButton.h>
#include <driver/adc.h>

#include <Fsm.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BATTERY_VOLTAGE_FULL            44.2
#define BATTERY_VOLTAGE_CUTOFF_START    37.4
#define BATTERY_VOLTAGE_CUTOFF_END      34.1

#define LED_ON HIGH
#define LED_OFF LOW

/* ---------------------------------------------- */

static boolean serverConnected = false;


#define STATE_POWER_UP        0
#define STATE_CONNECTING      1
#define STATE_CONNECTED       2
#define STATE_BATTERY_VOLTAGE_SCREEN  3

struct VESC_DATA
{
  float batteryVoltage;
  float motorCurrent;
  bool moving;
  float ampHours;
  float totalAmpHours;
  float odometer; // in kilometers
  float totalOdometer;
  uint8_t status;
};
VESC_DATA vescdata, oldvescdata;

bool oldMoving = false;

#include "utils.h"
#include "display.h"


enum EventsEnum {
  BUTTON_CLICK,
  SERVER_CONNECTED,
  SERVER_DISCONNECTED,
  MOVING,
  STOPPED_MOVING,
  HELD_POWERDOWN_WINDOW,
  HELD_CLEAR_TRIP_WINDOW,
  BUTTON_BEING_HELD,
  SENT_CLEAR_TRIP_ODO,
  HELD_RELEASED
} event;

void on_state_connecting_on_enter();
State state_connecting(
  &on_state_connecting_on_enter, 
  NULL, 
  NULL);

void on_state_connected_on_enter();
State state_connected(
  &on_state_connected_on_enter, 
  NULL, 
  NULL);

void on_state_battery_voltage_screen_on_enter();
void check_battery_voltage_changed();
State state_battery_voltage_screen(
  &on_state_battery_voltage_screen_on_enter, 
  &check_battery_voltage_changed,
  NULL); 

void on_state_motor_current_screen_on_enter();
void check_motor_current_changed();
State state_motor_current_screen(
  &on_state_motor_current_screen_on_enter, 
  &check_motor_current_changed,
  NULL); 

void check_page_two_data_changed();
void on_state_page_two_enter();
State state_page_two(
  &on_state_page_two_enter,
  &check_page_two_data_changed,
  NULL);

void on_button_held_powerdown_window_enter();
State state_button_held_powerdown_window(
  &on_button_held_powerdown_window_enter,
  NULL,
  NULL
);

void on_button_held_clear_trip_window_enter();
State state_button_held_clear_trip_window(
  &on_button_held_clear_trip_window_enter,
  NULL,
  NULL
);

void on_button_being_held_enter();
State state_button_being_held(
  &on_button_being_held_enter,
  NULL,
  NULL
);

Fsm fsm(&state_connecting);

void on_state_connecting_on_enter() {
  Serial.printf("on_state_connecting_on_enter()\n");
  lcdMessage("connecting");
}
void on_state_connected_on_enter() {
  lcdMessage("connected");
}
void on_state_battery_voltage_screen_on_enter() {
  drawBattery( getBatteryPercentage(vescdata.batteryVoltage) );
}
void check_battery_voltage_changed() {
  if (vescdata.batteryVoltage != oldvescdata.batteryVoltage) {
    oldvescdata = vescdata;
    drawBattery( getBatteryPercentage(vescdata.batteryVoltage) );
  }
}
void on_state_motor_current_screen_on_enter() {
  lcdMotorCurrent(vescdata.motorCurrent);
}
void check_motor_current_changed() {
  if (vescdata.motorCurrent != oldvescdata.motorCurrent) {
    oldvescdata = vescdata;
    lcdMotorCurrent(vescdata.motorCurrent);
  }
}
void on_state_page_two_enter() {
  lcdPage2(
    vescdata.ampHours, 
    vescdata.totalAmpHours, 
    vescdata.odometer, 
    vescdata.totalOdometer);
}
void check_page_two_data_changed() {
  if (vescdata.ampHours != oldvescdata.ampHours) {
    oldvescdata = vescdata;
    lcdPage2(
      vescdata.ampHours, 
      vescdata.totalAmpHours, 
      vescdata.odometer, 
      vescdata.totalOdometer);
  }
}
void on_button_held_powerdown_window_enter() { lcdMessage("powerd down?"); }
void on_button_held_clear_trip_window_enter() { lcdMessage("clear trip?"); }
void on_button_being_held_enter() { lcdMessage("..."); }

/* ---------------------------------------------- */

void bleConnected() {
  Serial.printf("serverConnected! \n");
  serverConnected = true;
  fsm.trigger( SERVER_CONNECTED );
}

void bleDisconnected() {
  serverConnected = false;
  Serial.printf("disconnected!");
  fsm.trigger( SERVER_DISCONNECTED );
}

void bleReceivedNotify() {
  //Serial.printf("Received: %.1fAh %.1fkm \n", vescdata.ampHours, vescdata.odometer);
}

#include "bleClient.h"

/* ---------------------------------------------- */

#define LedPin 19
#define IrPin 17
#define BuzzerPin 26
#define BtnPin 35
/* ---------------------------------------------- */

#define PULLUP		true
#define OFFSTATE	HIGH

void listener_Button(int eventCode, int eventPin, int eventParam);
myPushButton button(BtnPin, PULLUP, OFFSTATE, listener_Button);
void listener_Button(int eventCode, int eventPin, int eventParam) {
    
  bool sleepTimeSlot = eventParam >= 2 && eventParam <= 3;
  bool clearTripOdoSlot = eventParam >= 4 && eventParam <= 5;

	switch (eventCode) {
		case button.EV_BUTTON_PRESSED:
			Serial.println("EV_BUTTON_PRESSED");
			break;
		case button.EV_RELEASED:
			Serial.printf("EV_RELEASED %d\n", eventParam);
      if ( sleepTimeSlot ) {
        deepSleep();
        break;
      }      
      if ( clearTripOdoSlot ) {
        sendClearTripOdoToMonitor();
        break;
      }
      else if (eventParam < 2) {
        fsm.trigger( BUTTON_CLICK );
      }
      else {
        fsm.trigger( BUTTON_CLICK );
      }
			break;
		case button.EV_DOUBLETAP:
			break;
		case button.EV_HELD_SECONDS:
      Serial.printf("HELD %d seconds \n", eventParam);
      if ( sleepTimeSlot ) {
        Serial.printf("HELD_POWERDOWN_WINDOW \n");
        fsm.trigger( HELD_POWERDOWN_WINDOW );
          // lcdMessage("release!");
      }
      else if ( clearTripOdoSlot ) {
        Serial.printf("HELD_CLEAR_TRIP_WINDOW \n");
        fsm.trigger( HELD_CLEAR_TRIP_WINDOW );
      }
      else {
        Serial.printf("BUTTON_BEING_HELD \n");
        fsm.trigger( BUTTON_BEING_HELD );
          // lcdMessage("powering down");
      }
			break;
    }
}

void setup() {
    // put your setup code here, to run once:
    Wire.begin(21, 22, 100000);
    // u8x8.begin();
    u8g2.begin();

    Serial.begin(9600);
    Serial.println("\nStarting Arduino BLE Client application...");

    fsm.add_transition(&state_connecting, &state_connected, SERVER_CONNECTED, NULL);
    fsm.add_timed_transition(&state_connected, &state_battery_voltage_screen, 1000, NULL);
    // BUTTON_CLICK
    fsm.add_transition(&state_battery_voltage_screen, &state_page_two, BUTTON_CLICK, NULL);
    fsm.add_transition(&state_page_two, &state_motor_current_screen, BUTTON_CLICK, NULL);
    fsm.add_transition(&state_motor_current_screen, &state_battery_voltage_screen, BUTTON_CLICK, NULL);
    // SERVER_DISCONNECTED -> state_connecting
    fsm.add_transition(&state_battery_voltage_screen, &state_connecting, SERVER_DISCONNECTED, NULL);
    fsm.add_transition(&state_page_two, &state_connecting, SERVER_DISCONNECTED, NULL);
    fsm.add_transition(&state_motor_current_screen, &state_connecting, SERVER_DISCONNECTED, NULL);
    // MOVING -> state_motor_current_screen
    fsm.add_transition(&state_battery_voltage_screen, &state_motor_current_screen, MOVING, NULL);
    fsm.add_transition(&state_page_two, &state_motor_current_screen, MOVING, NULL);
    // STOPPED_MOVING
    fsm.add_transition(&state_motor_current_screen, &state_page_two, STOPPED_MOVING, NULL);

    //BUTTON_BEING_HELD               
    fsm.add_transition(&state_battery_voltage_screen, &state_button_being_held, BUTTON_BEING_HELD, NULL);
    fsm.add_transition(&state_page_two, &state_button_being_held, BUTTON_BEING_HELD, NULL);
    fsm.add_transition(&state_motor_current_screen, &state_button_being_held, BUTTON_BEING_HELD, NULL);
    fsm.add_transition(&state_button_held_clear_trip_window, &state_button_being_held, BUTTON_BEING_HELD, NULL);
    //HELD_POWERDOWN_WINDOW
    fsm.add_transition(&state_button_being_held, &state_button_held_powerdown_window, HELD_POWERDOWN_WINDOW, NULL);
    //HELD_CLEAR_TRIP_WINDOW
    fsm.add_transition(&state_button_held_powerdown_window, &state_button_held_clear_trip_window, HELD_CLEAR_TRIP_WINDOW, NULL);
    // SENT_CLEAR_TRIP_ODO
    fsm.add_transition(&state_button_held_clear_trip_window, &state_page_two, SENT_CLEAR_TRIP_ODO, NULL);
    // EVENT_HELD_RELEASED
    fsm.add_transition(&state_button_being_held, &state_page_two, BUTTON_CLICK, NULL);
    
    fsm.run_machine();

    setupPeripherals();

    // if button held then we can shut down
    button.serviceEvents();
    while (button.isPressed()) {
      fsm.run_machine();
      button.serviceEvents();
    }
    button.serviceEvents();
}

void loop()
{
    button.serviceEvents();

    if ( serverConnected == false ) {
      serverConnected = bleConnectToServer();
    }

    if ( oldMoving != vescdata.moving ) {
      oldMoving = vescdata.moving;
      if ( vescdata.moving ) {
        fsm.trigger( MOVING );
      }
      else {
        fsm.trigger( STOPPED_MOVING );
      }
    }

    fsm.run_machine();

    delay(100);
}

void buzzerBuzz() {
  for(int i=0;i<100;i++){
    digitalWrite(BuzzerPin,HIGH);
    delay(1);
    digitalWrite(BuzzerPin,LOW);
    delay(1);
  }
}

void sendToMaster() {
    Serial.printf("sending to master\n");
    char buff[6];
    ltoa(millis(), buff, 10);
    pRemoteCharacteristic->writeValue(buff, sizeof(buff));
    buzzerBuzz();
}

void setupPeripherals() {
    pinMode(LedPin, OUTPUT);
    pinMode(IrPin, OUTPUT);
    pinMode(BuzzerPin, OUTPUT);
    digitalWrite(LedPin, LED_ON);
    digitalWrite(BuzzerPin, LOW);
    u8g2.setFont(u8g2_font_4x6_tr);
}


void deepSleep() {
  digitalWrite(LedPin, LED_OFF);
  u8g2.setPowerSave(1);
  delay(500);
  pureDeepSleep();
}

#define CLEAR_TRIP_ODO_COMMAND  99

void sendClearTripOdoToMonitor() {
  Serial.printf("sending clear trip odo command to master\n");
  pRemoteCharacteristic->writeValue(CLEAR_TRIP_ODO_COMMAND, sizeof(uint8_t));
  buzzerBuzz();
  fsm.trigger( SENT_CLEAR_TRIP_ODO );
}

void pureDeepSleep() {
    // https://esp32.com/viewtopic.php?t=3083
    // esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    //   IMU.setSleepEnabled(true);
    delay(100);
    adc_power_off();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, LOW); //1 = High, 0 = Low
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}
