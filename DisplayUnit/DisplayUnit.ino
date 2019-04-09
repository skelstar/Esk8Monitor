#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <myPushButton.h>
#include <driver/adc.h>

#include <Fsm.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BATTERY_VOLTAGE_FULL 44.2
#define BATTERY_VOLTAGE_CUTOFF_START 37.4
#define BATTERY_VOLTAGE_CUTOFF_END 34.1

#define LED_ON HIGH
#define LED_OFF LOW

/* ---------------------------------------------- */

static boolean serverConnected = false;

#define STATE_POWER_UP 0
#define STATE_CONNECTING 1
#define STATE_CONNECTED 2
#define STATE_BATTERY_VOLTAGE_SCREEN 3

#define CHECK_BATT_VOLTS  1
#define CHECK_AMP_HOURS   2
#define CHECK_MOTOR_CURRENT 3

struct VESC_DATA
{
  float batteryVoltage;
  float motorCurrent;
  bool moving;
  float ampHours;
  float totalAmpHours;
  float odometer; // in kilometers
  float totalOdometer;
};
VESC_DATA vescdata, oldvescdata;

bool valueChanged(uint8_t measure) {
  switch (measure) {
    case CHECK_BATT_VOLTS:
      if (oldvescdata.batteryVoltage != vescdata.batteryVoltage) {
        oldvescdata.batteryVoltage = vescdata.batteryVoltage;
        return true;
      }
      return false;
    case CHECK_AMP_HOURS:
      if (oldvescdata.ampHours != vescdata.ampHours) {
        oldvescdata.ampHours = vescdata.ampHours;
        return true;
      }
      return false;
    case CHECK_MOTOR_CURRENT:
      if (oldvescdata.motorCurrent != vescdata.motorCurrent) {
        oldvescdata.motorCurrent = vescdata.motorCurrent;
        return true;
      }
      return false;
  }
}

#include "utils.h"
#include "display.h"

enum EventsEnum
{
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
// connecting
void enter_connecting() { lcdMessage("connecting"); }
State state_connecting(&enter_connecting, NULL, NULL);
// connected
void enter_connected() { lcdMessage("connected"); }
State state_connected(&enter_connected, NULL, NULL);
// battery_voltage_screen
void enter_battery_voltage_screen();
void check_battery_voltage_changed();
State state_battery_voltage_screen(
    &enter_battery_voltage_screen,
    &check_battery_voltage_changed,
    NULL);
void enter_battery_voltage_screen() { drawBattery(getBatteryPercentage(vescdata.batteryVoltage)); }
void check_battery_voltage_changed()
{
  if ( valueChanged(CHECK_BATT_VOLTS) ) {
    drawBattery( getBatteryPercentage(vescdata.batteryVoltage) );
  }
}
// motor_current_screen
void enter_motor_current_screen();
void check_motor_current_changed();
State state_motor_current_screen(
    &enter_motor_current_screen,
    &check_motor_current_changed,
    NULL);
void enter_motor_current_screen() { 
  lcdMovingScreen(vescdata.motorCurrent); 
}
void check_motor_current_changed()
{
  if (valueChanged(CHECK_MOTOR_CURRENT)) {
    lcdMovingScreen(vescdata.motorCurrent);
  }
}
// state_page_2
void enter_page_two();
void check_page_two_data_changed();
State state_page_two(
  &enter_page_two, 
  &check_page_two_data_changed, 
  NULL);
void enter_page_two() { lcdPage2(vescdata.ampHours, vescdata.totalAmpHours, vescdata.odometer, vescdata.totalOdometer); }
void check_page_two_data_changed()
{
  if (valueChanged(CHECK_AMP_HOURS)) {
    lcdPage2(vescdata.ampHours, vescdata.totalAmpHours, vescdata.odometer, vescdata.totalOdometer);
  }
}
// button_held_powerdown_window
void enter_button_held_powerdown_window() { lcdMessage("power down?"); }
State state_button_held_powerdown_window(&enter_button_held_powerdown_window, NULL, NULL);
// button_held_clear_trip_window
void enter_button_held_clear_trip_window() { lcdMessage("clear trip?"); }
State state_button_held_clear_trip_window(&enter_button_held_clear_trip_window, NULL, NULL);
// button_being_held
void enter_button_being_held() { lcdMessage("..."); }
State state_button_being_held(&enter_button_being_held, NULL, NULL);

Fsm fsm(&state_connecting);

void addFsmTransitions() {
  // SERVER_DISCONNECTED -> state_connecting
  uint8_t event = SERVER_DISCONNECTED;
  fsm.add_transition(&state_battery_voltage_screen, &state_connecting, event, NULL);
  fsm.add_transition(&state_page_two, &state_connecting, event, NULL);
  fsm.add_transition(&state_motor_current_screen, &state_connecting, event, NULL);
  // SERVER_CONNECTED
  event = SERVER_CONNECTED;
  fsm.add_transition(&state_connecting, &state_connected, event, NULL);
  fsm.add_timed_transition(&state_connected, &state_battery_voltage_screen, 1000, NULL);
  // BUTTON_CLICK
  event = BUTTON_CLICK;
  fsm.add_transition(&state_battery_voltage_screen, &state_page_two, event, NULL);
  fsm.add_transition(&state_page_two, &state_motor_current_screen, event, NULL);
  fsm.add_transition(&state_motor_current_screen, &state_battery_voltage_screen, event, NULL);
  // MOVING -> state_motor_current_screen
  event = MOVING;
  fsm.add_transition(&state_battery_voltage_screen, &state_motor_current_screen, event, NULL);
  fsm.add_transition(&state_page_two, &state_motor_current_screen, event, NULL);
  // STOPPED_MOVING
  event = STOPPED_MOVING;
  fsm.add_transition(&state_motor_current_screen, &state_page_two, event, NULL);
  //BUTTON_BEING_HELD
  event = BUTTON_BEING_HELD;
  fsm.add_transition(&state_connecting, &state_button_being_held, event, NULL);
  fsm.add_transition(&state_battery_voltage_screen, &state_button_being_held, event, NULL);
  fsm.add_transition(&state_page_two, &state_button_being_held, event, NULL);
  fsm.add_transition(&state_motor_current_screen, &state_button_being_held, event, NULL);
  fsm.add_transition(&state_button_held_clear_trip_window, &state_button_being_held, event, NULL);
  //HELD_POWERDOWN_WINDOW
  event = HELD_POWERDOWN_WINDOW;
  fsm.add_transition(&state_button_being_held, &state_button_held_powerdown_window, event, NULL);
  //HELD_CLEAR_TRIP_WINDOW
  event = HELD_CLEAR_TRIP_WINDOW;
  fsm.add_transition(&state_button_held_powerdown_window, &state_button_held_clear_trip_window, event, NULL);
  // SENT_CLEAR_TRIP_ODO
  event = SENT_CLEAR_TRIP_ODO;
  fsm.add_transition(&state_button_held_clear_trip_window, &state_page_two, event, NULL);
  // EVENT_HELD_RELEASED
  event = BUTTON_CLICK;
  fsm.add_transition(&state_button_being_held, &state_page_two, event, NULL);
}
/* ---------------------------------------------- */

void bleConnected()
{
  Serial.printf("serverConnected! \n");
  serverConnected = true;
  fsm.trigger(SERVER_CONNECTED);
}

void bleDisconnected()
{
  serverConnected = false;
  Serial.printf("disconnected!");
  fsm.trigger(SERVER_DISCONNECTED);
}

void bleReceivedNotify()
{
  Serial.printf("Received: %.1fV %.1fAh \n", vescdata.batteryVoltage, vescdata.ampHours);
}

#include "bleClient.h"

/* ---------------------------------------------- */

#define LedPin 19
#define IrPin 17
#define BuzzerPin 26
#define BtnPin 35
/* ---------------------------------------------- */

#define PULLUP true
#define OFFSTATE HIGH

void listener_Button(int eventCode, int eventPin, int eventParam);
myPushButton button(BtnPin, PULLUP, OFFSTATE, listener_Button);
void listener_Button(int eventCode, int eventPin, int eventParam)
{

  bool sleepTimeWindow = eventParam >= 2 && eventParam <= 3;
  bool clearTripWindow = eventParam >= 4 && eventParam <= 5;

  switch (eventCode)
  {
    case button.EV_BUTTON_PRESSED:
      break;

    case button.EV_RELEASED:
      if (sleepTimeWindow)
      {
        deepSleep();
        break;
      }
      if (clearTripWindow)
      {
        sendClearTripOdoToMonitor();
        break;
      }
      else
      {
        fsm.trigger(BUTTON_CLICK);
      }
      break;
    case button.EV_DOUBLETAP:
      break;
    case button.EV_HELD_SECONDS:
      Serial.printf("HELD %d seconds \n", eventParam);
      if (sleepTimeWindow)
      {
        fsm.trigger(HELD_POWERDOWN_WINDOW);
      }
      else if (clearTripWindow)
      {
        fsm.trigger(HELD_CLEAR_TRIP_WINDOW);
      }
      else
      {
        fsm.trigger(BUTTON_BEING_HELD);
      }
      break;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Wire.begin(21, 22, 100000);
  // u8x8.begin();
  u8g2.begin();

  Serial.begin(9600);
  Serial.println("\nStarting Arduino BLE Client application...");

  addFsmTransitions();
  fsm.run_machine();

  setupPeripherals();

  // if button held then we can shut down
  button.serviceEvents();
  while (button.isPressed())
  {
    fsm.run_machine();
    button.serviceEvents();
  }
  button.serviceEvents();
}

void loop()
{
  button.serviceEvents();

  if (serverConnected == false)
  {
    serverConnected = bleConnectToServer();
  }

  checkBoardMoving();

  fsm.run_machine();

  delay(100);
}

void checkBoardMoving() {
  if (oldvescdata.moving != vescdata.moving)
  {
    oldvescdata.moving = vescdata.moving;
    if (vescdata.moving)
    {
      fsm.trigger(MOVING);
    }
    else
    {
      fsm.trigger(STOPPED_MOVING);
    }
  }
}

void buzzerBuzz()
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BuzzerPin, HIGH);
    delay(1);
    digitalWrite(BuzzerPin, LOW);
    delay(1);
  }
}

void setupPeripherals()
{
  pinMode(LedPin, OUTPUT);
  pinMode(IrPin, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(LedPin, LED_ON);
  digitalWrite(BuzzerPin, LOW);
  u8g2.setFont(u8g2_font_4x6_tr);
}

void deepSleep()
{
  digitalWrite(LedPin, LED_OFF);
  u8g2.setPowerSave(1);
  delay(500);
  pureDeepSleep();
}

#define CLEAR_TRIP_ODO_COMMAND 99

void sendClearTripOdoToMonitor()
{
  Serial.printf("sending clear trip odo command to master\n");
  pRemoteCharacteristic->writeValue(CLEAR_TRIP_ODO_COMMAND, sizeof(uint8_t));
  buzzerBuzz();
  fsm.trigger(SENT_CLEAR_TRIP_ODO);
}

void pureDeepSleep()
{
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
