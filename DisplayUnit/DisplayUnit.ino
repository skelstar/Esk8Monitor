#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <myPushButton.h>
#include <driver/adc.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BATTERY_VOLTAGE_FULL            44.2
#define BATTERY_VOLTAGE_CUTOFF_START    37.4
#define BATTERY_VOLTAGE_CUTOFF_END      34.1

#define LED_ON HIGH
#define LED_OFF LOW

/* ---------------------------------------------- */
#define MODE_DISPLAY_SLEEPING   0
#define MODE_DISPLAY_TO_SLEEP   1
#define MODE_WAKE_DISPLAY       2
#define MODE_BATTERY_VOLTAGE    3
#define MODE_MOTOR_CURRENT      4
#define MODE_CONNECTING         5
#define MODE_CONNECTED          6
#define MODE_AMP_HOURS          7
#define MODE_DO_NOTHING         99

uint8_t display_mode = MODE_BATTERY_VOLTAGE;

struct VESC_DATA
{
  float batteryVoltage;
  float motorCurrent;
  bool moving;
  bool vescOnline;
  float ampHours;
  float totalAmpHours;
  float odometer; // in kilometers
  uint8_t status;
};
VESC_DATA vescdata, oldvescdata;

#define STATUS_BIT_POWER_DOWN_NORMAL 0
#define STATUS_BIT_CLEARED_TRIP      1

#include "bleClient.h"

#include "utils.h"
#include "display.h"

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
    
	switch (eventCode) {
		case button.EV_BUTTON_PRESSED:
			Serial.println("EV_BUTTON_PRESSED");
			break;
		case button.EV_RELEASED:
			Serial.println("EV_RELEASED");
      if (eventParam >= 2) {
        deepSleep();
      }      
      else if ( display_mode == MODE_BATTERY_VOLTAGE ) {  
        display_mode = MODE_MOTOR_CURRENT;
        updateNow = true;
      }
      else if ( display_mode == MODE_MOTOR_CURRENT ) {
        display_mode = MODE_AMP_HOURS;
        updateNow = true;
      }
      else if ( display_mode == MODE_AMP_HOURS ) {
        display_mode = MODE_BATTERY_VOLTAGE;
        updateNow = true;
      }
			break;
		case button.EV_DOUBLETAP:
			break;
		case button.EV_HELD_SECONDS:
      if (eventParam >= 2) {
          lcdMessage("release!");
      }
      else {
          lcdMessage("powering down");
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

    display_mode = MODE_CONNECTING;

    setupPeripherals();

    // if button held then we can shut down
    button.serviceEvents();
    while (button.isPressed()) {
      button.serviceEvents();
    }
    button.serviceEvents();
}

void loop()
{
    button.serviceEvents();

    if ( serverConnected == false ) {
      serverConnected = bleConnectToServer();
      display_mode = MODE_CONNECTING;
    }
    else {
      display_mode = updateDisplay(display_mode); // returns new mode
    }

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
