/*
Stick test
    hardware: M5Stack Stick
  
  please install the U8g2 library first ...
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <myPushButton.h>
#include "BLEDevice.h"
#include <driver/adc.h>

// static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
// static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// static BLEUUID    CHARACTERISTIC_UUID("0000ffe1-0000-1000-8000-00805f9b34fb");
// static BLEUUID    CHARACTERISTIC_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BATTERY_VOLTAGE_CUTOFF_START    37.4
#define BATTERY_VOLTAGE_CUTOFF_END      34.1
#define BATTERY_VOLTAGE_FULL            4.2 * 11

#define LED_ON HIGH
#define LED_OFF LOW

/* ---------------------------------------------- */
#define MODE_DISPLAY_SLEEPING   0
#define MODE_DISPLAY_TO_SLEEP   1
#define MODE_WAKE_DISPLAY       2
#define MODE_BATTERY_VOLTAGE    3
#define MODE_MOTOR_CURRENT      4

uint8_t display_mode = MODE_BATTERY_VOLTAGE;

struct STICK_DATA {
	float batteryVoltage;
	float motorCurrent;
	bool moving;
	bool vescOnline;
};
STICK_DATA stickdata;

int currentLocalBatteryCharge = 0;
/* ---------------------------------------------- */
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

#define LedPin 19
#define IrPin 17
#define BuzzerPin 26
#define BtnPin 35
/* ---------------------------------------------- */
// U8X8_SH1107_64X128_4W_HW_SPI u8x8(14, /* dc=*/ 27, /* reset=*/ 33);
U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ 14, /* dc=*/ 27, /* reset=*/ 33);

//https://github.com/olikraus/u8g2/wiki/fntgrpiconic#open_iconic_arrow_2x2

bool mpu9250_exis = false;
void mpu9250_read() {
    // uint8_t data = 0;
    // Wire.beginTransmission(0x68);         
    // Wire.write(0x75);                     
    // Wire.endTransmission(true);
    // Wire.requestFrom(0x68, 1);  
    // data = Wire.read();                   

    // Serial.print("mpu9250 addr: ");
    // Serial.println(data, HEX);
    // if(data == 0x71) {
    //     mpu9250_exis = true;
    // }
}


#define PULLUP		true
#define OFFSTATE	HIGH

void listener_Button(int eventCode, int eventPin, int eventParam);
myPushButton button(BtnPin, PULLUP, OFFSTATE, listener_Button);
void listener_Button(int eventCode, int eventPin, int eventParam) {
    
	switch (eventCode) {
		case button.EV_BUTTON_PRESSED:
			Serial.println("EV_BUTTON_PRESSED");
            //sendToMaster();
			break;
		case button.EV_RELEASED:
			Serial.println("EV_RELEASED");
            if (eventParam >= 2) {
                deepSleep();
            }            
            else if (display_mode == MODE_BATTERY_VOLTAGE) {
                display_mode = MODE_MOTOR_CURRENT;
            }
            else if (display_mode == MODE_MOTOR_CURRENT) {
                display_mode = MODE_BATTERY_VOLTAGE;
            }

			break;
		case button.EV_DOUBLETAP:
			Serial.println("EV_DOUBLETAP");
			break;
		case button.EV_HELD_SECONDS:
            if (eventParam >= 2) {
                lcdMessage("release!");
            }
            else {
                lcdMessage("powering down");
            }
			Serial.printf("EV_HELD_SECONDS %d\n", eventParam);
			break;
    }
}

//--    

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
      lcdMessage("connected! (1)");
  }

  void onDisconnect(BLEClient* pclient) {
    lcdMessage("disconnected!");
  }
};

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) {
			
	memcpy(&stickdata, pData, sizeof(stickdata));

	Serial.printf("Received batteryVoltage: %.1f \n", stickdata.batteryVoltage);
	Serial.printf("Received motor current: %.1f \n", stickdata.motorCurrent);

    if (button.isPressed() == false) {

        switch (display_mode) {

            case MODE_DISPLAY_SLEEPING:
                break;
            case MODE_DISPLAY_TO_SLEEP:
                u8g2.setPowerSave(1);
                display_mode = MODE_DISPLAY_SLEEPING;
                break;
            case MODE_WAKE_DISPLAY:
                u8g2.setPowerSave(0);
                display_mode = MODE_BATTERY_VOLTAGE;
                break;
            
            case MODE_BATTERY_VOLTAGE: {
                uint8_t ratio = ((stickdata.batteryVoltage - BATTERY_VOLTAGE_CUTOFF_END) / 
                    (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_CUTOFF_END)) * 100;
                if (ratio > 100) {
                    ratio = 100;
                }
                Serial.printf("ratio: %d\n", ratio);
                drawBattery(ratio);
                break;
            }
            case MODE_MOTOR_CURRENT:
                lcdMotorCurrent(stickdata.motorCurrent);
                break;
        }
    }
}

#define IP5306_ADDR           (117)
#define IP5306_REG_READ0      (0x70)
#define CHARGE_FULL_BIT       (   3)
#define BOOST_OUT_BIT         (0x02)
#define LIGHT_LOAD_BIT 		  (0x20)
#define CHARGE_OUT_BIT        (0x10)
#define BOOST_ENABLE_BIT      (0x80)

bool isCharging() {
	uint8_t data;
	Wire.beginTransmission(IP5306_ADDR);
	Wire.write(IP5306_REG_READ0);
	Wire.endTransmission(false);
	if (Wire.requestFrom(IP5306_ADDR, 1))
	{
		data = Wire.read();
		return data & (1 << CHARGE_FULL_BIT);
	}
	return false;
}

int8_t getBatteryLevel() {
    Wire.beginTransmission(0x75);
    Wire.write(0x78);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x75, 1)) {
        switch (Wire.read() & 0xF0) {
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default: return 0;
        }
    }
    return -1;
}

void setup() {
    // put your setup code here, to run once:
    Wire.begin(21, 22, 100000);
    // u8x8.begin();
    u8g2.begin();

    Serial.begin(9600);
    Serial.println("\nStarting Arduino BLE Client application...");

    setupPeripherals();

    // if button held then we can shut down
    button.serviceEvents();
    while (button.isPressed()) {
        button.serviceEvents();
    }
    button.serviceEvents();
    
    bleConnectToServer();
}

long now = 0;

void loop()
{
    button.serviceEvents();

    currentLocalBatteryCharge = getBatteryLevel();

    delay(200);
}

// u8g2.setFont(u8g2_font_tenfatguys_tf);
// u8g2.setFont(u8g2_font_tenthinguys_tf);

void lcdMotorCurrent(float current) {
    u8g2.clearBuffer();
    char buff2[8];
    char buff[8]; // Buffer big enough for 7-character float
    dtostrf(stickdata.motorCurrent, 2, 1, buff); // Leave room for too large numbers!
    sprintf(buff2, "%sA", buff);
    lcdNumber(buff2);
    if (stickdata.motorCurrent > 0) {
        lcdBarGraph(stickdata.motorCurrent/40.0);
    }
    u8g2.sendBuffer();
}

#define BAR_GRAPH_THICKNESS 5
void lcdBarGraph(float percentage) {
    u8g2.setDrawColor(1);
    float x2 = 128.0 * percentage;
    u8g2.drawBox(0, 64-BAR_GRAPH_THICKNESS, x2, BAR_GRAPH_THICKNESS);
}

void lcdNumber(char* number) {
    u8g2.setFontPosCenter();
    // u8g2.setFont(u8g2_font_inr24_mn);    // numbers only
    u8g2.setFont(u8g2_font_inr24_mf);       // full
    int width = u8g2.getStrWidth(number);
    u8g2.drawStr(128/2 - width/2, 64/2, number);
}

void lcdMessage(char* message) {
    u8g2.clearBuffer();
    u8g2.setFontPosCenter();    // vertical center
    u8g2.setFont(u8g2_font_tenthinnerguys_tf);
    int width = u8g2.getStrWidth(message);
    u8g2.drawStr(128/2 - width/2, 64/2, message);
    u8g2.sendBuffer();
}

void lcdWriteLine(uint8_t line, char* title, char* value) {
    // u8x8.drawString(0, line, title);
    // u8x8.drawString(0, line+1, value);
}

void buzzerBuzz() {
    for(int i=0;i<100;i++){
        digitalWrite(BuzzerPin,HIGH);
        delay(1);
        digitalWrite(BuzzerPin,LOW);
        delay(1);
    }
}

void setLed(int state) {

}

void sendToMaster() {
    Serial.printf("sending to master\n");
    char buff[6];
    ltoa(millis(), buff, 10);
    pRemoteCharacteristic->writeValue(buff, sizeof(buff));
    buzzerBuzz();
}

bool bleConnectToServer() {
    BLEDevice::init("");
    lcdMessage("searching");
    delay(1000);
    // pServerAddress = new BLEAddress("80:7d:3a:c4:50:9a");   
    pServerAddress = new BLEAddress("80:7d:3a:c5:6a:36");
    delay(1000);
    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    pClient->connect(*pServerAddress);
    Serial.println("Connected to server");
    lcdMessage("connected!");
    delay(500);
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic->canNotify()) {
        Serial.println("registering for notify");
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
}

void setupPeripherals() {
    pinMode(LedPin, OUTPUT);
    pinMode(IrPin, OUTPUT);
    pinMode(BuzzerPin, OUTPUT);
    // ledcSetup(1, 38000, 10);
    // ledcAttachPin(IrPin, 1);
    digitalWrite(LedPin, LED_ON);
    digitalWrite(BuzzerPin, LOW);
    // delay(1500);
    // mpu9250_read();
    u8g2.setFont(u8g2_font_4x6_tr);
}

#define BATTERY_WIDTH	100
#define BATTERY_HEIGHT	50
#define BORDER_SIZE 6
#define KNOB_HEIGHT 20

void drawBattery(int percent) {
    u8g2.clearBuffer();
    int outsideX = (128-(BATTERY_WIDTH+BORDER_SIZE))/2; // includes batt knob
    int outsideY = (64-BATTERY_HEIGHT)/2;
    u8g2.drawBox(outsideX, outsideY, BATTERY_WIDTH, BATTERY_HEIGHT);
    u8g2.drawBox(
        outsideX + BATTERY_WIDTH, 
        outsideY + (BATTERY_HEIGHT-KNOB_HEIGHT)/2, 
        BORDER_SIZE, 
        KNOB_HEIGHT
    );    // knob
    u8g2.setDrawColor(0);
    u8g2.drawBox(
        outsideX + BORDER_SIZE, 
        outsideY + BORDER_SIZE, 
        BATTERY_WIDTH - BORDER_SIZE*2, 
        BATTERY_HEIGHT - BORDER_SIZE*2
    );
    u8g2.setDrawColor(1);
    u8g2.drawBox(
        outsideX + BORDER_SIZE*2, 
        outsideY + BORDER_SIZE*2, 
        (BATTERY_WIDTH - BORDER_SIZE*4)*percent/100, 
        BATTERY_HEIGHT - BORDER_SIZE*4
    );
    u8g2.sendBuffer();
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


// #define 	BATTERY_WIDTH	20
// #define BATTERY_HEIGHT	8
// void oledSmallBattery(int percent, int x, int y) {
// 	int width = BATTERY_WIDTH;
// 	int height = BATTERY_HEIGHT;
	
// 	// outside box (solid)
// 	u8g2.setDrawColor(1);
// 	u8g2.drawBox(x+2, y, width, height); 
// 	// capacity
// 	u8g2.setDrawColor(0); 
// 	int cap = ((percent/100.0) * (width-2));
// 	int notcap = width - cap;
// 	u8g2.drawBox(x+2+1, y+1, notcap, height-2);
// 	// nipple
// 	u8g2.setDrawColor(1);
// 	u8g2.drawBox(x, y+2, 2, height - (2*2));
//     u8g2.sendBuffer();
// }