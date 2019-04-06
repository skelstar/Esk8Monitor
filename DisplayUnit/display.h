#include <U8g2lib.h>

//https://github.com/olikraus/u8g2/wiki/fntgrpiconic#open_iconic_arrow_2x2
U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/14, /* dc=*/27, /* reset=*/33);

bool updateNow;
// u8g2.setFont(u8g2_font_tenfatguys_tf);
// u8g2.setFont(u8g2_font_tenthinguys_tf);

#define FONT_SIZE_SMALL u8g2_font_profont12_tr
#define FONT_SIZE_MED   u8g2_font_profont17_tr
#define FONT_SIZE_MED_LINE_HEIGHT 16
#define FONT_SIZE_MED_LINE_1 0
#define FONT_SIZE_MED_LINE_2 FONT_SIZE_MED_LINE_HEIGHT
#define FONT_SIZE_MED_LINE_3 FONT_SIZE_MED_LINE_HEIGHT * 2
#define FONT_SIZE_MED_LINE_4 FONT_SIZE_MED_LINE_HEIGHT * 3


void lcdNumber(char *number)
{
  u8g2.setFontPosCenter();
  // u8g2.setFont(u8g2_font_inr24_mn);    // numbers only
  u8g2.setFont(u8g2_font_inr24_mf); // full
  int width = u8g2.getStrWidth(number);
  u8g2.drawStr(128 / 2 - width / 2, 64 / 2, number);
}
//--------------------------------------------------------------------------------
#define BAR_GRAPH_THICKNESS 5
void lcdBarGraph(float percentage)
{
  u8g2.setDrawColor(1);
  float x2 = 128.0 * percentage;
  u8g2.drawBox(0, 64 - BAR_GRAPH_THICKNESS, x2, BAR_GRAPH_THICKNESS);
}
//--------------------------------------------------------------------------------
void lcdMotorCurrent(float current)
{
  u8g2.clearBuffer();
  char buff2[8];
  char buff[8];                                // Buffer big enough for 7-character float
  dtostrf(vescdata.motorCurrent, 2, 1, buff); // Leave room for too large numbers!
  sprintf(buff2, "%sA", buff);
  lcdNumber(buff2);
  if (vescdata.motorCurrent > 0)
  {
    lcdBarGraph(vescdata.motorCurrent / 40.0);
  }
  u8g2.sendBuffer();
}
//--------------------------------------------------------------------------------
void lcd_medium_float_text(
    uint8_t x, 
    uint8_t y, 
    char* label, 
    char* paramtext, 
    float value) {
  u8g2.setFontPosTop();
  char buff2[8];
  char buff[8];                                // Buffer big enough for 7-character float
  dtostrf(value, 5, 1, buff);
  sprintf(buff2, paramtext, buff);
  u8g2.setFont( FONT_SIZE_MED ); // full
  int width = u8g2.getStrWidth(buff2);
  u8g2.drawStr(x, y, label);
  u8g2.drawStr(128-width, y, buff2);
}

void lcd_medium_char_text(
    uint8_t x, 
    uint8_t y, 
    char* label, 
    char* value) {
  u8g2.setFontPosTop();
  char buff2[8];
  sprintf(buff2, "%s", value);
  u8g2.setFont( FONT_SIZE_MED ); // full
  int width = u8g2.getStrWidth(buff2);
  u8g2.drawStr(x, y, label);
  u8g2.drawStr(128-width, y, buff2);
}

void lcdPage2(float ampHours, float totalAmpHours, float odo) {
  u8g2.clearBuffer();
  lcd_medium_float_text(0, FONT_SIZE_MED_LINE_1, "Trip", "%sAh", ampHours);
  lcd_medium_float_text(0, FONT_SIZE_MED_LINE_2, "Total", "%sAh", totalAmpHours);
  lcd_medium_float_text(0, FONT_SIZE_MED_LINE_3, "Trip", "%skm", odo);
  // lcd_medium_float_text(0, FONT_SIZE_MED_LINE_4, "Test2", "", 0.0);
  u8g2.sendBuffer();
}

//--------------------------------------------------------------------------------
void lcdMessage(char *message)
{
  u8g2.clearBuffer();
  u8g2.setFontPosCenter(); // vertical center
  u8g2.setFont(u8g2_font_tenthinnerguys_tf);
  int width = u8g2.getStrWidth(message);
  u8g2.drawStr(128 / 2 - width / 2, 64 / 2, message);
  u8g2.sendBuffer();
}
//--------------------------------------------------------------------------------
#define BATTERY_WIDTH 100
#define BATTERY_HEIGHT 50
#define BORDER_SIZE 6
#define KNOB_HEIGHT 20

void drawBattery(int percent)
{
  u8g2.clearBuffer();
  int outsideX = (128 - (BATTERY_WIDTH + BORDER_SIZE)) / 2; // includes batt knob
  int outsideY = (64 - BATTERY_HEIGHT) / 2;
  u8g2.drawBox(outsideX, outsideY, BATTERY_WIDTH, BATTERY_HEIGHT);
  u8g2.drawBox(
      outsideX + BATTERY_WIDTH,
      outsideY + (BATTERY_HEIGHT - KNOB_HEIGHT) / 2,
      BORDER_SIZE,
      KNOB_HEIGHT); // knob
  u8g2.setDrawColor(0);
  u8g2.drawBox(
      outsideX + BORDER_SIZE,
      outsideY + BORDER_SIZE,
      BATTERY_WIDTH - BORDER_SIZE * 2,
      BATTERY_HEIGHT - BORDER_SIZE * 2);
  u8g2.setDrawColor(1);
  u8g2.drawBox(
      outsideX + BORDER_SIZE * 2,
      outsideY + BORDER_SIZE * 2,
      (BATTERY_WIDTH - BORDER_SIZE * 4) * percent / 100,
      BATTERY_HEIGHT - BORDER_SIZE * 4);
  u8g2.sendBuffer();
}
//--------------------------------------------------------------------------------
bool display_needs_to_update(uint8_t mode)
{
  if (updateNow) {
    updateNow = false;
    return true;
  }
  bool changed = false;
  switch ( mode ) {
    case MODE_BATTERY_VOLTAGE:
      changed = vescdata.batteryVoltage != oldvescdata.batteryVoltage;
      break;
    case MODE_MOTOR_CURRENT:
      changed = vescdata.motorCurrent != oldvescdata.motorCurrent;
      break;
    case MODE_AMP_HOURS:
      changed = vescdata.ampHours != oldvescdata.ampHours;
      break;
    default:
      changed = mode == MODE_CONNECTING || mode == MODE_CONNECTED;
  }
  oldvescdata = vescdata;
  return changed;
}
//--------------------------------------------------------------------------------
uint8_t updateDisplay(uint8_t mode)
{
  if ( display_needs_to_update(mode) )
  {
    switch (mode)
    {
      case MODE_CONNECTING:
        lcdMessage("connecting");
        return MODE_CONNECTED;

      case MODE_CONNECTED:
        lcdMessage("connected");
        delay(500);
        updateNow = true;
        return MODE_BATTERY_VOLTAGE;

      case MODE_BATTERY_VOLTAGE:
        drawBattery(getBatteryPercentage(vescdata.batteryVoltage));
        break;

      case MODE_MOTOR_CURRENT:
        Serial.printf("data_changed (MODE_MOTOR_CURRENT) \n");
        lcdMotorCurrent(vescdata.motorCurrent);
        break;

      case MODE_AMP_HOURS:
        lcdPage2(vescdata.ampHours, vescdata.totalAmpHours, vescdata.odometer);
        break;
      default:
        return mode;
        break;
      }
  }
  return mode;
}
//--------------------------------------------------------------------------------
