#include <U8g2lib.h>

//https://github.com/olikraus/u8g2/wiki/fntgrpiconic#open_iconic_arrow_2x2
U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ 14, /* dc=*/ 27, /* reset=*/ 33);


// u8g2.setFont(u8g2_font_tenfatguys_tf);
// u8g2.setFont(u8g2_font_tenthinguys_tf);

void lcdNumber(char* number) {
    u8g2.setFontPosCenter();
    // u8g2.setFont(u8g2_font_inr24_mn);    // numbers only
    u8g2.setFont(u8g2_font_inr24_mf);       // full
    int width = u8g2.getStrWidth(number);
    u8g2.drawStr(128/2 - width/2, 64/2, number);
}

#define BAR_GRAPH_THICKNESS 5
void lcdBarGraph(float percentage) {
    u8g2.setDrawColor(1);
    float x2 = 128.0 * percentage;
    u8g2.drawBox(0, 64-BAR_GRAPH_THICKNESS, x2, BAR_GRAPH_THICKNESS);
}

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