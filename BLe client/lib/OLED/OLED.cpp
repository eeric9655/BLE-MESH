#include "OLED.h"
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/oled_scl, /* data=*/oled_sda, /* reset=*/oled_rst);

void printOLED(int x,int y, String text){
  u8g2.firstPage();
    do {      
      u8g2.drawStr(x, y, text.c_str());
    }while (u8g2.nextPage());
}
void setupOLED(void){
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_ncenR08_tr);
}
