#include <U8g2lib.h>
#include "Arduino.h"

#define oled_scl 18
#define oled_sda 17
#define oled_rst 21


void setupOLED(void);
void printOLED(int x,int y, String text);

