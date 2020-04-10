/*
  Simple example for receiving
  
  https://github.com/sui77/rc-switch/
*/


#include <DS3231.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
DS3231  rtc(SDA, SCL);


void setup() {
  lcd.begin(16, 2);

}

void loop() {      
 lcd.setCursor(0,0);
 lcd.print("Time:  ");
 lcd.print(rtc.getTimeStr());
 
 lcd.setCursor(0,1);
 lcd.print("Date: ");
 lcd.print(rtc.getDateStr());
 delay(1000);

}
