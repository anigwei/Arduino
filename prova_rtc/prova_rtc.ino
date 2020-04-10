
#include <DS3231.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
DS3231  rtc(SDA, SCL);
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status

void setup() {
  lcd.begin(16, 2);
 rtc.begin(); // Initialize the rtc object
  pinMode(2, INPUT);     // declare sensor as input
 
  
  // The following lines can be uncommented to set the date and time
  //rtc.setDOW(TUESDAY);     // Set Day-of-Week to SUNDAY
  //rtc.setTime(20,59, 10);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(24, 4, 2018);   // Set the date to January 1st, 2014
}

void loop() {      
 
 lcd.setCursor(0,0);
 lcd.print(rtc.getTimeStr());
 lcd.print("@");
 lcd.print(rtc.getDateStr());
  
  val = digitalRead(2);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
     lcd.setCursor(0,1);
     lcd.print("Hola!");
     if (pirState == LOW) {
      pirState = HIGH;
    }
  } else {
     lcd.setCursor(0,1);
     lcd.print("-----");
    if (pirState == HIGH){
      pirState = LOW;
    }
  }

 delay(1000);

}
