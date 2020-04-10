#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
RH_ASK driver;


void setup() {
  lcd.begin(16, 2);
      Serial.begin(9600); // Debugging only
    if (!driver.init())
         Serial.println("init failed");
}


void loop() {
     uint8_t buf[12];
    uint8_t buflen = sizeof(buf);
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      int i;
      // Message with a good checksum received, dump it.
      Serial.print("Message: ");
      Serial.println((char*)buf);         
    }
}
