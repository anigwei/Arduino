#include <dht.h>
#include <LiquidCrystal.h>


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

dht DHT;
#define DHT11_PIN 2
int roomTempCMP;

void setup(){
  lcd.begin(16, 2);
  lcd.print("Hola!");
 delay(500);
}

void loop()
{
  int chk = DHT.read11(DHT11_PIN);
  int roomTemp;
  
  roomTemp = int(DHT.temperature);
  

  
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(roomTemp);
  lcd.print((char)223);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Humitat: ");
  lcd.print(int(DHT.humidity));
  lcd.print("% ");

  delay(2000);
}
