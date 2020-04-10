#include <dht.h>
#include <DS3231.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

dht DHT;
#define DHT11_PIN 3

DS3231  rtc(SDA, SCL);


int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status


float calcDewpoint(float temp, float humi) {
  float k;
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}

double humidex(double tempC, double DewPoint)
{
  double e = 19.833625 - 5417.753 /(273.16 + DewPoint);
  double h = tempC + 3.3941 * exp(e) - 5.555;
  return h;
}


void setup(){
  lcd.begin(20, 4);
  lcd.print("   Benvinguts!");
  delay(500);
  rtc.begin(); // Initialize the rtc object
  pinMode(2, INPUT);     // declare sensor as input
  // The following lines can be uncommented to set the date and time
  //rtc.setDOW(TUESDAY);     // Set Day-of-Week to SUNDAY
  //rtc.setTime(20,59, 10);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(24, 4, 2018);   // Set the date to January 1st, 2014
}

void loop()
{
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(0,0); 
  lcd.print("T: ");
  lcd.print(int(DHT.temperature));
  lcd.print((char)223);
  lcd.print("C S:");
  lcd.print(humidex(DHT.temperature,calcDewpoint(DHT.temperature,DHT.humidity)));
  lcd.setCursor(0,1);
  lcd.print("Humitat R: ");
  lcd.print(int(DHT.humidity));
  lcd.print("% ");

 lcd.setCursor(0,3);
 lcd.print(rtc.getTimeStr());
 lcd.print("  ");
 lcd.print(rtc.getDateStr());
  
  val = digitalRead(2);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
     lcd.setCursor(0,2);
     lcd.print("Hola!");
     if (pirState == LOW) {
      pirState = HIGH;
    }
  } else {
     lcd.setCursor(0,2);
     lcd.print("-----");
    if (pirState == HIGH){
      pirState = LOW;
    }
  }


  delay(2000);
}
