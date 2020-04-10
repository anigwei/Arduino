#include <LiquidCrystal.h>

                       
LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

                   
#define RELAY1  2
#define RELAY2  3
#define RELAY3  4
#define RELAY4  5
#define RELAY5  6
#define RELAY6  7
#define RELAY7  8
#define RELAY8  9
#define sw1  11
#define sw2  12
#define sensorLlum 1
#define sensorTemp 0
enum {INDETERMINADO,CERRADA,ABIERTA}estadoPersiana;


int comptador_persiana=0;
int llumActual;

void iniciDisplay(){
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("Timer Temp  Llum");
}


void baixarpersiana1() {
    lcd.setCursor(0, 1);
    lcd.print("Baixant Persiana1");
    digitalWrite(RELAY1, LOW);
    delay(100);
    digitalWrite(RELAY2, LOW);
    delay(6000);
    digitalWrite(RELAY2, HIGH);
    delay(100);
    digitalWrite(RELAY1, HIGH);
    iniciDisplay();
}
void pujarpersiana1() {
    lcd.setCursor(0, 1);
    lcd.print("Pujant Persiana1");
    digitalWrite(RELAY1, LOW);
    delay(100);
    digitalWrite(RELAY3, LOW);
    delay(6000);
    digitalWrite(RELAY3, HIGH);
    delay(100);
    digitalWrite(RELAY1, HIGH);
    iniciDisplay();
}

void setup()

{    
// Serial.begin(9600);
 enum {INDETERMINADO,CERRADA,ABIERTA}estadoPersiana;
 pinMode(RELAY1, OUTPUT);       
 pinMode(RELAY2, OUTPUT);
 pinMode(RELAY3, OUTPUT);
 pinMode(RELAY4, OUTPUT);
 pinMode(RELAY5, OUTPUT);
 pinMode(RELAY6, OUTPUT);
 pinMode(RELAY7, OUTPUT);
 pinMode(RELAY8, OUTPUT); 
 pinMode(13, OUTPUT);
 pinMode(sw1, INPUT_PULLUP);
 pinMode(sw2, INPUT_PULLUP);

//INICIALITZACIO SORTIDES per defecte
 digitalWrite(RELAY1,HIGH);
 digitalWrite(RELAY2,HIGH);
 digitalWrite(RELAY3,HIGH);
// digitalWrite(RELAY4,HIGH);
//digitalWrite(RELAY5,HIGH);
// digitalWrite(RELAY6,HIGH);
//digitalWrite(RELAY7,HIGH);
// digitalWrite(RELAY8,HIGH);


//LCD SETUP
 lcd.begin(16, 2);
 lcd.print("Benvinguts!!");
 delay(1000);
 iniciDisplay();

}

  int SwitchState1;             // current state of the switch
  int PreviousSwitchState1;     // previous state of the switch
  int RelayState1;              // current state of relay pin
  int SwitchState2;             // current state of the switch
  int PreviousSwitchState2;     // previous state of the switch
  int RelayState2;              // current state of relay pin
 
 
void loop()
  {

  //Temperatura
  int reading = analogRead(sensorTemp);  
  float voltage = reading * 5.0;
  voltage /= 1024.0; 
  float temperatureC = (voltage - 0.5) * 100 ; 

  //Llum
  llumActual = analogRead(sensorLlum);

  
  int sw_pujar1 = digitalRead(sw1);
  int sw_baixar1 = digitalRead(sw2);
 
 if (sw_pujar1 == LOW){
   pujarpersiana1();
 }
 
 if (sw_baixar1 == LOW){
    baixarpersiana1();
 }
 
 /*
 tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(100);
    noTone(buzzer);     // Stop sound...*/  
/*  SwitchState1 = switch1;
  RelayState1 = digitalRead(RELAY1);


   if (SwitchState1 != PreviousSwitchState1) {
    // if the state has changed, check change direction
    if (SwitchState1 == LOW){

      // if the current state is HIGH then the switch went from off to on
      // instigate a relay state change:
      if (RelayState1 == HIGH)
          digitalWrite(RELAY1, LOW);
      else
          digitalWrite(RELAY1, HIGH);
        }
    }
PreviousSwitchState1 = SwitchState1;    

  int switch2 = digitalRead(sw2);
  SwitchState2 = switch2;
  RelayState2 = digitalRead(RELAY2);

   if (SwitchState2 != PreviousSwitchState2) {
    // if the state has changed, check change direction
    if (SwitchState2 == LOW){

      // if the current state is HIGH then the switch went from off to on
      // instigate a relay state change:
      if (RelayState2 == HIGH)
          digitalWrite(RELAY2, LOW);
      else
          digitalWrite(RELAY2, HIGH);
        }
    }
PreviousSwitchState2 = SwitchState2;  
*/

delay(50);
 lcd.setCursor(0, 1);
 lcd.print(millis()/1000);
 lcd.setCursor(6, 1);
 lcd.print(temperatureC);
 lcd.setCursor(12,1);
 lcd.print(llumActual);
}
