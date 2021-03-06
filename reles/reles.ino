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

void setup()

{    
// Serial.begin(9600);
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

 digitalWrite(RELAY1,HIGH);
 digitalWrite(RELAY2,HIGH);
 digitalWrite(RELAY3,HIGH);
 //digitalWrite(RELAY4,HIGH);
 //digitalWrite(RELAY5,HIGH);
 //digitalWrite(RELAY6,HIGH);
 //digitalWrite(RELAY7,HIGH);
 //digitalWrite(RELAY8,HIGH);
//LCD
 lcd.begin(16, 2);
 lcd.print("Hola Gemmeta!!");

}

  int SwitchState1;             // current state of the switch
  int PreviousSwitchState1;     // previous state of the switch
  int RelayState1;              // current state of relay pin
  int SwitchState2;             // current state of the switch
  int PreviousSwitchState2;     // previous state of the switch
  int RelayState2;              // current state of relay pin
 
 
void loop()
  {
  
  int switch1 = digitalRead(sw1);
  SwitchState1 = switch1;
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

delay(90);

 lcd.setCursor(0, 1);
 // print the number of seconds since reset:
 lcd.print(millis()/1000);
   
/*    
   if (switch1 == HIGH) {
    digitalWrite(13, LOW);
    digitalWrite(RELAY1, HIGH);

  } else {
    digitalWrite(13, HIGH);
    digitalWrite(RELAY1, LOW);

  }
 */   
}
