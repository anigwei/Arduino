#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

#define RELAY1  2
#define RELAY2  3
#define RELAY3  4
#define RELAY4  5
#define RELAY5  6
#define RELAY6  7
#define RELAY7  8
#define RELAY8  9
#define SW1  11
#define SW2  12
#define SW3  22
#define SW4  23
#define sensorLlum 1
#define sensorTemp 0
#define TEMPSPERSIANA1 5555

//Sensor Llum
int llumv;
int llum;

bool rele1;

//XARXA I MQTT

// Adreca MAC
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//IP Tarja
IPAddress ip(192, 168, 1, 15);
// IP del servidor
IPAddress mqtt_server(192, 168, 1, 9);

// Topic con el que trabajamos
const char* topicName = "test";

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

void callback(char* topic, byte* payload, unsigned int length) {
   char message_buff[100];
   int i = 0;

  //Serial.println("Message arrived:  topic: " + String(topic));
  //Serial.println("Length: " + String(length,DEC));
  
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  String msgString = String(message_buff);
  
  ///Serial.println("Payload: " + msgString);
  
  if (msgString.equals("true")) {
    rele1 = true;
  } else if (msgString.equals("false")) {
    rele1 = false;
  }
}


// DEFINICIO CARACTERS ESPECIALS LCD
/*byte graus[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000
};*/

byte fletxaAmunt[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};

byte fletxaAvall[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00100
};
// FI DEFINICIO CARACTERS ESPECIALS



int comptadorPersiana1=0;
long previousMillis = 0;

/*
 * estatPersiana:
 * 0: desconegut
 * 1: baixant
 * 2: pujant
 * 3: dalt
 * 4: baix
 */
 
int estatPersiana1=0; 
int percentPersiana1=0;

void iniciDisplay(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R1:");
  lcd.setCursor(7,0);
  lcd.write("P1:??");

 if (digitalRead(RELAY1) == HIGH)
      {
        lcd.setCursor(3, 0);
        lcd.print("OFF");
      }
      else 
      {
        lcd.setCursor(3, 0);
        lcd.print("ON ");
      }  
}

void baixarpersiana1() {
  lcd.setCursor(0, 1);
  lcd.print("Baixant Persiana1");
  digitalWrite(RELAY2, LOW);
  delay(100);
  digitalWrite(RELAY3, HIGH);
  delay(6000);
  //TORNA PER DEFECTE 
  digitalWrite(RELAY3, HIGH);
  delay(100);
  digitalWrite(RELAY2, HIGH);
  iniciDisplay();
}

void baixarpersianatotal1() {
  unsigned long currentMillis = millis();
  if ((estatPersiana1 == 3) || (estatPersiana1 == 0) || (percentPersiana1 != 0))
  { 
    previousMillis = currentMillis;
    lcd.clear();
    while (currentMillis - previousMillis < TEMPSPERSIANA1)
    { 
     currentMillis = millis(); 
     percentPersiana1 = ((currentMillis - previousMillis) * 100) / TEMPSPERSIANA1;
     lcd.setCursor(0, 1);
     lcd.print("Baixant Persiana1");
     lcd.setCursor(0, 0);
     lcd.print("T:");
     lcd.print(currentMillis - previousMillis);
     lcd.print("/");
     lcd.print(TEMPSPERSIANA1);
     lcd.print("; ");
     lcd.print(percentPersiana1);
     lcd.print("%");
     digitalWrite(RELAY2, LOW);
     delay(100);
     digitalWrite(RELAY3, HIGH);
     // Si es prem altre cop boto de pujar mentre s'esta pujant: surt del loop i atura  
     estatPersiana1 = 1;
     if (digitalRead(SW4) == LOW)
     {
        break;
     }
  
    }
     
     // Para motor
     digitalWrite(RELAY3, HIGH);
     delay(100);
     digitalWrite(RELAY2, HIGH);
     iniciDisplay();
     estatPersiana1 = 4;
     lcd.setCursor(3,1);
     lcd.write((uint8_t)2);
     lcd.setCursor(10,0);
     lcd.write((uint8_t)1);
     lcd.print("-");
     lcd.print(percentPersiana1);
     lcd.print("%");
  }
}

void pujarpersianatotal1() {
  unsigned long currentMillis = millis();
  if ((estatPersiana1 == 4) || (estatPersiana1 == 0) || (percentPersiana1 != 100))
  { 
    previousMillis = currentMillis;
    lcd.clear();
    while (currentMillis - previousMillis < TEMPSPERSIANA1)
    { 
     currentMillis = millis();   
     percentPersiana1 = ((currentMillis - previousMillis) * 100) / TEMPSPERSIANA1;
     lcd.setCursor(0, 1);
     lcd.print("Pujant Persiana1");
     lcd.setCursor(0, 0);
     lcd.print("T:");
     lcd.print(currentMillis - previousMillis);
     lcd.print("/");
     lcd.print(TEMPSPERSIANA1);
     lcd.print("; ");
     lcd.print(percentPersiana1);
     lcd.print("%");
     digitalWrite(RELAY2, LOW);
     delay(100);
     digitalWrite(RELAY3, LOW);
     estatPersiana1 = 2;
     // Si es prem altre cop boto de pujar mentre s'esta pujant: surt del loop i atura  
     if (digitalRead(SW3) == LOW)
     {
        break;
     }
    }
     
     // Para motor
     digitalWrite(RELAY3, HIGH);
     delay(100);
     digitalWrite(RELAY2, HIGH);
     iniciDisplay();
     estatPersiana1 = 3;
     lcd.setCursor(10,0);
     lcd.write((uint8_t)1);
     lcd.print("-");
     lcd.print(percentPersiana1);
     lcd.print("%");
  }
}

void pujarpersiana1() {
  lcd.setCursor(0, 1);
  lcd.print("Pujant Persiana1");
  digitalWrite(RELAY2, LOW);
  delay(150);
  digitalWrite(RELAY3, LOW); 
  //iniciDisplay();
}


void setup()
{      
  //Serial.begin(9600);
  pinMode(RELAY1, OUTPUT);       
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);
  pinMode(RELAY7, OUTPUT);
  pinMode(RELAY8, OUTPUT); 
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

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
  lcd.setCursor(0,1);
  lcd.print("Connectant..");  

  //lcd.createChar(0, graus);
  lcd.createChar(1, fletxaAmunt);
  lcd.createChar(2, fletxaAvall);

//MQTT I ETH INICIALITZACIO
  //Ethernet.begin(mac, ip);
 // setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {
    lcd.setCursor(0,1);
    lcd.print("Error IP");    
    for (;;);
  }
  lcd.setCursor(0,1);
  lcd.print(Ethernet.localIP());
  delay(2200);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);

  
  if (mqttClient.connect("Arduino","andreu","penis")) {
    mqttClient.subscribe("romani/test/rele1");
  } else {
    // connection failed
    // mqttClient.state() will provide more information
    // on why it failed.
}


  iniciDisplay();
}

//Variables DETECCIO EDGE
int SwitchState1;             // current state of the switch
int PreviousSwitchState1;     // previous state of the switch
int RelayState1;              // current state of relay pin
//

//Variables DETECCIO POLSACIONS
int current;         // Current state of the button
// (LOW is pressed b/c i'm using the pullup resistors)
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed 
//

//Parpadeig LED INTEGRAT
void ledblink(int times, int lengthms){
  for (int x=0; x<times;x++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay (lengthms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(lengthms);
  }
}


void loop()
{ 
   mqttClient.loop();

/*
  // INICI DETECCIO POLSACIONS ----------------------------------------------------------------------------------------------
  current = digitalRead(SW3);

  // if the button state changes to pressed, remember the start time 
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  // This if statement is a basic debouncing tool, the button must be pushed for at least
  // 100 milliseconds in a row for it to be considered as a push.
  if (millis_held > 50) {

    if (current == LOW && secs_held > prev_secs_held) {
      ledblink(1, 50); // Each second the button is held blink the indicator led
    }

    // check if the button was released since we last checked
    if (current == HIGH && previous == LOW) {
      // HERE YOU WOULD ADD VARIOUS ACTIONS AND TIMES FOR YOUR OWN CODE
      // ===============================================================================

      // Button pressed for less than 1 second
      if (secs_held <= 0) {
        //pujarpersiana1();      
        lcd.setCursor(0, 0);
        lcd.print("Pulsacio curta!");
        delay(555);
      }

      if (secs_held >= 1 && secs_held < 2) {
        lcd.setCursor(0, 0);
        lcd.print("Pulsacio mitjana!");
        delay(122);
        pujarpersianatotal1(); 
      }

      if (secs_held >= 2) {
        lcd.setCursor(0, 0);
        lcd.print("Pulsacio llarga!");
        lcd.setCursor(0, 1);
        lcd.print(secs_held);
        lcd.setCursor(4, 1);
        lcd.print("segons...");
        delay(555);
      }
      // ===============================================================================
        iniciDisplay();

    }
  }

  previous = current;
  prev_secs_held = secs_held;


  // FI DETECCIO POLSACIONS  ----------------------------------------------------------------------------------------------

*/

  // Deteccio EDGE POLSADOR 1

  SwitchState1 = digitalRead(SW1);
  RelayState1 = digitalRead(RELAY1);

  if (SwitchState1 != PreviousSwitchState1) {
    // if the state has changed, check change direction
    if (SwitchState1 == LOW){
      // if the current state is HIGH then the switch went from off to on
      // instigate a relay state change:
      if (RelayState1 == HIGH)
      {
        mqttClient.publish("romani/test/rele1", "true");
        digitalWrite(RELAY1, LOW);
        lcd.setCursor(3, 0);
        lcd.print("ON ");
        rele1 = true;
       }
      else 
      {
        mqttClient.publish("romani/test/rele1", "false");
        digitalWrite(RELAY1, HIGH);
        lcd.setCursor(3, 0);
        lcd.print("OFF ");
        rele1 = false;
       }  
    }
  }
  PreviousSwitchState1 = SwitchState1;

 if (rele1 == true)
 {
        digitalWrite(RELAY1, LOW);
        lcd.setCursor(3, 0);
        lcd.print("ON ");
        rele1 = true;
 }
 if (rele1 == false)
 {
        digitalWrite(RELAY1, HIGH);
        lcd.setCursor(3, 0);
        lcd.print("OFF ");
        rele1 = false;
 }

  // FI DETECCIO EDGE POLSADOR 1
  
    if (digitalRead(SW4) == LOW)
    {
      baixarpersianatotal1();
     }
  
    if (digitalRead(SW3) == LOW)
    {
      pujarpersianatotal1();
     }

    delay(50);

  lcd.setCursor(0, 1);
 //Llum
  int llum = analogRead(sensorLlum);   
  llumv = ((long)llum*1000*10)/((long)15*10*(1024-llum));   
  lcd.print(llum);
//const long A = 1000;     //Resistencia en oscuridad en KΩ
//const int B = 15;        //Resistencia a la luz (10 Lux) en KΩ
//const int Rc = 10;       //Resistencia calibracion en KΩ

  lcd.setCursor(9,1);
  lcd.print(millis()/1000);


}

