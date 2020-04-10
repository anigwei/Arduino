#include <SPI.h>
#include <Ethernet.h>
//#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/
//#include <dht.h>

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
//LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

//!!!!MOLT IMPORTANT: En activar una persiana, primer activar el Rele SENTIT i 200-300ms després Rele ACTIVAR!!!

//P= Rele Power  (Placa Reles Dreta)
//S= Rele Sentit (Placa Reles Esquerra)

#define RELAY1P 40
#define RELAY1S 22

#define RELAY2P 42
#define RELAY2S 24

#define RELAY3P 44
#define RELAY3S 26

#define RELAY4P 46
#define RELAY4S 28

#define RELAY5P 48
#define RELAY5S 30

#define RELAY6P 32
#define RELAY7P 34

#define SW1  6
#define SW2  7

//#define sensorLlum 1
//#define sensorTemp 11

#define TEMPSPUJARPERSIANA1 12290
#define TEMPSPUJARPERSIANA2 12290
#define TEMPSPUJARPERSIANA3 8850
#define TEMPSPUJARPERSIANA4 12999
#define TEMPSPUJARPERSIANA5 18000

#define TEMPSBAIXARPERSIANA1 12290
#define TEMPSBAIXARPERSIANA2 12290
#define TEMPSBAIXARPERSIANA3 9090
#define TEMPSBAIXARPERSIANA4 12290
#define TEMPSBAIXARPERSIANA5 16290


//Estat del rele1
bool rele1;
bool rele2;

// S'enten percentPersiana 100% DALT de tot i 0% BAIX de tot
int percentPersiana1 = 0;
int percentPersiana2 = 0;
int percentPersiana3 = 0;
int percentPersiana4 = 0;
int percentPersiana5 = 0;

//#define DHTTYPE DHT11
//dht DHT;


//XARXA I MQTT

// Adreca MAC
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress mqtt_server(192, 168, 1, 9);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

String pubString; // Cadena que s'utiltizara per publicar a MQTT (i convertir a char).
char pubStringChar[5];
unsigned long last_mqtt_reconnect_attempt = 0;

void callback(char* topic, byte* payload, unsigned int length) {
      char message_buff[100];
      int i = 0;
      //Serial.println("Message arrived:  topic: " + String(topic));
      //Serial.println("Length: " + String(length,DEC));
    
      // create character buffer with ending null terminator (string)
      for (i = 0; i < length; i++) {
        message_buff[i] = payload[i];
      }
      message_buff[i] = '\0';
      String msgString = String(message_buff);

    if (strcmp(topic,"romani/watchdog1")==0) {  // Si TOPIC es el del WATCHDOG
      if (msgString.equals("true")) {
           mqttClient.publish("romani/watchdog2", "true", true); 
      } else if (msgString.equals("false")) {
           mqttClient.publish("romani/watchdog2", "false", true);
      }  
    }

    
    if (strcmp(topic,"romani/test/rele1")==0) {  // Si TOPIC es el del RELE1
      if (msgString.equals("true")) {
         rele1 = true;
      } else if (msgString.equals("false")) {
         rele1 = false;
      }  
    }
    
    if (strcmp(topic,"romani/test/rele2")==0) {  // Si TOPIC es el del RELE2
      if (msgString.equals("true")) {
         rele2 = true;
      } else if (msgString.equals("false")) {
         rele2 = false;
      }  
    }
    
    
    if (strcmp(topic,"romani/test/persiana1")==0) {  // Si TOPIC es el del PERSIANA1
      percentPersiana1 = msgString.toInt();
      
      
  //Serial.println(percentPersiana1);
      if (percentPersiana1 != 0)
      {
        accioPersiana1(percentPersiana1,1);
      }
    }
    if (strcmp(topic,"romani/test/persiana2")==0) {  // Si TOPIC es el del PERSIANA2
      percentPersiana2 = msgString.toInt();
      
      
  //Serial.println(percentPersiana2);
      if (percentPersiana2 != 0)
      {
        accioPersiana1(percentPersiana2,2);
      }
    }
    if (strcmp(topic,"romani/test/persiana3")==0) {  // Si TOPIC es el del PERSIANA3
      percentPersiana3 = msgString.toInt();
      
      
  //Serial.println(percentPersiana3);
      if (percentPersiana3 != 0)
      {
        accioPersiana1(percentPersiana3,3);
      }
    }
    if (strcmp(topic,"romani/test/persiana4")==0) {  // Si TOPIC es el del PERSIANA4
      percentPersiana4 = msgString.toInt();
      
      
  //Serial.println(percentPersiana4);
      if (percentPersiana4 != 0)
      {
        accioPersiana1(percentPersiana4,4);
      }
    }
    if (strcmp(topic,"romani/test/persiana5")==0) {  // Si TOPIC es el del PERSIANA5
      percentPersiana5 = msgString.toInt();
      
      
  //Serial.println(percentPersiana5);
      if (percentPersiana5 != 0)
      {
        accioPersiana1(percentPersiana5,5);
      }
    }

}

// FI XARXA I MQTT

long previousMillis = 0;

float calcularTempsvsPercent(float percent, int tempstotal){
  return ((tempstotal * percent) / 100);
}

void accioPersiana1(int percentPersiana, int quina) {
  int sentit = 0; // 0 = Indeterminat; 1 = Baixar; 2 = Pujar
  unsigned long currentMillis = millis();
  int tempsConsigna;
  int caseTempsPersiana;
  int ReleP;
  int ReleS;
   
//Determina si cal pujar o baixar en funció del percentatge rebut (Inferior a 50:BAIXA; Superior a 51:PUJA)
  if (percentPersiana <= 50) {
     sentit = 1; //BAIXA
     percentPersiana = percentPersiana * 2;
  } else 
  {
     sentit = 2; //PUJA
     percentPersiana = (percentPersiana - 50) * 2;
  }
  switch (quina) {
    case 1:
      caseTempsPersiana = TEMPSPUJARPERSIANA1;
      ReleP = RELAY1P;
      ReleS = RELAY1S;
      break;
    case 2:
      caseTempsPersiana = TEMPSPUJARPERSIANA2;
      ReleP = RELAY2P;
      ReleS = RELAY2S;
      break;
    case 3:
      caseTempsPersiana = TEMPSPUJARPERSIANA3;
      ReleP = RELAY3P;
      ReleS = RELAY3S;
      break;
    case 4:
      caseTempsPersiana = TEMPSPUJARPERSIANA4;
      ReleP = RELAY4P;
      ReleS = RELAY4S;
      break;
    case 5:
      caseTempsPersiana = TEMPSPUJARPERSIANA5;
      ReleP = RELAY5P;
      ReleS = RELAY5S;
      break;
  }
    
    
    previousMillis = currentMillis;
    tempsConsigna = calcularTempsvsPercent(percentPersiana, caseTempsPersiana);
    while (currentMillis - previousMillis < tempsConsigna)
    //while (percentPersiana < 100)
    {
      currentMillis = millis();
      percentPersiana = ((currentMillis - previousMillis) * 100) / caseTempsPersiana;
      if (sentit == 1)
      {
        digitalWrite(ReleS, LOW);
        delay(250);
        digitalWrite(ReleP, LOW);
      }
      else
      {
        digitalWrite(ReleS, HIGH);
        delay(250);
        digitalWrite(ReleP, LOW);
       }
    }
    // Para motor
    digitalWrite(ReleP, HIGH);
    delay(400);
    digitalWrite(ReleS, HIGH);
}

void inicialitzaXarxa()
{
  //MQTT I ETH INICIALITZACIO
  //Ethernet.begin(mac, ip);
  // setup ethernet communication using DHCP
  Serial.println("Connectant ethernet...");

  if (Ethernet.begin(mac) == 0) {  
     Serial.print("PROBLEMA AMB ETHERNET!!!");
    delay(1000);
  }
  Serial.print("La IP es: ");
  Serial.println(Ethernet.localIP());
  delay(1200);
}

void inicialitzaMqtt()
{
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);

  //Connexio MQTT
  if (mqttClient.connect("Arduino", "andreu", "penis")) {
    mqttClient.subscribe("romani/test/rele1");
    mqttClient.subscribe("romani/test/rele2");
    mqttClient.subscribe("romani/test/persiana1");
    mqttClient.subscribe("romani/test/persiana2");
    mqttClient.subscribe("romani/test/persiana3");
    mqttClient.subscribe("romani/test/persiana4");
    mqttClient.subscribe("romani/test/persiana5");
    mqttClient.subscribe("romani/watchdog1");

  } else {
     //connection failed
     //mqttClient.state() will provide more information
    // on why it failed.
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(RELAY1P, OUTPUT);
  pinMode(RELAY1S, OUTPUT);
  pinMode(RELAY2P, OUTPUT);
  pinMode(RELAY2S, OUTPUT);
  pinMode(RELAY3P, OUTPUT);
  pinMode(RELAY3S, OUTPUT);
  pinMode(RELAY4P, OUTPUT);
  pinMode(RELAY4S, OUTPUT);
  pinMode(RELAY5P, OUTPUT);
  pinMode(RELAY5S, OUTPUT);
  pinMode(RELAY6P, OUTPUT);
  pinMode(RELAY7P, OUTPUT);
 
//  pinMode(SW1, INPUT_PULLUP);
//  pinMode(SW2, INPUT_PULLUP);
  //pinMode(SW3, INPUT_PULLUP);
  //pinMode(SW4, INPUT_PULLUP);
    
  //INICIALITZACIO SORTIDES per defecte
  digitalWrite(RELAY1P,HIGH);
  digitalWrite(RELAY1S,HIGH);
  digitalWrite(RELAY2P,HIGH);
  digitalWrite(RELAY2S,HIGH);
  digitalWrite(RELAY3P,HIGH);
  digitalWrite(RELAY3S,HIGH);
  digitalWrite(RELAY4P,HIGH);
  digitalWrite(RELAY4S,HIGH);
  digitalWrite(RELAY5P,HIGH);
  digitalWrite(RELAY5S,HIGH);
  digitalWrite(RELAY6P,HIGH);
  digitalWrite(RELAY7P,HIGH);
  
  inicialitzaXarxa();
  inicialitzaMqtt();
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

/*
//Parpadeig LED INTEGRAT
void ledblink(int times, int lengthms) {
  for (int x = 0; x < times; x++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay (lengthms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(lengthms);
  }
}
*/


void loop()
{
  if (!mqttClient.connected()) {
    Serial.println("Inici LOOP -> Reconnexio MQTT per desconnectat");
    unsigned long now = millis();
    if (now - last_mqtt_reconnect_attempt > 5000UL || last_mqtt_reconnect_attempt == 0) {
      Serial.println("Attempting to connect to MQTT");
      last_mqtt_reconnect_attempt = now;
      inicialitzaMqtt();
    }
    return;
  }

  if (!ethClient.connected()) {
    Serial.println();
    Serial.println("Ethernet desconnectat!!!");
    ethClient.stop();
    delay(1555);
    inicialitzaXarxa;
  }
  
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
  /*SwitchState1 = digitalRead(SW1);
  RelayState1 = digitalRead(RELAY6P);

  if (SwitchState1 != PreviousSwitchState1) {
    // if the state has changed, check change direction
    if (SwitchState1 == LOW) {
      // if the current state is HIGH then the switch went from off to on
      // instigate a relay state change:
      if (RelayState1 == HIGH)
      {
        mqttClient.publish("romani/test/rele1", "true", true); //Eardino l darrer true és per retenir el missatge
        digitalWrite(RELAY6P, LOW);
        rele1 = true;
      }
      else
      {
        mqttClient.publish("romani/test/rele1", "false", true); //El darrer true és per retenir el missatge
        digitalWrite(RELAY6P, HIGH);
        rele1 = false;
      }
    }
  }
  PreviousSwitchState1 = SwitchState1;
//FI Deteccio Edge Polsador 1
*/


  if (rele1 == true)
  {
    digitalWrite(RELAY6P, LOW);
  }
  if (rele1 == false)
  {
    digitalWrite(RELAY6P, HIGH);
  }


  if (rele2 == true)
  {
    digitalWrite(RELAY7P, LOW);
  }
  if (rele2 == false)
  {
    digitalWrite(RELAY7P, HIGH);
  }
 
  
// FI DETECCIO EDGE POLSADOR 1
  delay(100);
}
