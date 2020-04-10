//#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/
//#include <dht.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
//LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

//!!!!MOLT IMPORTANT: En activar una persiana, primer activar el Rele SENTIT i 200-300ms després Rele ACTIVAR!!!

//PLACA Reles1
#define RELAY1  2 //Test: Llum
#define RELAY2  3 //Test: Persiana1 Sentit
#define RELAY3  4 //Test: Persiana1 Activar
#define RELAY4  5 //Test: NC
#define RELAY5  6
#define RELAY6  7
#define RELAY7  8
#define RELAY8  9

//PLACA Reles2
#define RELAY9  2 
#define RELAY10  3 
#define RELAY11  4 
#define RELAY12  5 
#define RELAY13  6
#define RELAY14  7
#define RELAY15  8
#define RELAY16  9

#define SW1  6
#define SW2  7
#define SW3  8
#define SW4  9
#define SW5  NULL
#define SW6  NULL
#define SW7  NULL
#define SW8  NULL
#define SW9  NULL
#define SW10  NULL
#define SW11  NULL
#define SW12  NULL

//#define sensorLlum 1
//#define sensorTemp 11

#define TEMPSPUJARPERSIANA1 5555
#define TEMPSPUJARPERSIANA2 5555
#define TEMPSPUJARPERSIANA3 5555
#define TEMPSPUJARPERSIANA4 5555
#define TEMPSPUJARPERSIANA5 20000

#define TEMPSBAIXARPERSIANA1 5555
#define TEMPSBAIXARPERSIANA2 5555
#define TEMPSBAIXARPERSIANA3 5555
#define TEMPSBAIXARPERSIANA4 5555
#define TEMPSBAIXARPERSIANA5 20000


//Estat del rele1
bool rele1;
bool rele2;

//#define DHTTYPE DHT11
//dht DHT;


//XARXA I MQTT

// Adreca MAC
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IP del servidor
IPAddress mqtt_server(192, 168, 1, 9);

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

String pubString; // Cadena que s'utiltizara per publicar a MQTT (i convertir a char).
char pubStringChar[5];

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


  ///Serial.println("Payload: " + msgString);
}

// FI XARXA I MQTT


// DEFINICIO CARACTERS ESPECIALS LCD
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



long previousMillis = 0;

// S'enten percentPersiana 100% DALT de tot i 0% BAIX de tot
float percentPersiana1 = 0;
float percentPersiana2 = 0;
float percentPersiana3 = 0;
float percentPersiana4 = 0;
float percentPersiana5 = 0;

void iniciDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R1:");
  lcd.setCursor(7, 0);
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
  if (percentPersiana1 != 0)
  {
    previousMillis = currentMillis;
    lcd.clear();
    //while (currentMillis - previousMillis < TEMPSPUJARPERSIANA1)
    while (percentPersiana1 > 0)
    {
      currentMillis = millis();
      percentPersiana1 = ((currentMillis - previousMillis) * 100) / TEMPSBAIXARPERSIANA1;
      lcd.setCursor(0, 1);
      lcd.print("Baixant Persiana1");
      lcd.setCursor(0, 0);
      lcd.print("T:");
      lcd.print(currentMillis - previousMillis);
      lcd.print("/");
      lcd.print(TEMPSPUJARPERSIANA1);
      lcd.print("; ");
      lcd.print(percentPersiana1);
      lcd.print("%");
      digitalWrite(RELAY2, LOW);
      delay(100);
      digitalWrite(RELAY3, HIGH);
      // Si es prem altre cop boto de pujar mentre s'esta pujant: surt del loop i atura
      // Pero nomes a partir de 750ms
      if ((digitalRead(SW4) == LOW) && (currentMillis - previousMillis > 750))
      {
        break;
      }
    }

    // Para motor
    digitalWrite(RELAY3, HIGH);
    delay(100);
    digitalWrite(RELAY2, HIGH);
    iniciDisplay();
    lcd.setCursor(3, 1);
    lcd.write((uint8_t)2);
    lcd.setCursor(10, 0);
    lcd.write((uint8_t)1);
    lcd.print("-");
    lcd.print(percentPersiana1);
    lcd.print("%");
    pubString = String(percentPersiana1); // Convertir enter a cadena..
    pubString.toCharArray(pubStringChar, pubString.length()+1); // I despres a char...
    mqttClient.publish("romani/test/persiana1", pubStringChar, false); // per ser publicat correctament a MQTT
  }
}

float calcularTempsvsPercent(float percent, int tempstotal){
  return ((tempstotal * percent) / 100);
}

void pujarpersianatotal1(int persiana) {
  unsigned long currentMillis = millis();
  float tempsInicial;
  float percentPersiana;
/*
  switch (persiana):
    case 1: 
        tempsPujarPersiana = TEMPSPUJARPERSIANA1;
        percentPersiana = percentPersiana1
        releSentit = RELAY1;
        releActivar = RELAY2;
        switch = SW3;
        break;
*/
  if (percentPersiana1 != 100)
  {
    previousMillis = currentMillis;
    lcd.clear();
    //while (currentMillis - previousMillis < TEMPSPUJARPERSIANA1)
    //while (percentPersiana1 < 100)
    tempsInicial = calcularTempsvsPercent(percentPersiana1 , TEMPSPUJARPERSIANA1);
    while (tempsInicial < TEMPSPUJARPERSIANA1)
    {
      currentMillis = millis();
      tempsInicial = currentMillis - previousMillis;
      //percentPersiana1 = ((currentMillis - previousMillis) * 100) / TEMPSPUJARPERSIANA1;
      lcd.setCursor(0, 1);
      lcd.print("Pujant Persiana1");
      lcd.setCursor(0, 0);
      lcd.print("T:");
      lcd.print(currentMillis - previousMillis);
      lcd.print("/");
      lcd.print(TEMPSPUJARPERSIANA1);
      lcd.print("; ");
      lcd.print(percentPersiana1);
      lcd.print("%");
      digitalWrite(RELAY2, LOW);
      delay(100);
      digitalWrite(RELAY3, LOW);
      // Si es prem altre cop boto de pujar mentre s'esta pujant: surt del loop i atura
      if ((digitalRead(SW3) == LOW) && (currentMillis - previousMillis > 750))
      {
        break;        
      }
    }

    // Para motor
    digitalWrite(RELAY3, HIGH);
    delay(100);
    digitalWrite(RELAY2, HIGH);
    iniciDisplay();
    
    //Calcular Percentatge estat de la persiana
    percentPersiana1 = (tempsInicial * 100) / TEMPSPUJARPERSIANA1;

/*
  switch (persiana):
    case 1: 
        percentPersiana1 = (tempsInicial * 100) / TEMPSPUJARPERSIANA1;
*/
 
    lcd.setCursor(10, 0);
    lcd.write((uint8_t)1);
    lcd.print("-");
    lcd.print(percentPersiana1);
    lcd.print("%");
    pubString = String(percentPersiana1); // Convertir enter a cadena..
    pubString.toCharArray(pubStringChar, pubString.length()+1); // I despres a char...
    mqttClient.publish("romani/test/persiana1", pubStringChar, false); // per ser publicat correctament a MQTT    
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
 // pinMode(RELAY5, OUTPUT);
 // pinMode(RELAY6, OUTPUT);
 // pinMode(RELAY7, OUTPUT);
 // pinMode(RELAY8, OUTPUT);
 // pinMode(RELAY9, OUTPUT);
 // pinMode(RELAY10, OUTPUT);
 // pinMode(RELAY11, OUTPUT);
 // pinMode(RELAY12, OUTPUT);
 // pinMode(RELAY13, OUTPUT);
 // pinMode(RELAY14, OUTPUT);
 // pinMode(RELAY15, OUTPUT);
 // pinMode(RELAY16, OUTPUT);
 
  pinMode(SW1, INPUT_PULLUP);
//  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);
//  pinMode(SW5, INPUT_PULLUP);
//  pinMode(SW6, INPUT_PULLUP);
//  pinMode(SW7, INPUT_PULLUP);
//  pinMode(SW8, INPUT_PULLUP);
//  pinMode(SW9, INPUT_PULLUP);
//  pinMode(SW10, INPUT_PULLUP);
//  pinMode(SW11, INPUT_PULLUP);
//  pinMode(SW12, INPUT_PULLUP);
 // pinMode(LED_BUILTIN, OUTPUT);
    
  //INICIALITZACIO SORTIDES per defecte
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
  //digitalWrite(RELAY5,HIGH);
  // digitalWrite(RELAY6,HIGH);
  //digitalWrite(RELAY7,HIGH);
  // digitalWrite(RELAY8,HIGH);
  //digitalWrite(RELAY9,HIGH);
  // digitalWrite(RELAY10,HIGH);
  //digitalWrite(RELAY11,HIGH);
  // digitalWrite(RELAY12,HIGH);
  //digitalWrite(RELAY13,HIGH);
  // digitalWrite(RELAY14,HIGH);
  //digitalWrite(RELAY15,HIGH);
  // digitalWrite(RELAY16,HIGH);

  //LCD SETUP
  lcd.begin(16, 2);
  lcd.backlight();

  lcd.print("Benvinguts!!");
  lcd.setCursor(0, 1);
  lcd.print("Connectant..");
  delay(333);

  //lcd.createChar(0, graus);
  lcd.createChar(1, fletxaAmunt);
  lcd.createChar(2, fletxaAvall);

  //MQTT I ETH INICIALITZACIO
  //Ethernet.begin(mac, ip);
  // setup ethernet communication using DHCP
  if (Ethernet.begin(mac) == 0) {
    lcd.setCursor(0, 1);
    lcd.print("Error IP");
    delay(1000);
  }
  lcd.setCursor(0, 1);
  lcd.print(Ethernet.localIP());
  delay(2200);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);

  //Connexio MQTT
  if (mqttClient.connect("Arduino", "andreu", "penis")) {
    mqttClient.subscribe("romani/test/rele1");
    mqttClient.subscribe("romani/test/rele2");
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
    if (SwitchState1 == LOW) {
      // if the current state is HIGH then the switch went from off to on
      // instigate a relay state change:
      if (RelayState1 == HIGH)
      {
        mqttClient.publish("romani/test/rele1", "true", true); //Eardino l darrer true és per retenir el missatge
        digitalWrite(RELAY1, LOW);
        lcd.setCursor(3, 0);
        lcd.print("ON ");
        rele1 = true;
      }
      else
      {
        mqttClient.publish("romani/test/rele1", "false", true); //El darrer true és per retenir el missatge
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
  }
  if (rele1 == false)
  {
    digitalWrite(RELAY1, HIGH);
    lcd.setCursor(3, 0);
    lcd.print("OFF ");
  }


  if (rele2 == true)
  {
    digitalWrite(RELAY4, LOW);
  }
  if (rele2 == false)
  {
    digitalWrite(RELAY4, HIGH);
  }

  
// FI DETECCIO EDGE POLSADOR 1

  if (digitalRead(SW4) == LOW)
  {
    baixarpersianatotal1();
  }

  if (digitalRead(SW3) == LOW)
  {
    pujarpersianatotal1(1);
  }

  delay(50);

  lcd.setCursor(0, 1);
  //Temperatura
  // int chk = DHT.read11(sensorTemp);

  //lcd.print(DHT.humidity); 
  //lcd.print("-");
  //lcd.print(DHT.temperature);  
  lcd.setCursor(10, 1);
  lcd.print(millis() / 1000);
  delay(200);
}
