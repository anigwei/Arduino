void setup(){
  Wire.begin(); 
  int c=0;
  #ifdef EXC_DEBUG_MODE   
    Serial.begin(9600);      
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Sytem Start");   
  #endif
  #ifdef EXC_I2C_BOARD
    for (c=0;c<EXC_I2C_BOARD;c++){InitMCP23017(c);IoBoards[c].Inputs=ReadMCP23017(c);IoBoards[c].Outputs=0;}
  #endif
  #ifdef EXC_SERVER 
    for (byte pas = 0; pas <RemotePacketNumber; pas++){
     RemotePackets[pas].Data[0]='0';RemotePackets[pas].Data[1]='1';RemotePackets[pas].Data[2]='2';RemotePackets[pas].Data[3]='3';
     RemotePackets[pas].Data[4]='4';RemotePackets[pas].Data[5]='5';RemotePackets[pas].Data[6]='6';RemotePackets[pas].Data[7]='7';
     RemotePackets[pas].Data[8]='8';RemotePackets[pas].Data[9]='9';RemotePackets[pas].Data[10]='i';
     RemotePackets[pas].Data[12]=255;RemotePackets[pas].Data[13]='P';RemotePackets[pas].Data[14]='K';RemotePackets[pas].Data[15]='\0'; 
     RemotePackets[pas].Data[11]=pas+1;RemotePackets[pas].LastSend=0;RemotePackets[pas].Send=false; }
  #endif
  
   #ifdef SS_ETHERNET 
     pinMode(SS_ETHERNET, OUTPUT);//pines de control spi
     pinMode(SS_UNO,  OUTPUT);
   #endif
   #ifdef SS_SD 
     pinMode(SS_SD, OUTPUT);//pines de control spi
   #endif
   #ifdef SS_UNO 
     digitalWrite(SS_UNO, HIGH);
   #endif
  #ifdef SS_nRF24L01 
     pinMode(SS_nRF24L01, OUTPUT);  
   #endif
  #ifdef EXC_DEBUG_MODE   
      Serial.println("SPI Completed");
  #endif 
  byte ControlPersiana=0;
  

  
  for (c=0; c<Number_Output;c++){pinMode(PinOutput[c], OUTPUT);SetRelay(PinOutput[c],false);}
   
   
  //enum CircuitsTypes {Persiana,ConsignaTemp,};
  for (c=0;c<Number_Sensor;c++){Sensors[c].Device_Number=240;Sensors[c].Damaged=false;Sensors[c].Value=0; Sensors[c].Type=TypeSensors[c];}
  
  for (c=0;c<Number_Circuit;c++){
    
    circuits[c].Device_Number=0;
    circuits[c].Out1_Value=false;
    circuits[c].Out2_Value=false;
    circuits[c].Value=0;
    circuits[c].CopyRef=0;
    circuits[c].Type=TypeCircuits[c];

    #ifdef EXC_NumeroPersianas
      if ((circuits[c].Type==Persiana)||(circuits[c].Type==Toldo)||(circuits[c].Type==Persiana2)){LocalizadorPersiana[ControlPersiana]=c;circuits[c].Device_Number=ControlPersiana;ControlPersiana++;}
    #endif
  }

  for (c=0;c<Number_Input;c++){
    Inputs[c].InState = 0;
    Inputs[c].LastTimeInput = 0;
    Inputs[c].Type=TypeInputs[c];
    pinMode(PinInput[c], INPUT);
    #ifdef EXC_INTERNAL_RESISTOR
       digitalWrite(PinInput[c], HIGH);       // turn on pullup resistors
    #endif  
  }
  
   
  for (c=0; c < N_SETPOINTS; c++) {int Pos=EM_SETPOINTS_OFSSET + (c*2); Consignas[c]= word(EepromRead(Pos), EepromRead(Pos+1));}
  for (c=0; c < N_ALARMS; c++){Alarms[c]=EepromRead(EM_ALARMS_OFSSET+c);if (Alarms[c]>=AlarmSent){Alarms[c]=WithoutAlarm;}}
  
  #ifdef EXC_NumeroPersianas
    //Fijamo valores y posicion inicio persianas
    //Fijamos el tiempo de subida bajada Persianas
    for (int per=0; per< EXC_NumeroPersianas; per++){InDowPersiana[per]=false;InUpPersiana[per]=false;}
    ReiniciarTiempoPersianas();
  #endif 

  #ifdef SD_CARD
    EnableSD();    
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Initializing SD card...");             
    #endif  
    if (!SD.begin(SS_SD)) {
      #ifdef EXC_DEBUG_MODE   
        Serial.println("ERROR - SD card initialization failed!");           
      #endif        
        SdOk=false;
    }else{
       #ifdef EXC_DEBUG_MODE   
        Serial.println("SUCCESS - SD card initialized.");              
     #endif
      if (!SD.exists("HIT/")){SD.mkdir("HIT");} 
    }
    
    if (SdOk){
      
        //Log File
      SdFile = SD.open("log.txt", FILE_WRITE);
      String Log= "Start at " + ((String)hour) + ":" +((String)minute) + " - " +((String)dayOfMonth) + "/" + ((String)month) + "/"+((String)year) + "/";
      SdFile.println(Log);
      SdFile.close();
        
  
        ///lectura copia de seguridad old
     }

      LoadSecutityCopy();  
      EnableEthernet(); 
  #else
      LoadSecutityCopy();
  #endif   
   
 #ifdef EXC_WIFI_SHIELD
     if (WiFi.status() == WL_NO_SHIELD) {
       #ifdef EXC_DEBUG_MODE   
          Serial.println("WiFi shield not present"); 
        #endif
      while(true);// don't continue:
    }else{ConexionWifi();}
      
  #else
     #ifdef EXC_STATIC_IP  
      Ethernet.begin(mac,ip);
     #else    
      Ethernet.begin(mac);
     #endif
     Udp.begin(localPort);
  #endif
  #ifdef EXC_NumeroPersianas
    for (int per=0; per< EXC_NumeroPersianas; per++){CargaPosicionPersiana(per); }//las persianas ajustar posicion     
  #endif


 #ifdef EXC_RECEIVER_433  
    Init433Mhz();// Receiver on inerrupt 0 => that is pin #2
 #endif
 #ifdef EXC_TRANSMITER_433 
  mySwitch.enableTransmit(EXC_433enableTransmit);
  mySwitch.setPulseLength(EXC_433setPulseLength);//Pulse lenght
  mySwitch.setProtocol(EXC_setProtocol);//Tipo Protocolo
  mySwitch.setRepeatTransmit(EXC_433setRepeatTransmit);//Repeticiones
  //mySwitch.enableTransmit(15); //Default pin
  //mySwitch.setPulseLength(320);//Default pin Pulse lenght
  //mySwitch.setProtocol(1);//Default pin Tipo Protocolo
  //mySwitch.setRepeatTransmit(15);//Default pin Repeticiones
 #endif
 
  //Iniciamos Termostatos
 #ifdef THERMOSTAT_DS18B20_NUMBER
   InitDS18B20();
 #endif 
  #ifdef EXC_IR_RECIVE
   irrecv.enableIRIn();
 #endif
   //Iniciamos perro guardian
   #ifdef EXC_ENABLE_WATCH_DOG
     wdt_enable(WDTO_8S);
   #endif
   #ifdef THERMOSTAT_DTH22 
     dht.begin();  
   #endif 
   #ifdef ENABLE_SMODBUS
    #ifdef EXC_ARDUINO_MEGA
      modbus_configure(&Serial1, MODBUS_BAUD, SERIAL_8N2, 2, TxPin, HOLDING_REGS_SIZE, holdingRegs); 
    #else
      modbus_configure(&Serial, MODBUS_BAUD, SERIAL_8N2, 2, TxPin, HOLDING_REGS_SIZE, holdingRegs); 
    #endif 
    for (int m=0;m<32;m++){ModbusOuts[m]=false;}
  #endif  
  #ifdef EXC_NRF24  
    for (int m=0;m<32;m++){NrfOuts[m]=false;} 
    for (int k=0;k<UserRegNumber;k++){OldNRFRegs[k]=0;NRFRegs[k]=0;;}
    
    for (int dv=0;dv<DevNumber;dv++){for (int m=0;m<8;m++){NrfInRegs[dv][m]=0;OldNrfInRegs[dv][m]=-1;}}    
    NrfCiclos=1;
    IniciarNrf();

  #endif

  
  unsigned long currenMillis = millis();
  unsigned int reading;
  
  for (c=0;c<Number_Input;c++){
    if ((Inputs[c].Type==Swicth)||(Inputs[c].Type==Retroaviso)){
      reading = digitalRead(PinInput[c]);
      if (reading==EXC_InputOn){Inputs[c].InState=1;}else{Inputs[c].InState=0;}
      Inputs[c].LastTimeInput=currenMillis;
      if (Inputs[c].Type==Retroaviso){if (reading==EXC_InputOn){SwicthStateChange(c,HIGH);}else{SwicthStateChange(c,LOW);}}    
    }   
  }
  #ifdef EXC_LCD
    lcd.begin(20,4);loadCharsLCD(); 
    lcd.backlight();
    PrintMyLcd();
  #endif
  ExTimer=millis(); 
  
  UserSetup();

}
void loop(){
  #ifdef ExControlMail
    if ((Connecting==false)&&(EspRfrIp<60)){
     #ifdef EXC_WIFI_SHIELD
      if (EstadoWifiOk() == true) {SendAlarms();}  
     #else
      SendAlarms();
     #endif
    }
  #endif 
  SystemLoop();
}

void SystemLoop(){
  #ifdef ENABLE_SMODBUS

    for (int m=0;m<16;m++){if (ModbusOuts[m]){bitWrite(holdingRegs[OUT_REG1], m, 1); }else{bitWrite(holdingRegs[OUT_REG1], m, 0); }}
    for (int m=16;m<32;m++){if (ModbusOuts[m]){bitWrite(holdingRegs[OUT_REG2], m-16, 1); }else{bitWrite(holdingRegs[OUT_REG2], m-16, 0); }}
    
    int pa=0;int dv=0;
    for (int m=2;m<HOLDING_REGS_SIZE;m++){
      if (pa<6){
        if (OldholdingRegs[m]!= holdingRegs[m]){
          int r=dv*100;r=r+pa+1000;
          if (holdingRegs[m]==0){SwicthStateChange(r,LOW);}
          if (holdingRegs[m]==1){SwicthStateChange(r,HIGH);}
          if ((holdingRegs[m]>100)&& (holdingRegs[m]<110)){ShortInput(r);}
          if ((holdingRegs[m]==100)&& (OldholdingRegs[m]==110)){LongInputEnd(r);}
          if (holdingRegs[m]==110){LongInput(r);}
          OldholdingRegs[m]= holdingRegs[m];         
        } 
      }
      
      pa++;if (pa>7){pa=0;dv++;}
    }

      
    modbus_update();
  #endif
  
  #ifdef EXC_NRF24
    
    EnableRF24L01(); 
    
    network.update();  
    boolean ComExist=false;
    while ( network.available() )  {                    
      #ifdef EXC_DEBUB_NRF24 
        Serial.println("ENTRADA DATOS NRF24");
      #endif
      RF24NetworkHeader header;                            
      network.peek(header);
      
      switch (header.type){                             
        case 'I': {ComExist=true; network.read(header,NrfInRegs[0],NrfRS);if (OldNrfInRegs[0][0]==-1){NrfInputIni(0);}else{NrfInput(0);} break;}

        case 'X':{if (DevNumber>=1){ComExist=true; network.read(header,NrfInRegs[1],NrfRS);if (OldNrfInRegs[1][0]==-1){NrfInputIni(1);}else{NrfInput(1);}break;}}
        case 'Y':{if (DevNumber>=2){ComExist=true; network.read(header,NrfInRegs[2],NrfRS);if (OldNrfInRegs[2][0]==-1){NrfInputIni(2);}else{NrfInput(2);}break;}}
        case 'Z':{if (DevNumber>=3){ComExist=true; network.read(header,NrfInRegs[3],NrfRS);if (OldNrfInRegs[3][0]==-1){NrfInputIni(3);}else{NrfInput(3);}break;}}
        default:  {
          network.read(header,0,0); break;
          #ifdef EXC_DEBUB_NRF24 
            Serial.println("PAQUETE ERRONEO");
          #endif
          
          }
      }   
    }
    
    if (ComExist){
      Escucha=false;
      NRF24_CommOk(false);
      OutGest();
     
    }   
    for (int m=0;m<16;m++){if (NrfOuts[m]){bitWrite(NRFRegs[0], m, 1); }else{bitWrite(NRFRegs[0], m, 0); }}
    for (int m=16;m<32;m++){if (NrfOuts[m]){bitWrite(NRFRegs[1], m-16, 1); }else{bitWrite(NRFRegs[1], m-16, 0); }}
    if (NrfCiclos<1){for (int k=0;k<UserRegNumber;k++){if (NRFRegs[k]!=OldNRFRegs[k]){NrfCiclos=1;break;}}}

    if (NrfCiclos>0){
      #ifdef EXC_DEBUB_NRF24 
        Serial.println("INICIO ENVIOS NRF");
      #endif
      if (Escucha){
        //Serial.println("MODO ESCUCHA");
        unsigned long t = millis();
        if (t<TimSend){TimSend=t;}
        if ((t - TimSend) >= 120 ){Escucha=false;}
      }
      else{          
      #ifdef EXC_DEBUB_NRF24 
        Serial.println("ENVIANDO DATOS");
      #endif
  
          RF24NetworkHeader header(dest_address, 'O');
          if (network.write(header,NRFRegs,NrfRegS)){NrfCiclos=0;NRF24_CommOk(true);for (int k=0;k<UserRegNumber;k++){OldNRFRegs[k]=NRFRegs[k];}} 
          else{Escucha=true;NrfCiclos++;if (NrfCiclos>20){delay(1000);IniciarNrf();}}
          #ifdef EXC_DEBUB_NRF24 
            if (Escucha){Serial.println("Fallo Envio reg. salidas");}else{Serial.println("Envio Registro salida ok");}
          #endif
      }      
    }

    EnableEthernet(); 
  #endif
  
  #ifdef EXC_IR_RECIVE   
    ComprobarInfrarro();
  #endif 
  
  #ifdef EXC_RECEIVER_433  
    Recepcion433Mhz();
  #endif
  
  InputState();
  
  
   TimNow=millis();   
   if(TimNow < ExTimer ) {ExTimer=TimNow;}
   if((TimNow - ExTimer) >= 100) {ExTimer=TimNow;ExcTimer();}

   
   #ifdef EXC_WIFI_SHIELD
    if (TimConexion > 34){if ((WiFi.status() == WL_CONNECTED)||(status == WL_AP_LISTENING)){RecepcionPaqueteUDP();} else{ResetConexionWifi();}}  
   #else
    RecepcionPaqueteUDP();
   #endif
   
   #ifdef EXC_NumeroPersianas
     for (int p =0; p< EXC_NumeroPersianas;p++){GestionMovPersianas(p);} //Control de movimiento persianas
   #endif
   
   
   #ifdef EXC_ENABLE_WATCH_DOG
    wdt_reset();
   #endif 
   GestionCircuitos();
   OutGest();
   UserLoop();
  
}


void ExcTimer(){
  byte c=0;
  LoopNew100MillisSg();
  
  #ifdef EXC_RGB_LED
    RgbRandomColor();
  #endif
  CountMsg++;
  #ifdef EXC_I2C_BOARD
    for (c=0;c<EXC_I2C_BOARD;c++){
      byte rst =ReadMCP23017(c);
      if (rst!=IoBoards[c].Inputs){
        byte v;
        for (byte bt =0;bt<8;bt++){
          v=bitRead(rst,bt);
          if (v!=bitRead(IoBoards[c].Inputs, bt)){if (v==0){SwicthStateChange(3000 + (c*100) + bt, HIGH);}else{SwicthStateChange(3000 + (c*100) + bt, LOW);}}
        }
        IoBoards[c].Inputs=rst;
      }
     }
     GestionCircuitos();OutGest();
     if (CountMsg==5){for (c=0;c<EXC_I2C_BOARD;c++){WriteMCP23017(c ,IoBoards[c].Outputs);}}
  #endif  
      
  if (CountMsg>=10){
      CountMsg=0;CountSg++;LoopNewSecond();
      if (CountSg>=30){Exc30Seg();} 
      if (IntCom>0){IntCom--;}
      
      #ifdef EXC_WIFI_SHIELD        
         if (TimConexion==19){ConexionWifi();}//Inicio conexion
         else if (TimConexion==31){//Comprobar conexion completa
          bool cn=false;
          
          if ((WiFi.status() != WL_CONNECTED)&&(status != WL_AP_LISTENING)) {ResetConexionWifi();}
          else if (Udp.begin(localPort)==1){        
                TimConexion=35;
                #ifdef EXC_DEBUG_MODE 
                  Serial.println();
                  Serial.println("Wifi connection OK");
                  Serial.print("SSID: ");
                  Serial.println(WiFi.SSID());
                
                  // print your WiFi shield's IP address:
                  IPAddress ip = WiFi.localIP();
                  Serial.print("IP Address: ");
                  Serial.println(ip);
                
                  // print the received signal strength:
                  long rssi = WiFi.RSSI();
                  Serial.print("signal strength (RSSI):");
                  Serial.print(rssi);
                  Serial.println(" dBm");
                #endif            
              }else{ResetConexionWifi();} //Esperando dos 30 sg para reconexion
 
        }else if (TimConexion<35){TimConexion++;}

      #endif
      
      #ifdef THERMOSTAT_DS18B20_NUMBER
        if (CountSg==10){RefreshTemperature();}
      #endif
      if (SegUpdtHora==0){CargaHora();}
      else{SegUpdtHora--;}
      
      #ifdef THERMOSTAT_DTH22 
         if (CountSg==4){ReadDHT();}   
      #endif   
      #ifdef EXC_NRF24
        TimRcv++;
        if (TimRcv>=70){delay(1000);IniciarNrf();}
      #endif
      
  }  
}
void OutGest(){
  OutControl();
  #ifdef EXC_I2C_BOARD
    for (byte c=0;c<EXC_I2C_BOARD;c++){if (ov[c]!=IoBoards[c].Outputs){WriteMCP23017(c ,IoBoards[c].Outputs);ov[c]=IoBoards[c].Outputs;}}
  #endif
}
void Exc30Seg(){
   CountSg=0;
   #ifdef EXC_LCD
     PrintMyLcd();
   #endif     
   SecutityCopy();
   Loop30Sg();
   #ifdef ExControlMail
     if (EspRfrIp<1){connectAndRfr();}else{EspRfrIp--;}
   #endif 
   #ifdef EXC_SERVER
    if (TimUdp>=1){TimUdp--;}
    else{
      //delay(50000);
     #ifdef EXC_DEBUG_MODE        
      Serial.println("Reiniciando conexion, server timeout"); 
     #endif
     #ifdef EXC_WIFI_SHIELD        
          ResetConexionWifi();
     #else
          Udp.stop();
          #ifdef EXC_STATIC_IP  
            Ethernet.begin(mac,ip);
          #else    
            Ethernet.begin(mac);
          #endif
          Udp.begin(localPort);
     #endif
     #ifdef EXC_DEBUG_MODE        
        Serial.println("Completado"); 
     #endif
    }
  #endif 
}
void GestionCircuitos(){
      
  //  ,,,,,,,Persiana,ConsignaTemp,,
  for (int c=0;c<Number_Circuit;c++){
    if ((circuits[c].Type!=SensorView)&&(circuits[c].Type!=Reserva)&&(circuits[c].Type!=Timer)&&(circuits[c].Type!=Riego_Temporizado)){
      if (circuits[c].Type==Ado_3Etapas){
        switch (circuits[c].Value) {
          case 0:    
            circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
            break;
          case 1:    
            circuits[c].Out1_Value=true;circuits[c].Out2_Value=false;
            break;
          case 2:    
           circuits[c].Out1_Value=false;circuits[c].Out2_Value=true; 
            break;
           case 3:    
             circuits[c].Out1_Value=true;circuits[c].Out2_Value=true; 
            break;
           default:    
            circuits[c].Value=0;circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
            break;
         }    
      }
      
      else if ((circuits[c].Type==Persiana)||(circuits[c].Type==Toldo)||(circuits[c].Type==Persiana2)){
          #ifdef EXC_NumeroPersianas
           circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
           if ((OutDowPersiana[circuits[c].Device_Number]==true)||(OutUpPersiana[circuits[c].Device_Number]==true))
           {
             if ((OutDowPersiana[circuits[c].Device_Number]==true)&&(OutUpPersiana[circuits[c].Device_Number]==false)){circuits[c].Out1_Value=true; circuits[c].Out2_Value=true;}
             if ((OutDowPersiana[circuits[c].Device_Number]==false)&&(OutUpPersiana[circuits[c].Device_Number]==true)){circuits[c].Out1_Value=true; circuits[c].Out2_Value=false;}     
           }
          #endif
        }    
      else if ((circuits[c].Type==Frio)||(circuits[c].Type==Calor)){Termostato(c);}
      else if ((circuits[c].Type==High_trigger)||(circuits[c].Type==Low_trigger)){SensorTrigger(c);}
      else if ((circuits[c].Type==SetPoint_200)||(circuits[c].Type==SetPoint_2000)||(circuits[c].Type==SetPoint_20000)){}      
      else if (circuits[c].Type>100){
        boolean b=false;
        if (circuits[c].Value==1){b=true;}
        if (circuits[c].Out2_Value!=b){circuits[c].Out2_Value=b;circuits[c].Out1_Value=!circuits[c].Out1_Value;}    
      }
      else{
        if (circuits[c].Value>=1){circuits[c].Out1_Value=true;circuits[c].Out2_Value=false;}
        else{circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;}
      }
    }
  }      
}
   // else if (circuits[c].Type==High_trigger){HighTrigger(c);}
     // else if (circuits[c].Type==Low_trigger){LowTrigger(c);}
     
void SensorTrigger(byte  CirNumber){
  if ((CirNumber>=Number_Circuit)||(CirNumber<0)){return;}
   if (circuits[CirNumber].Value==0){ circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Out2_Value=false;}
   else{   
 
    Circuit ConsignaCir = circuits[circuits[CirNumber].Device_Number];
    if (Sensors[ConsignaCir.Device_Number].Damaged){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=249;return;}
    short t;  
    switch (ConsignaCir.Type) {
    case SetPoint_100:   
      t=ConsignaCir.Value;
      break;
    case SetPoint_200:
      t=ConsignaCir.Value;
      break;
    case SetPoint_2000:   
      t=ConsignaCir.Value * 10;
      break;   
     case SetPoint_20000:   
      t=ConsignaCir.Value * 100;
      break; 
    default:
      return;
   }
   if ((Sensors[ConsignaCir.Device_Number].Type==Sensor_Temperature)||(Sensors[ConsignaCir.Device_Number].Type==Sensor_Humidity)||(Sensors[ConsignaCir.Device_Number].Type==Sensor_Float)){t=t*10;}
   if ((ConsignaCir.Device_Number<0)||(ConsignaCir.Device_Number >= Number_Sensor)){return;}
   else{
     switch (circuits[CirNumber].Type) {
      case High_trigger:   
        if (Sensors[ConsignaCir.Device_Number].Value > t){circuits[CirNumber].Out1_Value=true;circuits[CirNumber].Value=249;}
        else{circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=1;}
        break;
      case Low_trigger:
       if (Sensors[ConsignaCir.Device_Number].Value > t){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=1;}
       else{circuits[CirNumber].Out1_Value=true;circuits[CirNumber].Value=249;}
       break;
      default:
        return;
      }
   }          
  }
}

void Termostato(byte  CirNumber){
  if ((CirNumber>=Number_Circuit)||(CirNumber<0)){return;}
  if (circuits[CirNumber].Value==0){ circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Out2_Value=false;return;}
  if ((circuits[CirNumber].Device_Number > Number_Circuit)||(circuits[CirNumber].Device_Number < 0 )){return;}
  
  Circuit ConsignaCir = circuits[circuits[CirNumber].Device_Number];
  if ((ConsignaCir.Device_Number > Number_Sensor)||(ConsignaCir.Device_Number < 0 )){return;}  
  else{
    
    if (Sensors[ConsignaCir.Device_Number].Damaged){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=249;return;}
    short t;  
    switch (ConsignaCir.Type) {
    case ConsignaTemp:   
      t=ConsignaCir.Value *10;
      break;
    case HomeTemperature:
      t=ConsignaCir.Value + 150;
      break;
    case TempNegative:   
      t=ConsignaCir.Value *  (-10);
      break;   
    default:
      return;
   }
   
   switch (circuits[CirNumber].Type) {
    case Calor:   
      if (Sensors[ConsignaCir.Device_Number].Value > (t + Histeresis)){circuits[CirNumber].Out1_Value=false;}
      else{if (Sensors[ConsignaCir.Device_Number].Value < (t - Histeresis)){circuits[CirNumber].Out1_Value=true;}}
      break;
    case Frio:
      if (Sensors[ConsignaCir.Device_Number].Value < (t - Histeresis)){circuits[CirNumber].Out1_Value=false;}
      else{if (Sensors[ConsignaCir.Device_Number].Value > (t + Histeresis)){circuits[CirNumber].Out1_Value=true;}}
      break;
    default:
      return;
   }   
  }
}


void InputState(){
  unsigned long InputMillis;
  unsigned int v=0;

  boolean acT=false;
  InputMillis = millis();
  if (InputMillis<TimAcDet) {TimAcDet=InputMillis;acT=true;}
  else if ((InputMillis - TimAcDet)>=2)  {TimAcDet=InputMillis;acT=true;}

  for (int i=0;i<Number_Input;i++){    
    switch (Inputs[i].Type) {
    case Button: {
      if (digitalRead(PinInput[i])==EXC_InputOn){v=1;}else{v=0;}
      InputMillis = millis();  
      if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
      else{
        InputMillis = InputMillis -Inputs[i].LastTimeInput;       
        if ((Inputs[i].InState>=4)||(Inputs[i].InState<0)){Inputs[i].InState=0;}     
        if (v==1){ 
          if ((Inputs[i].InState==0)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
          if ((Inputs[i].InState==1)&&(InputMillis>=440)){LongInput(i);Inputs[i].InState=2;}
          if (Inputs[i].InState==2){Inputs[i].LastTimeInput=millis();}
          if (Inputs[i].InState==3){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
         }
         else{
            if (Inputs[i].InState==0){Inputs[i].LastTimeInput=millis();}
            if (Inputs[i].InState==1){Inputs[i].LastTimeInput=millis();Inputs[i].InState=3;}
            if ((Inputs[i].InState==2) &&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;LongInputEnd(i);}
            if ((Inputs[i].InState==3)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;ShortInput(i);}
        }
      }  
      break;}
   case ExcIDetector: {

      if (acT==false){break;}
      v=analogRead(PinInput[i])/4;
      if (v>254){v=254;}

      byte Hv=highByte(Inputs[i].LastTimeInput); 
      byte Lv=lowByte(Inputs[i].LastTimeInput);

      if ((v<100)||(v>210)){if (Lv<110){Lv=Lv+4;}}
      else{
        if (v==Hv) {if (Lv>0){Lv--;}}
        else{ 
          byte d;
          if (v>Hv){d=v-Hv;}else{d=Hv-v;}
          if (d<=4){if (Lv>0){Lv--;}}else{if (Lv<110){Lv=Lv+4;}}           
        }
      }
      Inputs[i].LastTimeInput=word(v, Lv);
      
      if (Lv>100){v=1;} 
      else if (Lv==0){v=0;}
      else {break;}

      if (Inputs[i].InState!=v){
          Inputs[i].InState=v;
          if (v==1){SwicthStateChange(i,HIGH);}else{SwicthStateChange(i,LOW);}        
      }
      break;}

      
    default:{
      if (digitalRead(PinInput[i])==EXC_InputOn){v=1;}else{v=0;}
      InputMillis = millis();
      if (v==Inputs[i].InState){Inputs[i].LastTimeInput=InputMillis;}
      else{
        if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
        if ((InputMillis-Inputs[i].LastTimeInput)>=60){Inputs[i].LastTimeInput=InputMillis; Inputs[i].InState=v;if (v){SwicthStateChange(i,HIGH);}else{SwicthStateChange(i,LOW);}}    
      }
      break;}
    }
  }
} 

#ifdef EXC_NumeroPersianas
  
  /*************************************************************/
     //Gestion Persianas
  /*************************************************************/ 
  
  void GestionMovPersianas(int NPersiana){
    if (InUpPersiana[NPersiana] || InDowPersiana[NPersiana])
    {  //Funcionamiento Manual
      if (InUpPersiana[NPersiana] && InDowPersiana[NPersiana]){InUpPersiana[NPersiana] =false; InDowPersiana[NPersiana]=false;OutDowPersiana[NPersiana]=false;OutUpPersiana[NPersiana]=false;OutGest();delay(100);}
      else{
        if (InUpPersiana[NPersiana]){SubirPersiana(NPersiana);}else {BajarPersiana(NPersiana);}
        circuits[LocalizadorPersiana[NPersiana]].Value=PosicionPersiana[NPersiana];    
      }    
    }
    else
    {  //Funcionamiento Automatico;
      if (circuits[LocalizadorPersiana[NPersiana]].Value==PosicionPersiana[NPersiana]){OutDowPersiana[NPersiana]=false;OutUpPersiana[NPersiana]=false;}
      else
      {      
         if (circuits[LocalizadorPersiana[NPersiana]].Value > PosicionPersiana[NPersiana]){SubirPersiana(NPersiana);}
         else  {if (circuits[LocalizadorPersiana[NPersiana]].Value< PosicionPersiana[NPersiana])  {BajarPersiana(NPersiana);}}    
      }  
    }
  }
  
  void SubirPersiana(int NPersiana){

    unsigned long TiempoActual = micros();
    if (OutUpPersiana[NPersiana]==false){OutUpPersiana[NPersiana]=true;}
    else{
      unsigned long DiferenciaTiempo;
      if (TiempoActual<TiempoMovPersiana[NPersiana]){DiferenciaTiempo=TiempoActual;}else{DiferenciaTiempo = TiempoActual-TiempoMovPersiana[NPersiana];}
      if ((TiempoPosPersianaUp[NPersiana] + DiferenciaTiempo)<TimUpPersiana[NPersiana]){TiempoPosPersianaUp[NPersiana]=TiempoPosPersianaUp[NPersiana] + DiferenciaTiempo;}
      else{TiempoPosPersianaUp[NPersiana]=TimUpPersiana[NPersiana];}
      byte porcentajeSubida = TiempoPosPersianaUp[NPersiana] / (TimUpPersiana[NPersiana]/100);
      byte porcentajeBajada=100-porcentajeSubida;
      PosicionPersiana[NPersiana]=porcentajeSubida;
      TiempoPosPersianaDown[NPersiana]=porcentajeBajada*(TimDowPersiana[NPersiana]/100);
    }  
    TiempoMovPersiana[NPersiana]=TiempoActual;
  }
  
     
  void BajarPersiana(int NPersiana){
  
    unsigned long TiempoActual = micros();
    if (OutDowPersiana[NPersiana]==false){OutDowPersiana[NPersiana]=true;}
    else{
      unsigned long DiferenciaTiempo;
      if (TiempoActual<TiempoMovPersiana[NPersiana]){DiferenciaTiempo=TiempoActual;}else{DiferenciaTiempo = TiempoActual-TiempoMovPersiana[NPersiana];}
      if ((TiempoPosPersianaDown[NPersiana] + DiferenciaTiempo)<TimDowPersiana[NPersiana]){TiempoPosPersianaDown[NPersiana]=TiempoPosPersianaDown[NPersiana] + DiferenciaTiempo;}else{TiempoPosPersianaDown[NPersiana]=TimDowPersiana[NPersiana];}
     
      byte porcentajeBajada = TiempoPosPersianaDown[NPersiana] / (TimDowPersiana[NPersiana]/100);
      byte porcentajeSubida=100-porcentajeBajada;
      PosicionPersiana[NPersiana]=porcentajeSubida;
      TiempoPosPersianaUp[NPersiana]=porcentajeSubida*(TimUpPersiana[NPersiana]/100);
    }  
    TiempoMovPersiana[NPersiana]=TiempoActual;
  }
  
  void ReiniciarPosicionPersiana(int NumPersiana){ TiempoPosPersianaUp[NumPersiana]=0;TiempoPosPersianaDown[NumPersiana]=TimDowPersiana[NumPersiana];circuits[LocalizadorPersiana[NumPersiana]].Value=100;}
  void ReiniciarTiempoPersianas()
  {
    for ( byte c =0; c <  EXC_NumeroPersianas; c++){
      TimUpPersiana[c]=(EepromRead(EM_UP_TIM_SHUTTER_OFFSET + c))*  1000000; 
      TimDowPersiana[c]=(EepromRead(EM_DO_TIM_SHUTTER_OFFSET + c))* 1000000;
    }
  }
  void CargaPosicionPersiana(int NPersiana){
    if (circuits[LocalizadorPersiana[NPersiana]].Value>100){ReiniciarPosicionPersiana(NPersiana);}
    else{
      PosicionPersiana[NPersiana]=circuits[LocalizadorPersiana[NPersiana]].Value;
      byte porcentajeBajada=100-PosicionPersiana[NPersiana];
      TiempoPosPersianaDown[NPersiana]=porcentajeBajada*(TimDowPersiana[NPersiana]/100);
      TiempoPosPersianaUp[NPersiana]=PosicionPersiana[NPersiana]*(TimUpPersiana[NPersiana]/100);  
    }
  }
#endif
void RecepcionPaqueteUDP(){
 
  const char COMPLETED='%';
  
  int  indexstr = 0;
  unsigned int p,c;


  
  //if (Connecting==true){return;}//hay que probarlo con y sin!
  p = Udp.parsePacket();  
  
  if(p>0){    
     #ifdef EXC_DEBUG_MODE        
      Serial.print("UDP Packet recive, size"); Serial.println(p);  
    #endif
    if (p>96){
      #ifdef EXC_DEBUG_MODE        
        Serial.println("ERROR  UDP SOBRECARGADO"); 
      #endif
      Udp.stop();
      #ifdef EXC_DEBUG_MODE        
        Serial.println("Desconexion Red ok"); 
      #endif

      #ifdef EXC_DEBUG_MODE        
        Serial.println("Conexion Red"); 
      #endif

     #ifdef EXC_WIFI_SHIELD        
          if (Udp.begin(localPort)!=1){ ResetConexionWifi();}
     #else
          Udp.begin(localPort);  
     #endif
     return;  
    }
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
        //Test secure connection, return if not same.
    if  (SecureConnection){for(c=0; c<8;c++){if (ExControlPass[7-c]!=packetBuffer[p -(c+1)]){return;}}}   
    #ifdef EXC_SERVER
      TimUdp=190;
    #endif 
    // Test comand receiver. Execute and request.
    if (strncmp(packetBuffer, "COMCOMM", 7)==0){
      CommonOrders(packetBuffer[7]);
      strcpy(packetBuffer , "COMCOMOK");  
    }
    else if (strncmp(packetBuffer, "ALRM", 4)==0){
      strcpy(packetBuffer,"ESAL"); 
      for (c = 4; c<24;c++){packetBuffer[c]=Alarms[c-4]+1;}    
      packetBuffer[24]='\0';
    }
    else if (strncmp(packetBuffer, "SETNOTI", 7)==0){
       byte pos=packetBuffer[7]-1;
       Alarms[pos]=packetBuffer[8]-1;
       strcpy(packetBuffer, "WIALMO");  
       EepromWrite(EM_ALARMS_OFSSET + pos,Alarms[pos]);
    }
       
    else if (strncmp(packetBuffer, "SVAL", 4)==0){
      if (packetBuffer[4]<=Number_Circuit){ circuits[packetBuffer[4]-1].Value=packetBuffer[5]-1;}
      EnvioEstadoActual();
      return;
    }    
    else if (strncmp(packetBuffer, "VACT", 4)==0){
      char Respuesta[35]; 
      indexstr=4;   
      strcpy(packetBuffer, "VVAL");     
      for (c=0; c<30;c++)
      {
         if ((c)<Number_Circuit){ packetBuffer[indexstr]=circuits[c].Value+1;}
         else{packetBuffer[indexstr]=1;}  
         indexstr++;
      }
      packetBuffer[indexstr]='\0';
    }
     else if (strncmp(packetBuffer, "RESE", 4)==0){ 
       
      strcpy(packetBuffer, "VASC");
      indexstr=4;	  
      for (c=0; c < Number_Sensor; c++){packetBuffer[indexstr]=Sensors[c].Type;indexstr++;}
      packetBuffer[indexstr]='\0';
    }
     else if (strncmp(packetBuffer, "EESE", 4)==0){ 
      strcpy(packetBuffer, "VESC");
      indexstr=4;
      byte bl,bh;
      for (c=0; c < Number_Sensor; c++){
        bl= lowByte(Sensors[c].Value); bh= highByte(Sensors[c].Value);        
        if (bl<255){bl++;}
        if (bh<255){bh++;}        
        packetBuffer[indexstr]=bl;
        indexstr++;  
        packetBuffer[indexstr]=bh;
        indexstr++;
      }
      packetBuffer[indexstr]='\0';
    }
   else if (strncmp(packetBuffer, "REDN", 4)==0){ 
      strcpy(packetBuffer, "VADN");
      indexstr=4;
      for (c=0; c < 30; c++){
        if (c < Number_Circuit){packetBuffer[indexstr]=circuits[c].Device_Number+1;}
        else{packetBuffer[indexstr]=1;}
        indexstr++;
      }
      packetBuffer[indexstr]='\0';
    }
    else if (strncmp(packetBuffer, "CARG", 4)==0){ 
      strcpy(packetBuffer, "VALC");
      indexstr=4;
      for (c=0; c < 30; c++){
        if (c < Number_Circuit){
          if (circuits[c].Type>100){
            packetBuffer[indexstr]=circuits[c].Type-100;
          }
          else{
            packetBuffer[indexstr]=circuits[c].Type;
          }
        }
        else{
          packetBuffer[indexstr]=1;
        }
        indexstr++;
      }
      packetBuffer[indexstr]='\0';
    }
    	
    else if (strncmp(packetBuffer, "CLEARHORARIO", 11)==0){
      strcpy(packetBuffer , "HORARIOS BORRADOS");
      EnviarRespuesta(packetBuffer);
      #ifdef EXC_ENABLE_WATCH_DOG
        byte paso=0; 
      #endif

      for (p = EM_TRIGGER_OFFSET; p <= EM_TIME_ESPECIAL2_END; p++){          // Size slot 80 * 4bytes data *7 day of the week + 
        EepromWrite(p, 66);
        #ifdef EXC_ENABLE_WATCH_DOG
          paso++;
          if (paso>200){wdt_reset();paso=0;}          
        #endif
      }
      return;     
    }
    else if (strncmp(packetBuffer, "CLEARESPCDAY", 12)==0){
      strcpy(packetBuffer , "DIAS ESPECIALES BORRADOS");
      EnviarRespuesta(packetBuffer);
      #ifdef EXC_ENABLE_WATCH_DOG
        wdt_reset();
      #endif
      for (c = EM_DATE_ESPECIAL1_OFSSET; c <= EM_DATE_ESPECIAL2_END; c++){ EepromWrite(c, 0);} // size slot (100)  50 * 4bytes data * 2 special days
      return;     
    }
    else if (strncmp(packetBuffer, "SETFH", 5)==0){
      setDateCLOCK(packetBuffer[5] ,packetBuffer[6], packetBuffer[7], packetBuffer[8], packetBuffer[9], packetBuffer[10], packetBuffer[11]);
      CargaHora();
      strcpy(packetBuffer, "SETFHOK");
    }
    else if (strncmp(packetBuffer, "GETSENSOR", 9)==0){     
      String Rsp="SENSORx" + FreeText(packetBuffer[9]);
      Rsp.setCharAt(6, packetBuffer[9]);
      indexstr=Rsp.length();
      for (c=0; c < indexstr; c++){packetBuffer[c]=Rsp.charAt(c);}
      packetBuffer[indexstr]='\0';
    }  
    else if (strncmp(packetBuffer, "READDAY", 7)==0)
    {
      if (packetBuffer[7]=='2'){p=EM_DATE_ESPECIAL2_OFSSET;}// Pointer, first address memory slot. Eeprom.
      else{p=EM_DATE_ESPECIAL1_OFSSET;}
      strcpy(packetBuffer,"CFDA");             
           
      indexstr=4;                              // Index to string constructor.
      for (c = 0; c<EM_DATE_ESPECIAL_SIZE; c++){                  // Number of Iterations = slot size.
        packetBuffer[indexstr]=EepromRead(p);  // Read Eeprom, data stored in the same packetBuffer
        indexstr++;
        p++;
      }
      packetBuffer[indexstr]='\0';              // End of string. NULL termintation.        
              
    }    
    else if (strncmp(packetBuffer, "WRIDAYE", 7)==0)
    {       
       if (packetBuffer[7]==2){p=EM_DATE_ESPECIAL2_OFSSET;}
       else{p=EM_DATE_ESPECIAL1_OFSSET;}         
       indexstr=8;                               //Index to first data in packetBuffer.       
       for (c=0; c<EM_DATE_ESPECIAL_SIZE; c++){EepromWrite(p++, packetBuffer[indexstr++]); }   //Write de data.        
       packetBuffer[0]=COMPLETED;              
    }      
    else if (strncmp(packetBuffer, "RETRIGGER", 9)==0)
    {      
      strcpy(packetBuffer , "TIGR");
      indexstr=4;    
      for (c=EM_TRIGGER_OFFSET; c <= EM_TRIGGER_END; c++){packetBuffer[indexstr++] = EepromRead(c)+1;}
      packetBuffer[indexstr]='\0';      
    }    
    else if (strncmp(packetBuffer, "WTGR", 4)==0){
      indexstr=4;      
      for (c=EM_TRIGGER_OFFSET; c <= EM_TRIGGER_END; c++){EepromWrite(c, packetBuffer[indexstr++]);}
      packetBuffer[0]=COMPLETED;         
    }   
    else if (strncmp(packetBuffer, "READHOR", 7)==0){ 
       p=(packetBuffer[7] * EM_TIME_DAY_SIZE) + EM_TIME_WEEKLY_OFFSET;
       strcpy(packetBuffer, "EHR");       
       packetBuffer[3]=1;  
       indexstr=4;  
       for (c = 0; c<80; c++){packetBuffer[indexstr++]=EepromRead(p++)+1; }
       packetBuffer[indexstr]='\0';   
    }
    else if (strncmp(packetBuffer, "HOREAD", 6)==0){
      p=(packetBuffer[7] * EM_TIME_DAY_SIZE) + EM_TIME_WEEKLY_OFFSET;
      p+=(packetBuffer[6]-1) * 80;
      strcpy(packetBuffer, "EHR");
      packetBuffer[3]=packetBuffer[6];
      indexstr=4;
      
      for (c = 0; c<80; c++){packetBuffer[indexstr++]=EepromRead(p++)+1; }
      packetBuffer[indexstr]='\0';	   
    }
    else if (strncmp(packetBuffer, "HORWRI", 6)==0){
      p=(packetBuffer[7] * EM_TIME_DAY_SIZE) + EM_TIME_WEEKLY_OFFSET;
      byte data=packetBuffer[6];
      p+=(data-1)*80;
      indexstr=8;
      for (c=0; c<80 ;c++){ EepromWrite(p++, packetBuffer[indexstr++]);}
	    strcpy(packetBuffer, "HWRT");
	    packetBuffer[4]=data;
	    packetBuffer[5]='\0';
    }
    else if (strncmp(packetBuffer, "SSCE", 4)==0){     
      SelectScene(packetBuffer[4]);
      EnvioEstadoActual();
      return;
    }
    else if (strncmp(packetBuffer, "WESC", 4)==0){        
      p = EM_ESCENES_OFFSET + ((packetBuffer[4]-1) * S_ESCENES ) ;  
      for (c = 0; c < S_ESCENES; c++){EepromWrite(p+c, packetBuffer[5+c]-1);}
      packetBuffer[0]=COMPLETED;       
    }
    else if (strncmp(packetBuffer, "RESC", 4)==0){
       packetBuffer[0]='V';packetBuffer[1]='E';packetBuffer[2]='S';packetBuffer[3]='C';
       p = ((packetBuffer[4] -1) * S_ESCENES ) + EM_ESCENES_OFFSET;  
       indexstr=5;  
       byte data;
       for (c = 0; c < S_ESCENES; c++){
         data=EepromRead(p + c);
         if (data<=254){data++;}
         packetBuffer[indexstr++]=data;
       }
       packetBuffer[indexstr]='\0';      
    }        
    else if (strncmp(packetBuffer, "ESTADOINST",10)==0){      
      strcpy(packetBuffer, "ESTACT");     
      indexstr=6;
      packetBuffer[indexstr++]=TipoDia + 1;
      packetBuffer[indexstr++]=hour + 1;  
      packetBuffer[indexstr++]=minute + 1;
      packetBuffer[indexstr++]=dayOfMonth + 1;
      packetBuffer[indexstr++]=month + 1;
      packetBuffer[indexstr++]=year + 1;       
      packetBuffer[indexstr]='\0';  
    }

    else if (strncmp(packetBuffer, "ENABLEHOR", 9)==0 ){      
         
       strcpy(packetBuffer, "ENHOR");       
       indexstr=5;
       p=EM_EN_TIMETABLE_OFFSET;
       for (c=0; c<N_EN_TIMETABLE; c++){packetBuffer[indexstr ++]=EepromRead(p++)+1;}
       packetBuffer[indexstr]='\0';       
    }
    else if (strncmp(packetBuffer, "WHOR", 4)==0){
       indexstr=4;
       p=EM_EN_TIMETABLE_OFFSET;
       for (c = 0; c<N_EN_TIMETABLE; c++){EepromWrite(p++, packetBuffer[indexstr++]-1); }
       packetBuffer[0]=COMPLETED;
    }    
    else if (strncmp(packetBuffer, "CONENABLE", 9)==0){
              
       strcpy(packetBuffer, "ENCON"); 
       indexstr=5; 
       for (c = 0; c < 10; c++){
       if (Condicionados[c]==true){ packetBuffer[indexstr]=2;}else{packetBuffer[indexstr]=1;}indexstr++;}
       packetBuffer[indexstr]='\0';     
    }    
    else if (strncmp(packetBuffer, "WCON", 4)==0){  
      indexstr=4; 
      for (c = 0; c<10; c++){
         if ( packetBuffer[indexstr]==2){Condicionados[c] = true;}else{Condicionados[c] = false;}
         indexstr++;
       }
       packetBuffer[0]=COMPLETED;
    }   
    else if (strncmp(packetBuffer, "COMANDO", 7)==0){
      String Rsp= RunCommand(packetBuffer[7]);
      indexstr=Rsp.length();;
      for (c=0; c < indexstr;c++){packetBuffer[c]=Rsp.charAt(c);}
      packetBuffer[indexstr]='\0'; 
  
    } 
   else if (strncmp(packetBuffer, "DOCOMAN", 7)==0){
      PersonalFunctions(packetBuffer[7]);
      packetBuffer[0]=COMPLETED;       
    }  
    else if (strncmp(packetBuffer, "TIMPERSIANA", 11)==0){
      strcpy(packetBuffer, "LECPE");      
      indexstr=5;       
      p=EM_UP_TIM_SHUTTER_OFFSET;  
      for (c=0; c < (N_UP_TIM_SHUTTER *2) ; c++){packetBuffer[indexstr++]=EepromRead(p++)+1; }   
      packetBuffer[indexstr]='\0';      
    }
    else if (strncmp(packetBuffer, "WCOW", 4)==0){
      byte data1 = packetBuffer[5]-1;
      byte data2 = packetBuffer[6]-1;
      indexstr=packetBuffer[4]-1;
      p = (indexstr*2) + EM_SETPOINTS_OFSSET;      
      EepromWrite(p, data1);EepromWrite(p+1, data2);
      Consignas[indexstr]=word(data1, data2) ;      
      SetPointPacket();
    }
    else if (strncmp(packetBuffer, "SETPOINT", 8)==0){SetPointPacket(); } 

    else if (strncmp(packetBuffer, "WPERS", 5)==0){
      p=EM_UP_TIM_SHUTTER_OFFSET;
      indexstr=5;
      for (byte  i = 0; i< (N_UP_TIM_SHUTTER *2) ;i++){EepromWrite(p++, packetBuffer[indexstr++]-1); }
      #ifdef EXC_NumeroPersianas
        ReiniciarTiempoPersianas();
      #endif
      packetBuffer[0]=COMPLETED;       
    } 
    else if (strncmp(packetBuffer, "USEPA", 5)==0){
      #ifdef EXC_SERVER  
        char Rsp[]="USEPOX";
        Rsp[5]=packetBuffer[5];
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(Rsp);
        Udp.endPacket();
        InternalPacketIn(packetBuffer[6],packetBuffer[7],packetBuffer[8],packetBuffer[9],packetBuffer[10],packetBuffer[11],packetBuffer[12],packetBuffer[13],packetBuffer[14],packetBuffer[15]);
        return;
      #endif 
      return;    
    }  
    else if (strncmp(packetBuffer, "AlmOk", 5)==0){
      Alarms[packetBuffer[5]-1]=AlarmSent;
      IntCom=0;
      #ifdef EXC_DEBUG_MODE   
       Serial.print("Alarma Enviada - ");Serial.println((int)packetBuffer[5]-1); 
      #endif
      return;
    }  
    else if (strncmp(packetBuffer, "UsePa", 5)==0){
       #ifdef EXC_DEBUG_MODE   
       Serial.print("Envio Packete completo ");Serial.println((int)packetBuffer[5]-1);
      #endif
      
      #ifdef EXC_SERVER  
        IntCom=0;
        RemotePackets[packetBuffer[5]-1].Send=false;
      #endif
      return;      
    }  
    else if (strncmp(packetBuffer, "RESTPER", 7)==0){
      #ifdef EXC_NumeroPersianas  
        ReiniciarPosicionPersiana(packetBuffer[7]-1);
      #endif 
      strcpy(packetBuffer,"RESETEANDO PERSIANA");        
    }
        
    else if (strncmp(packetBuffer, "HIST", 4)==0){
      #ifdef SD_CARD
        
       byte b=packetBuffer[4];
       
       char Ruta[] = {'H', 'I', 'T', '/', '0', '0', '-','0', '0', '-','0', '0', '.','E','X','C','\0'};
       String Val;
       Val = String(b);
       if (Val.length()==2){
         Ruta[4]=Val.charAt(0);
         Ruta[5]=Val.charAt(1);
       }
       else{
         Ruta[5]=Val.charAt(0);
       }
       b=packetBuffer[5];
       Val = String(b);
       if (Val.length()==2){
         Ruta[7]=Val.charAt(0);
         Ruta[8]=Val.charAt(1);
       }else{
         Ruta[8]=Val.charAt(0);
       }
       b=packetBuffer[6];
       Val = String(b);
       if (Val.length()==2){
         Ruta[10]=Val.charAt(0);
         Ruta[11]=Val.charAt(1);
       }else{
         Ruta[11]=Val.charAt(0);
       }
      ReadHistorico(packetBuffer[7]-1,Ruta);
      #else
        strcpy(packetBuffer , "NOFOUND!!");
      #endif  
    }
    else {strcpy(packetBuffer,"REPETIRMSG");}   
       
    if (packetBuffer[0]== COMPLETED){ strcpy(packetBuffer,"COMPLETED");}
    EnviarRespuesta(packetBuffer);
    return;  
  }
  
  #ifdef EXC_SERVER
  if (IntCom > 0){return;}
  for (int a=0;a<=19;a++){
      if (Alarms[a]==SendingAlarm){
        Udp.beginPacket(ServerIP, SeverPort);
        char mS[]="ALARMA0";
        mS[6]=a+1;
        Udp.write(mS);
        Udp.endPacket();
        IntCom=3;
         #ifdef EXC_DEBUG_MODE   
              Serial.print("Send Alarm ");Serial.println((int)a+1);             
         #endif
        return;
      }  
    }
    unsigned int t=millis();
    if (InternalComPacket>=RemotePacketNumber){InternalComPacket=0;}
      for (byte pas = InternalComPacket; pas <RemotePacketNumber; pas++){
        if (RemotePackets[pas].Send){
          if (RemotePackets[pas].LastSend>t){RemotePackets[pas].LastSend=t;}
          if ((RemotePackets[pas].LastSend +3000)<= t){
            #ifdef EXC_DEBUG_MODE   
              Serial.print("Send User Packet Number ");Serial.println(pas);             
            #endif
            RemotePackets[pas].LastSend=t;
            Udp.beginPacket(ServerIP, SeverPort);
            Udp.write(RemotePackets[pas].Data);
            Udp.endPacket();
            InternalComPacket=pas+1;
            IntCom=3;
            break;
          }       
        }
        else{if (t>3000){RemotePackets[pas].LastSend=t-3000;}else{RemotePackets[pas].LastSend=0;}}
      }InternalComPacket=0;
    
      
  #endif
  
}
void SetPointPacket(){
  strcpy(packetBuffer, "SEPOI");
  int indexstr=5;        
  for (byte c=0; c<10; c++){
    byte bl,bh;bl= lowByte(Consignas[c]); bh= highByte(Consignas[c]);
    if (bl<255){bl++;}
    if (bh<255){bh++;}  
    packetBuffer[indexstr++]= bl;packetBuffer[indexstr++]= bh;
  }
  packetBuffer[indexstr]='\0';   
}
void EnviarRespuesta(char  *ReplyBuffer)
{
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
}

//MODIFICADO.
void EnvioEstadoActual()
{
  
  char Respuesta[35];
  byte  indexstr,c;
  
  strcpy(Respuesta, "EVAL"); 
  indexstr=4;
  
  for (c=0; c<30;c++)
  {
     if ((c)<Number_Circuit){
       Respuesta[indexstr]=circuits[c].Value+1;
     }
     else{
       Respuesta[indexstr]=1;
     }  
     indexstr++;
  }
 
  Respuesta[indexstr]='\0';
  EnviarRespuesta(Respuesta); 
}


void SelectScene(byte Dir)
{
  int p;
  byte c,v;
  
  p= (int) Dir;
  p= EM_ESCENES_OFFSET + ((p-1) * S_ESCENES );
  for (c =0 ; c<Number_Circuit; c++){v =EepromRead(p + c); if (v < 250){circuits[c].Value = v;}}
  Scene_Selected(Dir);
  #ifdef EXC_DEBUG_MODE   
      Serial.print("Scene Number ");Serial.print(Dir); Serial.println("Selected");             
   #endif
  
}
//Envento cada minuto.
void ActualizaMinuto()
{  
  int Reg;
  
    #ifdef SD_CARD
      if (minute==0){GuardaHistorico();}
      else{
        MinutesLstHist++;
        if (MinutesLstHist>=MinutesHistorico){GuardaHistorico();}
      }
    #endif

  
    //Adelanta la hora.Apartir del dia 25 de Marzo, busca el primer domingo
    //y cuando se han las 2 de la noche adelanta el reloj una hora
    if( minute==0 && Enable_DaylightSavingTime==true ){AutomaticDST();}
    if (hour==4){HoraRetrasa=false;}      

    minutoMemory=minute;
    
    if (ForcingSpecialDay1 || ForcingSpecialDay2)
    {
      if (ForcingSpecialDay1){TipoDia=8;}else{TipoDia=9;}
    }
    else{
        TipoDia=dayOfWeek;
        
        //Verificacion Dia Especial 1
        for (Reg=EM_DATE_ESPECIAL1_OFSSET; Reg <= EM_DATE_ESPECIAL1_END; Reg=Reg +S_DATE_ESPECIAL){
          if (month == EepromRead(Reg) && dayOfMonth== EepromRead(Reg+1)){      
              TipoDia=8;
          }
        }
        //Verificacion Dia Especial 2
        for (Reg=EM_DATE_ESPECIAL2_OFSSET; Reg <= EM_DATE_ESPECIAL2_END; Reg=Reg +S_DATE_ESPECIAL){
           if (month == EepromRead(Reg) && dayOfMonth== EepromRead(Reg+1)){      
              TipoDia=9;
          }
        }
    }

    
    int r= ((TipoDia-1)* EM_TIME_DAY_SIZE)+ EM_TIME_WEEKLY_OFFSET;
    
    //Read Timetable for days.
    boolean UpData=false;
    for (Reg=r; Reg <(r+EM_TIME_DAY_SIZE);Reg=Reg + S_TIME_ESPECIAL){if ( hour==EepromRead(Reg) && minute==EepromRead(Reg+1)){UpData=true; timeChangeCircuit(Reg+2);}}

    //Read Triggers 
    for (Reg=EM_TRIGGER_OFFSET; Reg < EM_TRIGGER_END ; Reg=Reg + S_TRIGGER){if ( hour==EepromRead(Reg) && minute==EepromRead(Reg+1)){UpData=true; EepromWrite(Reg, 66);timeChangeCircuit(Reg+2);}}

    if (UpData){SecutityCopy();}
    
    for (int c=0;c<Number_Circuit;c++){    
      if ((circuits[c].Type==Riego_Temporizado)||(circuits[c].Type==Timer)){
        if (circuits[c].Value>=1) {
          if (circuits[c].Out1_Value==true){circuits[c].Value--;}
          else{circuits[c].Out1_Value=true;}        
        }
        if (circuits[c].Value==0){circuits[c].Out1_Value=false;}
      }
    }
}
void timeChangeCircuit(int addressEE)
{
  byte ci=EepromRead(addressEE);  
  if (EepromRead(EM_EN_TIMETABLE_OFFSET+ci)==0){return;}
  byte val=EepromRead(addressEE+1);
  
  if (ci < Number_Circuit){circuits[ci].Value=val;}
  else if((ci < 40)&&(ci > 29)){SelectScene(ci-29);}  
  else if((ci < 50)&&(ci > 39)){if (val==1){Condicionados[ci-40]=true;}else{Condicionados[ci-40]=false;}} 
  else if((ci < 60)&&(ci > 49)){PersonalFunctions(ci-49);}  
}
void ReadDate(){
  char Respuesta[26];
  Respuesta[0]='E';
  Respuesta[1]='S';
  Respuesta[2]='T';
  Respuesta[3]='A';
  Respuesta[4]='C';
  Respuesta[5]='T';
       
  Respuesta[6]=TipoDia + 1;
  Respuesta[7]=hour + 1;
  Respuesta[8]=minute + 1;
  Respuesta[9]=dayOfMonth + 1;
  Respuesta[10]=month + 1;
  Respuesta[11]=year + 1;
       
  Respuesta[12]='\0';
  
  EnviarRespuesta(Respuesta); 
}


void CargaHora()
{
  getDateClock(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  if (minute != minutoMemory){ActualizaMinuto();NewMinute();  }
}

/******************************************************************/
//  FUNCIONES RELOJ
/*****************************************************************/

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val){ return ( (val/10*16) + (val%10) );}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val){return ( (val/16*10) + (val%16) );}


/*void setDateCLOCK(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99*/

void setDateCLOCK(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
   Wire.beginTransmission(DS_RTC);
   Wire.write( (byte)0);
   //Wire.write(0);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
}

// Gets the date and time from the ds1307
void getDateClock(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,byte *month,byte *year)
{
  #ifdef EXC_DEBUG_MODE   
      Serial.print("Get Date Time ");               
  #endif
  // Reset the register pointer
  Wire.beginTransmission(DS_RTC);
  Wire.write( (byte)0);
  //Wire.write(0);
  if (Wire.endTransmission()==0){
    Wire.requestFrom(DS_RTC, 7);
      if (Wire.available()==7){   
      *second     = bcdToDec(Wire.read() & 0x7f);
      *minute     = bcdToDec(Wire.read());
      *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
      *dayOfWeek  = bcdToDec(Wire.read());
      *dayOfMonth = bcdToDec(Wire.read());
      *month      = bcdToDec(Wire.read());
      *year       = bcdToDec(Wire.read());
      #ifdef EXC_DEBUG_MODE   
        Serial.println("OK");               
      #endif
      SegUpdtHora=30;
      return;
    }
  }
  SegUpdtHora=6;
  #ifdef EXC_DEBUG_MODE   
    Serial.println("ERROR");               
  #endif
}

/******************************************************************/
// REFRESCAR
/*****************************************************************/
#ifdef ExControlMail
boolean Notification(String Text){
    EspRfrIp=60;
    #ifdef EXC_WIFI_SHIELD
       if (EstadoWifiOk()==false){return false;}
    #endif

    if ((Connecting)||(ExControlMail=="")){return false;}
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Notification = "+ Text);               
    #endif
    Text.replace(" ", "%20%20");
    boolean result =CreateCabHTTP("GET http://www.ex-connect.es/Users/Noti?Mail=",Text);
    if (result){result=ComproRespuestaHTTP();}
    return result;
 }
  void connectAndRfr(){
    EspRfrIp=60;
    #ifdef EXC_WIFI_SHIELD
       if (EstadoWifiOk()==false){return;}
    #endif
    if ((Connecting)||(ExControlMail=="")){return;}   
      
    boolean result = CreateCabHTTP("GET http://www.ex-connect.es/Users/IpSet?Mail=","");
    if (result){ComproRespuestaHTTP(); }    
  }

 
boolean CreateCabHTTP(String URL, String Key2){
  
     #ifdef EXC_ENABLE_WATCH_DOG
      wdt_disable();
    #endif 
      
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Coneccting http server");               
    #endif
    if (client.connect("www.ex-connect.es", 80)) {
       #ifdef EXC_DEBUG_MODE   
        Serial.println("Conected");               
      #endif
      
      client.print(URL);
      //if (Key2==""){client.print(Mail + "&Key=" + Key);}
      
      if (Key2==""){client.print(ExControlMail);client.print("&Key="); client.print(ExControlPass); }    
      else{client.print(ExControlMail);client.print("&Key=");client.print(ExControlPass);client.print("&Key2=");client.print(Key2);}  
      //{client.print(Mail + "&Key=" + Key + "&Key2=" + Key2);}
          
      client.println(" HTTP/1.1");
      client.println("Host: www.ex-connect.es");
      client.println("Connection: close");
      client.println();
      Connecting=true;
      #ifdef EXC_ENABLE_WATCH_DOG
        wdt_enable(WDTO_8S); 
      #endif 
      return true;
    }
    #ifdef EXC_DEBUG_MODE   
      Serial.println("ERROR Coneccting server");               
    #endif
    #ifdef EXC_ENABLE_WATCH_DOG
        wdt_enable(WDTO_8S); 
      #endif 
    Connecting=false;
    EspRfrIp=65;
    return false;  

}

boolean ComproRespuestaHTTP(){
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Esperando Respuesta Server");               
    #endif
    int Reintento=0;
    while(true){
      if (client.available()) {
         while(client.connected()) {
           while (client.available()) {
             char c = client.read();
             #ifdef EXC_DEBUG_MODE   
                Serial.write(c);              
              #endif             
           }
         }
        client.stop();
        EspRfrIp=60;
        Connecting=false;
        #ifdef EXC_DEBUG_MODE   
          Serial.println("Respuesta Completa");               
        #endif
        return true;
      }
      else{
        unsigned int TimIni=millis();
        boolean Wait=true;
        while (Wait){
          SystemLoop();
          unsigned int t=millis();
          if (t<TimIni){TimIni=t;}
          else if ((TimIni + 100 )<t){Wait=false;}       
        }
        //while (TimEnd>millis()){SystemLoop();}
        if (Reintento >= 160){
          #ifdef EXC_DEBUG_MODE   
              Serial.println("No se recibio respuesta de servidor");               
          #endif
          client.stop();
          client.flush();
          Connecting=false;
          EspRfrIp=65;
          return false;
        }
      }
      Reintento++;
    }
  }

  void SendAlarms(){
    for (int a=0;a<=19;a++){
        if (Alarms[a]==SendingAlarm){
          boolean res=Notification(GetAlarmsName(a));
          if (res){Alarms[a]=AlarmSent;}
          break;
        }  
      }
    
  }
#endif 
void LoadSecutityCopyEEP(){
  
  #ifdef EXC_DEBUG_MODE   
    Serial.println("Load Security copy");               
  #endif
  byte c;
  for (c=0;c<Number_Circuit;c++){
    byte v=EepromRead( EM_STATECOPY_OFSSET + c );
    if (v==255){v=0;}
    if (circuits[c].Type>100){        
        if (v>=100){circuits[c].Out1_Value=true;v=v-100;}
        if(v==1){circuits[c].Value=1; circuits[c].Out2_Value=true;}else{circuits[c].Value=0; circuits[c].Out2_Value=false;}  
    }else{circuits[c].Value=v;}    
  }
  for (c=0;c<10;c++){
    byte b=EepromRead( EM_STATECOPY_OFSSET +30 + c );
    if (b==1){Condicionados[c]=true;}else{Condicionados[c]=false;}
  }
}
void LoadSecutityCopy(){
  
  #ifdef SD_CARD
     LoadSecutityCopySD();
   #else
     LoadSecutityCopyEEP();
   #endif  
}
void SecutityCopy(){
  boolean Exit=true;
    int c=0;
   for (c=0;c<Number_Circuit;c++){if (circuits[c].Value!=circuits[c].CopyRef){Exit=false;circuits[c].CopyRef=circuits[c].Value; }}
    
    short ver=0; byte p=1;
    for (c=0;c<10;c++){
      if (Condicionados[c]){ver+=p;}
      p=p*2;
    }
    if (ver!=CondiVer){CondiVer=ver;Exit=false;}        
    
    if (Exit){return;}
   #ifdef SD_CARD
     SecutityCopySD();
   #else
     SecutityCopyEEP();
   #endif  
}
void SecutityCopyEEP(){
  byte c;

  for (c=0;c<Number_Circuit;c++){
    byte v=circuits[c].Value;         
    if (circuits[c].Type>100){if (circuits[c].Out1_Value){v=v+100;}}
    EepromWrite(EM_STATECOPY_OFSSET + c,v);
  }    
  for (c=0;c<10;c++){
    byte b=0;
    if (Condicionados[c]){b=1;}    
    EepromWrite(EM_STATECOPY_OFSSET +30 + c,b);
  }
}
#ifdef SD_CARD
  void LoadSecutityCopySD(){
    if (SdOk==false){LoadSecutityCopyEEP();return;}
    EnableSD();
    if (SD.exists("elc.txt")) {
            #ifdef EXC_DEBUG_MODE   
              Serial.println("Load circuit state");           
            #endif  
            SdFile = SD.open("Elc.txt");
            
            if (SdFile) {
              int c=0; 
              while ((SdFile.available())&&(c<Number_Circuit)) {
                byte v =SdFile.read();
                if (v==255){v=0;} 
                if (circuits[c].Type>100){
                  if (v>=100){circuits[c].Out1_Value=true;v=v-100;}
                  if(v==1){circuits[c].Value=1; circuits[c].Out2_Value=true;}else{circuits[c].Value=0; circuits[c].Out2_Value=false;}  
                }else{circuits[c].Value=v;}                 
                c++;}
              while ((SdFile.available())&&((c-Number_Circuit)<10)) {byte v=SdFile.read(); if (v==1){Condicionados[c-Number_Circuit]=true;}else {Condicionados[c-Number_Circuit]=false;}c++;}
              SdFile.close();
            }
          }
        else{ 
          
          #ifdef EXC_DEBUG_MODE   
            Serial.println("ERROR - Found file Elc.");
          #endif
        }
       EnableEthernet();        
  }
  void SecutityCopySD(){
    if (SdOk==false){SecutityCopyEEP();return;}  
    
  
    EnableSD();
    if (SD.exists("elc.txt")) {SD.remove("elc.txt");}
    SdFile = SD.open("elc.txt", FILE_WRITE);
    if (SdFile) {
      int c=0;
      for (c=0;c<Number_Circuit;c++){
        byte v=circuits[c].Value;        
        if (circuits[c].Type>100){if (circuits[c].Out1_Value){v=v+100;}}
        SdFile.write(v);
      }      
          
      for (c=0;c<10;c++){byte b=0;if (Condicionados[c]){b=1;}SdFile.write(b);}    
    
      SdFile.close();
      #ifdef EXC_DEBUG_MODE   
         Serial.println("Security copy sd full");               
      #endif
      
    }  
    
    else { 
      #ifdef EXC_DEBUG_MODE   
         Serial.println("error opening sd file");               
      #endif
    }
    EnableEthernet();
  }
  
  void GuardaHistorico(){
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Iniciando escritura  fichero historico");   
    #endif
    MinutesLstHist=0;
    if (SdOk==false){return;}
    EnableSD();
    #ifdef EXC_ENABLE_WATCH_DOG
    wdt_reset();
    #endif
    char Ruta[] = {'H', 'I', 'T', '/', '0', '0', '-','0', '0', '-','0', '0', '.','E','X','C','\0'};
    String Val;
    Val = String(year);
    if (Val.length()==2){Ruta[4]=Val.charAt(0);Ruta[5]=Val.charAt(1);}else{Ruta[5]=Val.charAt(0);}
    Val = String(month);
    if (Val.length()==2){Ruta[7]=Val.charAt(0);Ruta[8]=Val.charAt(1);}else{Ruta[8]=Val.charAt(0);}
    Val = String(dayOfMonth);
    if (Val.length()==2){Ruta[10]=Val.charAt(0);Ruta[11]=Val.charAt(1);}else{Ruta[11]=Val.charAt(0);}
    SdFile = SD.open(Ruta, FILE_WRITE);
     #ifdef EXC_DEBUG_MODE   
      Serial.println(Ruta);   
    #endif
    if (SdFile) {
      SdFile.write(hour);SdFile.write(minute);    
      for (int b=0;b<Number_Sensor;b++){
        byte bl= lowByte(Sensors[b].Value);
        byte bh= highByte(Sensors[b].Value);
        SdFile.write(bl);SdFile.write(bh);      
      }                
      //SdFile.println(Line);
      SdFile.close();
      #ifdef EXC_DEBUG_MODE   
        Serial.println("escritura fichero historico completa");   
      #endif
    }
    else
      {
        #ifdef EXC_DEBUG_MODE   
        Serial.println("Error escritura fichero historico");   
        Serial.println(Ruta);
        #endif
     }
     EnableEthernet();
}
byte EndHitorico(){
  strcpy(packetBuffer, "HT"); 
  packetBuffer[2]=255;packetBuffer[3]=255;packetBuffer[4]='\0';
}
void ReadHistorico(byte Linea,char Ruta[]){
    EnableSD();
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Iniciando lectura  fichero historico");   
      Serial.print("File: "); Serial.println(Ruta); 
    #endif
    int HistSize=( Number_Sensor * 2 ) +2;   
    if (SdOk==false){EndHitorico();return;}
    
    
    #ifdef EXC_ENABLE_WATCH_DOG
      wdt_reset();
    #endif
    
    SdFile = SD.open(Ruta);
    
    
    if (SdFile) {    
      unsigned long Sze=SdFile.size();      
      int Pos=Linea*HistSize;
      /*
      Serial.print("Linea ");Serial.println(Linea);
      Serial.print("Size ");Serial.println(Sze);
      Serial.print("HistSize ");Serial.println(HistSize);
      
      Serial.print("Pos ");Serial.println(Pos);
      Serial.print("Total ");Serial.println(Pos+HistSize);*/
      if ((Pos+HistSize) >= Sze){SdFile.close();EnableEthernet();EndHitorico();return;}
      else{
        SdFile.seek(Pos);//Buscamos posicion 
        strcpy(packetBuffer, "HT"); //Cabecera
        for(int c=0;c<HistSize;c++){
          byte b=SdFile.read();
          if (b<255){b++;}
          packetBuffer[c+2]=b;
        } 
        packetBuffer[HistSize+2]='\0'; 
        SdFile.close();EnableEthernet();
        return;     
      }      
    }
    else{
      #ifdef EXC_DEBUG_MODE   
        Serial.println("NO EXISTE FICHERO");   
      #endif
    }
    EnableEthernet();EndHitorico();return;
}
String ReadFileLine(byte Linea,char Ruta[]){
    
    if (SdOk==false){return"NOFOUND!!";}
    EnableSD();
    #ifdef EXC_ENABLE_WATCH_DOG
      wdt_reset();
    #endif
    
    byte Lin=0;
    String Resultado;
    byte Bin;
    SdFile = SD.open(Ruta);
    if (SdFile) {
    while (SdFile.available()) {
      Bin=SdFile.read();
      if (Bin==13){Lin++;SdFile.read();}
      else
      {
        if (Lin==Linea){Resultado=Resultado+(char(Bin));}
        if (Lin>Linea){SdFile.close();EnableEthernet();
          if (Resultado==""){return"NOFOUND!!";}else {return Resultado;}
        }
      }
      }
      SdFile.close();EnableEthernet();return"NOFOUND!!";
    
    }
    EnableEthernet();return"NOFOUND!!";
}

#endif
#ifdef EXC_NRF24 
  void NrfInput(int dv){  
    TimRcv=0;
    for (int m=0;m<8;m++){
      if (m<6){
        if (NrfInRegs[dv][m]!= OldNrfInRegs[dv][m]){
          int r=dv*100;r=r+m+2000;
          if (NrfInRegs[dv][m]==0){SwicthStateChange(r,LOW);}
          if (NrfInRegs[dv][m]==1){SwicthStateChange(r,HIGH);}
          if ((NrfInRegs[dv][m]>100)&& (OldNrfInRegs[dv][m]<110)){ShortInput(r);}
          if ((NrfInRegs[dv][m]==100)&& (OldNrfInRegs[dv][m]==110)){LongInputEnd(r);}
          if (NrfInRegs[dv][m]==110){LongInput(r);}
          OldNrfInRegs[dv][m]= NrfInRegs[dv][m];         
        }
      } 
    }    
  }

  void NrfInputIni(byte dv){TimRcv=0; for (int m=0;m<8;m++){OldNrfInRegs[dv][m]=NrfInRegs[dv][m];}}
  void IniciarNrf(){    
      EnableRF24L01(); 
      NrfCiclos=0;
      TimRcv=0;
      // Setup and configure rf radio
      radio.begin();
      radio.setPALevel(RF24_PA_HIGH);
      
      #ifdef EXC_NRF24SPEED 
        boolean res=false;
        switch (EXC_NRF24SPEED){
          case 0:{res= radio.setDataRate(RF24_250KBPS);}  
          case 1:{res= radio.setDataRate(RF24_1MBPS);} 
          case 2:{res= radio.setDataRate(RF24_2MBPS);}
          default:{res=true;}
        } 
        #ifdef EXC_DEBUG_MODE  
          if (res){Serial.println("Nrf new speed ok");}else{Serial.println("Nrf new speed ERROR");} 
        #endif     
      #endif
      
      radio.setChannel(RF24channel); 
      network.begin( this_node );
      //network.begin(/*channel*/ RF24channel, /*node address*/ this_node );
      EnableEthernet();
  }     
void EnableRF24L01(){
    #ifdef EXC_DEBUG_MODE  
        //Serial.println("SPI nRF24L01 ENABLED.");
   #endif
   #ifdef SS_ETHERNET 
     digitalWrite(SS_ETHERNET, HIGH);
   #endif
   #ifdef SS_SD 
     digitalWrite(SS_SD, HIGH);
   #endif
   #ifdef SS_UNO 
     digitalWrite(SS_UNO, HIGH);
   #endif
  
   #ifdef SS_nRF24L01
    digitalWrite(SS_nRF24L01,LOW);  
   #endif
   delay(1);
  } 
#endif 

void EnableEthernet(){
    #ifdef EXC_DEBUG_MODE   
    //Serial.println("SPI ETHERNET ENABLED.");
   #endif
   #ifdef SS_ETHERNET 
     digitalWrite(SS_ETHERNET, LOW);
   #endif 
   #ifdef SS_SD
     digitalWrite(SS_SD, HIGH);
   #endif
   #ifdef SS_UNO
     digitalWrite(SS_UNO, LOW);
   #endif
   #ifdef SS_nRF24L01   
    digitalWrite(SS_nRF24L01,HIGH); 
  #endif
  delay(1);
}

#ifdef SD_CARD
void EnableSD(){
    #ifdef EXC_DEBUG_MODE   
         //Serial.println("SPI SD ENABLED.");
    #endif
    //Para comunicar con sd desabilitamos w5100 spi (pin 10 HIGH)
    // Para comunicar con sd habilitamos sd spi (pin 4 low)
    #ifdef SS_nRF24L01   
       digitalWrite(SS_nRF24L01,HIGH); 
    #endif  
   #ifdef  SS_ETHERNET
     digitalWrite(SS_ETHERNET, HIGH); 
   #endif  
   #ifdef  SS_SD
     digitalWrite(SS_SD, LOW); 
   #endif 
   #ifdef  SS_UNO
    digitalWrite(SS_UNO, HIGH);
   #endif 
   #ifdef SS_nRF24L01   
       delay(1);
    #endif  
    delay(1);
  }
 #endif  
#ifdef THERMOSTAT_DTH22 

  void ReadDHT(){
    float t = dht.readTemperature();
    if ( isnan(t) ) {DhtReadError();}
    else{
       TemperatureDHT =  t;
       float h = dht.readHumidity();
       if ( isnan(h) ) {DhtReadError();return;}
       HumedadDHT=h;
       DhtErrorCount=0;
        #ifdef EXC_DEBUG_MODE   
          Serial.println("Lectura  DHT OK");  
          Serial.print(HumedadDHT, 1);
          Serial.print(",\t");
          Serial.print(TemperatureDHT, 1);
          Serial.print(",\t");  
          Serial.println(""); 
        #endif       
    } 
  }
  void DhtReadError(){
    if (DhtErrorCount<6){DhtErrorCount++;}
    else{ AveriaDHT=true;TemperatureDHT=0;HumedadDHT=0;}     
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Error lectura sensor DHT");   
    #endif   
  }
#endif 

#ifdef THERMOSTAT_DS18B20_NUMBER
  void InitDS18B20(){
      sensorTemp.begin();
      for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){sensorTemp.setResolution(Ds18B20Addres[c], TEMPERATURE_PRECISION);}
      sensorTemp.setWaitForConversion(true);
      RefreshTemperature();
      //sensorTemp.setWaitForConversion(false);
   }

   void RefreshTemperature(){
     sensorTemp.requestTemperatures();
     float f=0;
     for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){f= sensorTemp.getTempC(Ds18B20Addres[c]); if (f>-100){Temperature[c] =f;}}
    
    /* if (THERMOSTAT_DS18B20_NUMBER==1){Temperature[0] = sensorTemp.getTempCByIndex(0);}
     else{for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){Temperature[c] = sensorTemp.getTempC(Ds18B20Addres[c]);}}*/
  }
#endif 

#ifdef EXC_WIFI_SHIELD
boolean EstadoWifiOk(){
    if ((TimConexion > 34) && (WiFi.status() == WL_CONNECTED)){return true;}  
    else{return false;}
}
void ResetConexionWifi() {
    #ifdef EXC_DEBUG_MODE   
        Serial.println("Reseteando Conexon Wifi");  
    #endif
    //WiFi.disconnect(); delay(1000);
     Udp.stop();TimConexion=0;WiFi.disconnect();   
     #ifdef EXC_SERVER
      TimUdp=190;
     #endif   
}
void ConexionWifi() {
    #ifdef EXC_DEBUG_MODE   
        Serial.println("Iniciando Conexon Wifi");  
    #endif
    //WiFi.disconnect(); delay(1000);
   
    if ((Net_Type == A_POINT)&&(ApOn==false)){
      TimConexion=0;  
      
      #ifdef EXC_DEBUG_MODE   
        Serial.println("AP NOT ENABLED");  
      #endif
      return;
    }
    
    #ifdef EXC_STATIC_IP  
        WiFi.config(ip);
    #endif
    
    if (Net_Type == OPEN){status = WiFi.begin(ssid);  }//Open Network
    if (Net_Type == WPA){status = WiFi.begin(ssid, pass); }//WPA NETWORK
    if (Net_Type == WEP){status = WiFi.begin(ssid, keyIndex, pass); }//WEP 
    if (Net_Type == A_POINT){
      status  = WiFi.beginAP(ssid);
      #ifdef EXC_DEBUG_MODE
        if (status != WL_AP_LISTENING) {Serial.println("Creating access point failed");
        }else{Serial.println("Creating access point Ok");}
      #endif    
    }
    TimConexion=20;       
}
#endif

//Funciones alarmas
void SetAlarm(int Number){if ((Number<=19)&&(Alarms[Number]==WithoutAlarm)){Alarms[Number]=SendingAlarm;}}
void ResetAlarm(int Number){if ((Number<=19)&&(Alarms[Number]==AlarmSent)){Alarms[Number]=WithoutAlarm;}}
#ifdef EXC_ARDUINO_MEGA
//    ARDUINO MEGA..........................................................   
    

    byte EepromRead ( unsigned short eeaddress){return EEPROM.read(eeaddress);}
    void EepromWrite ( unsigned short eeaddress, byte data ){if(EepromRead(eeaddress) != data) EEPROM.write(eeaddress, data);}
   
#else
//    ARDUINO CON Atmeg328 (ARDUINO UNO,ARDUINO ETHERNET. ETC.............
///   USO DE MEMORIA EXTERNA.
  #ifdef IC24C32_I2C_ADDRESS 
  void InitI2CEeprom(unsigned short eeaddress ){
        Wire.beginTransmission(IC24C32_I2C_ADDRESS);
        Wire.write(highByte(eeaddress)); // MSB
        Wire.write(lowByte(eeaddress) ); // LSB
  }
  byte EepromRead( unsigned short eeaddress ) {
      byte R =0;      
      while (R<4){
        InitI2CEeprom(eeaddress);
        if (Wire.endTransmission()==0){
          Wire.requestFrom(IC24C32_I2C_ADDRESS,1);
          if (Wire.available() == 1){return Wire.read();}
        }
        #ifdef EXC_DEBUG_MODE   
          Serial.print("EEPROM READ ERROR - ");   
          Serial.println(R);
        #endif 
        delay(10);
        R++;
      }
		  return 0;  
  }
    
  void EepromWrite( unsigned short eeaddress, byte data ) {
    
    if(EepromRead(eeaddress) == data){return; }
      byte R =0;
      while (R<4){
        delay(5);
        InitI2CEeprom(eeaddress); 
        Wire.write(data);
        if (Wire.endTransmission()==0) {delay(5);return; }
        else{
          #ifdef EXC_DEBUG_MODE   
            Serial.print("EEPROM WRITE ERROR - ");   
            Serial.println(R);
          #endif   
          R++;              
        }           
      }
    }
  #else
    byte EepromRead ( unsigned short eeaddress){return 0;}
    void EepromWrite ( unsigned short eeaddress, byte data ){return;}
  #endif
#endif

#ifdef EXC_I2C_BOARD
  void SetI2CRelay(byte OutNumber, boolean On){byte b=0;while (OutNumber>7){ OutNumber=OutNumber-10;b++;}if (On){bitWrite(IoBoards[b].Outputs, OutNumber, 1);}else{bitWrite(IoBoards[b].Outputs, OutNumber, 0);}}
  byte ReadMCP23017(byte Device){
     byte R =0;      
     while (R<4){
      Wire.beginTransmission(0x20 + Device);  
      Wire.write(0x13);
      
      if (Wire.endTransmission()==0){
            Wire.requestFrom(0x20 + Device, 1);//Wire.requestFrom(IC24C32_I2C_ADDRESS,1);
            if (Wire.available() == 1){return Wire.read();}
          }
          #ifdef EXC_DEBUG_MODE   
            Serial.print("i2c INPUT  READ ERROR - ");   
            Serial.println(R);
          #endif 
          delay(10);
          R++;
        }
        InitMCP23017(Device);
        return 0;  
  }
  void WriteMCP23017(byte Device, byte  regValue){
    byte R =0;
        while (R<4){
          delay(5);//COMPROBAR CON Y SIN
          Wire.beginTransmission(0x20 + Device); 
          Wire.write(0x12);
          Wire.write(regValue);
          
          if (Wire.endTransmission()==0) {delay(5);return; }
          else{
            #ifdef EXC_DEBUG_MODE   
              Serial.print("i2c OUTPUT  WRITE ERROR - "); Serial.println(R);
            #endif   
            R++;              
          }           
        }
        InitMCP23017(Device);
    }
  
  void InitMCP23017(byte Device){

    byte R=0;
    while (R<4){//Configure Output
      Wire.beginTransmission(0x20 + Device);
      Wire.write(0x00); // IODIRA register
      Wire.write(0x00); // set all of port A to outputs
      Wire.write(0xFF); // set all of port b to input    
      if (Wire.endTransmission()==0) {
        
        #ifdef EXC_DEBUG_MODE   
          Serial.print("Init i2c setting ok, Device  ");Serial.print(Device);
        #endif
        break;}
      else{
        #ifdef EXC_DEBUG_MODE   
          Serial.print("Init i2c config error, Device  ");Serial.print(Device);Serial.print("ERROR COUNT"); Serial.println(R);
        #endif   
        R++;              
      }           
    }
  }
#endif
#ifdef EXC_RGB_LED
  void RgbRandomColor(){    
    switch (RGBrandomCount) {
    case 2: 
      
      if ((RGBredVal + RGBSpeed)>254){RGBgreenVal = 0;RGBredVal = 255;RGBrandomCount=0;}
      else{
        RGBredVal += RGBSpeed;
        RGBblueVal -= RGBSpeed; 
        RGBgreenVal=0;
      }
      break;
    case 1:  
      if ((RGBblueVal + RGBSpeed)>254){RGBredVal = 0;RGBblueVal = 255;RGBrandomCount=2;}
      else{
        RGBblueVal += RGBSpeed;
        RGBgreenVal -= RGBSpeed; 
        RGBredVal=0;
      }
      break;
    case 0: 
      if ((RGBgreenVal + RGBSpeed)>254){RGBblueVal = 0;RGBgreenVal = 255;RGBrandomCount=1;}
      else{
        RGBgreenVal += RGBSpeed;
        RGBredVal -= RGBSpeed; 
        RGBblueVal=0;
      }
      break;
    } 
  }
  
  void SetRGBLed(byte RedPin, byte GreenPin, byte BluePin,  byte Value, boolean InvertirSalida){  
    byte R,G,B;
    
    switch (Value) {

    case 199:    //Random
      R=RGBredVal;G=RGBgreenVal;B=RGBblueVal;
      //analogWrite( RedPin, 255 - RGBredVal );
      //analogWrite( GreenPin, 255 - RGBgreenVal );
      //analogWrite( BluePin, 255 - RGBblueVal );
      break;
    case 1:    //// 8388736 Purple
      R=128;G=0;B=28;
      //analogWrite( RedPin, 128);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 28);
      break;
    case 2://15631086 Viole
      R=238;G=130;B=238;
      //analogWrite( RedPin, 238);
      //analogWrite( GreenPin, 130);
      //analogWrite( BluePin, 238);
      break;
    case 3: //9055202 BlueViolet
      R=128;G=43;B=226;
      //analogWrite( RedPin, 138);
      //analogWrite( GreenPin, 43);
      //analogWrite( BluePin, 226);
      break;
    case 4://6970061 SlateBlue
      R=128;G=43;B=226;
      //analogWrite( RedPin, 106);
      //analogWrite( GreenPin, 90);
      //analogWrite( BluePin, 205);
      break;
    case 5:// 8087790 MediumSlateBlue
      R=123;G=104;B=238;
      //analogWrite( RedPin, 123);
      //analogWrite( GreenPin, 104);
      //analogWrite( BluePin, 238);
      break;
    case 6://255 blue
      R=0;G=0;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 255);
      break;
    case 7://65535 Aqua
      R=0;G=255;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 255);
      break;
    case 8://11591910 PowderBlue
      R=176;G=224;B=230;
      //analogWrite( RedPin, 176);
      //analogWrite( GreenPin, 224);
      //analogWrite( BluePin, 230);
      break;

    case 9://49151 DeepSkyBlue
      R=0;G=191;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 191);
      //analogWrite( BluePin, 255);     
      break;
    case 10://2142890 LightSeaGreen
      R=32;G=178;B=170;
      //analogWrite( RedPin, 32);
      //analogWrite( GreenPin, 178);
      //analogWrite( BluePin, 170);
      break;
    
    case 11: //6737322 MediumAquamarine
      R=102;G=205;B=170;
      //analogWrite( RedPin, 102);
      //analogWrite( GreenPin, 205);
      //analogWrite( BluePin, 170);
      break;  
    case 12:  //10025880 PaleGreen
      R=152;G=251;B=152;
      //analogWrite( RedPin, 152);
      //analogWrite( GreenPin, 251);
      //analogWrite( BluePin, 152);
      break; 
    case 13: //64154 MediumSpringGreen
      R=0;G=250;B=154;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 250);
      //analogWrite( BluePin, 154);
      break;  
    case 14:  //32768 Green
      R=0;G=128;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 128);
      //analogWrite( BluePin, 0);
      break;  
    case 15: //65280  green1 
      R=0;G=255;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 0);
      break;   
      
      
      
    
    case 16: //14423100 Crimson
      R=220;G=20;B=60;
      //analogWrite( RedPin, 220);
      //analogWrite( GreenPin, 20);
      //analogWrite( BluePin, 60);
      break; 
    case 17: //13047173 MediumVioletRed
      R=199;G=21;B=133;
      //analogWrite( RedPin, 199);
      //analogWrite( GreenPin, 21);
      //analogWrite( BluePin, 133);
      break;  
    case 18: //16729344 OrangeRed
      R=255;G=69;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 69);
      //analogWrite( BluePin, 0);
      break;   
    case 19: //16776960 Yelow
      R=255;G=255;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 0);
      break;  
    case 20: //10824234 Brown
      R=165;G=42;B=42;
      //analogWrite( RedPin, 165);
      //analogWrite( GreenPin, 42);
      //analogWrite( BluePin, 42);
      break; 
  case 21: //16711680 Red
      R=255;G=0;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 0);
      break; 
    case 22: //16716947 DeepPink
      R=255;G=20;B=147;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 20);
      //analogWrite( BluePin, 147);
      break;   
    
    case 23: //16747520 DarkOrange
      R=255;G=140;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 140);
      //analogWrite( BluePin, 0);
      break; 
    case 24: //12092939 DarkGoldenrod
      R=184;G=134;B=11;
      //analogWrite( RedPin, 184);
      //analogWrite( GreenPin, 134);
      //analogWrite( BluePin, 11);
      break;
    case 25: //16032864 SandyBrown
      R=244;G=164;B=96;
      //analogWrite( RedPin, 244);
      //analogWrite( GreenPin, 164);
      //analogWrite( BluePin, 96);
      break; 
      
      
      
    
    case 26: //16737094 tomato
      R=255;G=99;B=70;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 99);
      //analogWrite( BluePin, 70);
      break; 
     
    
    case 27: //16761035 Pink
      R=255;G=192;B=203;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 192);
      //analogWrite( BluePin, 203);
      break; 
    
    
    case 28: //16752762 LightSalmon
      R=255;G=160;B=122;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 160);
      //analogWrite( BluePin, 122);
      break; 
     
     
    case 29: //15787660 Khaki
      R=240;G=230;B=140;
      //analogWrite( RedPin, 240);
      //analogWrite( GreenPin, 230);
      //analogWrite( BluePin, 140);
      break; 
    
    case 30: //16113331 wheat
      R=245;G=222;B=179;
      //analogWrite( RedPin, 245);
      //analogWrite( GreenPin, 222);
      //analogWrite( BluePin, 179);
      break; 

    default:   // apagado
      R=0;G=0;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 0);
      break;
    }
    if(InvertirSalida){
      analogWrite( RedPin,255 - R);
      analogWrite( GreenPin,255 -  G);
      analogWrite( BluePin,255 -  B);   
    }
    else{
      analogWrite( RedPin, R);
      analogWrite( GreenPin, G);
      analogWrite( BluePin, B);  
    
    }
  }
#endif

 
