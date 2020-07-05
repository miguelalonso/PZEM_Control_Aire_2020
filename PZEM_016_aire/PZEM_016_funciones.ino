void control_aire(){

  if((millis()-timeOld5)<20000){
    return;
  }
  timeOld5=millis();
  
 //Serial.println("conf.temp_mode");Serial.println(conf.temp_mode);
  if (conf.temp_mode==0){control_sonoff(0, 0);return;}
  if (conf.temp_mode==1){control_sonoff(1, 1);return;}
  //if (conf.temp_mode==2){} //auto
  if (conf.temp_mode==3){return;} //manual

  float potencia_FV;

  potencia_FV= inversor.Power+pzem.power; //sumar las demas cuando se tengan
 
  if (potencia_FV > conf.temp_max){
    control_sonoff(1, 1);
    }
    if (potencia_FV < conf.temp_min){
     control_sonoff(1, 0);
    }
    
}

void control_sonoff(int inversor, int aire){

  if(inversor>0){
    //encender inversor
    control_inversor(1);
    }else{
      //apagar inversor
      control_inversor(0);
      }

      if(aire>0){
    //encender aire http://192.168.1.72/cm?cmnd=Power%20on
    control_aire(1);
    }else{
      //apagar aire http://192.168.1.72/cm?cmnd=Power%20off
      control_aire(0);
      }
      
      
}


void leer_PZEM016(){

  if((millis()-timeOld3)<10000){
    return;
  }
  timeOld3=millis();
  
  uint8_t result;
  uint16_t data[6];
  yield();
  node.clearResponseBuffer();
  ESP.wdtDisable();
  result = node.readInputRegisters(0x0000, 9);
  ESP.wdtEnable(1);
  //Serial.println("Nueva Medida:");
  //Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    pzem.voltage       = (node.getResponseBuffer(0x00) / 10.0f);
    pzem.current       = (node.getResponseBuffer(0x01) / 1000.000f);
    pzem.power         = (node.getResponseBuffer(0x03) / 10.0f);
    pzem.energy        = (node.getResponseBuffer(0x05) / 1000.0f);
    pzem.frequency     = (node.getResponseBuffer(0x07) / 10.0f);
    pzem.power_factor  = (node.getResponseBuffer(0x08) / 100.0f);

    //Serial.print("VOLTAGE:           ");   //Serial.println(pzem.voltage);       // V
    //Serial.print("CURRENT_USAGE:     ");   //Serial.println(pzem.current, 3);    // A
    //Serial.print("ACTIVE_POWER:      ");   //Serial.println(pzem.power);        // W
    //Serial.print("ACTIVE_ENERGY:     ");   //Serial.println(pzem.energy, 3);    // kWh
    //Serial.print("FREQUENCY:         ");   //Serial.println(pzem.frequency);           // Hz
    //Serial.print("POWER_FACTOR:      ");   //Serial.println(pzem.power_factor);
    //Serial.println("====================================================");
  }
  else {
    Serial.println("fallo en lectura PZEM-016");
    pzem.voltage       = 0;
    pzem.current       = 0;
    pzem.power         = 0;
    pzem.energy        = 0;
    pzem.frequency     = 0;
    pzem.power_factor  = 0;
     }  
  yield();
  leer_estado_tasmotas();
}

void resetEnergy(uint8_t slaveAddr){
  //The command to reset the slave's energy is (total 4 bytes):
  //Slave address + 0x42 + CRC check high byte + CRC check low byte.
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  mySerial.write(slaveAddr);
  mySerial.write(resetCommand);
  mySerial.write(lowByte(u16CRC));
  mySerial.write(highByte(u16CRC));
  delay(1000);
}


//**************************************************//
//*************LEER HTTP TASMOTAS*******************//
//**************************************************//

void leer_http_tasmotas() {

  if((millis()-timeOld4)<20000){
    return;
  }
  timeOld4=millis();
  
  if ((WiFi.status() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;
    
    if (http.begin(client, "http://"+String(conf.ip_inversor)+"/cm?cmnd=status%208")) {  // Inversor
      int httpCode = http.GET();
      if (httpCode > 0) {
        //Serial.printf("Tasmota Inversor abajo");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            payload_inversor = http.getString();
            printparameters( payload_inversor,1);
       }
      } 
     http.end();
    }
    if (http.begin(client, "http://"+String(conf.ip_aire)+"/cm?cmnd=status%208")) {  // Aire
      int httpCode = http.GET();
      if (httpCode > 0) {
        //Serial.printf("Tasmota AIre");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            payload_aire = http.getString();
            printparameters( payload_aire,0);
       }
      } 
     http.end();
    }
     
  }  //END

 
}

void printparameters(String payload,int device){
//{"StatusSNS":{"Time":"2020-06-29T22:22:24","ENERGY":{"TotalStartTime":"2020-06-24T09:25:29","Total":41.573,"Yesterday":6.147,"Today":8.286,"Power":2,"ApparentPower":22,"ReactivePower":21,"Factor":0.08,"Voltage":230,"Current":0.094}}}
   
            StaticJsonDocument<400> doc;
            DeserializationError error = deserializeJson(doc, payload);
            //serializeJson(doc, Serial);
            //Serial.println("*****************************************");
            //Serial.println("Response:");
            const char* cadena = doc["StatusSNS"]["Time"];
            //Serial.println(cadena);
            float Power= doc["StatusSNS"]["ENERGY"]["Power"];
            //Serial.print("Power :");//Serial.println(Power);
            float Total= doc["StatusSNS"]["ENERGY"]["Total"];
            //Serial.print("Total :");//Serial.println(Total);
            float Yesterday= doc["StatusSNS"]["ENERGY"]["Yesterday"];
            //Serial.print("Yesterday :");//Serial.println(Yesterday);
            float Today= doc["StatusSNS"]["ENERGY"]["Today"];
            //Serial.print("Today :");//Serial.println(Today);
            float ApparentPower= doc["StatusSNS"]["ENERGY"]["ApparentPower"];
            //Serial.print("ApparentPower :");//Serial.println(ApparentPower);
            float ReactivePower= doc["StatusSNS"]["ENERGY"]["ReactivePower"];
            //Serial.print("ReactivePower :");//Serial.println(ReactivePower);
            float Factor= doc["StatusSNS"]["ENERGY"]["Factor"];
            //Serial.print("Factor :");//Serial.println(Factor);
            float Voltage= doc["StatusSNS"]["ENERGY"]["Voltage"];
            //Serial.print("Voltage :");//Serial.println(Voltage);
            float Current= doc["StatusSNS"]["ENERGY"]["Current"];
            //Serial.print("Current :");//Serial.println(Current);

            if (device){
                inversor.Total=Total;
                inversor.Today=Today;
                inversor.Yesterday=Yesterday;
                inversor.Power=Power;
                inversor.ApparentPower=ApparentPower;
                inversor.ReactivePower=ReactivePower;
                inversor.Factor=Factor;
                inversor.Voltage=Voltage;
                inversor.Current=Current;
            }else{
                aire.Total=Total;
                aire.Today=Today;
                aire.Yesterday=Yesterday;
                aire.Power=Power;
                aire.ApparentPower=ApparentPower;
                aire.ReactivePower=ReactivePower;
                aire.Factor=Factor;
                aire.Voltage=Voltage;
                aire.Current=Current;
            }
 }
  

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  delay(0);
}

void postTransmission()
{
  delay(0);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void control_aire(int estado) {
        if (estado!=aire_estado_anterior){
          aire_estado_anterior=estado;
          WiFiClient client;
          HTTPClient http;
          String comando;
          if (estado){
              comando="http://"+String(conf.ip_aire)+"/cm?cmnd=Power%20on";
            }else{
              comando="http://"+String(conf.ip_aire)+"/cm?cmnd=Power%20off";
            }
          Serial.println(comando);
          if (http.begin(client, comando)) {  
            int httpCode = http.GET();
            if (httpCode > 0) {
             Serial.println("Aire controlado");
           } 
           http.end();
          }
        }
}

void control_inversor(int estado) {
    if (estado!=inversor_estado_anterior){
          inversor_estado_anterior=estado;
          WiFiClient client;
          HTTPClient http;
          String comando;
        if (estado){
            comando="http://"+String(conf.ip_inversor)+"/cm?cmnd=Power%20on";
          }else{
            comando="http://"+String(conf.ip_inversor)+"/cm?cmnd=Power%20off";
            //comando="http://"+String(conf.ip_inversor)+"/cm?cmnd=Power%20on";
          }
        Serial.println(comando);
        if (http.begin(client, comando)) {  
          int httpCode = http.GET();
          if (httpCode > 0) {
           Serial.println("Inversor controlado");
         } 
         http.end();
        }
    }
}

void leer_estado_tasmotas() {

    
  if ((WiFi.status() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;
    
    if (http.begin(client, "http://"+String(conf.ip_inversor)+"/cm?cmnd=status")) {  // Inversor
      int httpCode = http.GET();
      if (httpCode > 0) {
        //Serial.printf("Tasmota Inversor abajo");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload_inv = http.getString();
            printstatus( payload_inv,1);
       }
      } 
     http.end();
    }
    if (http.begin(client, "http://"+String(conf.ip_aire)+"/cm?cmnd=status")) {  // Aire
      int httpCode = http.GET();
      if (httpCode > 0) {
        //Serial.printf("Tasmota AIre");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload_air = http.getString();
            printstatus( payload_air,0);
       }
      } 
     http.end();
    }
     
  }  //END

 
}

void printstatus(String payload,int device){
//{"Status":{"Module":43,"DeviceName":"Tasmota Aire","FriendlyName":["Tasmota Aire"],"Topic":"tasmota_506AF9","ButtonTopic":"0","Power":0,"PowerOnState":3,"LedState":1,"LedMask":"FFFF","SaveData":1,"SaveState":1,"SwitchTopic":"0","SwitchMode":[0,0,0,0,0,0,0,0],"ButtonRetain":0,"SwitchRetain":0,"SensorRetain":0,"PowerRetain":0}} 
            StaticJsonDocument<400> doc;
            DeserializationError error = deserializeJson(doc, payload);
            //serializeJson(doc, Serial);
            //Serial.println("*****************************************");
            //Serial.println("Response:");
            int estado = doc["Status"]["Power"];
           if (device){
                inversor.estado=estado;
                inversor_estado_anterior=estado;
            }else{
                aire.estado=estado;
                aire_estado_anterior=estado;
            }
 }
 
