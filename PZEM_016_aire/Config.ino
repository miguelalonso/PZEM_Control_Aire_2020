void setupConfig(){

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  
  mySerial.begin(9600, SWSERIAL_8N1, D5, D6, false, 95, 11);
  node.begin(1, mySerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
    for ( int x = 0 ; x < 4 ; x++ ) {
    pinMode(GPIOPIN[x], OUTPUT);
    digitalWrite(GPIOPIN[x],HIGH);
  }
  
  Serial.begin ( 9600 );
  
    strcpy(conf.pwd,"121212");
    conf.IP_1            = 129;
    conf.IP_2            = 6;
    conf.IP_3            = 15;
    conf.IP_4            = 28;
    conf. EEPROM_chk     =123;
    conf.Latitud         =40.41;
    conf.Longitud        =-3.731;
    strcpy(conf.DevName,"nombre del dispositivo");
    strcpy(conf.username,username);
    strcpy(conf.password,password);
    strcpy(conf.clientID,clientID);
    strcpy(conf.ip_aire,"192.168.1.72");
    strcpy(conf.ip_inversor,"192.168.1.66");
    
    loadConfig();

    //conf.temp_mode=3; // control manual al inicio quitar en modo final
   
    Serial.println("Comenzando programa");
    StartWiFi();
    Serial.println("Comenzando programa");
    Serial.println(conf.username);
    Cayenne.begin(conf.username, conf.password, conf.clientID);
    //Cayenne.begin(username, password, clientID);

  Serial.print("DevName :");
  Serial.println(conf.DevName);
  Serial.println("____________________________");
  Serial.print("DevName en conf :");
  Serial.println(conf.DevName);
  Serial.println("____________________________");
 if (ERROR_CAYENNE){
    Serial.println("error en la conexion a cayenne");
  }
  Serial.println ( "" );
  Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );

  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS Mount failed");
  } else {
    Serial.println("SPIFFS Mount succesfull");
  }




  }

void loadConfig() {
       if(!SPIFFS.begin()){
        Serial.println("Card Mount Failed");
        return;
        }
        File myFile = SPIFFS.open("/conf.txt", "r");
        myFile.read((byte *)&timers, sizeof(timers));
        myFile.read((byte *)&conf, sizeof(conf));
        myFile.close();
        Serial.println("leyendo configuracion");
        Serial.println(conf.DevName);
        //Serial.printf( "read: %s, %.5f\n", myStruct.someString, myStruct.someFloat );
}

void saveConfig() {
  Serial.println("saving config");
   if(!SPIFFS.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    File myFile = SPIFFS.open("/conf.txt", "w");
    myFile.write((byte *)&timers, sizeof(timers));
    myFile.write((byte *)&conf, sizeof(conf));
    myFile.close(); 
    Serial.println("guardando configuracion");
    Serial.println(conf.DevName); 
}
