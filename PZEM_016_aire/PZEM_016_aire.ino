/*
 * Adaptado por Miguel Alonso Abella en junio de 2020
 * 
 * ESP8266 + DHT22 + BMP180 + BOOTSTRAP + SPIFFS + GOOGLE CHARTS
 * Copyright (C) 2017 http://www.projetsdiy.fr - http://www.diyprojects.io
 * https://randomnerdtutorials.com/esp8266-ds18b20-temperature-sensor-web-server-with-arduino-ide/
 * https://roboindia.com/tutorials/ds18b20-temp-sensor-nodemcu/
 * Modificado por Miguel Alonso Abella Mayo 2020
 * 
 * PZEM-016 Modbus
 * Lee los valores de los sonoff flasheados con Tasmota
 * en función de la potencia FV disponible enciende o apaga el sonoff del aire
 * *Equivalencia de pines del NodeMCU
#define D0 16
#define D1 5 // I2C Bus SCL (clock)
#define D2 4 // I2C Bus SDA (data)
#define D3 0
#define D4 2 // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO 
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
#define D9 3 // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

 RS485_HalfDuplex.pde - example using ModbusMaster library to communicate
  with EPSolar LS2024B controller using a half-duplex RS485 transceiver.

  This example is tested against an EPSolar LS2024B solar charge controller.
  See here for protocol specs:
  http://www.solar-elektro.cz/data/dokumenty/1733_modbus_protocol.pdf

  Library:: ModbusMaster
  Author:: Marius Kintel <marius at kintel dot net>
  max 485 pra leer el puerto serie del pzem-016 con modbus

 * D6 --> D1
 * D1 --> DE
 * 
 * D2 --> RE
 * D5 --> RO
 *  
 * 
 * 
 */


   
int ERROR_CAYENNE=0;

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif

#define CONFIG_VERSION "v01"
#define CONFIG_START 0
//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial

#include <CayenneMQTTESP8266.h>


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DHT.h>
#include <FS.h>
#include <ArduinoJson.h>  
#include <DNSServer.h>
#include <WiFiManager.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h> 
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <math.h>
//#include <SunPos.h> //Sunpos de la plataforma solar de Almería
#include <spa.h>    //spa del NREL

#include "variables.h" 

SoftwareSerial mySerial;
ModbusMaster node;


void setup() {
  setupConfig();
  
  server.on("/tabmesures.json", sendTabMesures2);
  server.on("/mesures.json", sendMesures);
  server.on("/gpio", updateGpio);
  server.on("/readgpio.json", sendGPIO);
  server.on("/hora.json", sendTime);
  server.on("/sendDataTimers.json", sendDataTimers);
  server.on("/sendDataConf.json", sendDataConf);
  server.on("/save", save);
  server.on("/savemode", savemode);
  server.on("/saveconfig", saveconfig_http);
  server.on("/savetemp", savetemp);

  server.serveStatic("/js", SPIFFS, "/js");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/img", SPIFFS, "/img");
  server.serveStatic("/html", SPIFFS, "/html");
  server.serveStatic("/", SPIFFS, "/index.html");
  server.begin();
  Serial.println ( "HTTP server started" );
}

void loop() {
  server.handleClient();
//  t=25;//temperatureC;     //DS18B20 temperatura;
//  h=980;//mV_TDS;           //mV del TSD ADS1115 A0_A1;
//  pa=145;//tdsValue;        //TDS value en ppm;
  if (!ERROR_CAYENNE){Cayenne.loop();} //libreria cayenne modificada para detectar error
  
  CheckNTPtime();
  server_read();
  horasolar();
 
  for (j=0;j<Num_reles;j++){  controlTimers(j);}
  leer_http_tasmotas();
  leer_PZEM016();
  control_aire();

  if(WiFi.status() != WL_CONNECTED)
  {
     Serial.println("reconectando WIFI");
     StartWiFi();
  }
}
