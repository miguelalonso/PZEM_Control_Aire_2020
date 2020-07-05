#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
ESP8266WiFiMulti WiFiMulti;
#include <ArduinoJson.h>

const char* ssid = "MiFibra-F870";
const char* password = "P9EnMuxD";


void setup() {
  Serial.begin(115200);
 for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }
 WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(ssid, password);
}

void loop() {
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;
    
    if (http.begin(client, "http://192.168.1.66/cm?cmnd=status%208")) {  // HTTP
      int httpCode = http.GET();
      if (httpCode > 0) {
        Serial.printf("Tasmota Inversor abajo");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = http.getString();
            printparameters( payload);
       }
      } 
     http.end();
    }
    if (http.begin(client, "http://192.168.1.72/cm?cmnd=status%208")) {  // HTTP
      int httpCode = http.GET();
      if (httpCode > 0) {
        Serial.printf("Tasmota AIre");
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = http.getString();
            printparameters( payload);
       }
      } 
     http.end();
    }
     
  }  //END

  delay(5000);
}

void printparameters(String payload){
//{"StatusSNS":{"Time":"2020-06-29T22:22:24","ENERGY":{"TotalStartTime":"2020-06-24T09:25:29","Total":41.573,"Yesterday":6.147,"Today":8.286,"Power":2,"ApparentPower":22,"ReactivePower":21,"Factor":0.08,"Voltage":230,"Current":0.094}}}
   
            StaticJsonDocument<400> doc;
            DeserializationError error = deserializeJson(doc, payload);
            serializeJson(doc, Serial);
            Serial.println("*****************************************");
            Serial.println("Response:");
            const char* cadena = doc["StatusSNS"]["Time"];
            Serial.println(cadena);
            float power= doc["StatusSNS"]["ENERGY"]["Power"];
            Serial.print("Power :");Serial.println(power);
            float Yesterday= doc["StatusSNS"]["ENERGY"]["Yesterday"];
            Serial.print("Yesterday :");Serial.println(Yesterday);
            float Today= doc["StatusSNS"]["ENERGY"]["Today"];
            Serial.print("Today :");Serial.println(Today);
            float ApparentPower= doc["StatusSNS"]["ENERGY"]["ApparentPower"];
            Serial.print("ApparentPower :");Serial.println(ApparentPower);
            float ReactivePower= doc["StatusSNS"]["ENERGY"]["ReactivePower"];
            Serial.print("ReactivePower :");Serial.println(ReactivePower);
            float Factor= doc["StatusSNS"]["ENERGY"]["Factor"];
            Serial.print("Factor :");Serial.println(Factor);
            float Voltage= doc["StatusSNS"]["ENERGY"]["Voltage"];
            Serial.print("Voltage :");Serial.println(Voltage);
            float Current= doc["StatusSNS"]["ENERGY"]["Current"];
            Serial.print("Current :");Serial.println(Current);
  
  }
