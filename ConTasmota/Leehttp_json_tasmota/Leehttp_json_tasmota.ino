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
    Serial.print("[HTTP] begin...\n");
    if (http.begin(client, "http://192.168.1.66/cm?cmnd=status%208")) {  // HTTP
      Serial.print("[HTTP] GET...\n");
      int httpCode = http.GET();
     if (httpCode > 0) {
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);
       if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String payload = http.getString();
            Serial.println(payload);
             Serial.println("***********NOW JSON******************");
            StaticJsonBuffer<600> jsonBuffer;
            JsonObject& doc = jsonBuffer.parseObject(payload);
            doc.printTo(Serial);
            Serial.println();
            Serial.println("Response:");
            const char* energ = doc["StatusSNS"]["Time"];
            Serial.println(energ);
            float power= doc["StatusSNS"]["ENERGY"]["Power"];
            Serial.print("Power :");Serial.println(power);
            
            //{"StatusSNS":{"Time":"2020-06-29T22:22:24","ENERGY":{"TotalStartTime":"2020-06-24T09:25:29","Total":41.573,"Yesterday":6.147,"Today":8.286,"Power":2,"ApparentPower":22,"ReactivePower":21,"Factor":0.08,"Voltage":230,"Current":0.094}}}
                
            
       }
      } else {
        Serial.printf("error1");
      }
     http.end();
    } else {
      Serial.printf("[HTTP} Unable to connect\n");
    }
  }  //END

  delay(5000);
}
