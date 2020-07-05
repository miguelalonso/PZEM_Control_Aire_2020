/*
  xnrg_05_pzem_ac.ino - PZEM-014,016 Modbus AC energy sensor support for Tasmota

  Copyright (C) 2020  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_PZEM_AC
/*********************************************************************************************\
 * PZEM-004T V3 - AC 220V 10/100A Energy
 * PZEM-014     - AC 220V 10A Energy
 * PZEM-016     - AC 220V 100A Energy
 *
 * Based on:
 *   PZEM-014,016 docs https://pan.baidu.com/s/1B0MdMgURyjtO1oQa2lavKw password ytkv
 *
 * Hardware Serial will be selected if GPIO1 = [62 PZEM0XX Tx] and GPIO3 = [98 PZEM016 Rx]
\*********************************************************************************************/

#define XNRG_05                    5

const uint8_t PZEM_AC_DEVICE_ADDRESS = 0x01;  // PZEM default address
const uint32_t PZEM_AC_STABILIZE = 30;        // Number of seconds to stabilize configuration

//#include <TasmotaModbus.h>
//TasmotaModbus *PzemAcModbus;
//SoftwareSerial mySerial;
#include <TasmotaSerial.h>
#include <ModbusMaster.h>


ModbusMaster node;
TasmotaSerial mySerial(14,12);

#define MAX485_DE      5  //D1
#define MAX485_RE_NEG  4  //D2


///////////////////////////////////////////añadido mio


float voltage      = 0;
float current      = 0;
float active_power       = 0;
float active_energy      = 0;
float frequency          = 0;
float power_factor       = 0;

unsigned long lastTime2 = 0;
unsigned long timerDelay2 = 5000;



/////////////////////////////

struct PZEMAC {
  float energy = 0;
  float last_energy = 0;
  uint8_t send_retry = 0;
  uint8_t phase = 0;
  uint8_t address = 0;
  uint8_t address_step = ADDR_IDLE;
} PzemAc;

void PzemAcEverySecond(void)
{
  Serial.println("---------MIO--------------Leyendo PzemAcEverySecond-----");
  uint8_t result;
  uint16_t data[6];
  node.clearResponseBuffer();
  result = node.readInputRegisters(0x0000, 9);
  Serial.println("Nueva Medida:");
 
  if (result == node.ku8MBSuccess)
  {
    voltage            = (node.getResponseBuffer(0x00) / 10.0f);
    current            = (node.getResponseBuffer(0x01) / 1000.000f);
    active_power       = (node.getResponseBuffer(0x03) / 10.0f);
    active_energy      = (node.getResponseBuffer(0x05) / 1000.0f);
    frequency          = (node.getResponseBuffer(0x07) / 10.0f);
    power_factor       = (node.getResponseBuffer(0x08) / 100.0f);
  }else {
    Serial.println("Failed to read modbus");  
  }
        Energy.data_valid[PzemAc.phase] = 0;
        Energy.voltage[PzemAc.phase] =        voltage; 
        Energy.current[PzemAc.phase] =        current;        
        Energy.active_power[PzemAc.phase] =   active_power;     
        Energy.frequency[PzemAc.phase] =      frequency;        
        Energy.power_factor[PzemAc.phase] =   power_factor;       

        PzemAc.energy +=  active_energy;  
            if (uptime > PZEM_AC_STABILIZE) {
              EnergyUpdateTotal(PzemAc.energy, false);
            }
         PzemAc.last_energy = PzemAc.energy;
         PzemAc.energy = 0;
         
  Serial.println(voltage);
  datosinversor();

}

void PzemAcSnsInit(void)
{
    Serial.println("Inicializando");
    PzemAc.phase = 0;
    energy_flg = XNRG_05;

    
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

    mySerial.begin(9600);
    node.begin(1, mySerial);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);  
  
  Serial.println("Inicializado");
}

void PzemAcDrvInit(void)
{
  
    energy_flg = XNRG_05;
  
}

bool PzemAcCommand(void)
{
  bool serviced = true;
  return serviced;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg05(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_ENERGY_EVERY_SECOND:
      if (uptime > 4) { PzemAcEverySecond(); }  // Fix start up issue #5875
      break;
    case FUNC_COMMAND:
      result = PzemAcCommand();
      break;
    case FUNC_INIT:
      PzemAcSnsInit();
      break;
    case FUNC_PRE_INIT:
      PzemAcDrvInit();
      break;
  }
  return result;
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

//********DATOS DE LOS DEMAS Snooff *******//
// Inversor abajo 192.168.1.66
//aire 192.168.1.72
//unsigned long lastTime = 0;
 // unsigned long timerDelay = 5000;

void datosinversor(){
  

 if ((Wifi.status == WL_CONNECTED) && ((millis()-lastTime2)>timerDelay2)) {
    lastTime2=millis();
    WiFiClient client;
    HTTPClient http;
     if (http.begin(client, "http://192.168.1.66/cm?cmnd=status%208")) {  // HTTP
     int httpCode = http.GET();
     if (httpCode > 0) {
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

        PzemAc.phase=1;
        Energy.data_valid[PzemAc.phase] = 0;
        Energy.voltage[PzemAc.phase] =        voltage; 
        Energy.current[PzemAc.phase] =        current;        
        Energy.active_power[PzemAc.phase] =   active_power;     
        Energy.frequency[PzemAc.phase] =      frequency;        
        Energy.power_factor[PzemAc.phase] =   power_factor;   
        PzemAc.phase=0;   
       }
      } else {
        Serial.printf("error1");
      }
     http.end();
    } else {
      Serial.printf("[HTTP} Unable to connect\n");
    }
  }  //END
  
  }
//*********FIN DE LECTURA DE DATOS *****/

#endif  // USE_PZEM_AC
#endif  // USE_ENERGY_SENSOR
