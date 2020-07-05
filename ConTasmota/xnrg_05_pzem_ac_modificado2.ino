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
//ModbusMaster node;
TasmotaSerial mySerial(14,12);

#define MAX485_DE      D1
#define MAX485_RE_NEG  D2
///////////////////////////////////////////añadido mio


float voltage      = 0;
float current      = 0;
float active_power       = 0;
float active_energy      = 0;
float frequency          = 0;
float power_factor       = 0;




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
  //node.clearResponseBuffer();
  //result = node.readInputRegisters(0x0000, 9);
  Serial.println("Nueva Medida:");
 
//  if (result == node.ku8MBSuccess)
//  {
//    voltage            = (node.getResponseBuffer(0x00) / 10.0f);
//    current            = (node.getResponseBuffer(0x01) / 1000.000f);
//    active_power       = (node.getResponseBuffer(0x03) / 10.0f);
//    active_energy      = (node.getResponseBuffer(0x05) / 1000.0f);
//    frequency          = (node.getResponseBuffer(0x07) / 10.0f);
//    power_factor       = (node.getResponseBuffer(0x08) / 100.0f);
//  }else {
//    Serial.println("Failed to read modbus");  
//  }
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

}

void PzemAcSnsInit(void)
{
    PzemAc.phase = 0;
    energy_flg = ENERGY_NONE;

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

    mySerial.begin(9600);
    //node.begin(1, mySerial);
   // node.preTransmission(preTransmission);
   // node.postTransmission(postTransmission);  

   
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


#endif  // USE_PZEM_AC
#endif  // USE_ENERGY_SENSOR
