/*http://192.168.1.53:80
 *Equivalencia de pines del NodeMCU
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

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <FS.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h> 

SoftwareSerial mySerial;
ModbusMaster node;
#define MAX485_DE      D1
#define MAX485_RE_NEG  D2

double voltage            = 0;
double current            = 0;
double power              = 0;
double energy             = 0;
double frequency          = 0;
double power_factor       = 0;


void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1);
}

void postTransmission()
{
  delay(3);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial.begin(9600);
  mySerial.begin(9600, SWSERIAL_8N1, D5, D6, false, 95, 11);
  node.begin(1, mySerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  delay(1000);
}

bool state = true;

void loop()
{
  uint8_t result;
  uint16_t data[6];
  yield();
  node.clearResponseBuffer();
  ESP.wdtDisable();
  result = node.readInputRegisters(0x0000, 9);
  ESP.wdtEnable(1);
  Serial.println("Nueva Medida:");
  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    voltage       = (node.getResponseBuffer(0x00) / 10.0f);
    current       = (node.getResponseBuffer(0x01) / 1000.000f);
    power         = (node.getResponseBuffer(0x03) / 10.0f);
    energy        = (node.getResponseBuffer(0x05) / 1000.0f);
    frequency     = (node.getResponseBuffer(0x07) / 10.0f);
    power_factor  = (node.getResponseBuffer(0x08) / 100.0f);

    Serial.print("VOLTAGE:           ");   Serial.println(voltage);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(power);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(energy, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor);
    Serial.println("====================================================");
  }
  else {
      Serial.println("Failed to read PZEM Device 1");
      Serial.println("PZEM Device 1 Data");
      }  
  delay(2000);
  yield();
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
