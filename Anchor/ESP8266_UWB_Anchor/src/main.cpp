#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>
#include "wifi_handler.hpp"
#include "logger.hpp"
#include "dw1000/DW1000.hpp"



WifiHandler wifiHandler;
Logger logger(&wifiHandler);
DW1000 dw1000;

uint32_t counter = 0;

void setup()
{
  Serial.begin(9600);
  wifiHandler.begin();
  pinMode(LED_BUILTIN, OUTPUT); 
  dw1000.initialize();
  dw1000.addLogger(&logger);
  uint8_t devIDNetID[4] = { 0x34, 0x12, 0xCD, 0xAB };
  dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
}

void loop()
{
  wifiHandler.loop();
  char message[100];
  //uint8_t devIDNetID[4] = { 0x34, 0x12, 0xCD, 0xAB };
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  uint8_t data[4];
  
  logger.output(std::to_string(counter++).c_str());
  dw1000.getPrintableDeviceIdentifier(message);
  dw1000.readNetworkIdAndDeviceAddress(data);

  logger.output(message);
  logger.output("%08X %08X %08X %08X", data[0], data[1], data[2], data[3]);
  delay(500);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}