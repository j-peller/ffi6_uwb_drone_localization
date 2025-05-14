#include <ArduinoWebsockets.h>
//#include "wifi_handler.hpp"
#include "logger.hpp"
#include "DW1000.hpp"
#include "DW1000Ranging.hpp"

DeviceType deviceType = ANCHOR;

//WifiHandler wifiHandler;
Logger logger(nullptr);
DW1000 dw1000;

DW1000Ranging* dw1000ranging = nullptr;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);  
  
  //wifiHandler.begin();
  //pinMode(LED_BUILTIN, OUTPUT); 
  dw1000.addLogger(&logger);
  dw1000ranging = new DW1000Ranging(deviceType, &dw1000);

  
}

void loop()
{
  //uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  delay(2500);
  //wifiHandler.loop(); /* Todo! timeintensive right now*/
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  dw1000ranging->loop();
  logger.outputBuffer();
}