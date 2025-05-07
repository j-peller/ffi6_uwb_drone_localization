#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>
#include "wifi_handler.hpp"
#include "logger.hpp"
#include "DW1000.hpp"
#include "DW1000Ranging.hpp"

WifiHandler wifiHandler;
Logger logger(&wifiHandler);
DW1000 dw1000;

DW1000Ranging* dw1000ranging = nullptr;
RangingResult result;

bool tag = false;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  wifiHandler.begin();
  pinMode(LED_BUILTIN, OUTPUT); 
  dw1000.addLogger(&logger);
  if(tag)
  {
    dw1000ranging = new DW1000RangingTag(dw1000, 0xAAAA, 0xBBBB);
  } else
  {
    dw1000ranging = new DW1000RangingAnchor(dw1000, 0xBBBB, 0xAAAA);
  }
  logger.setDeviceID(dw1000ranging->getDeviceAddress());
}

void loop()
{
  //uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  delay(500);
  wifiHandler.loop(); /* Todo! timeintensive right now*/
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  dw1000ranging->loop();
  logger.outputBuffer();
  
  /* */
  if(tag) {
    if(result.state == RangingState::DONE) {
      logger.output("Yippie yeah, we got a distance of %x", result.distance);
    }
    if(result.state != RangingState::MEASURING) {
      DW1000RangingTag* tag = (DW1000RangingTag*) dw1000ranging;
      tag->getDistanceToAnchor(0xBBBB, &result);
    }
  }
}