#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>
#include "wifi_handler.hpp"
#include "logger.hpp"



WifiHandler wifiHandler;
Logger logger(&wifiHandler);

uint32_t counter = 0;

void setup()
{
  Serial.begin(9600);
  wifiHandler.begin();
  pinMode(LED_BUILTIN, OUTPUT); 
}

void loop()
{
  wifiHandler.loop();
  
  logger.output(std::to_string(counter++).c_str());
  delay(500);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}