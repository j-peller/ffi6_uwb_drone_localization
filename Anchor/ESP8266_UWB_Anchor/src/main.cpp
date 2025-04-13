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
  dw1000.soft_reset();
  delayMicroseconds(10000);
  dw1000.loadLDECode();
  delayMicroseconds(10000);
  uint8_t test1[4] = {};
  dw1000.readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, test1, 4);
  logger.output("testTESTTEST %x %x %x %x", test1[3], test1[2], test1[1], test1[0]);
  
  uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  
  dw1000.setFrameLength(STANDARD_FRAME_LEN);
  dw1000.enableInterrupts(
    (InterruptTable)
    (
      INTERRUPT_ALL | INTERRUPT_ON_AUTOMATIC_ACK
    ) 
  );

  

}

void loop()
{
  uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  //wifiHandler.loop(); /* Todo! Timeintensive right now*/
  char message[100];
  //uint8_t devIDNetID[4] = { 0x34, 0x12, 0xCD, 0xAB };
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  uint8_t data[4];
  logger.output(std::to_string(counter++).c_str());
  dw1000.getPrintableDeviceIdentifier(message);
  dw1000.readNetworkIdAndDeviceAddress(data);

  uint8_t tx_message[2] = {0xFE, 0xAF};
  dw1000.transmit(tx_message, 2);

  logger.output(message);
  logger.output("%08X %08X %08X %08X", data[3], data[2], data[1], data[0]);
  delay(500);

  uint32_t sys_status = 0;
  dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
  logger.output("Gotcha not! %x", sys_status);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}