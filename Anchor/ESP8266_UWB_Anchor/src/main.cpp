#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>
#include "wifi_handler.hpp"
#include "logger.hpp"
#include "DW1000.hpp"



WifiHandler wifiHandler;
Logger logger(&wifiHandler);
DW1000 dw1000;

uint32_t counter = 0;

void setup()
{
  Serial.begin(9600);
  wifiHandler.begin();
  pinMode(LED_BUILTIN, OUTPUT); 
  dw1000.addLogger(&logger);
  dw1000.initialize();
  //dw1000.setClock(ClockSpeed::automatic);
  delay(1000);
  
  //dw1000.soft_reset();
  delay(1000);
  
  uint8_t test1[4] = {};
  dw1000.readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, test1, 4);
  logger.output("TX_FCTRL_ID %x %x %x %x", test1[3], test1[2], test1[1], test1[0]);
  logger.output("SLOW: %x %d", ClockSpeed::slow.pmsc0_clock, ClockSpeed::slow.spiSettings._clock);
  uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  
  //dw1000.setFrameLength(FrameLength::STANDARD); # todo removed
  dw1000.enableInterrupts(
    (InterruptTable)
    (
      InterruptTable::INTERRUPT_ALL | InterruptTable::INTERRUPT_ON_AUTOMATIC_ACK
    ) 
  );
  //dw1000.loadLDECode();
  dw1000.readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, test1, 4);
  logger.output("TX_FCTRL_ID %x %x %x %x", test1[3], test1[2], test1[1], test1[0]);

  uint32_t test = 0;
  dw1000.readBytes(SYS_MASK_ID, NO_SUB_ADDRESS, &test);
  logger.output("SYS_MASK_ID %x %x %x %x", test1[3], test1[2], test1[1], test1[0]);



}

void loop()
{
  uint8_t devIDNetID[4] = { 0xAB, 0xAC, 0xFE, 0xAF }; /* AFFE - PAN | ACAB - ADDR*/
  //dw1000.writeNetworkIdAndDeviceAddress(devIDNetID);
  //wifiHandler.loop(); /* Todo! Timeintensive right now*/


  uint8_t pmsc_ctrl0[PMSC_CTRL0_LEN] = {0};
  dw1000.readBytes(PMSC_ID, NO_SUB_ADDRESS, pmsc_ctrl0, PMSC_CTRL0_LEN);

  logger.output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);


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
  delay(1000);

  uint32_t sys_status = 0;
  dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
  logger.output("Gotcha not! %x", sys_status);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}