#pragma once
#include <ESP8266WiFi.h>
#include <ArduinoWebsockets.h>

class WifiHandler
{
    private:
        websockets::WebsocketsClient client;

        const void connectWS();
        const void connectWifi();
    public:
        WifiHandler();
        const void begin();
        bool logData(uint32_t deviceID, const char* message);
        const void loop();
};