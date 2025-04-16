#pragma once
#include "wifi_handler.hpp"

#define MAX_MESSAGE_LENGTH 100
#define MAX_MESSAGES 10
class Logger
{
    private:
        WifiHandler* wifiHandler;
        char messages[MAX_MESSAGES][MAX_MESSAGE_LENGTH];
        uint8_t next_free_buffer = 0;
        uint32_t deviceID;
    public:
        Logger(WifiHandler* wifiHandler);
        const void output(const char* message, ...);
        const void addBuffer(const char* message, ...);
        const void outputBuffer();
        void setDeviceID(uint32_t deviceID);
};