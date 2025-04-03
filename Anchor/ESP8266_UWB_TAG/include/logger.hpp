#pragma once
#include "wifi_handler.hpp"

class Logger
{
    private:
        WifiHandler* wifiHandler;
    public:
        Logger(WifiHandler* wifiHandler);
        const void output(const char* message);
};