#include "logger.hpp"

Logger::Logger(WifiHandler* wifiHandler){
    this->wifiHandler = wifiHandler;
}
const void Logger::output(const char* message)
{
    Serial.println(message);
    this->wifiHandler->logData(message);
}