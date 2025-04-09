#include "logger.hpp"
#include <cstdarg>  // für va_list, va_start, va_end
#include <cstdio>   // für vsnprintf
#include <cstring>  // für strlen

Logger::Logger(WifiHandler* wifiHandler){
    this->wifiHandler = wifiHandler;
}
const void Logger::output(const char* message, ...)
{
    constexpr size_t BUFFER_SIZE = 1024;
    char buffer[BUFFER_SIZE];

    va_list args;
    va_start(args, message);
    vsnprintf(buffer, BUFFER_SIZE, message, args);
    va_end(args);

    Serial.println(buffer);
    this->wifiHandler->logData(buffer);
}