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

const void Logger::addBuffer(const char* message, ...)
{
    if(next_free_buffer < MAX_MESSAGES-1) {
    
    char buffer[MAX_MESSAGE_LENGTH] = {0};

    va_list args;
    va_start(args, message);
    vsnprintf(buffer, MAX_MESSAGE_LENGTH, message, args);
    va_end(args);

    strncpy(this->messages[next_free_buffer], buffer, MAX_MESSAGE_LENGTH);
    this->messages[next_free_buffer][MAX_MESSAGE_LENGTH - 1] = '\0'; 
    next_free_buffer++;
    
    }
}

const void Logger::outputBuffer()
{
    for(uint8_t i = 0; i < next_free_buffer; i++)
    {
        output(this->messages[i]);
    }
    next_free_buffer = 0;
}