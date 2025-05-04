#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

#define DW1000_TIMEOUT  1e7     //< In Nanoseconds (100ms)
#define RX_TIMEOUT      1e7     //< In Nanoseconds (100ms)
#define RX_RETRY        2       //< Retrys

typedef enum : uint8_t {
    SUCCESS = 0x00,
    ERROR   = 0x01,
    TIMEOUT = 0x02,
} dwm_com_error_t;

#endif