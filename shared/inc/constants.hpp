#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

#define DW1000_TIMEOUT  10000000      //< In Nanoseconds (10ms)
#define RX_TIMEOUT      100000000      //< In Nanoseconds (100ms)
#define RX_RETRY        2               //< Retrys -- Remove this? 
#define MAX_RETRY_ON_FAILURE 3          //< Maximum number of retries on failure of a ranging operation
#define INITIAL_ANTENNA_DELAY 16157.05f  // Initial guess for TX and RX antenna delay

typedef enum : uint8_t {
    SUCCESS = 0x00,
    ERROR   = 0x01,
    TIMEOUT = 0x02,
} dwm_com_error_t;

#endif