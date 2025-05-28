#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

// Default anchor response delay.
#define ANC_RESP_DLY_DEFAULT_MS 150000000
// Default tag response delay.
#define TAG_RESP_DLY_DEFAULT_MS 200000000

#define RX_TIMEOUT_ANCHOR   120000000      //< In Nanoseconds (120ms)
#define RX_TIMEOUT          100000000      //< In Nanoseconds (100ms)
#define DW1000_TIMEOUT      10000000      //< In Nanoseconds (10ms)
#define INITIAL_ANTENNA_DELAY 16482.37f  // Initial guess for TX and RX antenna delay

typedef enum : uint8_t {
    SUCCESS = 0x00,
    ERROR   = 0x01,
    TIMEOUT = 0x02,
} dwm_com_error_t;

#endif