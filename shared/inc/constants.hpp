#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

// Default anchor response delay.
#define ANC_RESP_DLY_DEFAULT_MS 150000000
// Default tag response delay.
#define TAG_RESP_DLY_DEFAULT_MS 200000000
// Default timeout on hardware
#define DW1000_TIMEOUT      10000000      //< In Nanoseconds (10ms)
// good default value for 5m range using jopel mode
#define INITIAL_ANTENNA_DELAY 16482  // Initial guess for TX and RX antenna delay

typedef enum : uint8_t {
    SUCCESS = 0x00,
    ERROR   = 0x01,
    TIMEOUT = 0x02,
} dwm_com_error_t;

#endif