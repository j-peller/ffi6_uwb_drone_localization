#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

#define DW1000_TIMEOUT  1e6     //< In Nanoseconds (1ms)
#define RX_TIMEOUT      1e8     //< In Nanoseconds (100ms)

typedef enum : uint8_t {
    SUCCESS = 0x00,
    ERROR   = 0x01,
} dwm_com_error_t;

/**
 * @brief Calculate the difference in nanoseconds between two timespecs.
 *
 * @param end The end timespec.
 * @param start The start timespec.
 * @return uint64_t The difference in nanoseconds.
 */
static inline uint64_t timespec_delta_nanoseconds(struct timespec* end, struct timespec* start) {
    return (((end->tv_sec - start->tv_sec) * 1.0e9) + (end->tv_nsec - start->tv_nsec));
}

#endif