#pragma once

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>	
#include <time.h>

#define DW1000_TIMEOUT  1e8     //< In Nanoseconds (100ms)
#define RX_TIMEOUT      1e8     //< In Nanoseconds (100ms)
#define RX_RETRY        2       //< Retrys

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

/**
 * @brief Busy wait for a specified number of nanoseconds.
 */
/*

Kann das nicht compilen - Mario

static inline void busywait_nanoseconds(uint64_t ns) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    } while (timespec_delta_nanoseconds(&now, &start) < ns);
}
*/
#endif