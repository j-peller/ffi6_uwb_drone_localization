#pragma once

#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <stdint.h>
#include <time.h>


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
static inline void busywait_nanoseconds(uint64_t ns) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    } while (timespec_delta_nanoseconds(&now, &start) < ns);
}

#endif