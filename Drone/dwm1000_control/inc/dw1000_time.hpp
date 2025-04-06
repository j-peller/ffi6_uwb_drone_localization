#pragma once

#ifndef DW1000_TIME_HPP
#define DW1000_TIME_HPP

#include <stdint.h>

class DW1000Time {

public:
    DW1000Time();
    DW1000Time(uint64_t timeStamp);
    DW1000Time(uint8_t* timeStamp);
    DW1000Time(const DW1000Time& copy);
    ~DW1000Time();

    /* Setters */
    void set_timestamp(uint64_t timeStamp);
    void set_timestamp(uint8_t* data);
    void set_timestamp(const DW1000Time& copy);

    /* Getters */
    uint64_t get_timestamp() const;
    void     get_timestamp(uint8_t* data) const;

    /* */
    DW1000Time& operator=(const DW1000Time& copy);
    DW1000Time& operator-=(const DW1000Time& copy);
    DW1000Time& operator+=(const DW1000Time& copy);

    DW1000Time  operator-(const DW1000Time& copy) const;
    DW1000Time  operator+(const DW1000Time& copy) const;

    /* compare */
    bool operator==(const DW1000Time& copy) const;
    bool operator!=(const DW1000Time& copy) const;



private:
    uint64_t _timeStamp;  /* 40-bit time stamp */

protected:
    /* Time resolution in microseconds. Calculated as 1 / (128 * 499.2) */
    static constexpr float TIME_RESOLUTION_US   =  0.000015650040064103f;
    static constexpr float SPEED_OF_LIGHT_M_US  =  299.792458f;
    static constexpr float DISTANCE_PER_US_M    =  (SPEED_OF_LIGHT_M_US * TIME_RESOLUTION_US);

    /* DW1000 Timer has a resolution of 40bits */
    static const uint8_t LENGTH_TIMESTAMP   = 5;
    static const uint64_t TIMER_MAX         = 0xffffffffff;
    static const uint64_t TIMER_OVERFLOW    = TIMER_MAX + 1;

    /* Time Factors relative to microseconds */
    static const uint64_t SECONDS       = 1e6 ;
    static const uint64_t MILLISECONDS  = 1e3;
    static const uint64_t MICROSECONDS  = 1;
    static const uint64_t NANOSECONDS   = 1e-3;
};

#endif