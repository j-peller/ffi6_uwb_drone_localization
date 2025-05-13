#pragma once

#ifndef DW1000_TIME_HPP
#define DW1000_TIME_HPP

#include <stdint.h>

class DW1000Time {
    friend class DWMRanging;

public:
    DW1000Time();
    DW1000Time(int64_t timeStamp);
    DW1000Time(uint8_t* timeStamp);
    DW1000Time(const DW1000Time& copy);
    ~DW1000Time();

    /* Setters */
    void set_timestamp(int64_t timeStamp);
    void set_timestamp(uint8_t* data);
    void set_timestamp(const DW1000Time& copy);

    /* Getters */
    int64_t  get_timestamp() const;
    void     get_timestamp(uint8_t* data) const;
    double   get_as_meters() const;

    /* */
    DW1000Time& operator=(const DW1000Time& copy);
    DW1000Time& operator-=(const DW1000Time& copy);
    DW1000Time& operator+=(const DW1000Time& copy);
    DW1000Time& operator*=(const DW1000Time& copy);
    DW1000Time& operator*=(float factor);
    DW1000Time& operator/=(const DW1000Time& copy);
    DW1000Time& operator/=(float factor);

    DW1000Time  operator-(const DW1000Time& copy) const;
    DW1000Time  operator+(const DW1000Time& copy) const;
    DW1000Time  operator*(const DW1000Time& copy) const;
    DW1000Time  operator*(float factor) const;
    DW1000Time  operator/(const DW1000Time& copy) const;
    DW1000Time  operator/(float factor) const;

    /* compare */
    bool operator==(const DW1000Time& copy) const;
    bool operator!=(const DW1000Time& copy) const;

    /* helper */
    DW1000Time& wrap();

private:
    int64_t _timeStamp;  /* 40-bit time stamp */

public:
    /* Time resolution in microseconds. Calculated as 1 / (128 * 499.2) */
    static constexpr double TIME_RESOLUTION_US          =  0.000015650040064103f;
    static constexpr double DW1000_TIME_UNITS_PER_US    =  1 / TIME_RESOLUTION_US;
    static constexpr double SPEED_OF_LIGHT_M_US         =  299.792458f;
    static constexpr double DISTANCE_PER_US_M           =  (SPEED_OF_LIGHT_M_US * TIME_RESOLUTION_US);

protected:
    /* DW1000 Timer has a resolution of 40bits */
    static const uint8_t LENGTH_TIMESTAMP   = 5;
    static const uint64_t TIMER_MAX         = 0xffffffffff;
    static const uint64_t TIMER_OVERFLOW    = TIMER_MAX + 1;
};

#endif