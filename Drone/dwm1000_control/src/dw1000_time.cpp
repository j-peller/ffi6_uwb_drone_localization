#include "../inc/dw1000_time.hpp"

#include <stdio.h>
#include <string.h>

/**
 * 
 */
DW1000Time::DW1000Time()
    : _timeStamp(0)
{

}


/**
 * 
 */
DW1000Time::DW1000Time(uint64_t timeStamp)
    : _timeStamp(0)
{
    set_timestamp(timeStamp);
}


/**
 * 
 */
DW1000Time::DW1000Time(uint8_t* data)
    : _timeStamp(0)
{
    set_timestamp(data);
}


/**
 * 
 */
DW1000Time::DW1000Time(const DW1000Time& copy)
{
    set_timestamp(copy);
}


/**
 * 
 */
DW1000Time::~DW1000Time()
{
    /* Not needed? */
}


/**
 * @brief Set the timestamp value and handle 40bit overflow
 * 
 * @param timeStamp 40-bit timestamp value
 */
void DW1000Time::set_timestamp(uint64_t timeStamp)
{
    _timeStamp = timeStamp & TIMER_MAX;
}


/**
 * @brief Set the timestamp value from a byte array
 * 
 * @param data Pointer to byte array containing five bytes with array[0] containing the least significant byte (LSB)
 */
void DW1000Time::set_timestamp(uint8_t* data)
{
    uint64_t timeStamp = 0;
    for (uint8_t i = 0; i < LENGTH_TIMESTAMP; i++) {
        timeStamp |= ((uint64_t)data[i] << (i * 8));
    }
    _timeStamp = timeStamp & TIMER_MAX;
}


/**
 * @brief Set the timestamp value from another DW1000Time object
 * 
 * @param copy Reference to the DW1000Time object to copy from
 */
void DW1000Time::set_timestamp(const DW1000Time& copy)
{
    _timeStamp = copy.get_timestamp() & TIMER_MAX;
}


/**
 * @brief Get the timestamp value as a byte array
 * 
 * @param data Pointer to the byte array to store the timestamp data which is expected to be of size LENGTH_TIMESTAMP
 */
void DW1000Time::get_timestamp(uint8_t* data) const
{
    if (data == NULL) {
        return;
    }

    memset(data, 0, LENGTH_TIMESTAMP);

    for (uint8_t i = 0; i < LENGTH_TIMESTAMP; i++) {
        data[i] = (uint8_t)((_timeStamp >> (i * 8)) & 0xFF);
    }
}


/**
 * @brief Get the timestamp value
 * 
 * @return uint64_t The 40bit timestamp value
 */
uint64_t DW1000Time::get_timestamp() const
{
    return _timeStamp;
}


/**
 * @brief Assignment operator for DW1000Time
 */
DW1000Time& DW1000Time::operator=(const DW1000Time& copy)
{
    if (this != &copy) {
        this->set_timestamp(copy);
    }
    return *this;
}


/**
 * 
 */
DW1000Time& DW1000Time::operator-=(const DW1000Time& copy)
{
    this->_timeStamp = (this->_timeStamp - copy.get_timestamp()) & TIMER_MAX;
    return *this;
}


/**
 * 
 */
DW1000Time& DW1000Time::operator+=(const DW1000Time& copy) 
{
    this->_timeStamp = (this->_timeStamp + copy.get_timestamp()) & TIMER_MAX;
    return *this;
}


/**
 * 
 */
DW1000Time& DW1000Time::operator*=(const DW1000Time& copy)
{
    this->_timeStamp = (this->_timeStamp * copy.get_timestamp()) & TIMER_MAX;
    return *this;
}


/**
 * 
 */
DW1000Time& DW1000Time::operator/=(const DW1000Time& copy)
{
    this->_timeStamp = (this->_timeStamp / copy.get_timestamp()) & TIMER_MAX;
    return *this;
}



/**
 * @brief Subtraction operator for DW1000Time
 */
DW1000Time DW1000Time::operator-(const DW1000Time& copy) const
{
    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
    DW1000Time result(_timeStamp - copy.get_timestamp());
    return result;
}


/**
 * 
 */
DW1000Time DW1000Time::operator+(const DW1000Time& copy) const
{
    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
    DW1000Time result(_timeStamp + copy.get_timestamp());
    return result;
}


/**
 * 
 */
DW1000Time DW1000Time::operator*(const DW1000Time& copy) const
{
    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
    DW1000Time result(_timeStamp * copy.get_timestamp());
    return result;
}


/**
 * 
 */
DW1000Time DW1000Time::operator/(const DW1000Time& copy) const
{
    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
    DW1000Time result(_timeStamp / copy.get_timestamp());
    return result;
}


/**
 * @brief Comparison operator for DW1000Time
 * 
 * @param copy Reference to the DW1000Time object to compare with
 * @return true if the timestamps are equal, false otherwise
 */
bool DW1000Time::operator==(const DW1000Time& copy) const
{
    return (_timeStamp == copy.get_timestamp());
}


/**
 * @brief Inequality operator for DW1000Time
 * 
 * @param copy Reference to the DW1000Time object to compare with
 * @return true if the timestamps are not equal, false otherwise
 */
bool DW1000Time::operator!=(const DW1000Time& copy) const
{
    return (_timeStamp != copy.get_timestamp());
}