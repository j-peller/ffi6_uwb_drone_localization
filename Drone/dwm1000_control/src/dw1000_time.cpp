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
DW1000Time::DW1000Time(int64_t timeStamp)
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
void DW1000Time::set_timestamp(int64_t timeStamp)
{
    _timeStamp = timeStamp;
}


/**
 * @brief Set the timestamp value from a byte array
 * 
 * @param data Pointer to byte array containing five bytes with array[0] containing the least significant byte (LSB)
 */
void DW1000Time::set_timestamp(uint8_t* data)
{
    int64_t timeStamp = 0;
    for (uint8_t i = 0; i < LENGTH_TIMESTAMP; i++) {
        timeStamp |= ((int64_t)data[i] << (i * 8));
    }
    _timeStamp = timeStamp;
}


/**
 * @brief Set the timestamp value from another DW1000Time object
 * 
 * @param copy Reference to the DW1000Time object to copy from
 */
void DW1000Time::set_timestamp(const DW1000Time& copy)
{
    _timeStamp = copy.get_timestamp();
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
 * @return int64_t The 40bit timestamp value
 */
int64_t DW1000Time::get_timestamp() const
{
    return _timeStamp;
}


/**
 * 
 */
double DW1000Time::get_as_meters() const
{
    return (double)_timeStamp * DISTANCE_PER_US_M;
}


///**
// * @brief Assignment operator for DW1000Time
// */
//DW1000Time& DW1000Time::operator=(const DW1000Time& copy)
//{
//    if (this != &copy) {
//        this->set_timestamp(copy);
//    }
//    return *this;
//}
//
//
///**
// * 
// */
//DW1000Time& DW1000Time::operator-=(const DW1000Time& copy)
//{
//    this->_timeStamp = (this->_timeStamp - copy.get_timestamp());
//    return *this;
//}
//
//
///**
// * 
// */
//DW1000Time& DW1000Time::operator+=(const DW1000Time& copy) 
//{
//    this->_timeStamp = (this->_timeStamp + copy.get_timestamp());
//    return *this;
//}
//
//
///**
// * 
// */
//DW1000Time& DW1000Time::operator*=(const DW1000Time& copy)
//{
//    this->_timeStamp = (this->_timeStamp * copy.get_timestamp());
//    return *this;
//}
//
//
///**
// * 
// */
//DW1000Time& DW1000Time::operator/=(const DW1000Time& copy)
//{
//    this->_timeStamp = (this->_timeStamp / copy.get_timestamp());
//    return *this;
//}
//
//
//
///**
// * @brief Subtraction operator for DW1000Time
// */
//DW1000Time DW1000Time::operator-(const DW1000Time& copy) const
//{
//    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
//    DW1000Time result(_timeStamp - copy.get_timestamp());
//    return result;
//}
//
//
///**
// * 
// */
//DW1000Time DW1000Time::operator+(const DW1000Time& copy) const
//{
//    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
//    DW1000Time result(_timeStamp + copy.get_timestamp());
//    return result;
//}
//
//
///**
// * 
// */
//DW1000Time DW1000Time::operator*(const DW1000Time& copy) const
//{
//    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
//    DW1000Time result(_timeStamp * copy.get_timestamp());
//    return result;
//}
//
//
///**
// * 
// */
//DW1000Time DW1000Time::operator/(const DW1000Time& copy) const
//{
//    /* we are initializing a new DW1000 Object - constructor handles overflow correction */
//    DW1000Time result(_timeStamp / copy.get_timestamp());
//    return result;
//}
//
//
///**
// * @brief Comparison operator for DW1000Time
// * 
// * @param copy Reference to the DW1000Time object to compare with
// * @return true if the timestamps are equal, false otherwise
// */
//bool DW1000Time::operator==(const DW1000Time& copy) const
//{
//    return (_timeStamp == copy.get_timestamp());
//}
//
//
///**
// * @brief Inequality operator for DW1000Time
// * 
// * @param copy Reference to the DW1000Time object to compare with
// * @return true if the timestamps are not equal, false otherwise
// */
//bool DW1000Time::operator!=(const DW1000Time& copy) const
//{
//    return (_timeStamp != copy.get_timestamp());
//}


/**
 * Converts negative values due overflow of one node to correct value
 * @example:
 * Maximum timesamp is 1000.
 * Node N1 sends 999 as timesamp. N2 recieves and sends delayed and increased timestamp back.
 * Delay is 10, so timestamp would be 1009, but due overflow 009 is sent back.
 * Now calculate TOF: 009 - 999 = -990 -> not correct time, so wrap()
 * Wrap calculation: -990 + 1000 = 10 -> correct time 
 * @return 
 */
DW1000Time& DW1000Time::wrap() {
	if(_timeStamp < 0) {
		_timeStamp += TIMER_OVERFLOW;
	}
	return *this;
}

// assign
DW1000Time& DW1000Time::operator=(const DW1000Time& assign) {
	if(this == &assign) {
		return *this;
	}
	_timeStamp = assign.get_timestamp();
	return *this;
}

// add
DW1000Time& DW1000Time::operator+=(const DW1000Time& add) {
	_timeStamp += add.get_timestamp();
	return *this;
}

DW1000Time DW1000Time::operator+(const DW1000Time& add) const {
	return DW1000Time(*this) += add;
}

// subtract
DW1000Time& DW1000Time::operator-=(const DW1000Time& sub) {
	_timeStamp -= sub.get_timestamp();
	return *this;
}

DW1000Time DW1000Time::operator-(const DW1000Time& sub) const {
	return DW1000Time(*this) -= sub;
}

// multiply
DW1000Time& DW1000Time::operator*=(float factor) {
	//float tsValue = (float)_timestamp*factor;
	//_timestamp = (int64_t)tsValue;
	_timeStamp *= factor;
	return *this;
}

DW1000Time DW1000Time::operator*(float factor) const {
	return DW1000Time(*this) *= factor;
}

DW1000Time& DW1000Time::operator*=(const DW1000Time& factor) {
	_timeStamp *= factor.get_timestamp();
	return *this;
}

DW1000Time DW1000Time::operator*(const DW1000Time& factor) const {
	return DW1000Time(*this) *= factor;
}

// divide
DW1000Time& DW1000Time::operator/=(float factor) {
	//_timestamp *= (1.0f/factor);
	_timeStamp /= factor;
	return *this;
}

DW1000Time DW1000Time::operator/(float factor) const {
	return DW1000Time(*this) /= factor;
}

DW1000Time& DW1000Time::operator/=(const DW1000Time& factor) {
	_timeStamp /= factor.get_timestamp();
	return *this;
}

DW1000Time DW1000Time::operator/(const DW1000Time& factor) const {
	return DW1000Time(*this) /= factor;
}

// compare
bool DW1000Time::operator==(const DW1000Time& cmp) const {
	return _timeStamp == cmp.get_timestamp();
}

bool DW1000Time::operator!=(const DW1000Time& cmp) const {
	//return !(*this == cmp); // seems not as intended
	return _timeStamp != cmp.get_timestamp();
}