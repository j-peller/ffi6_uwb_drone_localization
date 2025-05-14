#pragma once
#include "DW1000.hpp"
enum DeviceType {
    TAG,
    ANCHOR,
};

class DW1000Ranging
{
    private:
        DeviceType deviceType;
        DW1000* dw1000;
    public:
        DW1000Ranging(DeviceType deviceMode, DW1000* dw1000);
        void loop();

};