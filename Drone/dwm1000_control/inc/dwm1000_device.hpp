#pragma once

#ifndef DWM1000_DEVICE_HPP
#define DWM1000_DEVICE_HPP

#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_time.hpp"


/**
 * 
 */
class DWM1000Device {
    /* DWMRanging can access private attributes */
    friend class DWMRanging;

public:
    DWM1000Device();
    ~DWM1000Device();

    /* */
    void set_short_address(uint16_t short_address);
    void set_long_address(uint64_t long_address);

private:
    uint16_t _short_address;      //< Short address of the device
    uint64_t _long_address[8];    //< Long UID address of the device EUI

    /* Timestamps to remember by Drone */
    DW1000Time  _t_poll_tx;     //< Poll transmission timestamp     -- Drone Initiator
    DW1000Time  _t_resp_rx;     //< Response reception timestamp    -- Anchor Responder
    DW1000Time  _t_final_tx;    //< Final transmission timestamp    -- Drone Initiator

    /* Timestamps provided by Anchor */
    DW1000Time  _t_poll_rx;     //< Poll reception timestamp
    DW1000Time  _t_resp_tx;     //< Response transmission timestamp
    DW1000Time  _t_final_rx;    //< Report transmission timestamp
};


#endif