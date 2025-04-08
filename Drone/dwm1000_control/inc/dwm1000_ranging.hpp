#pragma once

#ifndef DWM1000_RANGING_HPP
#define DWM1000_RANGING_HPP

#include "dwm1000_ctrl.hpp"
#include "dw1000_time.hpp"
#include "dwm1000_device.hpp"

#include <vector>
#include <memory>

/**
 * Perform ranging to a set of anchors
 */
class DWMRanging {
public:
    DWMRanging(DWMController* controller);
    ~DWMRanging();

    /* */
    void add_anchor(DWM1000Device* anchor);

    /* Ranging */
    void start_ranging();

private:
    DW1000Time calculateToF();

private:
    /* DWM1000 of the Drone to send and receive messages */
    DWMController*  _controller;

    /* Identification and storing of timestamps */
    std::vector<DWM1000Device*> _anchors;
};


#endif