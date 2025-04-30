#pragma once

#ifndef DWM1000_RANGING_ANCHOR_HPP
#define DWM1000_RANGING_ANCHOR_HPP

#include "dwm1000_ctrl.hpp"
#include "dw1000_time.hpp"
#include "../../../shared/inc/constants.hpp"
#include "../../../shared/inc/helpers.hpp"
#include "../../../shared/inc/anchor_addresses.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"
#include "../../coords_calc/coords_calc.h"

#include <vector>
#include <memory>

/**
 * Perform ranging to a set of anchors
 */
class DWMRangingAnchor {
public:
    DWMRangingAnchor();
    DWMRangingAnchor(DWMController* controller) {
        _controller = controller;
    }
    ~DWMRangingAnchor();

    /* Ranging */
    dwm_com_error_t run_state_machine();
    

private:
    dwm_com_error_t do_init_state();
    dwm_com_error_t do_response_ack_state(uint16_t anchor_addr);
    dwm_com_error_t do_final_state();
    dwm_com_error_t do_report_state(uint16_t anchor_addr);

    static void waitOutError();


private:
    /* DWM1000 of the Drone to send and receive messages */
    DWMController*  _controller;
    DW1000Time  _init_rx_ts;
    DW1000Time  _resp_tx_ts;
    DW1000Time  _final_rx_ts;

};


#endif