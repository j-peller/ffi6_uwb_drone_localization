#pragma once

#ifndef DWM1000_RANGING_HPP
#define DWM1000_RANGING_HPP

#include "dwm1000_ctrl.hpp"
#include "dw1000_time.hpp"
#include "dwm1000_device.hpp"
#include "../../../shared/inc/constants.hpp"
#include "../../../shared/inc/anchor_addresses.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"
#include "../../coords_calc/coords_calc.h"

#include <vector>
#include <memory>

/**
 * Perform ranging to a set of anchors
 */
class DWMRanging {
public:
    DWMRanging();
    DWMRanging(DWMController* controller) {
        _controller = controller;
    }
    ~DWMRanging();

    /* Ranging */
    dwm_com_error_t get_distances_to_anchors(distances* distances);
    

private:
    dwm_com_error_t get_distance_to_anchor(uint16_t anchor_addr, double* distance);
    dwm_com_error_t do_init_state(DW1000Time& init_tx_ts, uint16_t anchor_addr);
    dwm_com_error_t do_response_ack_state(DW1000Time& ack_rx_ts);
    dwm_com_error_t do_final_state(DW1000Time& fin_tx_ts, uint16_t anchor_addr);
    dwm_com_error_t do_report_state(DW1000Time& esp_init_rx_ts, DW1000Time&
         esp_resp_tx_ts, DW1000Time& esp_fin_rx_t);

    static double timestamps2distance(
        DW1000Time& init_tx_ts, DW1000Time& ack_rx_ts, DW1000Time& fin_tx_ts, 
        DW1000Time& esp_init_rx_ts, DW1000Time& esp_resp_tx_ts, DW1000Time& esp_fin_rx_ts);
    static void waitOutError();


private:
    /* DWM1000 of the Drone to send and receive messages */
    DWMController*  _controller;

};


#endif