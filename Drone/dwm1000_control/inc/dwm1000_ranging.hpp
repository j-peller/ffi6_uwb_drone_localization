#pragma once

#ifndef DWM1000_RANGING_HPP
#define DWM1000_RANGING_HPP

#include "dwm1000_ctrl.hpp"
#include "dw1000_time.hpp"
#include "../../../shared/inc/constants.hpp"
#include "../../../shared/inc/anchor_addresses.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"
#include "../../coords_calc/coords_calc.h"

#include <memory>


// Helper macro for state transitions (put in header)
#define HANDLE_STATE_TRANSITION(ret_val, next_state, timeout_state, timeout_occurred) \
    if (ret_val == dwm_com_error_t::SUCCESS) { \
        state = next_state; \
    } \
    else if (ret_val == dwm_com_error_t::TIMEOUT) { \
        state = timeout_state; \
        timeout_occurred = true; \
        break; \
    } else { \
        return ret_val; \
    }

/**
 * Perform ranging to a set of anchors
 */
class DWMRanging {
public:
    static DWMRanging* create_instance(DWMController* controller);
    ~DWMRanging();

    /* Ranging */
    dwm_com_error_t get_distances_to_anchors(distances* distances);
    dwm_com_error_t get_distance_to_anchor(uint16_t anchor_addr, double* distance);

private:
    DWMRanging();
    DWMRanging(DWMController* controller);

    dwm_com_error_t do_init_state(DW1000Time& t_sp, uint16_t anchor_addr);
    dwm_com_error_t do_response_ack_state(DW1000Time& t_ra);
    dwm_com_error_t do_final_state(DW1000Time& t_sf, uint16_t anchor_addr);
    dwm_com_error_t do_report_state(DW1000Time& t_rp, DW1000Time& t_sa, DW1000Time& t_rf);

    double timestamps2distance( DW1000Time& t_sp, DW1000Time& t_ra, DW1000Time& t_sf,
        DW1000Time& t_rp, DW1000Time& t_sa, DW1000Time& t_rf);

    static void waitOutError();

    inline bool checkForTimeout(dwm_com_error_t ret) {
        return (ret == dwm_com_error_t::TIMEOUT);
    };

public:
    enum class RangingState {
        INIT,
        RESP_ACK,
        FINAL,
        REPORT,
        COMPLETE
    };

private:
    /* DWM1000 of the Drone to send and receive messages */
    DWMController*  _controller;


};


#endif