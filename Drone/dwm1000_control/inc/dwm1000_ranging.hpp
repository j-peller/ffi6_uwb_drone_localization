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
 * 
 */
class DWMRanging {
public:
    static DWMRanging* create_instance(DWMController* controller);
    virtual ~DWMRanging() = 0;
    dwm_com_error_t perform_reset(bool performHardReset) const;
    

protected:
    DWMRanging();
    DWMRanging(DWMController* controller);

public:
    enum class RangingState { 
        INIT,
        RESP_ACK,
        FINAL,
        REPORT,
        COMPLETE
    };

protected:
    /* DWM1000 Controller */
    DWMController*  _controller;
};


/**
 * 
 */
class DWMRangingDrone: public DWMRanging {
friend class DWMRanging;

public:
    ~DWMRangingDrone() override {
        if (_controller != NULL)
            delete _controller;
    };

    /* Ranging */
    dwm_com_error_t get_distances_to_anchors(distances* distances);
    dwm_com_error_t get_distance_to_anchor(uint16_t anchor_addr, double* distance);
    
    /* Calibration */
    dwm_com_error_t calibrate_antenna_delay(double known_distance_m, double allowed_error_m, int max_iterations);

private:
    DWMRangingDrone() : DWMRanging() {};
    DWMRangingDrone(DWMController* controller) : DWMRanging(controller) {};

    dwm_com_error_t do_init_state(uint16_t anchor_addr);
    dwm_com_error_t do_response_ack_state(DW1000Time& t_sp, DW1000Time& t_ra);
    dwm_com_error_t do_final_state(uint16_t anchor_addr);
    dwm_com_error_t do_report_state(DW1000Time& t_sf, DW1000Time& t_rp, DW1000Time& t_sa, DW1000Time& t_rf);

    double timestamps2distance( DW1000Time& t_sp, DW1000Time& t_ra, DW1000Time& t_sf,
        DW1000Time& t_rp, DW1000Time& t_sa, DW1000Time& t_rf);

    static void waitOutError();

    inline bool checkForTimeout(dwm_com_error_t ret) {
        return (ret == dwm_com_error_t::TIMEOUT);
    };

};

/**
 * 
 */
class DWMRangingAnchor : public DWMRanging {
friend class DWMRanging;
public:
    ~DWMRangingAnchor() override {
        if (_controller != NULL)
            delete _controller;
    };


    /* Ranging */
    dwm_com_error_t run_state_machine();
    
    /* Calibration */
    dwm_com_error_t calibrate_antenna_delay(int max_iterations);

private:
    DWMRangingAnchor() : DWMRanging() {};
    DWMRangingAnchor(DWMController* controller) : DWMRanging(controller) {};

    dwm_com_error_t do_init_state();
    dwm_com_error_t do_response_ack_state(uint16_t anchor_addr);
    dwm_com_error_t do_final_state();
    dwm_com_error_t do_report_state(uint16_t anchor_addr);
    
    static void waitOutError();


private:
    DW1000Time  _init_rx_ts;
    DW1000Time  _resp_tx_ts;
    DW1000Time  _final_rx_ts;

};




#endif