#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_time.hpp"
#include <time.h>

/**
 * @brief Calculate the distance to an anchor given all the timestamps.
 * 
 * @param init_tx_ts Timestamp of poll msg transmission.
 * @param ack_rx_ts Timestamp of poll acknowledge reception.
 * @param fin_tx_ts Timestamp of final msg transmission.
 * @param rprt_rx_ts Timestamp of report msg reception.
 * @param esp_init_rx_ts Timestamp of poll msg reception.
 * @param esp_resp_tx_ts Timestamp of poll acknowledge transmission.
 * @param esp_fin_rx_ts Timestamp of final msg reception.
 * @return double Represents the distance in meters to the anchor.
 */
double DWMRanging::timestamps2distance(
    DW1000Time& init_tx_ts, DW1000Time& ack_rx_ts, DW1000Time& fin_tx_ts,
    DW1000Time& esp_init_rx_ts, DW1000Time& esp_resp_tx_ts,
    DW1000Time& esp_fin_rx_ts
) {
    // convert timestampts to 64 bit ints containing raw number of ticks
    uint64_t t_sp = init_tx_ts.get_timestamp();
    uint64_t t_rp = esp_init_rx_ts.get_timestamp();
    uint64_t t_sa = esp_resp_tx_ts.get_timestamp();
    uint64_t t_ra = ack_rx_ts.get_timestamp();
    uint64_t t_sf = fin_tx_ts.get_timestamp();
    uint64_t t_rf = esp_fin_rx_ts.get_timestamp();
    
    // calculate time of flight in seconds
    double time_of_flight = 
        ((double) (t_ra - t_sp) * (t_rf - t_sa) - (t_sa - t_rp) * (t_sf - t_ra)) 
        / ((t_ra - t_sp) + (t_rf - t_sa) + (t_sa - t_rp) + (t_sf - t_ra)); 

    // calculate and return distance from TOF
    return time_of_flight * (double) DW1000Time::TIME_RESOLUTION_US
        * (double) DW1000Time::SPEED_OF_LIGHT_M_US;
}

/**
 * @brief Wait out errors to cause anchor to also go into an error state if a
 * response is expected.
 * 
 */
void DWMRanging::waitOutError()
{
    timespec start, now;
    clock_gettime(CLOCK_MONOTONIC_RAW,  &start);
    do { 
        clock_gettime(CLOCK_MONOTONIC_RAW,  &now);
    } while (timespec_delta_nanoseconds(&now, &start) < (RX_RETRY * RX_TIMEOUT));
}

/**
 * @brief Perform ranging with all 4 anchors.
 * 
 * @param distances Struct containing all distances as doubles.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::get_distances_to_anchors(distances* distances)
{
    dwm_com_error_t a1_status = get_distance_to_anchor(
        ANCHOR_1, &(distances->d1));
    dwm_com_error_t a2_status = get_distance_to_anchor(
        ANCHOR_2, &(distances->d1));
    dwm_com_error_t a3_status = get_distance_to_anchor(
        ANCHOR_3, &(distances->d1));
    dwm_com_error_t a4_status = get_distance_to_anchor(
        ANCHOR_4, &(distances->d1));

    if (
        a1_status == dwm_com_error_t::ERROR
            || a2_status == dwm_com_error_t::ERROR
            || a3_status == dwm_com_error_t::ERROR
            || a4_status == dwm_com_error_t::ERROR
    ) {
        return dwm_com_error_t::ERROR;
    } else {
        return dwm_com_error_t::SUCCESS;
    }
}


/**
 * @brief Complete actions taken in the init state of the ranging process.
 * 
 * @param init_tx_ts Timestamp of poll msg transmission.
 * @param anchor_addr Address of current anchor.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::do_init_state(DW1000Time& init_tx_ts, uint16_t anchor_addr)
{
    twr_message_t init_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {0xCA, 0xDE},
            .destAddr = { anchor_addr & 0xff, anchor_addr >> 8 },
            .srcAddr = { MASTER & 0xff, MASTER >> 8 }
        },
        .payload = { .init = (twr_init_message_t) {
            .type = twr_msg_type_t::TWR_MSG_TYPE_POLL,
            .anchorShortAddr = {anchor_addr & 0xff, anchor_addr >> 8},
            .responseDelay = 0x00
        }}
    };
    _controller->write_transmission_data((uint8_t*)&init_msg, sizeof(twr_message_t));
    _controller->start_transmission();
    
    // poll and check for error
    dwm_com_error_t tx_state = _controller->poll_tx_status();
    if (tx_state == dwm_com_error_t::ERROR) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }
    
    _controller->get_tx_timestamp(init_tx_ts);

    return dwm_com_error_t::SUCCESS;
}


/**
 * @brief Complete actions taken in the response acknowledge state.
 * 
 * @param ack_rx_ts Timestamp of poll acknowledge reception.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::do_response_ack_state(DW1000Time& ack_rx_ts)
{
    twr_message_t* ack_return;
    uint8_t ack_len;

    _controller->start_receiving();
    // poll and check for error
    dwm_com_error_t tx_state = _controller->poll_rx_status();
    if (tx_state == dwm_com_error_t::ERROR) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }
    ack_return = (twr_message_t*) _controller->read_received_data(&ack_len);
    _controller->get_rx_timestamp(ack_rx_ts);
    
    if (ack_return->payload.response.type != twr_msg_type_t::TWR_MSG_TYPE_RESPONSE) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }

    return dwm_com_error_t::SUCCESS;
}


/**
 * @brief Complete actions taken in the final state of the ranging process.
 * 
 * @param fin_tx_ts Timestamp of final msg transmission.
 * @param anchor_addr Address of current anchor.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::do_final_state(DW1000Time& fin_tx_ts, uint16_t anchor_addr) 
{
    twr_message_t final_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {0xCA, 0xDE},
            .destAddr = { anchor_addr & 0xff, anchor_addr >> 8 },
            .srcAddr = { MASTER & 0xff, MASTER >> 8 }
        },
        .payload = { .final = {.type = twr_msg_type_t::TWR_MSG_TYPE_FINAL,}}
    };
    _controller->write_transmission_data((uint8_t*)&final_msg, sizeof(twr_message_t));
    _controller->start_transmission();

    // poll and check for error
    dwm_com_error_t tx_state = _controller->poll_tx_status();
    if (tx_state == dwm_com_error_t::ERROR) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }

    _controller->get_tx_timestamp(fin_tx_ts);

    return dwm_com_error_t::SUCCESS;
}


/**
 * @brief Complete actions taken in the report state of the ranging process.
 * 
 * @param esp_init_rx_ts Timestamp of poll msg reception.
 * @param esp_resp_tx_ts Timestamp of poll acknowledge transmission.
 * @param esp_fin_rx_ts Timestamp of final msg reception.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::do_report_state(DW1000Time& esp_init_rx_ts, DW1000Time& esp_resp_tx_ts, DW1000Time& esp_fin_rx_ts) 
{
    twr_message_t* rprt_return;
    uint8_t rprt_len;

    _controller->start_receiving();
    // poll and check for error
    dwm_com_error_t tx_state = _controller->poll_rx_status();
    if (tx_state == dwm_com_error_t::ERROR) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }
    rprt_return = (twr_message_t*) _controller->read_received_data(&rprt_len);
    
    if (rprt_return->payload.report.type != twr_msg_type_t::TWR_MSG_TYPE_REPORT) {
        waitOutError();
        return dwm_com_error_t::ERROR;
    }

    esp_init_rx_ts.set_timestamp(rprt_return->payload.report.pollRx);
    esp_resp_tx_ts.set_timestamp(rprt_return->payload.report.responseTx);
    esp_fin_rx_ts.set_timestamp(rprt_return->payload.report.finalTx);

    return dwm_com_error_t::SUCCESS;
}


/**
 * @brief Get the distance to a given anchor.
 * 
 * @param anchor_addr Address of current anchor.
 * @param distance Pointer to write distance to.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRanging::get_distance_to_anchor(uint16_t anchor_addr, double* distance)
{
    // variables in method scope
    DW1000Time init_tx_ts, ack_rx_ts, fin_tx_ts;
    DW1000Time esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts;

    // go through state machine state by state and do the error handling
    // accordingly
    if (do_init_state(init_tx_ts, anchor_addr) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;
    if (do_response_ack_state(ack_rx_ts) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;
    if (do_final_state(fin_tx_ts, anchor_addr) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;
    if (do_report_state(esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;

    // return distance procedurally
    *distance = timestamps2distance(
        init_tx_ts, ack_rx_ts, fin_tx_ts,
        esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts
    );

    return dwm_com_error_t::SUCCESS;
}