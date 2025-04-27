#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_time.hpp"


/**
 * 
 */
DWMRanging::~DWMRanging() 
{
    /* TODO */
}


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

    fprintf(stdout, "init_tx_ts: %ld\nesp_init_rx_ts: %ld\nesp_resp_tx_ts: %ld\nack_rx_ts: %ld\nfin_tx_ts: %ld\nesp_fin_rx_rs: %ld\n", t_sp, t_rp, t_sa, t_ra, t_sf, t_rf);
    
    // calculate time of flight as dw1000 timer ticks
    double time_of_flight = ((t_ra - t_sp) * (t_rf - t_sa) - (t_sa - t_rp) * (t_sf - t_ra)) \
        / ((t_ra - t_sp) + (t_rf - t_sa) + (t_sa - t_rp) + (t_sf - t_ra)); 

    // calculate and return distance from TOF
    return time_of_flight * DW1000Time::DISTANCE_PER_US_M;
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
    dwm_com_error_t a1_status = get_distance_to_anchor(ANCHOR_1, &(distances->d1));
    dwm_com_error_t a2_status = get_distance_to_anchor(ANCHOR_2, &(distances->d2));
    dwm_com_error_t a3_status = get_distance_to_anchor(ANCHOR_3, &(distances->d3));
    dwm_com_error_t a4_status = get_distance_to_anchor(ANCHOR_4, &(distances->d4));

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
    dwm_com_error_t ret = SUCCESS;

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
        }}
    };

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&init_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();
    
    /* Poll for completion of transmission */
    ret = _controller->poll_tx_status();
    if (ret != SUCCESS) {
        fprintf(stdout, "Error polling for TX Status: %d\n", ret);
        return ret;
    }
    
    /* Note time of transmission */
    _controller->get_tx_timestamp(init_tx_ts);
    fprintf(stdout, "Got init_tx_ts: %ld\n", init_tx_ts.get_timestamp());

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
    twr_message_t* ack_return = NULL;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;  


    /* Start reception of packets */
    _controller->start_receiving();
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_rx_status();
        if (ret != SUCCESS)
        {
            waitOutError();
        } else {
            ret = _controller->read_received_data(&ack_len, (uint8_t**)&ack_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && ack_return->payload.init.type == TWR_MSG_TYPE_RESPONSE)
                break;
        }
    }

    /* Note Timestamp of Reception */
    _controller->get_rx_timestamp(ack_rx_ts);

    /* cleanup */
    delete ack_return;
    
    return SUCCESS;
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
    dwm_com_error_t ret = SUCCESS;

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

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&final_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

    /* Poll for completion of transmission */
    ret = _controller->poll_tx_status();
    if (ret != SUCCESS) {
        waitOutError();
        return ret;
    }

    /* Note time of transmission */
    _controller->get_tx_timestamp(fin_tx_ts);
    fprintf(stdout, "Got fin_tx_ts: %ld\n", fin_tx_ts.get_timestamp());

    return SUCCESS;
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
    twr_message_t* rprt_return = NULL;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;  

    /* Start reception of packets */
    _controller->start_receiving();
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_rx_status();
        if (ret != SUCCESS)
        {
            waitOutError();
        } else {
            ret = _controller->read_received_data(&ack_len, (uint8_t**)&rprt_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && rprt_return->payload.init.type == TWR_MSG_TYPE_REPORT)
                break;
        }
    }

    /* Note Timestamps recorded by Anchor */
    esp_init_rx_ts.set_timestamp(rprt_return->payload.report.pollRx);
    esp_resp_tx_ts.set_timestamp(rprt_return->payload.report.responseTx);
    esp_fin_rx_ts.set_timestamp(rprt_return->payload.report.finalRx);

    /* cleanup */
    delete rprt_return;

    return SUCCESS;
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

    // go through state machine state by state and do the error handling accordingly
    if (do_init_state(init_tx_ts, anchor_addr) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;

    fprintf(stdout, "Init Sent\n");

    if (do_response_ack_state(ack_rx_ts) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;

    fprintf(stdout, "Got response\n");

    if (do_final_state(fin_tx_ts, anchor_addr) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;

    fprintf(stdout, "Sent Final\n");

    if (do_report_state(esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts) == dwm_com_error_t::ERROR)
        return dwm_com_error_t::ERROR;

    fprintf(stdout, "got report\n");

    // return distance procedurally
    *distance = timestamps2distance(
        init_tx_ts, ack_rx_ts, fin_tx_ts,
        esp_init_rx_ts, esp_resp_tx_ts, esp_fin_rx_ts
    );

    return dwm_com_error_t::SUCCESS;
}