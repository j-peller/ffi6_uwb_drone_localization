#include "../inc/dwm1000_ranging.hpp"

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
double DWMRangingDrone::timestamps2distance(
    DW1000Time& t_sp, DW1000Time& t_ra, DW1000Time& t_sf,
    DW1000Time& t_rp, DW1000Time& t_sa, DW1000Time& t_rf 
) {

    #if DEBUG
    fprintf(stdout, "t_sp: %ld\n", t_sp.get_timestamp());
    fprintf(stdout, "t_ra: %ld\n", t_ra.get_timestamp());
    fprintf(stdout, "t_sf: %ld\n", t_sf.get_timestamp());
    fprintf(stdout, "t_rp: %ld\n", t_rp.get_timestamp());
    fprintf(stdout, "t_sa: %ld\n", t_sa.get_timestamp());
    fprintf(stdout, "t_rf: %ld\n", t_rf.get_timestamp());
    #endif 
    
    DW1000Time t_round1 = (t_ra - t_sp).wrap();
    DW1000Time t_round2 = (t_rf - t_sa).wrap();
    DW1000Time t_reply1 = (t_sa - t_rp).wrap();
    DW1000Time t_reply2 = (t_sf - t_ra).wrap();

    #if DEBUG
    fprintf(stdout, "r_round1: %ld\n", t_round1.get_timestamp());
    fprintf(stdout, "r_round2: %ld\n", t_round2.get_timestamp());
    fprintf(stdout, "r_reply1: %ld\n", t_reply1.get_timestamp());
    fprintf(stdout, "r_reply2: %ld\n", t_reply2.get_timestamp());
    #endif

    DW1000Time time_of_flight = ((t_round1 * t_round2) - (t_reply1 * t_reply2)) / (t_round1 + t_round2 + t_reply1 + t_reply2);


    #if DEBUG
    fprintf(stdout, "ToF: %ld\n", time_of_flight.get_timestamp());
    #endif

    // calculate and return distance from TOF
    return time_of_flight.get_as_meters();
}


/**
 * @brief Perform ranging with all 4 anchors.
 * 
 * @param distances Struct containing all distances as doubles.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingDrone::get_distances_to_anchors(distances* distances)
{
    dwm_com_error_t ret = SUCCESS;

    if ( (ret = get_distance_to_anchor(ANCHOR_1, &(distances->d1))) != dwm_com_error_t::SUCCESS) {
        return ret;
    }
    
    if ( (ret = get_distance_to_anchor(ANCHOR_2, &(distances->d2))) != dwm_com_error_t::SUCCESS) {
        return ret;
    }
    
    if ( (ret = get_distance_to_anchor(ANCHOR_3, &(distances->d3))) != dwm_com_error_t::SUCCESS) {
        return ret;
    }
    
    if ( (ret = get_distance_to_anchor(ANCHOR_4, &(distances->d4))) != dwm_com_error_t::SUCCESS) {
        return ret;
    }

    return ret;
}


/**
 * @brief Complete actions taken in the init state of the ranging process.
 * 
 * @param init_tx_ts Timestamp of poll msg transmission.
 * @param anchor_addr Address of current anchor.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingDrone::do_init_state(uint16_t anchor_addr)
{
    dwm_com_error_t ret = SUCCESS;

    twr_message_t init_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {DEFAULT_PAN & 0xff, DEFAULT_PAN >> 8},
            .destAddr = { anchor_addr & 0xff, anchor_addr >> 8 },
            .srcAddr = { MASTER & 0xff, MASTER >> 8 }
        },
        .payload = { 
            .init = (twr_init_message_t) {
                .type = twr_msg_type_t::TWR_MSG_TYPE_POLL,
                .anchorShortAddr = {anchor_addr & 0xff, anchor_addr >> 8},
            }
        }
    };

    /* */
    //_controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&init_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();
    
    return SUCCESS;
}


/**
 * @brief Complete actions taken in the response acknowledge state.
 * 
 * @param ack_rx_ts Timestamp of poll acknowledge reception.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingDrone::do_response_ack_state(DW1000Time& init_tx_ts, DW1000Time& ack_rx_ts)
{
    twr_message_t* ack_return = NULL;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;  

    /* Start reception of packets */
    //_controller->start_receiving();
    //_controller->set_receiver_auto_reenable(true);
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_status_bit(SYS_STATUS_RXDFR, TAG_RESP_DLY_DEFAULT_MS);
        if (ret != SUCCESS) {
            return ret;
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

    /* Note Timestamp of Reception and Transmission or prev. Poll */
    _controller->get_tx_timestamp(init_tx_ts);
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
dwm_com_error_t DWMRangingDrone::do_final_state(uint16_t anchor_addr) 
{
    dwm_com_error_t ret = SUCCESS;

    twr_message_t final_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {DEFAULT_PAN & 0xff, DEFAULT_PAN >> 8},
            .destAddr = { anchor_addr & 0xff, anchor_addr >> 8 },
            .srcAddr = { MASTER & 0xff, MASTER >> 8 }
        },
        .payload = { 
            .final = {
                .type = twr_msg_type_t::TWR_MSG_TYPE_FINAL,
            }
        }
    };

    /* */
    //_controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&final_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

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
dwm_com_error_t DWMRangingDrone::do_report_state(DW1000Time& fin_tx_ts, DW1000Time& esp_init_rx_ts, DW1000Time& esp_resp_tx_ts, DW1000Time& esp_fin_rx_ts) 
{
    twr_message_t* rprt_return = NULL;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;  

    /* Start reception of packets */
    //_controller->start_receiving();
    //_controller->set_receiver_auto_reenable(true);
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_status_bit(SYS_STATUS_RXDFR, TAG_RESP_DLY_DEFAULT_MS);
        if (ret != SUCCESS)
        {
            //waitOutError();
            return ret;
        } else {

            /**
             * DONT REMOVE DIS SLEEP
             * I USED TO KNOW WHATS GOING ON HERE
             * NOW ONLY GOD KNOWS.
             */

            busywait_nanoseconds(500000); //< I mean dis, do not remove!!11ELF

            /**
             * THANKS TO THE GUYS AT THE DWM1000 TEAM
             * FOR THIS PIECE OF CRAPPY HARDWARE
             */

            ret = _controller->read_received_data(&ack_len, (uint8_t**)&rprt_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && rprt_return->payload.report.type == TWR_MSG_TYPE_REPORT)
                break;
        }
    }

    /* Collect time of transmission of prev. final state */
    _controller->get_tx_timestamp(fin_tx_ts);

    /* Note Timestamps recorded by the Anchor */
    esp_init_rx_ts.set_timestamp(rprt_return->payload.report.pollRx);
    esp_resp_tx_ts.set_timestamp(rprt_return->payload.report.responseTx);
    esp_fin_rx_ts.set_timestamp(rprt_return->payload.report.finalRx);

    #if DEBUG
    //fprintf(stdout, "Got esp_init_rx_ts: %ld\n", esp_fin_rx_ts.get_timestamp());
    //fprintf(stdout, "Got esp_init_rx_tx in msg: %02X %02X %02X %02X %02X\n", 
    //        rprt_return->payload.report.finalRx[0],
    //        rprt_return->payload.report.finalRx[1],
    //        rprt_return->payload.report.finalRx[2],
    //        rprt_return->payload.report.finalRx[3],
    //        rprt_return->payload.report.finalRx[4]);
    #endif

    /* cleanup */
    delete rprt_return;

    return SUCCESS;
}


/**
 * 
 */
dwm_com_error_t DWMRangingDrone::get_distance_to_anchor(uint16_t anchor_addr, double* distance)
{
    int retries = 0;
    RangingState state = RangingState::INIT;
    dwm_com_error_t ret = SUCCESS;

    // variables in method scope
    DW1000Time t_sp, t_ra, t_sf;
    DW1000Time t_rp, t_sa, t_rf;

    while (state != RangingState::COMPLETE) {

        switch (state) {
            case RangingState::INIT:
                fprintf(stdout, "INIT\n");
                ret = do_init_state(anchor_addr);
                if (ret == SUCCESS)
                    state = RangingState::RESP_ACK;
                else
                    return ret;
                break;

            case RangingState::RESP_ACK:
                fprintf(stdout, "RESP\n");
                ret = do_response_ack_state(t_sp, t_ra);
                if (ret == SUCCESS)
                    state = RangingState::FINAL;
                else
                    return ret;
                break;

            case RangingState::FINAL:
                fprintf(stdout, "FINAL\n");
                ret = do_final_state(anchor_addr);
                if (ret == SUCCESS)
                    state = RangingState::REPORT;
                else
                    return ret;
                break;

            case RangingState::REPORT:
                fprintf(stdout, "REPORT\n");
                ret = do_report_state(t_sf, t_rp, t_sa, t_rf);
                if (ret == SUCCESS)
                    state = RangingState::COMPLETE;
                else
                    return ret;
                break;

            case RangingState::COMPLETE:
                return SUCCESS;
        }

        #if DEBUG
        fprintf(stdout, "Cur State: %d\n", (int)state);
        #endif 
    }

    /* Calculate ToF */
    *distance = timestamps2distance(
        t_sp, t_ra, t_sf,
        t_rp, t_sa, t_rf 
    );

    return ret;
}

/**
 * 
 */
dwm_com_error_t DWMRangingDrone::calibrate_antenna_delay(double known_distance_m, double allowed_error_m, int max_iterations)
{
    dwm_com_error_t ret = SUCCESS;
    double antd = INITIAL_ANTENNA_DELAY; // Initial guess for TX and RX antenna delay
    double current_distance_m = 0.0f;

    /* TODO: Adjust RX Power Level for antenna Calibration */

    int i = 0;
    while (i < max_iterations) {
        /* Set the current antenna delay */
        _controller->set_tx_antenna_delay((uint16_t)(antd + 0.5f));
        _controller->set_rx_antenna_delay((uint16_t)(antd + 0.5f));

        /* Perform ranging with new antenna delay */
        int k = 0;
        double sum_distance_m = 0.0f;
        while (k < 5) {
            ret = get_distance_to_anchor(ANCHOR_1, &current_distance_m);
            if (ret != SUCCESS) {
                continue; // Skip this iteration if the distance is invalid
            }

            /* Accumulate the distance */
            sum_distance_m += current_distance_m;

            k++;
        }

        /* Is current measurement good enough? */
        double error_m = ( sum_distance_m / 5.0 ) - known_distance_m;
        if (fabs(error_m) <= allowed_error_m) {
            printf("Converged at iteration %d: Delay = %.2f (Measured: %.4f m, Error: %.4f m)\n",
                   i + 1, antd, sum_distance_m / 5.0, error_m);
            return SUCCESS;
        }

        /* convert range error to time error */
        double time_error_us = (error_m / DW1000Time::SPEED_OF_LIGHT_M_US); // Convert to microseconds
        double delay_units = time_error_us * DW1000Time::DW1000_TIME_UNITS_PER_US;

        /* devide by 4.0 because we adjust for tx and rx antenna on both drone and anchor */
        antd += (delay_units / 4.0f); // Adjust the antenna delay based on the error

        /* */
        if (antd < 0.0) antd = 0.0;
        else if (antd > 0xFFFF) antd = 0xFFFF;

        /* Wait */
        busywait_nanoseconds(100000000);

        /* Send calculated antenna delay to anchor device */
        _controller->send_antenna_calibration_value((uint16_t)(antd));
        
        busywait_nanoseconds(100000000);
        
        printf("Iteration %d: Delay = %.2f units, Measured = %.4f m, Error = %.4f m, Correction = %.2f units\n",
               i + 1, antd, sum_distance_m / 5.0, error_m, delay_units);
        
        i++;
    }

    printf("Calibration failed to converge after %d iterations.\n", max_iterations);

    return ERROR;
}
