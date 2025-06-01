#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_time.hpp"


/**
 * @brief Complete actions taken in the init state of the ranging process.
 * 
 * @param init_tx_ts Timestamp of poll msg transmission.
 * @param anchor_addr Address of current anchor.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingAnchor::do_init_state()
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
        ret = _controller->poll_status_bit(SYS_STATUS_RXDFR, ANC_RESP_DLY_DEFAULT_MS);
        if (ret == TIMEOUT)
        {
            return ret;
        } else if (ret == ERROR) {
            return ret;
        } else {
            ret = _controller->read_received_data(&ack_len, (uint8_t**)&ack_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && ack_return->payload.init.type == TWR_MSG_TYPE_POLL)
                break;
            else {
                fprintf(stdout, "Got unexpected message type: %d\n", ack_return->payload.init.type);
                return ERROR;
            }
        }
    }

    /* Remember receive timestamp */
    _controller->get_rx_timestamp(_init_rx_ts);

    #if DEBUG
    fprintf(stdout, "Got init_rx_ts: %ld\n", _init_rx_ts.get_timestamp());
    #endif

    return SUCCESS;
}


/**
 * @brief Complete actions taken in the response acknowledge state.
 * 
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingAnchor::do_response_ack_state(uint16_t anchor_addr)
{
    dwm_com_error_t ret = SUCCESS;

    twr_message_t resp_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {DEFAULT_PAN & 0xff, DEFAULT_PAN >> 8},
            .destAddr = { MASTER & 0xff, MASTER >> 8 },
            .srcAddr = { anchor_addr & 0xff, anchor_addr >> 8 }
        },
        .payload = { 
            .response = {
                .type = twr_msg_type_t::TWR_MSG_TYPE_RESPONSE
            }
        }
    };

    /* disable receiver auto reenable */
    //_controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&resp_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

    return SUCCESS;
}


/**
 * @brief Complete actions taken in the final state of the ranging process.
 * 
 * @param fin_tx_ts Timestamp of final msg transmission.
 * @param anchor_addr Address of current anchor.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingAnchor::do_final_state() 
{
    twr_message_t* fin_return = NULL;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;  

    /* Start reception of packets */
    //_controller->start_receiving();
    //_controller->set_receiver_auto_reenable(true);
        
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_status_bit(SYS_STATUS_RXDFR, ANC_RESP_DLY_DEFAULT_MS);
        if (ret != SUCCESS)
        {
            return ret;
        } else {
            ret = _controller->read_received_data(&ack_len, (uint8_t**)&fin_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && fin_return->payload.final.type == TWR_MSG_TYPE_FINAL)
                break;
            else {
                fprintf(stdout, "Got unexpected message type: %d\n", fin_return->payload.final.type);
                return ERROR;
            }
        }
    }

    /* Remember receive and transmit timestamp */
    _controller->get_tx_timestamp(_resp_tx_ts);
    _controller->get_rx_timestamp(_final_rx_ts);

    #if DEBUG
    fprintf(stdout, "Sent resp_tx_ts: %ld\n", _resp_tx_ts.get_timestamp());
    fprintf(stdout, "Got final_rx_ts: %ld\n", _final_rx_ts.get_timestamp());
    #endif

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
dwm_com_error_t DWMRangingAnchor::do_report_state(uint16_t anchor_addr) 
{
    dwm_com_error_t ret = SUCCESS;

    twr_message_t report_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {DEFAULT_PAN & 0xff, DEFAULT_PAN >> 8},
            .destAddr = { MASTER & 0xff, MASTER >> 8 },
            .srcAddr = { anchor_addr & 0xff, anchor_addr >> 8 }
        },
        .payload = { .report = {
            .type = TWR_MSG_TYPE_REPORT,
        }}
    };

    _init_rx_ts.get_timestamp(report_msg.payload.report.pollRx);
    _resp_tx_ts.get_timestamp(report_msg.payload.report.responseTx);
    _final_rx_ts.get_timestamp(report_msg.payload.report.finalRx);

    #if DEBUG
    //fprintf(stdout, "finalRx in DW100Time: %ld\n", _final_rx_ts.get_timestamp());
    //fprintf(stdout, "pollRx in msg: %02X %02X %02X %02X %02X\n", 
    //        report_msg.payload.report.finalRx[0],
    //        report_msg.payload.report.finalRx[1],
    //        report_msg.payload.report.finalRx[2],
    //        report_msg.payload.report.finalRx[3],
    //        report_msg.payload.report.finalRx[4]);
    #endif

    /* disable receiver auto reenable */
    //_controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&report_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

    /* Poll for completion of transmission, otherwise transmission will be interrupted by the next reception cycle */
    ret = _controller->poll_tx_status();    
    if (ret != SUCCESS) {
        fprintf(stdout, "Error while sending Response: %d\n", ret);
        return ret;
    }

    return SUCCESS;
}


/**
 * 
*/
dwm_com_error_t DWMRangingAnchor::run_state_machine()
{
    int retries = 0;
    RangingState state = RangingState::INIT;
    dwm_com_error_t ret = SUCCESS;

    while (state != RangingState::COMPLETE) {

        switch (state) {
            case RangingState::INIT:
                fprintf(stdout, "INIT\n");
                ret = do_init_state();
                if (ret == SUCCESS)
                    state = RangingState::RESP_ACK;
                else
                    return ret;
                break;

            case RangingState::RESP_ACK:
                fprintf(stdout, "RESP\n");
                ret = do_response_ack_state(_controller->_dev_instance.short_addr);
                if (ret == SUCCESS)
                    state = RangingState::FINAL;
                else
                    return ret;
                break;

            case RangingState::FINAL:
                fprintf(stdout, "FINAL\n");
                ret = do_final_state();
                if (ret == SUCCESS)
                    state = RangingState::REPORT;
                else
                    return ret;
                break;

            case RangingState::REPORT:
                fprintf(stdout, "REPORT\n");
                ret = do_report_state(_controller->_dev_instance.short_addr);
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

    return SUCCESS;
}


/**
 * 
 */
dwm_com_error_t DWMRangingAnchor::calibrate_antenna_delay(int max_iterations)
{
    double antd = INITIAL_ANTENNA_DELAY; // Initial guess for TX and RX antenna delay
    uint16_t tmp = 0;

    int i = 0;
    while (i < max_iterations) {
        /* Set the current antenna delay */
        _controller->set_tx_antenna_delay((uint16_t)(antd + 0.5f));
        _controller->set_rx_antenna_delay((uint16_t)(antd + 0.5f));

        /* Run state machine for ranging once */
        while (true) {
            if (run_state_machine() == SUCCESS) {
                /* we got successfull ranging skip to receiving correction value */
                break;
            }
        }
        
        /* wait for correction value from drone */
        _controller->wait_for_antenna_calibration_value(&tmp);
        antd = (double)tmp; // convert to meters
        printf("Iteration %d: Delay = %.2f units\n", i + 1, antd);
        i++;
    }

    printf("Calibration failed to converge after %d iterations.\n", max_iterations);

    return ERROR;
}