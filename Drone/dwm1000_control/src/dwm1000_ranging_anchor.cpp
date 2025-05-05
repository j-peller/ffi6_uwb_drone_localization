#include "../inc/dwm1000_ranging_anchor.hpp"
#include "../inc/dw1000_time.hpp"
#include <time.h>


/**
 * 
 */
DWMRangingAnchor::~DWMRangingAnchor()
{
    /* TODO */
}


/**
 * @brief Wait out errors to cause anchor to also go into an error state if a
 * response is expected.
 * 
 */
void DWMRangingAnchor::waitOutError()
{
    timespec start, now;
    clock_gettime(CLOCK_MONOTONIC_RAW,  &start);
    do { 
        clock_gettime(CLOCK_MONOTONIC_RAW,  &now);
    } while (timespec_delta_nanoseconds(&now, &start) < (RX_RETRY * RX_TIMEOUT));
}

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
    _controller->set_receiver_auto_reenable(true);
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_rx_status();
        if (ret == TIMEOUT)
        {
            continue; //< we dont care for timeout in init state
        } else if (ret == ERROR) {
            //waitOutError();
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
        }
    }

    /* Remember receive timestamp */
    _controller->get_rx_timestamp(_init_rx_ts);
    fprintf(stdout, "Got init_rx_ts: %ld\n", _init_rx_ts.get_timestamp());

    return SUCCESS;
}


/**
 * @brief Complete actions taken in the response acknowledge state.
 * 
 * @param ack_rx_ts Timestamp of poll acknowledge reception.
 * @return dwm_com_error_t 
 */
dwm_com_error_t DWMRangingAnchor::do_response_ack_state(uint16_t anchor_addr)
{
    dwm_com_error_t ret = SUCCESS;

    twr_message_t resp_msg = {
        .header = (twr_frame_header_t) {
            .frameCtrl = {0x41, 0x88},
            .seqNum = 0x00,
            .panID = {0xCA, 0xDE},
            .destAddr = { MASTER & 0xff, MASTER >> 8 },
            .srcAddr = { anchor_addr & 0xff, anchor_addr >> 8 }
        },
        .payload = { .response = {.type = twr_msg_type_t::TWR_MSG_TYPE_RESPONSE,}}
    };

    /* disable receiver auto reenable */
    _controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&resp_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

    /* Poll for completion of transmission */
    ret = _controller->poll_tx_status();    
    if (ret != SUCCESS) {
        fprintf(stdout, "Error while sending Response: %d\n", ret);
        return ret;
    }

    /* Note time of transmission */
    _controller->get_tx_timestamp(_resp_tx_ts);
    fprintf(stdout, "Got resp_tx_ts: %ld\n", _resp_tx_ts.get_timestamp());

    return dwm_com_error_t::SUCCESS;
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
    _controller->set_receiver_auto_reenable(true);
    
    // poll and check for error
    while (true)
    {
        /* Poll for the reception of a packet */
        ret = _controller->poll_rx_status();
        if (ret != SUCCESS)
        {
            // for testing only...
            return ret;
        } else {
            ret = _controller->read_received_data(&ack_len, (uint8_t**)&fin_return);
            if (ret != SUCCESS) {
                /* Error handling */
                continue;
            }

            /* Check if we got expected message type and only escape if valid */
            if ( ack_len == sizeof(twr_message_t) && fin_return->payload.init.type == TWR_MSG_TYPE_FINAL)
                break;
        }
    }

    /* Remember receive timestamp */
    _controller->get_rx_timestamp(_final_rx_ts);
    fprintf(stdout, "Got final_rx_ts: %ld\n", _final_rx_ts.get_timestamp());


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
            .panID = {0xCA, 0xDE},
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

    /* disable receiver auto reenable */
    _controller->set_receiver_auto_reenable(false);

    /* Write Packet payload to tx buffer */
    _controller->write_transmission_data((uint8_t*)&report_msg, sizeof(twr_message_t));

    /* Start transmission of the answer */
    _controller->start_transmission();

    /* Poll for completion of transmission */
    ret = _controller->poll_tx_status();    
    if (ret != SUCCESS) {
        fprintf(stdout, "Error while sending Response: %d\n", ret);
        return ret;
    }

    return SUCCESS;
}

//dwm_com_error_t DWMRangingAnchor::run_state_machine()
//{
//    dwm_com_error_t ret = SUCCESS;
//
//    if ( (ret =  do_init_state()) != SUCCESS ) {
//        fprintf(stdout, "Error in init state %d\n", ret);
//        return ret;
//    }
//    fprintf(stdout, "Got Init\n");
//
//    if ( (ret = do_response_ack_state(ANCHOR_1)) != SUCCESS) {
//        fprintf(stdout, "Error in response state %d\n", ret);
//        return ret;
//    }
//    fprintf(stdout, "Sent Response\n");
//    
//    if ( (ret = do_final_state()) != SUCCESS) {
//        fprintf(stdout, "Error in final state %d\n", ret);
//        return ret;
//    }
//    fprintf(stdout, "Got Final\n");
//
//    if ( (ret = do_report_state(ANCHOR_1)) != SUCCESS) {
//        fprintf(stdout, "Error in report state %d\n", ret);
//        return ret;
//    }
//    fprintf(stdout, "Send Report\n");
//
//    return ret;
//}


dwm_com_error_t DWMRangingAnchor::run_state_machine()
{
    int retries = 0;
    RangingState state = RangingState::INIT;
    dwm_com_error_t ret = SUCCESS;

    // variables in method scope
    bool timeout_occurred = false;

    while (state != RangingState::COMPLETE) {

        switch (state) {
            case RangingState::INIT:
                ret = do_init_state();
                HANDLE_STATE_TRANSITION(ret, RangingState::RESP_ACK, RangingState::INIT, timeout_occurred);
                break;

            case RangingState::RESP_ACK:
                ret = do_response_ack_state(ANCHOR_1);
                HANDLE_STATE_TRANSITION(ret, RangingState::FINAL, RangingState::INIT, timeout_occurred);
                break;

            case RangingState::FINAL:
                ret = do_final_state();
                HANDLE_STATE_TRANSITION(ret, RangingState::REPORT, RangingState::INIT, timeout_occurred);
                break;

            case RangingState::REPORT:
                ret = do_report_state(ANCHOR_1);
                HANDLE_STATE_TRANSITION(ret, RangingState::COMPLETE, RangingState::INIT, timeout_occurred);
                break;

            case RangingState::COMPLETE:
                break;
        }

        if (timeout_occurred) {
            retries++;
            timeout_occurred = false;
            if (retries >= MAX_RETRY_ON_FAILURE) {
                fprintf(stdout, "Max retries reached. Exiting...\n");
                return dwm_com_error_t::ERROR;
            }
            waitOutError();
        }
    }

    return dwm_com_error_t::SUCCESS;
}