#include "DW1000Ranging.hpp"


DW1000RangingAnchor::DW1000RangingAnchor(DeviceType deviceMode, DW1000& dw1000, uint16_t deviceAddress, uint16_t tagAddress) : DW1000Ranging(deviceMode, dw1000)
{
    dw1000.setDeviceID(deviceAddress);
    this->tag_address = tagAddress;
    this->deviceAddress = deviceAddress; //TODO setter + in parent class
}

void DW1000RangingAnchor::pollStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_LDEDONE)
    {
        clear_mask |= SYS_STATUS_LDEDONE;
        dw1000.get_rx_timestamp(esp_init_rx_ts);
        dw1000.removeCustomInterruptHandler();

        /* Send response */
        //TODO evaluate if a send should happen in the last interrupt? For now we will do it in the main loop!

        systemState = STATE_MEASURING_ACTIVE;
        currentCommState = RESPONSE;
    }

    uint32_t data = {0};
    dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &data);
    data &= ~(clear_mask);
    dw1000.writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data);
}

void DW1000RangingAnchor::ackStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_TXFRS)
    {
        clear_mask |= SYS_STATUS_TXFRS;
        dw1000.get_tx_timestamp(esp_resp_tx_ts);
    }
    if(sys_status & SYS_STATUS_LDEDONE)
    {
        clear_mask |= SYS_STATUS_LDEDONE;
        dw1000.get_rx_timestamp(esp_fin_rx_ts);
        systemState = STATE_MEASURING_ACTIVE;
        currentCommState = REPORT;
    }


    uint32_t data = {0};
    dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &data);
    data &= ~(clear_mask);
    dw1000.writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data);
}

void DW1000RangingAnchor::loop()
{
    DW1000Ranging::loop();
    switch(currentCommState)
    {
        case POLL:
        {
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                systemState = STATE_MEASURING_WAITING;
                dw1000.startReceiving();
                dw1000.setReceiverAutoReenable(true);
                
                dw1000.addCustomInterruptHandler(InterruptTable::INTERRUPT_ON_LDE_DONE, [this](uint32_t value) { this->pollStateIRQHandler(value); });
            }
            break;
        }
            
        case RESPONSE:
        {
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                
                dw1000.setReceiverAutoReenable(false);

                twr_message_t ack_msg = {
                    .header = (twr_frame_header_t) {
                        .frameCtrl = {0x41, 0x88},
                        .seqNum = 0x00,
                        .panID = {0xCA, 0xDE},
                        .destAddr = { this->tag_address & 0xff, this->tag_address >> 8 },
                        .srcAddr = { this->deviceAddress & 0xff, this->deviceAddress >> 8 }
                    },
                    .payload = { .response = (twr_response_message_t) {
                        .type = twr_msg_type_t::TWR_MSG_TYPE_RESPONSE,
                    }}
                };
                systemState = STATE_MEASURING_WAITING;
                dw1000.addCustomInterruptHandler((InterruptTable) (SYS_STATUS_TXFRS | SYS_STATUS_LDEDONE), [this](uint32_t value) { this->pollStateIRQHandler(value);});
                dw1000.transmit((uint8_t*)&ack_msg, sizeof(twr_message_t));
            }
            break;
        }
        case FINAL:
        {
            break;
        }      
        case REPORT:
        {
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                systemState = STATE_MEASURING_WAITING;
                twr_message_t report_msg = {
                    .header = (twr_frame_header_t) {
                        .frameCtrl = {0x41, 0x88},
                        .seqNum = 0x00,
                        .panID = {0xCA, 0xDE},
                        .destAddr = { this->tag_address & 0xff, this->tag_address >> 8 },
                        .srcAddr = { this->deviceAddress & 0xff, this->deviceAddress >> 8 }
                    },
                    .payload = { .report = (twr_report_message_t) {
                        .type = twr_msg_type_t::TWR_MSG_TYPE_REPORT,
                        .pollRx = esp_init_rx_ts.get_timestamp(),
                        .responseTx = esp_resp_tx_ts.get_timestamp(),
                        .finalRx = esp_fin_rx_ts.get_timestamp(),
                    }}   
                };
                dw1000.transmit((uint8_t*)&report_msg, sizeof(twr_message_t));

                /* Todo we should go into an interrupt here and then switch the state! */
                currentCommState = POLL;
                systemState = STATE_IDLE;
            }
            break;
        }
    }

}
void DW1000RangingAnchor::resetTimestamps()
{
    esp_init_rx_ts.set_timestamp((uint64_t) 0);
    esp_resp_tx_ts.set_timestamp((uint64_t) 0);
    esp_fin_rx_ts.set_timestamp((uint64_t) 0);
}

