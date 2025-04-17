#include "DW1000Ranging.hpp"


DW1000RangingAnchor::DW1000RangingAnchor(DeviceType deviceMode, DW1000& dw1000) : DW1000Ranging(deviceMode, dw1000)
{
    
}

void DW1000RangingAnchor::pollStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_LDEDONE)
    {
        clear_mask |= SYS_STATUS_LDEDONE;
        dw1000.get_rx_timestamp(esp_init_rx_ts);
        clear_mask |= (SYS_STATUS_LDEDONE);
        dw1000.removeCustomInterruptHandler();
        systemState = STATE_MEASURING_ACTIVE;
        currentCommState = RESPONSE;
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
            dw1000.startReceiving();
            dw1000.setReceiverAutoReenable(true);

            dw1000.addCustomInterruptHandler(InterruptTable::INTERRUPT_ON_LDE_DONE, [this](uint32_t value) { this->pollStateIRQHandler(value); });
            
            uint8_t* message = nullptr;
            uint16_t length = 0;
            dw1000.readReceivedData(&message, &length);
            if(message != nullptr)
            {
                dw1000.get_rx_timestamp(esp_init_rx_ts);
                delete[] message;
                dw1000.logger->output("RX:");
                currentCommState = RESPONSE;
            }
            break;
        }
            
        case RESPONSE:
        {
            dw1000.setReceiverAutoReenable(false);
            twr_message_t ack_msg = {
                .header = (twr_frame_header_t) {
                    .frameCtrl = {0x41, 0x88},
                    .seqNum = 0x00,
                    .panID = {0xCA, 0xDE},
                    .destAddr = { tag_id & 0xff, tag_id >> 8 },
                    .srcAddr = { anchor_id & 0xff, anchor_id >> 8 }
                },
                .payload = { .response = (twr_response_message_t) {
                    .type = twr_msg_type_t::TWR_MSG_TYPE_RESPONSE,
                }}
            };

            dw1000.transmit((uint8_t*)&ack_msg, sizeof(twr_message_t));


            dw1000.logger->output("TX:");
            currentCommState = POLL;
            break;
        }
    }

}