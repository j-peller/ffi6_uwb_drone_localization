#include "DW1000Ranging.hpp"


DW1000RangingTag::DW1000RangingTag(DeviceType deviceMode, DW1000& dw1000) : DW1000Ranging(deviceMode, dw1000)
{
    
}

void DW1000RangingTag::loop()
{
    DW1000Ranging::loop();
    switch(currentState)
    {
        case POLL:
        {
            twr_message_t init_msg = {
                .header = (twr_frame_header_t) {
                    .frameCtrl = {0x41, 0x88},
                    .seqNum = 0x00,
                    .panID = {0xCA, 0xDE},
                    .destAddr = { anchor_id & 0xff, anchor_id >> 8 },
                    .srcAddr = { tag_id & 0xff, tag_id >> 8 }
                },
                .payload = { .init = (twr_init_message_t) {
                    .type = twr_msg_type_t::TWR_MSG_TYPE_POLL,
                    .anchorShortAddr = {anchor_id & 0xff, anchor_id >> 8},
                    .responseDelay = {0x0},
                }}
            };

            dw1000.transmit((uint8_t*)&init_msg, sizeof(twr_message_t));

            currentState = RESPONSE;
            break;
        }
            
        case RESPONSE:
        {
            dw1000.startReceiving();
            dw1000.setReceiverAutoReenable(true);
            uint8_t* message = nullptr;
            uint16_t length = 0;
            dw1000.readReceivedData(&message, &length);
            if(message != nullptr)
            {
                dw1000.get_tx_timestamp(init_tx_ts);
                dw1000.get_rx_timestamp(ack_rx_ts);
                delete[] message;
                dw1000.logger->output("TX:");
                currentState = POLL;
            }
            


            
            break;
        }
    }

}