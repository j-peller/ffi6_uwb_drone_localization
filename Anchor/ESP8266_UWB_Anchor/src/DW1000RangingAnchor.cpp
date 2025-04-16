#include "DW1000Ranging.hpp"


DW1000RangingAnchor::DW1000RangingAnchor(DeviceType deviceMode, DW1000& dw1000) : DW1000Ranging(deviceMode, dw1000)
{
    
}

void DW1000RangingAnchor::loop()
{
    DW1000Ranging::loop();
    switch(currentState)
    {
        case POLL:
        {
            dw1000.startReceiving();
            dw1000.setReceiverAutoReenable(true);
            uint8_t* message = nullptr;
            uint16_t length = 0;
            dw1000.readReceivedData(&message, &length);
            if(message != nullptr)
            {
                dw1000.get_rx_timestamp(esp_init_rx_ts);
                delete[] message;
                dw1000.logger->output("RX:");
                currentState = RESPONSE;
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
            currentState = POLL;
            break;
        }
    }

}