#include "DW1000Ranging.hpp"


DW1000RangingTag::DW1000RangingTag(DeviceType deviceMode, DW1000& dw1000) : DW1000Ranging(deviceMode, dw1000)
{
    
}
void DW1000RangingTag::pollStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_TXFRS)
    {
        dw1000.get_tx_timestamp(init_tx_ts);
        dw1000.logger->addBuffer("TX TIMESTAMP");
        clear_mask |= (SYS_STATUS_TXFRS);
        dw1000.startReceiving();
    }
    if(sys_status & SYS_STATUS_LDEDONE)
    {
        dw1000.get_rx_timestamp(ack_rx_ts);
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
void DW1000RangingTag::loop()
{
    DW1000Ranging::loop();
    switch(currentCommState)
    {
        case POLL:
        {
            if(systemState != STATE_MEASURING_WAITING)
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

                systemState = STATE_MEASURING_WAITING;
                dw1000.logger->output("TRYING TO SEND: %x", sizeof(twr_message_t));
                dw1000.addCustomInterruptHandler(InterruptTable::INTERRUPT_ON_TX | InterruptTable::INTERRUPT_ON_LDE_DONE, [this](uint32_t value) { this->pollStateIRQHandler(value); });
                dw1000.transmit((uint8_t*)&init_msg, sizeof(twr_message_t));
            }
            
            break;
        }
            
        case RESPONSE:
        {
            dw1000.logger->output("RX: %x %x", init_tx_ts.get_timestamp(), ack_rx_ts.get_timestamp());
            
            break;
        }
        default:
            break;
    }

}