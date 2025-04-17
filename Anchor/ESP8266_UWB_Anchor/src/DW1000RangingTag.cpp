#include "DW1000Ranging.hpp"


DW1000RangingTag::DW1000RangingTag(DeviceType deviceMode, DW1000& dw1000, uint16_t deviceAddress, uint16_t anchorAddress) : DW1000Ranging(deviceMode, dw1000)
{
    dw1000.setDeviceID(deviceAddress);
    this->anchor_address = anchorAddress;
    this->deviceAddress = deviceAddress;
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
        currentCommState = FINAL;
    }
    if(clear_mask)
    {
        uint32_t data = {0};
        dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &data);
        data &= ~(clear_mask);
        dw1000.writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data);
    }
    

}

void DW1000RangingTag::finalStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_TXFRS)
    {
        dw1000.get_tx_timestamp(fin_tx_ts);
        dw1000.logger->addBuffer("Fin TX TIMESTAMP");
        clear_mask |= (SYS_STATUS_TXFRS);
        dw1000.startReceiving();
        dw1000.removeCustomInterruptHandler();
        dw1000.addCustomInterruptHandler((InterruptTable) SYS_STATUS_RXDFR, [this](uint32_t value) { this->pollStateIRQHandler(value); });
        
        uint32_t data = {0}; //todo eigene funktion zum clearen in dw1000 wäre gut
        dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &data);
        data &= ~(clear_mask);
        dw1000.writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data);
    
    }
    
}

void DW1000RangingTag::reportStateIRQHandler(uint32_t sys_status)
{
    uint32_t clear_mask = 0;
    if(sys_status & SYS_STATUS_RXDFR) {
        /*/ We can parse the received data now */
        currentCommState = FINAL;
        systemState = STATE_MEASURING_ACTIVE;
        clear_mask |= SYS_STATUS_RXDFR;
    
    uint32_t data = {0};
    dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &data);
    data &= ~(clear_mask);
    dw1000.writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data);
    }
}
void DW1000RangingTag::loop()
{
    DW1000Ranging::loop();
    switch(currentCommState)
    {
        case POLL:
        {
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                systemState = STATE_MEASURING_WAITING;
                twr_message_t init_msg = {
                    .header = (twr_frame_header_t) {
                        .frameCtrl = {0x41, 0x88},
                        .seqNum = 0x00,
                        .panID = {0xCA, 0xDE},
                        .destAddr = { this->anchor_address & 0xff, this->anchor_address >> 8 },
                        .srcAddr = { this->deviceAddress & 0xff, this->deviceAddress >> 8 }
                    },
                    .payload = { .init = (twr_init_message_t) {
                        .type = twr_msg_type_t::TWR_MSG_TYPE_POLL,
                        .anchorShortAddr = {this->anchor_address & 0xff, this->anchor_address >> 8},
                        .responseDelay = {0x0},
                    }}
                };

                
                dw1000.logger->output("TRYING TO SEND: %x", sizeof(twr_message_t));
                dw1000.addCustomInterruptHandler(InterruptTable::INTERRUPT_ON_TX | InterruptTable::INTERRUPT_ON_LDE_DONE,
                    [this](uint32_t value) { this->pollStateIRQHandler(value); });

                dw1000.transmit((uint8_t*)&init_msg, sizeof(twr_message_t));
            }
            
            break;
        }
        case RESPONSE:
        {
            /* Everything is already handled in the irqs*/
            break;
        }
            
        case FINAL:
        {
            
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                dw1000.logger->output("RX: %x %x", init_tx_ts.get_timestamp(), ack_rx_ts.get_timestamp());
                twr_message_t final_msg = {
                    .header = (twr_frame_header_t) {
                        .frameCtrl = {0x41, 0x88},
                        .seqNum = 0x00,
                        .panID = {0xCA, 0xDE},
                        .destAddr = { this->anchor_address & 0xff, this->anchor_address >> 8 },
                        .srcAddr = { this->deviceAddress & 0xff, this->deviceAddress >> 8 }
                    },
                    .payload = { .final = {.type = twr_msg_type_t::TWR_MSG_TYPE_FINAL,}}
                };
                systemState = STATE_MEASURING_WAITING;

                dw1000.addCustomInterruptHandler((InterruptTable) SYS_STATUS_TXFRS, [this](uint32_t value) { this->finalStateIRQHandler(value); });
                dw1000.transmit((uint8_t*)&final_msg, sizeof(twr_message_t));
            }
            break;
        }
     

        case REPORT:
        {
            if(systemState == STATE_MEASURING_ACTIVE)
            {
                twr_message_t* rprt_return;
                uint16_t length;
                dw1000.readReceivedData((uint8_t**) &rprt_return, &length);
                
                if (rprt_return->payload.report.type != twr_msg_type_t::TWR_MSG_TYPE_REPORT) {
                    //waitOutError();
                    //return dwm_com_error_t::ERROR;
                }

                esp_init_rx_ts.set_timestamp(rprt_return->payload.report.pollRx);
                esp_resp_tx_ts.set_timestamp(rprt_return->payload.report.responseTx);
                esp_fin_rx_ts.set_timestamp(rprt_return->payload.report.finalRx);

                //TODO, what the fuck do we want to do next, for now we wil just step back to the beginning!
                currentCommState = POLL;
                systemState = STATE_IDLE;
            }
            break;
        }
        default:
            break;
    }

}

void DW1000RangingTag::getDistanceToAnchor(uint16_t anchor_address)
{
    if(systemState == STATE_IDLE)
    {
        /* We can start a new measurement! */
    }
    if(systemState == STATE_MEASURING_ACTIVE)
    {
        /* We just have to reset our timestamps etc */
    }
    if(systemState == STATE_MEASURING_WAITING)
    {
        /* We are currently waiting for an irq routine (to start and) to end! */
        /* The IRQ routine is not running yet, otherwise we wouldnt be able to execute this code!*/
        dw1000.forceIdle();
        dw1000.removeCustomInterruptHandler();
        dw1000.setReceiverAutoReenable(false);
        /* The safest way out of this is waiting for a Timeout or the IRQ to end and then start the new measurement */
        /* Busy waiting is an option! BUT*/
    }

    resetTimestamps();
    this->anchor_address = anchor_address;
    systemState = STATE_MEASURING_ACTIVE;
    currentCommState = POLL;
}

void DW1000RangingTag::resetTimestamps()
{
    esp_init_rx_ts.set_timestamp((uint64_t) 0);
    esp_resp_tx_ts.set_timestamp((uint64_t) 0);
    esp_fin_rx_ts.set_timestamp((uint64_t) 0);

    init_tx_ts.set_timestamp((uint64_t) 0);
    ack_rx_ts.set_timestamp((uint64_t) 0);
    fin_tx_ts.set_timestamp((uint64_t) 0);
}