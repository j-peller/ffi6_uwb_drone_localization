#include "DW1000Ranging.hpp"

uint16_t pan = 0xAFFE;
uint16_t tag_id = 0xAAAA;
uint16_t anchor_id = 0xBBBB;

DW1000Ranging::DW1000Ranging(DeviceType deviceType, DW1000 &dw1000) : deviceType(deviceType), dw1000(dw1000)
{
    this->deviceType = deviceType;
    this->dw1000 = dw1000;
    delay(1000);
    //dw1000.soft_reset();
    delay(1000);
    dw1000.initialize();
    

    
    uint16_t device_id = 0;
    InterruptTable interrupts = (InterruptTable) 0;

    Mode test_mode {
        .channel = channel5,
        .prf = prf64,
        .bitrate = bitrate_850k,
        .preamble_code = 0x09,
        .preamble_length = TX_FCTRL_TXPSR_PE_1024,
        .sfd = SFD::STD,
    };

    Mode thotro110 {
        .channel = channel5,
        .prf = prf16,
        .bitrate = bitrate_110k,
        .preamble_code = 0x04,
        .preamble_length = TX_FCTRL_TXPSR_PE_2048,
        .sfd = SFD::STD,
    };
    
    dw1000.loadLDECode();
    dw1000.setPANAdress(pan);
    
    switch (this->deviceType)
    {
        case TAG:
            device_id = tag_id;
            interrupts |= InterruptTable::INTERRUPT_ALL;
            break;
        case ANCHOR:
            device_id = anchor_id;
            interrupts |= InterruptTable::INTERRUPT_ALL;
            dw1000.setReceiverAutoReenable(true);
            break;
        default:
            break;
    }
    dw1000.setDeviceID(device_id);
    dw1000.enableInterrupts(interrupts);
   
    dw1000.setMode(thotro110);
    if(dw1000.logger != nullptr)
    {
        dw1000.logger->setDeviceID(dw1000.getDeviceID());
        dw1000.logger->output("Device started");
    } 
}

void DW1000Ranging::loop()
{
    uint8_t tx_message[3] = {0xFE, 0xAF, 0x00};
    static uint8_t counter = 0;

    uint8_t* message = nullptr;
    uint16_t length = 0;
}

void DW1000Ranging::twr_send(twr_message_t message)
{

}