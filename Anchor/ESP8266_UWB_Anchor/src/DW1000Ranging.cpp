#include "DW1000Ranging.hpp"

DW1000Ranging::DW1000Ranging(DeviceType deviceType, DW1000 &dw1000) : deviceType(deviceType), dw1000(dw1000)
{
    this->deviceType = deviceType;
    this->dw1000 = dw1000;
    delay(1000);
    //dw1000.soft_reset();
    delay(1000);
    dw1000.initialize();
    

    uint16_t pan = 0xAFFE;
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
            device_id = 0xAAAA;
            interrupts |= InterruptTable::INTERRUPT_ALL;
            break;
        case ANCHOR:
            device_id = 0xBBBB;
            interrupts |= (InterruptTable) 0b11111111111111111111111111111111;
            dw1000.setReceiverAutoReenable(true);
            break;
        default:
            break;
    }
    dw1000.setDeviceID(device_id);
    dw1000.enableInterrupts(interrupts);
   
    dw1000.setMode(thotro110);

    uint32_t read = 0;
    dw1000.readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("chan_trl %x", read);
}

void DW1000Ranging::loop()
{
    uint8_t tx_message[3] = {0xFE, 0xAF, 0x00};
    uint8_t counter = 0;

    uint32_t read = 0;
    dw1000.readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("chan_trl %x", read);

    read = 0;
    dw1000.readBytes(DEV_ID_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("devid %x", read);

    read = 0;
    dw1000.readBytes(PANADR_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("panaddr %x", read);

    uint8_t* message = nullptr;
    uint16_t length = 0;
    dw1000.readReceivedData(&message, &length);

    if(message!=nullptr)
    {
        for (int i = 0; i < length; i++) {
            dw1000.logger->output("%02x ", message[i]);
        }
        delete[] message;
    } else {
        dw1000.logger->output("null");
    }

    switch(this->deviceType)
    {
        case TAG:
            tx_message[2] = counter++;
            dw1000.transmit(tx_message, 3);
            dw1000.logger->addBuffer("Transmit!");
            break;
        case ANCHOR:
            dw1000.startReceiving();
            break;
    }
}