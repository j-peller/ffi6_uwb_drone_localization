#include "DW1000Ranging.hpp"

DW1000Ranging::DW1000Ranging(DeviceType deviceType, DW1000 &dw1000) : deviceType(deviceType), dw1000(dw1000)
{
    this->deviceType = deviceType;
    this->dw1000 = dw1000;
    delay(1000);
    dw1000.soft_reset();
    delay(1000);
    dw1000.initialize();
    

    uint16_t pan = 0xDECA;
    uint16_t device_id = 0;
    InterruptTable interrupts = (InterruptTable) 0;

    
    switch (this->deviceType)
    {
        case TAG:
            device_id = 0xAAAA;
            interrupts |= InterruptTable::INTERRUPT_ALL;
            break;
        case ANCHOR:
            device_id = 0xAFFE;
            interrupts |= (InterruptTable) SYS_MASK_MRXDFR;
            break;
        default:
            break;
    }


    dw1000.enableInterrupts(interrupts);
   
    dw1000.setMode(JOPEL110);

    dw1000.setPANAdress(pan);
    delay(100);
    dw1000.setDeviceAddress(device_id);
    delay(100);
    uint32_t read = 0;
    dw1000.readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("chan_trl %x", read);
}

void DW1000Ranging::loop()
{
    uint8_t tx_message[3] = {0xFE, 0xAF, 0x00};
    static uint8_t counter = 0;

    uint32_t read = 0;
    dw1000.readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("chan_trl %x", read);

    read = 0;
    dw1000.readBytes(DEV_ID_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("devid %x", read);

    read = 0;
    dw1000.readBytes(PANADR_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("panaddr %x", read);

    read = 0;
    dw1000.readBytes(SYS_MASK_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("sys_mask %x", read);
    
    read = 0;
    dw1000.readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &read);
    dw1000.logger->addBuffer("sys_status %x", read);

    uint8_t* message = nullptr;
    uint16_t length = 0;
    //dw1000.readReceivedData(&message, &length);

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
            dw1000.setReceiverAutoReenable(true);
            break;
    }
}