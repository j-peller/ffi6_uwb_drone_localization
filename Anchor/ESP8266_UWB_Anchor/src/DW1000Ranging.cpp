#include "DW1000Ranging.hpp"

uint16_t pan = 0xAFFE;

DW1000Ranging::DW1000Ranging(DW1000 &dw1000) : dw1000(dw1000)
{
    this->dw1000 = dw1000;
    delay(1000);
    //dw1000.soft_reset();
    delay(1000);
    dw1000.initialize();
    
    InterruptTable interrupts = (InterruptTable) 0;

    Mode test_mode {
        .channel = channel5,
        .prf = prf64,
        .bitrate = bitrate_850k,
        .preamble_code = 0x09,
        .preamble_length = TX_FCTRL_TXPSR_PE_1024,
        .sfd = SFD::STD,
        .tune = tuneTest,
    };

    Mode thotro110 {
        .channel = channel5,
        .prf = prf16,
        .bitrate = bitrate_110k,
        .preamble_code = 0x04,
        .preamble_length = TX_FCTRL_TXPSR_PE_2048,
        .sfd = SFD::STD,
        .tune = tuneTest,
    };
    
    dw1000.loadLDECode();
    dw1000.setPANAdress(pan);
    interrupts |= InterruptTable::INTERRUPT_ALL;
    interrupts |= InterruptTable::INTERRUPT_ALL_ALL;
    dw1000.enableInterrupts(interrupts);
    dw1000.setDiagnostic(true);
    dw1000.setMode(thotro110);
    if(dw1000.logger != nullptr)
    {
        dw1000.logger->setDeviceID(dw1000.getDeviceID());
        dw1000.logger->output("Device started");
    } 
}

void DW1000Ranging::loop()
{
    dw1000.log();
}
uint16_t DW1000Ranging::getDeviceAddress()
{
    return dw1000.getDeviceID();
}

void DW1000Ranging::twr_send(twr_message_t message)
{

}

void DW1000Ranging::updateTime()
{
    lastActive = micros64();
}
bool DW1000Ranging::isTimedOut()
{
    return (micros64()-lastActive >= DW1000_TIMEOUT / 1000); /* we only have microseconds accuracy */
}