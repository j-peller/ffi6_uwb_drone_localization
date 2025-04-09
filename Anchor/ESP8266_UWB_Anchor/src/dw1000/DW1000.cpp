#include "DW1000.hpp"
#include <Arduino.h>

static void IRAM_ATTR dw1000_interrupt_handler(void* arg) {
    DW1000* instance = static_cast<DW1000*>(arg);
    instance->handleInterrupt();
}

void DW1000::initialize()
{
    pinMode(this->irq, INPUT);
    SPI.begin();
    //attachInterrupt(digitalPinToInterrupt(irq), DW1000::handleInterrupt, RISING);
    attachInterruptArg(digitalPinToInterrupt(irq), dw1000_interrupt_handler, this, RISING);


    pinMode(chip_select, OUTPUT);
    digitalWrite(chip_select, HIGH);
}

void DW1000::addLogger(Logger* logger)
{
    this->logger = logger;
}

void DW1000::readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length)
{
    /* Build SPI Transaction Header according to 2.2.1.2 p4 DW1000 User Manual */
    dw1000_spi_cmd_t cmd = {
        .reg = reg,
        .subindex = offset != 0,
        .operation = READ,
        .extended = offset > 0x7F,
        .subaddress = offset
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t)(offset),
        [2] = (uint8_t)(offset >> 7)
    };

    uint8_t header_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;
    spi_transceive(header, header_len, data, length);
    /*
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		data[i] = SPI.transfer(0x00); // read values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    */

}
void DW1000::spi_transceive(uint8_t header[], uint8_t header_length, uint8_t data[], uint16_t data_length)
{
    if(logger != nullptr)
    {
        logger->output("SPI TX Communcation: %X - %X --- %X - %X", data[0], data[1], data[2], data[3]);
    }
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_length; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < data_length; i++) {
		data[i] = SPI.transfer(data[i]); // read/write values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    if(logger != nullptr)
    {
        logger->output("SPI RX Communcation: %X - %X --- %X - %X", data[0], data[1], data[2], data[3]);
    }
}

void DW1000::writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length)
{
    /* Build SPI Transaction Header according to 2.2.1.2 p4 DW1000 User Manual */
    dw1000_spi_cmd_t cmd = {
        .reg = reg,
        .subindex = offset != 0,
        .operation = (offset != 0 ? WRITE_SUB >> 7 : WRITE >> 7),
        .extended = offset > 0x7F,
        .subaddress = offset
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t)(offset),
        [2] = (uint8_t)(offset >> 7)
    };

    /* do we have a 1,2 or 3 octet header? */
    uint8_t header_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;
    spi_transceive(header, header_len, data, length);

}

void DW1000::getPrintableDeviceIdentifier(char message[])
{
    uint8_t data[DEV_ID_LEN];
	readBytes(DEV_ID_ID, NO_SUB_ADDRESS, data, DEV_ID_LEN);
    uint32_t device_id = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    sprintf(message, "0x%08X - %02X - model: %d, version: %d, revision: %d", device_id,
        (uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}

void DW1000::readNetworkIdAndDeviceAddress(uint8_t* data)
{
	readBytes(PANADR_ID, NO_SUB_ADDRESS, data, PANADR_LEN);
}
void DW1000::writeNetworkIdAndDeviceAddress(uint8_t* data)
{
    writeBytes(PANADR_ID, NO_SUB_ADDRESS, data, PANADR_LEN);
}



void DW1000::handleInterrupt() {
    //TODO test
}
void DW1000::idle() {

}