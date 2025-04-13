#include "DW1000.hpp"
#include <Arduino.h>
#include <cassert>

static void IRAM_ATTR dw1000_interrupt_handler(void* arg) {
    DW1000* instance = static_cast<DW1000*>(arg);
    instance->handleInterrupt();
}

void DW1000::initialize()
{
    pinMode(this->irq, INPUT);
    SPI.begin();
    //attachInterrupt(digitalPinToInterrupt(irq), DW1000::handleInterrupt, RISING);
    attachInterruptArg(digitalPinToInterrupt(irq), dw1000_interrupt_handler, this, RISING); // TODO was rising


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
    //spi_transceive(header, header_len, data, length);
    //noInterrupts();
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		data[i] = SPI.transfer(0x00); // read values, write junk
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    //interrupts();
    
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
void DW1000::readBytes(uint8_t reg, uint16_t offset, uint32_t* data)
{
    uint8_t cache[4] = {0};
    readBytes(reg, offset, cache, 4);
    *data = (cache[3]) << 24 |  (cache[2]) << 16 | (cache[1]) << 8 | (cache[0]);
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
void DW1000::soft_reset() 
 {
    /* Set SYSCLKS to 01 */
    uint8_t reg = 0;
    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, &reg, sizeof(uint8_t));

    /* clear bits 0:1 */
    reg &= ~(0x03);

    /* set SYSCLKS to 19.2MHz as per Documentation: p191 DW1000 User Manual */
    reg |= PMSC_CTRL0_SYSCLKS_19M;

    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, &reg, sizeof(uint8_t));

    /* Reset HIF, TX, RX and PMSC */
    reg = PMSC_CTRL0_RESET_ALL;
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));

    /* DW1000 needs a 10us sleep to let clk PLL lock after reset */
    delayMicroseconds(10);

    /* Finish reset */
    reg = PMSC_CTRL0_RESET_CLEAR; 
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));
 }
void DW1000::loadLDECode()
{
    /* see 2.5.5.10 LDELOAD */
    uint8_t otpctrl[OTP_CTRL_LEN] = {0};
    uint8_t pmsc_ctrl0[PMSC_CTRL0_LEN] = {0};
    uint8_t pmsc_ctrl0_cache[PMSC_CTRL0_LEN] = {0};

    //readBytes(OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    otpctrl[1] |= 1 << 7; /* LDELOAD Bit */
    pmsc_ctrl0[0] = 0x01;
    pmsc_ctrl0[1] = 0x03;
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    writeBytes(OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
    delayMicroseconds(150);

    pmsc_ctrl0[0] = 0x00;
	pmsc_ctrl0[1] = 0x02;
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, 2);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);

    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl0, PMSC_CTRL0_LEN);
    logger->output("pmsc %x %x %x %x", pmsc_ctrl0[3], pmsc_ctrl0[2], pmsc_ctrl0[1], pmsc_ctrl0[0]);
}
void DW1000::transmit(uint8_t data[], uint16_t length)
{
    assert(length < (1 << 10) && "length exceeds 10-bit maximum (1023)");
    // TODO enable frame check - add 2 bytes length etc...
    writeBytes(TX_BUFFER_ID, NO_SUB_ADDRESS, data, length);
    
    uint32_t frame_control = 0;
    readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, &frame_control);

    frame_control &= ~((1 << 9) - 1); /* Reset LEN Bits */

    frame_control |= length + 2; /* TFLEN: set first 10 bits to length*/

    frame_control &= ~(0b11 << 13); /* Reset transmit bitrate */
    frame_control |= TX_FCTRL_TXBR_110k; /* TXBR: transmit bitrate */

    frame_control &= ~(0b111 << 18); /* Reset TXPSR */
    frame_control |= TX_FCTRL_TXPSR_PE_4096; /* TXPSR: Length of transmitted preamble sequence*/

    logger->output("test %x", frame_control);
    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, frame_control);

    uint32_t system_ctrl = 0;
    readBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, &system_ctrl);
    system_ctrl |= SYS_CTRL_TXSTRT; /* Start Transmission Now Bit */
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, system_ctrl);
}

void DW1000::setFrameLength(FrameLengthType frame_length)
{
    uint8_t sys_cfg[SYS_CFG_LEN] = {0};
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, sys_cfg, SYS_CFG_LEN);
    sys_cfg[2] &= ~(0x3); // reset PHR value
    sys_cfg[2] |= frame_length; // update PHR value
    writeBytes(SYS_CFG_ID, 0, sys_cfg, SYS_CFG_LEN);
}

void DW1000::enableInterrupts(enum InterruptTable table)
{
    writeBytes(SYS_MASK_ID, NO_SUB_ADDRESS, table);
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
    //spi_transceive(header, header_len, data, length);
    //noInterrupts();
    SPI.beginTransaction(spiSettings);
    digitalWrite(chip_select, LOW);
    for(uint16_t i = 0; i < header_len; i++) {
	    SPI.transfer(header[i]); // send header
	}
    for(uint16_t i = 0; i < length; i++) {
		SPI.transfer(data[i]); // write values
	}

    delayMicroseconds(5);
	digitalWrite(chip_select, HIGH);
	SPI.endTransaction();
    //interrupts();

}
/**
 * @brief Write data to a specific register using an offset and data of type uint32_t
 * Converts uint32_t data to the correct uint8_t[] format (little endian)
 */
void DW1000::writeBytes(uint8_t reg, uint16_t offset, uint32_t data)
{
    //data = __builtin_bswap32(data); // swap to little endian decoding before writing as uint8_t array
    writeBytes(reg, offset, (uint8_t*)&data, sizeof(uint32_t));
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



void DW1000::handleInterrupt()
{
    uint32_t sys_status = 0;
    readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
    if(logger!=nullptr) logger->output("Gotcha! %x", sys_status);
    clearStatusRegister();
}

void DW1000::idle() {

}

void DW1000::setDataRate(uint8_t rate)
{
    //TODO
}
/* clear all SYS_Status Register by writing 1 in every bit*/
void DW1000::clearStatusRegister()
{
    uint8_t data[SYS_STATUS_LEN] = {0};
    memset(data, 0xFF, SYS_STATUS_LEN);
    writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, data, SYS_STATUS_LEN);
}