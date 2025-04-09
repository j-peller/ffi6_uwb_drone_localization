#pragma once
#include <cstdint>
#include "Mode.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"
#include <logger.hpp>
/**
 * @brief SPI command structure for the DWM1000
 */
typedef struct {
    uint32_t reg:6;          //!< Indicates the register to be read or write into
    uint32_t subindex:1;     //!< Indicates offset address of the register
    uint32_t operation:1;    //!< Read or Write operation
    uint32_t extended:1;     //!< If subaddress is higher than 128
    uint32_t subaddress:15;  //!< Indicates subaddress of register
} dw1000_spi_cmd_t;

    /* SPI Transaction Header operation modes  */
const uint8_t    READ        = 0x00;
const uint8_t    READ_SUB    = 0x40;
const uint8_t    WRITE       = 0x80;
const uint8_t    WRITE_SUB   = 0x80;

/* If we just wanna read a normal register... */
static const uint8_t    NO_SUB_ADDRESS    = 0x00;

class DW1000 {
    public:
        void initialize();
        void getPrintableDeviceIdentifier(char msgBuffer[]);
        void handleInterrupt();
        void idle();
        void readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length);
        void writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t length);
        void readNetworkIdAndDeviceAddress(uint8_t* data);
        void writeNetworkIdAndDeviceAddress(uint8_t* data);

        void addLogger(Logger* logger);
    private:
        uint8_t irq = D2;
        uint8_t chip_select = D8;
        DeviceMode current_mode = IDLE;
        SPISettings spiSettings = SPISettings(20000000L, MSBFIRST, SPI_MODE0);
        void spi_transceive(uint8_t header[], uint8_t header_length, uint8_t data[], uint16_t data_length);
        Logger* logger = nullptr;
};