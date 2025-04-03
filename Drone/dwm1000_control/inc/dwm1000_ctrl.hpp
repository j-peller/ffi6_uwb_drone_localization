#pragma once

#ifndef DWM1000_CTRL_HPP
#define DWM1000_CTRL_HPP

#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"

#include <stdint.h>

#define MAX_SPI_BAUDRATE 20000000 // 20MHz

/**
 * @brief DWM1000 device instance structure
 */
typedef struct {
    const char* spi_dev;
    int         spi_baudrate;
    uint8_t     spi_bits_per_word;
    uint8_t     spi_mode;
    uint8_t     irq_pin;
    uint8_t     rst_pin;
} dw1000_dev_instance_t;

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

/**
 * 
 */
typedef enum : uint8_t {
    IDLE_MODE   = 0x00,
    RX_MODE     = 0x01,
    TX_MODE     = 0x02,
} dw1000_dev_mode_t;


/**
 * @brief DWM1000 Controller class. Communication Interface to the DWM1000 using SPI
 */
class DWMController {

public:
    static DWMController*   create_instance(dw1000_dev_instance_t* spi_dev);
    ~DWMController();


    /* Transmission */
    void start_transmission();
    void stop_transmission();

    /* Receiving */
    void start_receiving();
    void stop_receiving();

    /* Write Payload */
    void write_transmission_data();

    /* Read Payload */
    void read_received_data();

    /* Setters */

    /* Getters */
    void get_device_id(uint32_t* device_id);
    
private:
    DWMController(int spi_fd, dw1000_dev_instance_t* device);

    /* Basic SPI Read and Write */
    void readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);
    void writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);

    /* DW1000 Mode Control */
    void forceIdle();

    /* Cleanup DW1000 Status Registers */
    void clearReceiveStatus();
    void clearTransmitStatus();


private:
    int                     _spi_fd;
    dw1000_dev_instance_t   _dev_instance;
    dw1000_dev_mode_t       _dev_mode;

protected:
    /* SPI Transaction Header operation modes  */
    static const uint8_t    READ        = 0x00;
    static const uint8_t    READ_SUB    = 0x40;
    static const uint8_t    WRITE       = 0x80;
    static const uint8_t    WRITE_SUB   = 0x80;

    /* If we just wanna read a normal register... */
    static const uint8_t    NO_SUB_ADDRESS    = 0x00;

};


#endif