#pragma once

#ifndef DWM1000_CTRL_HPP
#define DWM1000_CTRL_HPP

#include "../../../shared/inc/constants.hpp"
#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"
#include "dw1000_time.hpp"

#include <stdint.h>
#include <gpiod.h>

#define FAST_SPI    20000000    //< 20MHz
#define SLOW_SPI    2000000     //< 2MHz

#define AUTO_CLOCK  0x00    //< Switch over to PLL Clock when ready
#define XTI_CLOCK   0x01    //< 19.2 MHz Clock
#define PLL_CLOCK   0x02    //< 125 MHz Clock


/**
 * @brief DWM1000 device instance structure
 */
typedef struct {
    const char* spi_dev;
    uint32_t    spi_baudrate;
    uint8_t     spi_bits_per_word;
    uint8_t     spi_mode;
    const char* gpiod_chip;
    uint8_t     irq_gpio_pin;
    uint8_t     rst_gpio_pin;
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

    /* DW1000 Configuration */
    dwm_com_error_t do_init_config();

    /* Transmission */
    void write_transmission_data(uint8_t* data, uint8_t len);
    void start_transmission();
    void get_tx_timestamp(DW1000Time& time);

    /* Receiving */
    void start_receiving();
    uint8_t* read_received_data(uint8_t* len);
    void get_rx_timestamp(DW1000Time& time);

    /* Poll Status Bit */
    dwm_com_error_t poll_status_bit(uint64_t status_bit, uint64_t timeout);

    inline dwm_com_error_t poll_tx_status() {
        return poll_status_bit(SYS_STATUS_ALL_TX, DW1000_TIMEOUT);
    }

    inline dwm_com_error_t poll_rx_status() {
        return poll_status_bit(SYS_STATUS_ALL_RX_GOOD, RX_TIMEOUT);
    }

    /* Reset */
    void reset();
    void soft_reset();

    /* Setters */
    void set_device_short_addr(uint16_t short_addr);

    /* Getters */
    void get_device_id(uint32_t* device_id);
    void get_device_short_addr(uint16_t* short_addr);

    /* Testing Functions */
    dwm_com_error_t test_transmission_timestamp(DW1000Time& tx_time);
    dwm_com_error_t test_receiving_timestamp(DW1000Time& rx_time);
    
    
private:
    DWMController(int spi_fd, dw1000_dev_instance_t* device);

    /* Basic SPI Read and Write */
    void readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);
    void writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);

    /* OTP Read and Write */
    void readBytesOTP(uint16_t addr, uint8_t* data, uint32_t len);
    void _readBytesOTP(uint16_t addr, uint8_t* data);

    /* */
    uint8_t getReceivedDataLength();
    
    /* DW1000 Mode Control */
    void forceIdle();

    /* DW1000 Clock Control */
    void setSysClockSource(uint8_t source);

    /* config procedures */
    void loadLDECode();
    
    /* SPI helper */
    dwm_com_error_t spiSetBaud(uint32_t baudrate);

private:
    int                     _spi_fd;
    uint32_t                _cur_spi_baud;
    dw1000_dev_instance_t   _dev_instance;
    dw1000_dev_mode_t       _dev_mode;

    /* GPIO Control for Reset */
    struct gpiod_chip*      _gpio_chip;
    struct gpiod_line*      _rst_line;

protected:
    /* SPI Transaction Header operation modes  */
    static const uint8_t    READ        = 0x00;
    static const uint8_t    WRITE       = 0x01;

    /* If we just wanna read a normal register... */
    static const uint16_t    NO_SUB_ADDRESS    = 0x0000;
};


#endif