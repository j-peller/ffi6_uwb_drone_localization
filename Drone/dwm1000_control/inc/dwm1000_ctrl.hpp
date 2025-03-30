#pragma once

#ifndef DWM1000_CTRL_HPP
#define DWM1000_CTRL_HPP

#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"

#include <stdint.h>

#define MAX_SPI_BAUDRATE 20000000 // 20MHz

typedef struct {
    const char* spi_dev;
    int         spi_baudrate;
    uint8_t     spi_bits_per_word;
    uint8_t     spi_mode;
    uint8_t     irq_pin;
    uint8_t     rst_pin;
} dw1000_dev_instance_t;


class DWMController {

public:
    static DWMController*   create_instance(dw1000_dev_instance_t* spi_dev);
    ~DWMController();

    void get_device_id(uint32_t* device_id);
    
private:
    DWMController(int spi_fd, dw1000_dev_instance_t* device);

    void readBytes(uint16_t reg, uint8_t* data, int len);
    void writeBytes(uint16_t reg, uint8_t* data, int len);

private:
    int                     _spi_fd;
    dw1000_dev_instance_t   _dev_instance;

};


#endif