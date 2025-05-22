#pragma once

#ifndef DWM1000_CTRL_HPP
#define DWM1000_CTRL_HPP

#include "../../../shared/inc/constants.hpp"
#include "../../../shared/inc/helpers.hpp"
#include "../extern/uwb-dw1000/hw/drivers/uwb/uwb_dw1000/include/dw1000/dw1000_regs.h"
#include "dw1000_time.hpp"

#include <stdint.h>
#include <gpiod.h>

#define FAST_SPI    20000000    //< 20MHz
#define SLOW_SPI    2000000     //< 2MHz

#define AUTO_CLOCK  0x00    //< Switch over to PLL Clock when ready
#define XTI_CLOCK   0x01    //< 19.2 MHz Clock
#define PLL_CLOCK   0x02    //< 125 MHz Clock

#define SYS_STATUS_ALL_TX_GOOD      (SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | SYS_STATUS_TXPHS | SYS_STATUS_TXFRS)


/**
 * 
 */
typedef enum {
    DRONE,
    ANCHOR,
} dwm1000_role_t;


/**
 * 
 */
typedef enum {
    THOTRO,
    JOPEL,
} dw1000_mode_enum_t;


/**
 * @brief DWM1000 device instance structure
 */
typedef struct {
    dwm1000_role_t  role;
    uint16_t        short_addr;
    const char*     spi_dev;
    uint32_t        spi_baudrate;
    uint8_t         spi_bits_per_word;
    uint8_t         spi_mode;
    const char*     gpiod_chip;
    uint8_t         irq_gpio_pin;
    uint8_t         rst_gpio_pin;
    dw1000_mode_enum_t   mode;
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
typedef struct {
    uint8_t rf_rxctrlh; /* see Table 37 for 0x28:0B– RF_RXCTRLH*/
    uint32_t rf_txctrl; 
    uint8_t tc_pgdelay;
    uint32_t fs_pllcfg;
} dw1000_channel_t;


/**
 * 
 */
typedef struct {
    uint32_t txprf;
    uint32_t rxfprf;
} dw1000_prf_t;


/**
 * 
 */
typedef struct {
    uint16_t drx_tune0b;
    uint16_t drx_tune1a;
    uint16_t drx_tune1b;
    uint32_t drx_tune2;
    uint8_t fs_plltune;
} dw1000_tuning_t;


/**
 * 
 */
typedef struct {
    uint16_t agc_tune1;
    uint32_t agc_tune2;
} dw1000_fixed_config_t;


/**
 * 
 */
typedef struct {
    uint32_t txbr;      //< tx_fctrl register
    uint32_t rxm110k;   //< sys_cfg register
} dw1000_bitrate_t;


/**
 * 
 */
enum SFD {
    STD,
    DecaWave,
};


/**
 * 
 */
typedef struct {
    uint8_t channel_num;
    dw1000_channel_t        channel_config;
    dw1000_tuning_t         tune_config;
    dw1000_fixed_config_t   fixed_config;
    dw1000_prf_t            prf_config;
    dw1000_bitrate_t        bitrate_config;
    uint32_t                preamble_code;
    uint32_t                preamble_length;
    SFD                     sfd;
} dw1000_mode_t;


/* Deklaration der konstanten Konfiguration */
extern const dw1000_mode_t THOTRO110;
extern const dw1000_mode_t JOPEL110;


/**
 * @brief DWM1000 Controller class. Communication Interface to the DWM1000 using SPI
 */
class DWMController {

friend class DWMRanging;

public:
    static DWMController*   create_instance(dw1000_dev_instance_t* spi_dev);
    ~DWMController();

    /* DW1000 Configuration */
    dwm_com_error_t set_mode(dw1000_mode_t mode);
    void enable_frame_filtering(uint16_t enable);

    /* Transmission */
    dwm_com_error_t write_transmission_data(uint8_t* data, uint8_t len);
    void start_transmission();
    void start_transmission(bool wait4resp);
    void get_tx_timestamp(DW1000Time& time);

    /* Receiving */
    dwm_com_error_t start_receiving();
    dwm_com_error_t read_received_data(uint16_t* len_out, uint8_t** data_out);
    void set_receiver_auto_reenable(bool enable);
    void get_rx_timestamp(DW1000Time& time);

    /* Poll Status Bit */
    dwm_com_error_t poll_status_bit(uint32_t status_mask, uint64_t timeout);

    /**
     * @brief All TX events
     */
    inline dwm_com_error_t poll_tx_status() {
        //return poll_status_bit(SYS_STATUS_ALL_TX_GOOD, DW1000_TIMEOUT);
        return poll_status_bit(SYS_STATUS_TXFRS, DW1000_TIMEOUT);
    }

    /**
     * @brief All RX events after a correct packet reception
     */
    inline dwm_com_error_t poll_rx_status() {
        //return poll_status_bit(SYS_STATUS_ALL_RX_GOOD, RX_TIMEOUT);
        return poll_status_bit(SYS_STATUS_RXDFR, RX_TIMEOUT);
    }

    /* Reset */
    void reset();
    void soft_reset();

    /* Setters */
    void set_device_short_addr(uint16_t short_addr);
    void set_device_long_addr(uint64_t long_addr);
    void set_device_pan_id(uint16_t pan_id);
    void set_tx_antenna_delay(uint16_t delay);
    void set_rx_antenna_delay(uint16_t delay);

    /* Getters */
    void get_device_id(uint32_t* device_id);
    void get_device_short_addr(uint16_t* short_addr);
    void get_device_long_addr(uint64_t* long_addr);
    void get_device_pan_id(uint16_t* pan_id);
    dwm1000_role_t get_role();


    /* Testing Functions */
    dwm_com_error_t test_transmission_timestamp(DW1000Time& tx_time, uint8_t* payload, uint16_t payload_len);
    dwm_com_error_t test_receiving_timestamp(DW1000Time& rx_time);

    /* Calibration Fuckery */
    dwm_com_error_t send_antenna_calibration_value(uint16_t delay);
    dwm_com_error_t wait_for_antenna_calibration_value(uint16_t* delay);
    
    
    void setIRQMask(uint32_t irq_mask);
private:
    DWMController(int spi_fd, dw1000_dev_instance_t* device);

    /* Basic SPI Read and Write */
    void readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);
    void readBytes(uint8_t reg, uint16_t offset, uint32_t* data);
    void writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t len);
    void writeBytes(uint8_t reg, uint16_t offset, uint32_t data);

    /* OTP Read and Write */
    void readBytesOTP(uint16_t addr, uint8_t* data, uint32_t len);
    void _readBytesOTP(uint16_t addr, uint8_t* data);

    /* helpers */
    uint16_t getReceivedDataLength();
    void deleteReceivedDataLength();
    void clearStatusEvent(uint64_t event_mask);
    
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
    uint64_t                _last_sys_status;

    /* GPIO Control for Reset */
    struct gpiod_chip*      _gpio_chip;
    struct gpiod_line*      _rst_line;
    struct gpiod_line*      _irq_line;

protected:
    /* dw1000 device configuration */
    dw1000_dev_instance_t   _dev_instance;

    /* SPI Transaction Header operation modes  */
    static const uint8_t    READ        = 0x00;
    static const uint8_t    WRITE       = 0x01;

    /* If we just wanna read a normal register... */
    static const uint16_t    NO_SUB_ADDRESS    = 0x0000;
};


#endif