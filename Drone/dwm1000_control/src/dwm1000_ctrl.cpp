#include "../inc/dwm1000_ctrl.hpp"
#include "../../../shared/inc/anchor_addresses.hpp"
#include "../../../shared/inc/twr_dw1000_frame_spec.hpp"

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpiod.h>


DWMController* DWMController::create_instance(dw1000_dev_instance_t* device)
{
    if (device == NULL) {
        return NULL;
    }

    // Open SPI device
    int fd = open(device->spi_dev, O_RDWR);
    if (fd < 0) {
        perror("Failed to open SPI device");
        return NULL;
    }

    // Set SPI mode
    uint8_t mode = device->spi_mode;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(fd);
        return NULL;
    }

    // Set SPI speed
    if (device->spi_baudrate <= 0 && device->spi_baudrate > FAST_SPI) {
        perror("Invalid SPI baudrate");
        close(fd);
        return NULL;
    }
    uint32_t speed = device->spi_baudrate;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        close(fd);
        return NULL;
    }

    // Set SPI bits per word
    uint8_t bits = device->spi_bits_per_word;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("Failed to set SPI bits per word");
        close(fd);
        return NULL;
    }

    /* Init DWMController */
    DWMController* instance = new DWMController(fd, device);
    if (instance == NULL) {
        perror("Failed to create DWMController instance");
        close(fd);
        return NULL;
    }

    /* Setup GPIO */
    instance->_gpio_chip = gpiod_chip_open(device->gpiod_chip);
    if (!instance->_gpio_chip) {
        perror("Failed to open GPIO Chip");
        delete instance;
        close(fd);
        return NULL;
    }

    instance->_rst_line = gpiod_chip_get_line(instance->_gpio_chip, device->rst_gpio_pin);
    if (!instance->_rst_line) {
        perror("Failed to open GPIO Pin");
        gpiod_chip_close(instance->_gpio_chip);
        delete instance;
        close(fd);
        return NULL;
    }

    instance->_irq_line = gpiod_chip_get_line(instance->_gpio_chip, device->irq_gpio_pin);
    if (!instance->_irq_line) {
        perror("Failed to open GPIO Pin");
        gpiod_chip_close(instance->_gpio_chip);
        delete instance;
        close(fd);
        return NULL;
    }

    if (gpiod_line_request_output(instance->_rst_line, "DWM1000Reset", 0) < 0) {
        perror("Failed to set RST GPIO Pin to OUTPUT");
        gpiod_chip_close(instance->_gpio_chip);
        delete instance;
        close(fd);
        return NULL;
    }

    /**
     * DWM1000 IRQ Pin stays asserted!!
     */

    struct gpiod_line_request_config config = {
        .consumer = "DWM1000Interrupt",
        .request_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE,
        .flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN
    };


    if (gpiod_line_request(instance->_irq_line, &config, 0) < 0) {
        perror("Failed to set IRQ GPIO Pin to INPUT");
        gpiod_chip_close(instance->_gpio_chip);
        delete instance;
        close(fd);
        return NULL;
    }

    /* Test SPI Connection to DWM1000 */
    uint32_t device_id = 0;
    instance->get_device_id(&device_id);
    if (device_id != 0xDECA0130) {
        fprintf(stderr, "DWM1000 returned invalid device ID: 0x%08X\n", device_id);
        perror("DWM1000 not detected or SPI not working");
        delete instance;
        close(fd);
        return NULL;
    }

    fprintf(stdout, "DWM1000 detected with ID: 0x%08X\n", device_id);

    /* Test SPI Write - and Read Back Value */
    instance->set_device_short_addr(MASTER);
    uint16_t short_addr_read_back = 0;
    instance->get_device_short_addr(&short_addr_read_back);
    if (short_addr_read_back != MASTER) {
        fprintf(stderr, "DWM1000 returned invalid Short Address: 0x%04X\n", short_addr_read_back);
        perror("DWM1000 not detected or SPI not working");
        delete instance;
        close(fd);
        return NULL;
    }

    instance->set_device_pan_id(DEFAULT_PAN);
    uint16_t pan_id_read_back = 0;
    instance->get_device_pan_id(&pan_id_read_back);
    if (pan_id_read_back != DEFAULT_PAN) {
        fprintf(stderr, "DWM1000 returned invalid PAN ID: 0x%04X\n", pan_id_read_back);
        perror("DWM1000 not detected or SPI not working");
        delete instance;
        close(fd);
        return NULL;
    }

    fprintf(stdout, "DWM1000 Setup successful!\n\t - Short Address:\t0x%04X\n\t - PAN ID:\t\t0x%04X\n", short_addr_read_back, pan_id_read_back);

    return instance;
}


DWMController::DWMController(int spi_fd, dw1000_dev_instance_t* device)
    : _spi_fd(spi_fd), _cur_spi_baud(device->spi_baudrate), _last_sys_status(0),  _dev_instance(*device),  _gpio_chip(NULL), _rst_line(NULL)
{
    spiSetBaud(device->spi_baudrate);
}   


DWMController::~DWMController()
{
    /* Close GPIO Chip */
    if (_gpio_chip) {
        gpiod_line_release(_irq_line);
        gpiod_line_release(_rst_line);
        gpiod_chip_close(_gpio_chip);
    }

    if (_spi_fd >= 0) {
        close(_spi_fd);
    }

}


/**
 * 
 */
dwm_com_error_t DWMController::set_mode(dw1000_mode_t mode)
{
    uint32_t sys_cfg = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_cfg, SYS_CFG_LEN);

    //sys_cfg |= SYS_CFG_FFE;         //< Enable Frame Filtering. This requires SHORT_ADDR to be set beforehand.
    //sys_cfg |= SYS_CFG_FFAD;        //< Allow Data Frame
    sys_cfg &= ~SYS_CFG_FFE;
    sys_cfg &= ~SYS_CFG_FFAD;
    sys_cfg |= SYS_CFG_PHR_MODE_00; //< Standard Frame mode IEEE 802.15.4 compliant
    sys_cfg |= mode.bitrate_config.rxm110k;

    writeBytes(SYS_CFG_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_cfg, SYS_CFG_LEN);

     /**
     * Required Configuration for Transmitter on Channel 5
     */

    uint32_t rf_txctrl_val = mode.channel_config.rf_txctrl;     //< See DW1000 User Manual Page 148 Table 38
    writeBytes(RF_CONF_ID, RF_TXCTRL_OFFSET, rf_txctrl_val);

    uint8_t tc_pgdelay_val = mode.channel_config.tc_pgdelay;    //< See DW1000 User Manual Page 155 Table 40 
    writeBytes(TX_CAL_ID, TC_PGDELAY_OFFSET, &tc_pgdelay_val, sizeof(uint8_t));

    uint32_t fs_pllcfg_val = mode.channel_config.fs_pllcfg;     //< See DW1000 User Manual Page 157 Table 43
    writeBytes(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pllcfg_val);

    uint8_t tx_fctrl[TX_FCTRL_LEN] = {0};
    readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, tx_fctrl, TX_FCTRL_LEN);

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXBR_MASK; //< Clear current setting
    *(uint64_t *) tx_fctrl |= mode.bitrate_config.txbr;

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPRF_MASK;
    *(uint64_t *) tx_fctrl |= mode.prf_config.txprf;

    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPSR_MASK;
    *(uint64_t *) tx_fctrl &= ~TX_FCTRL_TXPSR_PE_MASK;
    *(uint64_t *) tx_fctrl |= mode.preamble_length;

    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, tx_fctrl, TX_FCTRL_LEN);

     /**
     * Required Configuration for Receiver on Channel 5
     */
    uint8_t rf_rxctrlh_val = mode.channel_config.rf_rxctrlh; //< See DW1000 User Manual Page 148 Table 37
    writeBytes(RF_CONF_ID, RF_RXCTRLH_OFFSET, &rf_rxctrlh_val, sizeof(uint8_t));


    /* Channel Control Settings for both Receiver and Transmitter */
    uint8_t chan_ctrl[CHAN_CTRL_LEN] = {0};
    readBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, chan_ctrl, CHAN_CTRL_LEN);

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_TX_CHAN_MASK;           //< Clear current TX Channel
    *(uint32_t *) chan_ctrl |= (mode.channel_num << CHAN_CTRL_TX_CHAN_SHIFT);  //< Set TX Channel to 5

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_RX_CHAN_MASK;           //< Clear current RX Channel
    *(uint32_t *) chan_ctrl |= (mode.channel_num << CHAN_CTRL_RX_CHAN_SHIFT);  //< Set RX Channel to 5

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_RXFPRF_MASK;            //< Clear current RX PRF
    *(uint32_t *) chan_ctrl |= mode.prf_config.rxfprf;               //< Set RX PRF to 64 MHz to match Transmitter

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_TX_PCOD_MASK;           //< Clear current Preamble Code for Transceiver
    *(uint32_t *) chan_ctrl |= (mode.preamble_code) << CHAN_CTRL_TX_PCOD_SHIFT;  //< Set Preamble Code 9. Supported according to page 214 table 61

    *(uint32_t *) chan_ctrl &= ~CHAN_CTRL_RX_PCOD_MASK;           //< Clear current Preamble Code for Receiver
    *(uint32_t *) chan_ctrl |= (mode.preamble_code) << CHAN_CTRL_RX_PCOD_SHIFT;  //< Set Preamble Code 9. Supported according to page 214 table 61

    switch(mode.sfd){
        case SFD::STD:
        {

            break;
        }
        case SFD::DecaWave:
        {
            // 110 k mode
            *(uint32_t *) chan_ctrl |= CHAN_CTRL_DWSFD;
            *(uint32_t *) chan_ctrl &= ~(CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD);
            // When using 110k mode, the SFD length is always 64Bytes 


            // 850 mode
            //*(uint32_t *) chan_ctrl |= (CHAN_CTRL_DWSFD | CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD);
            //uint8_t sfd_len = DW_NS_SFD_LEN_850K; //< 10 Bytes
            //writeBytes(USR_SFD_ID, NO_SUB_ADDRESS, &sfd_len, sizeof(uint8_t));
            break;
        }
    }
    
    writeBytes(CHAN_CTRL_ID, NO_SUB_ADDRESS, chan_ctrl, CHAN_CTRL_LEN);

    /**
     * Default configurations that should be modified according to Section 2.5.5 page 17 
     */
    writeBytes(DRX_CONF_ID, DRX_TUNE0b_OFFSET, (uint8_t*)&mode.tune_config.drx_tune0b, sizeof(uint16_t));
    writeBytes(DRX_CONF_ID, DRX_TUNE1a_OFFSET, (uint8_t*)&mode.tune_config.drx_tune1a, sizeof(uint16_t));
    writeBytes(DRX_CONF_ID, DRX_TUNE1b_OFFSET, (uint8_t*)&mode.tune_config.drx_tune1b, sizeof(uint16_t));
    writeBytes(DRX_CONF_ID, DRX_TUNE2_OFFSET, (uint8_t*)&mode.tune_config.drx_tune2, sizeof(uint32_t));
    writeBytes(AGC_CTRL_ID, AGC_TUNE1_OFFSET, (uint8_t*)&mode.fixed_config.agc_tune1, sizeof(uint16_t));
    writeBytes(AGC_CTRL_ID, AGC_TUNE2_OFFSET, (uint8_t*)&mode.fixed_config.agc_tune2, sizeof(uint32_t));

    uint8_t lde_cfg1 = 0;
    readBytes(LDE_IF_ID, LDE_CFG1_OFFSET, (uint8_t*)&lde_cfg1, sizeof(uint8_t));
    lde_cfg1 &= ~LDE_CFG1_NSTDEV_MASK;  //< Clear current setting which is set to 0x0C
    lde_cfg1 |= 0x0D;                   //< Set 0x0D as described in Section 2.5.5.4 for better performance
    writeBytes(LDE_IF_ID, LDE_CFG1_OFFSET, (uint8_t*)&lde_cfg1, sizeof(uint8_t));

    uint16_t lde_cfg2 = 0x1607;     //< See DW1000 User Manual Page 177 Table 50
    writeBytes(LDE_IF_ID, LDE_CFG2_OFFSET, (uint8_t*)&lde_cfg2, sizeof(uint16_t));

    uint16_t lde_repc = LDE_REPC_PCODE_4 >> 3; //< See Page 178 
    writeBytes(LDE_IF_ID, LDE_REPC_OFFSET, (uint8_t*)&lde_repc, sizeof(uint16_t));

    uint32_t tx_power_val = 0x0E082848; //< See DW1000 User Manual Section 2.5.5.6 
    writeBytes(TX_POWER_ID, NO_SUB_ADDRESS, (uint8_t*)&tx_power_val, sizeof(uint32_t));

    writeBytes(FS_CTRL_ID, FS_PLLTUNE_OFFSET, (uint8_t*)&mode.tune_config.fs_plltune, sizeof(uint8_t));

    /* Procedure to load LDE Coad into ROM */
    loadLDECode();

    /* Load LDOTUNE_CAL value from OTP into LDOTUNE Register as described in Section 2.5.5.11 page 18*/
    uint64_t ldotune_cal_val = 0;
    readBytesOTP(0x0004, (uint8_t*)&ldotune_cal_val, sizeof(uint64_t));
    writeBytes(RF_CONF_ID, 0x30, (uint8_t*)&ldotune_cal_val, 5);

    /* Ramp up SPI */
    spiSetBaud(FAST_SPI);

    return SUCCESS;
}

/**
 * @brief Write frame data to the TX buffer of the DW1000 and set the TX frame control register accordingly
 * 
 * @param data Pointer to the data to be transmitted
 * @param len  Length of the data to be transmitted
 * 
 *        Write our frame to the TX buffer of the DW1000. UWB Frames can be up to 127 bytes long.
 *        The value specified here determines the length of the data portion of the transmitted frame.
 *        This length includes the two-octet CRC appended automatically at the end of the frame, 
 *        unless SFCST (in Register file: 0x0D – System Control Register) is use to suppress the FCS.
 */
dwm_com_error_t DWMController::write_transmission_data(uint8_t* data_in, uint8_t len)
{
    if (data_in == NULL || len == 0) {
        fprintf(stderr, "Invalid data or length for transmission\n");
        return ERROR;
    }

    /* TODO: Check if CRC is used, right now we always use crc*/
    len = (len + 2) & TX_FCTRL_TFLEN_MASK;

    /* Write the data to be transmitted */
    writeBytes(TX_BUFFER_ID, NO_SUB_ADDRESS, data_in, len);

    /* Set Transmit Frame Length accordingly */
    uint32_t tx_fctrl = 0;
    readBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, &tx_fctrl);
    tx_fctrl &= ~(TX_FCTRL_TFLEN_MASK | TX_FCTRL_TFLE_MASK); //< Clear current setting
    tx_fctrl |= len ; //< 7 bit TFLEN
    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, tx_fctrl);

    return SUCCESS;
}


/**
 * @brief Start receiving data from the DWM1000
 * 
 *        This method sets the TXSTRT bit in the SYS_CTRL register,
 *        which commands the DW1000 to begin transmission.
 */
void DWMController::start_transmission() {

    /* Idle mode required to start new transmission */
    this->forceIdle();

    /* clear tx status registers */
    clearStatusEvent(SYS_STATUS_ALL_TX);

    /* WAIT4RESP maybe an option here to enable the receiver immediatly after transmission completed */
    /* Start Transmitter */
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, SYS_CTRL_TXSTRT | SYS_CTRL_WAIT4RESP);
}


/**
 * @brief Get the 40-bit full adjusted TX timestamp from the DW1000
 * @param time Reference to the DW1000Time object to store the timestamp
 */
void DWMController::get_tx_timestamp(DW1000Time& time) 
{
    uint8_t data[TX_STAMP_LEN] = {0};
    readBytes(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, data, TX_STAMP_LEN);

    /* Get the TX timestamp */
    time.set_timestamp(data);
}


/**
 * @brief Start receiving data from the DWM1000
 * 
 *        This method sets the RXENAB bit in the SYS_CTRL register,
 *        which commands the DW1000 to begin receiving.
 */
dwm_com_error_t DWMController::start_receiving()
{
    forceIdle();

    /* clear rx status registers */
    clearStatusEvent(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR);

    /* Start Receiver */
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, SYS_CTRL_RXENAB);

    return SUCCESS;
}


/**
 * @brief Read len bytes of received data from the RX buffer of the DW1000
 * 
 * @param len_out Pointer to the variable to store the length of the received data
 * @return Pointer to buffer containing received data
 *
 */
dwm_com_error_t DWMController::read_received_data(uint16_t* len_out, uint8_t** data_out)
{
    /* FCS Error */
    //if (_last_sys_status & SYS_STATUS_RXFCE) {
    //    fprintf(stdout, "Error in FCS\n");
    //    return ERROR;
    //}

    /* get length of received data from Frame Info register */
    uint16_t len = getReceivedDataLength();
    if (len <= 0) {
        fprintf(stderr, "Invalid length: %d\n", len);
        return ERROR;
    }
    fprintf(stdout, "Received data with length: %d\n", len);

    uint8_t* rx_data = new uint8_t[len];
    if (rx_data == NULL) {
        fprintf(stderr, "Failed to allocate memory for received data\n");
        return ERROR;
    }

    /* Read received data from RX_BUFFER of DW1000 */
    readBytes(RX_BUFFER_ID, NO_SUB_ADDRESS, rx_data, len);

    /* update the length of the received buffer */
    *len_out = len;
    *data_out = rx_data;

    /* delte length information in RX_FINFO register */
    deleteReceivedDataLength();

    /* return received data */
    return SUCCESS;
}


/**
 * 
 */
void DWMController::set_receiver_auto_reenable(bool enable)
{
    uint32_t sys_cfg = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, &sys_cfg);

    if (enable) {
        sys_cfg |= SYS_CFG_RXAUTR;
    } else {
        sys_cfg &= ~SYS_CFG_RXAUTR;
    }

    writeBytes(SYS_CFG_ID, NO_SUB_ADDRESS, sys_cfg);
}



/**
 * @brief Get the 40-bit full adjusted RX timestamp from the DW1000
 * @param time Reference to the DW1000Time object to store the timestamp
 */
void DWMController::get_rx_timestamp(DW1000Time& time) 
{
    uint8_t data[RX_STAMP_LEN] = {0};
    readBytes(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, data, RX_STAMP_LEN);

    /* Get the RX timestamp */
    time.set_timestamp(data);
}


/**
 * @brief Poll System Event Status Register for a specific status bit
 * @param status_bit The status bit to poll
 * @param timeout The timeout in nanoseconds
 * 
 * TODO: may not continously poll for status bits via SPI. Consider using interrupts via GPIO pins
 */
dwm_com_error_t DWMController::poll_status_bit(uint32_t status_mask, uint64_t timeout)
{
    uint32_t sys_status = 0;
    struct gpiod_line_event event;
    dwm_com_error_t ret = SUCCESS;
    timespec timeout_ts = {.tv_sec = timeout / 1000000000, .tv_nsec = timeout % 1000000000};
    int gpio_ret;

    /**
     * Mask only desired interrupts.
     * Sys Status Register is cleared by receiver / transmitter enable
     */
    //setIRQMask(status_mask);

    /* wait for event on irq pin */
    gpio_ret = gpiod_line_event_wait(_irq_line, &timeout_ts);
    if (gpio_ret > 0) {

        /* Read the event type */
        if (gpiod_line_event_read(_irq_line, &event) < 0) {
            fprintf(stderr, "GPIO Read failed\n");
            return ERROR;
        }

        /* is it our desired rising edge event? */
        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {

            /* we are only interested in the first 32bits... */
            readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
        
            /* check status */
            if ((sys_status & status_mask) == status_mask) {
                _last_sys_status = sys_status;
                fprintf(stdout, "Gotcha! %04X\n", status_mask);
                clearStatusEvent(status_mask);
            }
                
            /* Drain remaining events without status checks */
            //if (gpiod_line_event_wait(_irq_line, &remaining_ts) > 0) {
                //fprintf(stdout, "Draining remaining events\n");
                //while(gpiod_line_event_read(_irq_line, &event) > 0) {
                //}
            //}
                    
            return SUCCESS;
        }


    }
    else if (gpio_ret == 0) {
        //readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, &sys_status);
        fprintf(stderr, "No Events Available, Timeout\n", sys_status);
        return TIMEOUT;
        /* if we dont do this, we get stuck after some time because interrupts dont get cleared */
        //clearStatusEvent(status_mask);
    } 
    else {
        fprintf(stderr, "Some error? Ready: %d\n", gpio_ret);
        return ERROR;
    }

    return ret;
}


/**
 * @brief External hard reset of the DWM1000 device
 * 
 */
void DWMController::reset()
{
    /*  */
    gpiod_line_request_output(_rst_line, "DWM1000Reset", 0);

    /* */
    gpiod_line_set_value(_rst_line, 0);

    /* Reset Pin should be manually driven low for at least 50ns to ensure correct reset operation */
    busywait_nanoseconds(1000);

    /* */
    gpiod_line_request_input(_rst_line, "DWM1000Reset");

    /* busywait for 10ms */
    busywait_nanoseconds(10000000);

    /* Force the DWM1000 into idle mode */
    forceIdle();
}


/**
 * @brief API to do softreset on dw1000 by writing data into PMSC_CTRL0_SOFTRESET_OFFSET.
 */
 void DWMController::soft_reset() 
 {
    /* set SYSCLKS to 19.2MHz as per Documentation: p191 DW1000 User Manual */
    setSysClockSource(XTI_CLOCK);

    /* Reset HIF, TX, RX and PMSC */
    uint8_t reg = PMSC_CTRL0_RESET_ALL;
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));

    /* DW1000 needs a 10us sleep to let clk PLL lock after reset */
    busywait_nanoseconds(10000);

    /* Finish reset */
    reg = PMSC_CTRL0_RESET_CLEAR; 
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));

    /* set into idle */
    forceIdle();
 }


/**
 * 
 */
void DWMController::set_device_short_addr(uint16_t short_addr)
{
    writeBytes(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, (uint8_t*)&short_addr, sizeof(uint16_t));
}


/**
 * 
 */
void DWMController::set_device_pan_id(uint16_t pan_id)
{
    writeBytes(PANADR_ID, PANADR_PAN_ID_OFFSET, (uint8_t*)&pan_id, sizeof(uint16_t));
}


/**
 * 
 */
void DWMController::set_antenna_delay(uint16_t delay)
{
    writeBytes(TX_ANTD_ID, NO_SUB_ADDRESS, (uint8_t*)&delay, sizeof(uint16_t));
}


/**
 * @brief Get the device ID of the DWM1000 -- Read from the DEV_ID register = 0x00
 * @param device_id Pointer to store the device ID
 * 
 */
void DWMController::get_device_id(uint32_t* device_id)
{
    readBytes(DEV_ID_ID, NO_SUB_ADDRESS, device_id);
}


/**
 * 
 */
void DWMController::get_device_short_addr(uint16_t* short_addr)
{
    readBytes(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, (uint8_t*)short_addr, sizeof(uint16_t));
}


/**
 * 
 */
void DWMController::get_device_pan_id(uint16_t* pan_id)
{
    readBytes(PANADR_ID, PANADR_PAN_ID_OFFSET, (uint8_t*)pan_id, sizeof(uint16_t));
}


/**
 * 
 */
dwm_com_error_t DWMController::test_transmission_timestamp(DW1000Time& tx_time, uint8_t* payload)
{
    dwm_com_error_t ret = SUCCESS;

    /* write our payload to transmit buffer */
    ret = write_transmission_data(payload, sizeof(twr_message_t));
    if (ret != SUCCESS) {
        return ret;
    }

    /* start transmission of our data */
    start_transmission();
    
    /* wait until our data has been successfully transmitted */
    ret = poll_tx_status();
    if (ret != SUCCESS) {
        return ret;
    }
    
    /* get transmission timestamp */
    get_tx_timestamp(tx_time);

    return SUCCESS;
}


/**
 * @brief Simple test procedure for receiving a packet 
 */
dwm_com_error_t DWMController::test_receiving_timestamp(DW1000Time& rx_time)
{
    twr_message_t* msg;
    uint16_t ack_len;
    dwm_com_error_t ret = SUCCESS;

    /* Start receiver */
    ret = start_receiving();
    if (ret != SUCCESS) {
        return ret;
    }

    /* Poll for successfull reception of a packet */
    ret = poll_rx_status();
    if (ret != SUCCESS) {
        return ret;
    }

    /* we should now have data in our buffer */
    ret = read_received_data(&ack_len, (uint8_t**)&msg);
    if (ret != SUCCESS) {
        return ret;
    }

    /* Printout received data packet */
    fprintf(stdout,
        "TWR Message Frame:\n"
        "  Frame Control     : 0x%02X 0x%02X\n"
        "  Sequence Number   : 0x%02X\n"
        "  PAN ID            : 0x%02X 0x%02X\n"
        "  Destination Addr  : 0x%02X 0x%02X\n"
        "  Source Addr       : 0x%02X 0x%02X\n"
        "  Message Type      : 0x%02X\n"
        "  Final Timestamp   : 0x%02X%02X%02X%02X%02X\n",
        msg->header.frameCtrl[0], msg->header.frameCtrl[1],
        msg->header.seqNum,
        msg->header.panID[0], msg->header.panID[1],
        msg->header.destAddr[0], msg->header.destAddr[1],
        msg->header.srcAddr[0], msg->header.srcAddr[1],
        msg->payload.report.type,
        msg->payload.report.finalRx[0], msg->payload.report.finalRx[1], msg->payload.report.finalRx[2], msg->payload.report.finalRx[3], msg->payload.report.finalRx[4]
    );

    /* */
    get_rx_timestamp(rx_time);

    /* cleanup */
    delete msg;

    return ret;
}


/**
 * @brief Read data from the DWM1000 device
 * 
 * @param reg The register address to read from
 * @param offset The subaddress / offset within the register
 * @param data Pointer to the buffer to store the read data with data[0] containing the LSB
 * @param len The length of the data to read
 */
void DWMController::readBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t n)
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

    uint8_t cmd_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;

    /* Allocate TX and RX buffers */
    uint8_t tx_buf[cmd_len + n];
    uint8_t rx_buf[cmd_len + n];
    memset(tx_buf, 0, sizeof(cmd_len + n));
    memset(rx_buf, 0, sizeof(cmd_len + n));

    /* Write header to tx_buffer */
    memcpy(tx_buf, header, cmd_len);

    /* prepare SPI transfer */
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = cmd_len + n,
        .cs_change = 0,
    };

    /* syscall to SPI Kernel driver to read data from requested register */
    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to read from SPI device");
        return;
    }

    /* copy received data */
    memcpy(data, rx_buf + cmd_len, n);
}


/**
 * 
 */
void DWMController::readBytes(uint8_t reg, uint16_t offset, uint32_t* data)
{
    readBytes(reg, offset, (uint8_t*)data, 4);
}


/**
 * @brief Write data to the DWM1000 device
 * 
 * @param reg The register address to write to
 * @param offset The subaddress / offset within the register
 * @param data Pointer to the data to write with data[0] containing the LSB
 * @param len The length of the data to write
 * 
 */
void DWMController::writeBytes(uint8_t reg, uint16_t offset, uint8_t* data, uint32_t n)
{
    /* Build SPI Transaction Header according to 2.2.1.2 p4 DW1000 User Manual */
    dw1000_spi_cmd_t cmd = {
        .reg = reg,
        .subindex = offset != 0,
        .operation = WRITE,
        .extended = offset > 0x7F,
        .subaddress = offset
    };

    uint8_t header[] = {
        [0] = cmd.operation << 7 | cmd.subindex << 6 | cmd.reg,
        [1] = cmd.extended << 7 | (uint8_t)(offset),
        [2] = (uint8_t)(offset >> 7)
    };

    /* do we have a 1,2 or 3 octet header? */
    uint8_t cmd_len = cmd.subindex ? (cmd.extended ? 3 : 2) : 1;

    /* prepare tx buffer */
    uint8_t tx_buf[cmd_len + n];
    memcpy(tx_buf, header, cmd_len);
    memcpy(tx_buf + cmd_len, data, n);

    /* prepare SPI transfer */
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .len = cmd_len + n,
    };

    /* Issue SPI Transaction */
    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to write to SPI device");
        return;
    }
}

/**
 * @brief Write data to a specific register using an offset and data of type uint32_t
 * Converts uint32_t data to the correct uint8_t[] format (little endian)
 */
void DWMController::writeBytes(uint8_t reg, uint16_t offset, uint32_t data)
{
    writeBytes(reg, offset, (uint8_t*)&data, sizeof(uint32_t));
}


/**
 * @brief Read data from the OTP memory of the DWM1000
 */
void DWMController::readBytesOTP(uint16_t addr, uint8_t* data, uint32_t len)
{
    for (uint32_t i = 0; i < (len / 4); i++) {
        _readBytesOTP(addr + i, data + (i * sizeof(uint32_t)));
    }
}


/**
 * @brief OTP Data is always returned in 4 byte words.
 */
void DWMController::_readBytesOTP(uint16_t addr, uint8_t* data)
{
    uint16_t otp_addr = addr;
    /* Write OTP Address for read */
    writeBytes(OTP_IF_ID, OTP_ADDR, (uint8_t*)&otp_addr, sizeof(uint16_t));

    /* Perform OTP Read - Manual read mode has to be set */
    uint8_t otp_cmd = OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN;
    writeBytes(OTP_IF_ID, OTP_CTRL, (uint8_t*)&otp_cmd, sizeof(uint8_t));
    otp_cmd = OTP_CTRL_OTPRDEN;
    writeBytes(OTP_IF_ID, OTP_CTRL, (uint8_t*)&otp_cmd, sizeof(uint8_t));

    /* Read data is available after 40ns */
    busywait_nanoseconds(1000);

    /* Read data from OTP */
    readBytes(OTP_IF_ID, OTP_RDAT, data, sizeof(uint32_t));

    otp_cmd = 0x00;
    writeBytes(OTP_IF_ID, OTP_CTRL, (uint8_t*)&otp_cmd, sizeof(uint8_t));
}


/**
 * @brief Get the length of the received data
 * 
 *        This value is copied from the PHR of the received frame when
 *        a good PHR is detected (when the RXPHD status bit is set). The frame length from the PHR is 
 *        used in the receiver to know how much data to receive and decode, and where to find the 
 *        FCS (CRC) to validate the received data
 */
uint16_t DWMController::getReceivedDataLength() {
    uint32_t data = 0;
    readBytes(RX_FINFO_ID, NO_SUB_ADDRESS, &data);

    /* Get the length of the received data */
    uint16_t len = data & (RX_FINFO_RXFLE_MASK | RX_FINFO_RXFLEN_MASK);

    if (len > 2)
        len -= 2;

    return len; //< minus 2 bytes for FCS data
}


/**
 * @brief Clear RX_FINFO register
 */
void DWMController::deleteReceivedDataLength()
{
    uint32_t data = 0;
    
    //data = ~(RX_FINFO_RXFLE_MASK | RX_FINFO_RXFLEN_MASK);
    writeBytes(RX_FINFO_ID, NO_SUB_ADDRESS, data);
}


/**
 * @brief Clear status bits in SYS_STATUS register
 */
void DWMController::clearStatusEvent(uint64_t event_mask)
{
    uint8_t sys_status[SYS_STATUS_LEN] = {0};

    memcpy(sys_status, &event_mask, SYS_STATUS_LEN);

    writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, sys_status, SYS_STATUS_LEN);
}


/**
 * 
 */
void DWMController::setIRQMask(uint32_t irq_mask)
{
    writeBytes(SYS_MASK_ID, NO_SUB_ADDRESS, irq_mask);
}


/**
 *  @brief Force the DWM1000 into idle mode 
 * 
 *  This method sets the TRXOFF bit in the SYS_CTRL register,
 *  which immediately forces the DWM1000 transceiver into idle mode.
 */
void DWMController::forceIdle() {
    uint32_t sys_ctrl = SYS_CTRL_TRXOFF;

    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_ctrl ,SYS_CTRL_LEN);
}


/**
 * 
 */
void DWMController::setSysClockSource(uint8_t source)
{
    uint32_t pmsc_ctrl = 0;
    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t*)&pmsc_ctrl, PMSC_CTRL0_LEN);

    switch (source)
    {
    case AUTO_CLOCK:
        spiSetBaud(FAST_SPI);
        pmsc_ctrl &= ~(0x3UL); //< clear sysclk bits
        pmsc_ctrl |= PMSC_CTRL0_SYSCLKS_AUTO; //< set sysclk to auto
        break;
    
    case XTI_CLOCK:
        spiSetBaud(SLOW_SPI);
        pmsc_ctrl &= ~(0x3UL); //< clear sysclk bits
        pmsc_ctrl |= PMSC_CTRL0_SYSCLKS_19M; //< set sysclk to xti
        break;

    case PLL_CLOCK:
        spiSetBaud(FAST_SPI);
        pmsc_ctrl &= ~(0x3UL); //< clear sysclk bits
        pmsc_ctrl |= PMSC_CTRL0_SYSCLKS_125M; //< set sysclk to pll
        break;
    }

    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, (uint8_t*)&pmsc_ctrl, PMSC_CTRL0_LEN);
    
}


/**
 * 
 */
void DWMController::loadLDECode() 
{
    uint8_t otp_ctrl[OTP_CTRL_LEN]      = {0};
    uint8_t pmsc_ctrl[PMSC_CTRL0_LEN]   = {0};

    /* Get current register values */
    readBytes(OTP_IF_ID, OTP_CTRL, otp_ctrl, OTP_CTRL_LEN);
    readBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl, PMSC_CTRL0_LEN);

    /* Set value according to 2.5.5.1.0 DW1000 User Manual: Step L-1 */
    *(uint32_t*)pmsc_ctrl &= ~(0x0000FFFF);
    *(uint32_t*)pmsc_ctrl |= 0x00000301;
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl, PMSC_CTRL0_LEN);
    
    /* Set value according to 2.5.5.1.0 DW1000 User Manual: Step L-2 */
    *(uint16_t*)otp_ctrl &= ~(0xFFFF);;
    *(uint16_t*)otp_ctrl |= OTP_CTRL_LDELOAD;
    writeBytes(OTP_IF_ID, OTP_CTRL, otp_ctrl, OTP_CTRL_LEN);

    /* Wait for 150 microseconds */
    busywait_nanoseconds(150000);

    /* Set value according to 2.5.5.1.0 DW1000 User Manual: Step L-3 */
    *(uint32_t*)pmsc_ctrl &= ~(0x0000FFFF);
    *(uint32_t*)pmsc_ctrl |= 0x00000200;
    writeBytes(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_ctrl, PMSC_CTRL0_LEN);
}


/**
 * @brief helper function to set the SPI baudrate
 * @param baudrate The baudrate to set
 */
dwm_com_error_t DWMController::spiSetBaud(uint32_t baudrate)
{
    if (baudrate <= SLOW_SPI && baudrate > FAST_SPI) {
        perror("Invalid SPI baudrate");
        return ERROR;
    }

    if (baudrate != _cur_spi_baud) {
        uint32_t speed = baudrate;
        if (ioctl(_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            perror("Failed to set SPI speed");
            return ERROR;
        }
        _cur_spi_baud = baudrate;
    }

    return SUCCESS;
}