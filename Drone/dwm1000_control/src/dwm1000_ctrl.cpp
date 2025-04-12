#include "../inc/dwm1000_ctrl.hpp"
#include "../../../shared/inc/anchor_addresses.hpp"

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


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
    if (device->spi_baudrate <= 0 && device->spi_baudrate > MAX_SPI_BAUDRATE) {
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

    if (gpiod_line_request_output(instance->_rst_line, "DWM1000Reset", 0) < 0) {
        perror("Failed to set GPIO Pin to OUTPUT");
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
    uint16_t read_back = 0;
    instance->get_device_short_addr(&read_back);
    if (read_back != MASTER) {
        fprintf(stderr, "DWM1000 returned invalid Short Address: 0x%04X\n", read_back);
        perror("DWM1000 not detected or SPI not working");
        delete instance;
        close(fd);
        return NULL;
    }

    fprintf(stdout, "DWM1000 Setup successful with Short Address: 0x%04X\n", read_back);


    return instance;
}


DWMController::DWMController(int spi_fd, dw1000_dev_instance_t* device)
    : _spi_fd(spi_fd), _dev_instance(*device)
{

}   


DWMController::~DWMController()
{
    /* Close GPIO Chip */
    if (_gpio_chip) {
        gpiod_chip_close(_gpio_chip);
    }

}


/**
 * 
 */
dwm_com_error_t DWMController::do_init_config()
{
    /* Sys Config Part */
    uint32_t sys_cfg = 0;
    readBytes(SYS_CFG_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_cfg, SYS_CFG_LEN);

    sys_cfg |= SYS_CFG_FFE;     //< Enable Frame Filtering
    sys_cfg |= SYS_CFG_FFAD;    //< Allow Data Frame



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
void DWMController::write_transmission_data(uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0) {
        fprintf(stderr, "Invalid data or length for transmission\n");
        return;
    }

    /* TODO: Check if CRC is used, right now we always use crc*/
    len = (len + 2) & TX_FCTRL_TFLEN_MASK;

    /* Write the data to be transmitted */
    writeBytes(TX_BUFFER_ID, NO_SUB_ADDRESS, data, len);

    /* Set Transmit Frame Length accordingly */
    this->_tx_fctrl &= ~TX_FCTRL_TFLEN_MASK;
    this->_tx_fctrl |= len;
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

    /* currently not required to keep track of DW1000 operating mode */
    //this->_dev_mode = TX_MODE;

    uint32_t sys_ctrl = SYS_CTRL_TXSTRT;
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_ctrl ,SYS_CTRL_LEN);
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
void DWMController::start_receiving() 
{
    /* Idle mode required */
    this->forceIdle();

    /* currently not required to keep track of DW1000 operating mode */
    //this->_dev_mode = RX_MODE;

    uint32_t sys_ctrl = SYS_CTRL_RXENAB;
    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_ctrl ,SYS_CTRL_LEN);
}


/**
 * @brief Read len bytes of received data from the RX buffer of the DW1000
 * 
 * @param data Pointer to the buffer to store the received data
 * @param n Pointer to the variable to store the length of the received data
 *
 */
uint8_t* DWMController::read_received_data(uint8_t* n)
{
    /* get length of received data from Frame Info register */
    uint8_t len = getReceivedDataLength();
    if (len <= 0) {
        fprintf(stderr, "Invalid length for received data\n");
        return NULL;
    }

    uint8_t* rx_data = new uint8_t[len];
    if (rx_data == NULL) {
        fprintf(stderr, "Failed to allocate memory for received data\n");
        return NULL;
    }

    /* Read received data from RX_BUFFER of DW1000 */
    readBytes(RX_BUFFER_ID, NO_SUB_ADDRESS, rx_data, len);

    /* update the length of the received buffer */
    *n = len;

    return rx_data;
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
 */
dwm_com_error_t DWMController::poll_status_bit(uint64_t status_bit, uint64_t timeout)
{
    uint8_t sys_status[SYS_STATUS_LEN] = {0};

    timespec start, now;

    clock_gettime(CLOCK_MONOTONIC_RAW,  &start);
    do {

        readBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, sys_status, SYS_STATE_LEN);
        clock_gettime(CLOCK_MONOTONIC_RAW,  &now);

        if (timespec_delta_nanoseconds(&now, &start) > timeout) {
            // fprintf(stderr, "Timeout waiting for status bit\n");
            return ERROR;
        }

    } while (! (*(uint64_t*)(sys_status) & (0x1ULL << status_bit)) );

    /* clear status bit */
    (*(uint64_t*)(sys_status)) &= (0x1ULL << status_bit);
    writeBytes(SYS_STATUS_ID, NO_SUB_ADDRESS, sys_status, SYS_STATUS_LEN);

    return SUCCESS;
}


/**
 * @brief External hard reset of the DWM1000 device
 * 
 */
void DWMController::reset()
{
    /*  */
    gpiod_line_request_output(this->_rst_line, "DWM1000Reset", 0);

    /* */
    gpiod_line_set_value(this->_rst_line, 0);

    /* Reset Pin should be manually driven low for at least 50ns to ensure correct reset operation */
    busywait_nanoseconds(1000);

    /* */
    gpiod_line_request_input(this->_rst_line, "DWM1000Reset");

    /* busywait for 10ms */
    busywait_nanoseconds(10000000);

    /* Force the DWM1000 into idle mode */
    this->forceIdle();
}


/**
 * @brief API to do softreset on dw1000 by writing data into PMSC_CTRL0_SOFTRESET_OFFSET.
 */
 void DWMController::soft_reset() 
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
    busywait_nanoseconds(10000);

    /* Finish reset */
    reg = PMSC_CTRL0_RESET_CLEAR; 
    writeBytes(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, &reg, sizeof(uint8_t));
 }


/**
 * 
 */
void DWMController::set_device_short_addr(uint16_t short_addr)
{
    writeBytes(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, (uint8_t*)&short_addr, sizeof(uint16_t));
}


/**
 * @brief Get the device ID of the DWM1000 -- Read from the DEV_ID register = 0x00
 * @param device_id Pointer to store the device ID
 * 
 */
void DWMController::get_device_id(uint32_t* device_id)
{
    uint8_t data[DEV_ID_LEN] = {0};
    readBytes(DEV_ID_ID, NO_SUB_ADDRESS, data, DEV_ID_LEN);
    *device_id = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}


/**
 * 
 */
void DWMController::get_device_short_addr(uint16_t* short_addr)
{
    readBytes(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, (uint8_t*)short_addr, sizeof(uint16_t));
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
 * 
 */
void DWMController::setupTXFrameControl() {
    uint32_t tx_fctrl = 0;

    /* Set the TX frame control register */
    writeBytes(TX_FCTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&tx_fctrl ,TX_FCTRL_LEN);
}


/**
 * @brief Get the length of the received data
 * 
 *        This value is copied from the PHR of the received frame when
 *        a good PHR is detected (when the RXPHD status bit is set). The frame length from the PHR is 
 *        used in the receiver to know how much data to receive and decode, and where to find the 
 *        FCS (CRC) to validate the received data
 */
uint8_t DWMController::getReceivedDataLength() {
    uint8_t data[RX_FINFO_LEN] = {0};
    readBytes(RX_FINFO_ID, NO_SUB_ADDRESS, data, RX_FINFO_LEN);

    /* Get the length of the received data */
    uint8_t len = (data[0] & RX_FINFO_RXFLEN_MASK);
    return len;
}


/**
 *  @brief Force the DWM1000 into idle mode 
 * 
 *  This method sets the TRXOFF bit in the SYS_CTRL register,
 *  which immediately forces the DWM1000 transceiver into idle mode.
 */
void DWMController::forceIdle() {
    uint32_t sys_ctrl = SYS_CTRL_TRXOFF;

    this->_dev_mode = IDLE_MODE;

    writeBytes(SYS_CTRL_ID, NO_SUB_ADDRESS, (uint8_t*)&sys_ctrl ,SYS_CTRL_LEN);
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
