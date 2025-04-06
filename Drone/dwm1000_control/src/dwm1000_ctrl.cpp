#include "../inc/dwm1000_ctrl.hpp"
#include "../inc/dw1000_time.hpp"

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

    /* TODO: Setup GPIO */

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

    return instance;
}


DWMController::DWMController(int spi_fd, dw1000_dev_instance_t* device)
    : _spi_fd(spi_fd), _dev_instance(*device)
{

}   


DWMController::~DWMController()
{

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
 * @brief Reset the DWM1000 device
 * 
 */
void DWMController::reset()
{
    /* TODO */


    /* Force the DWM1000 into idle mode */
    this->forceIdle();
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
 * @brief Read data from the DWM1000 device
 * 
 * @param reg The register address to read from
 * @param offset The subaddress / offset within the register
 * @param data Pointer to the buffer to store the read data
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

    /* prepare SPI transfer */
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)header,
        .rx_buf = (unsigned long)data,
        .len = cmd_len + n,
    };

    /* syscall to SPI Kernel driver to read data from requested register */
    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to read from SPI device");
        return;
    }
}


/**
 * @brief Write data to the DWM1000 device
 * 
 * @param reg The register address to write to
 * @param offset The subaddress / offset within the register
 * @param data Pointer to the data to write
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