#include "../inc/dwm1000_ctrl.hpp"

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


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
 * @brief Get the device ID of the DWM1000 -- Read from the DEV_ID register = 0x00
 * @param device_id Pointer to store the device ID
 * 
 */
void DWMController::get_device_id(uint32_t* device_id)
{
    uint8_t data[DEV_ID_LEN] = {0};
    readBytes(DEV_ID_ID, data, DEV_ID_LEN);
    *device_id = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}


/**
 * @brief Read data from the DWM1000 device
 * 
 * @param reg The register address to read from
 * @param data Pointer to the buffer to store the read data
 * @param len The length of the data to read
 */

void DWMController::readBytes(uint16_t reg, uint8_t* data, int len)
{
    /* 2 for address header */
    uint8_t tx_buf[2] = {0};
    /* len for expected data to receive */
    uint8_t* rx_buf = new uint8_t[len];

    tx_buf[0] = (reg & 0xFF00) >> 8;
    tx_buf[1] = (reg & 0x00FF);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = 2 + len,
    };

    /* syscall to SPI Kernel driver to */
    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to read from SPI device");
        delete[] rx_buf;
        return;
    }

    // Read the data
    for (int i = 0; i < len; i++) {
        data[i] = rx_buf[2 + i];    /* skip the first 2 bytes, which are the register address */
    }

    delete[] rx_buf;
}

/**
 * @brief Write data to the DWM1000 device
 * 
 * @param reg The register address to write to
 * @param data Pointer to the data to write
 * @param len The length of the data to write
 * 
 */
void DWMController::writeBytes(uint16_t reg, uint8_t* data, int len)
{
    /* 2 for address header */
    uint8_t tx_buf[2 + len];
    tx_buf[0] = (reg & 0xFF00) >> 8;
    tx_buf[1] = (reg & 0x00FF);

    for (int i = 0; i < len; i++) {
        tx_buf[2 + i] = data[i];    /* skip the first 2 bytes, which are the register address */
    }

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .len = 2 + len,
    };

    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("Failed to write to SPI device");
        return;
    }
}