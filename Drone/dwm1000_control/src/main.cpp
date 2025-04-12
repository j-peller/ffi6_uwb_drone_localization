#include "../inc/dwm1000_ranging.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

int main() {
    dw1000_dev_instance_t device = {
        .spi_dev = "/dev/spidev0.0",
        .spi_baudrate = 2000000, //< currently at 20MHz issues with test setup (schlechte lötstellen)
        .spi_bits_per_word = 8,
        .spi_mode = SPI_MODE_0,
        .gpiod_chip = "/dev/gpiochip4",
        .irq_gpio_pin = 17,
        .rst_gpio_pin = 27
    };
    DWMController* controller = DWMController::create_instance(&device);
    if (controller == NULL) {
        fprintf(stderr, "Failed to create DWMController instance\n");
        return EXIT_FAILURE;
    }

    uint16_t i = 0;
    for (;;) {
        uint16_t shorta = 0;
        controller->set_device_short_addr(i);
        controller->get_device_short_addr(&shorta);
        fprintf(stdout, "Short Address: 0x%04X\n", shorta);
        i++;
        usleep(1000000);
    }

    //DWMRanging* ranging = new DWMRanging(controller);
    //if (ranging == NULL) {
    //    fprintf(stderr, "Failed to create DWMRanging instance\n");
    //    delete controller;
    //    return EXIT_FAILURE;
    //}

    //distances d;
    //ranging->get_distances_to_anchors(&d);

    return EXIT_SUCCESS;
}