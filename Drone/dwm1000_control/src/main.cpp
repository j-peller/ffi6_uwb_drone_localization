#include "../inc/dwm1000_ctrl.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>

int main() {
    dw1000_dev_instance_t device = {
        .spi_dev = "/dev/spidev0.0",
        .spi_baudrate = 20000000,
        .spi_bits_per_word = 8,
        .spi_mode = SPI_MODE_0,
        .irq_pin = 17,
        .rst_pin = 27
    };
    DWMController* controller = DWMController::create_instance(&device);
    if (controller == NULL) {
        fprintf(stderr, "Failed to create DWMController instance\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}