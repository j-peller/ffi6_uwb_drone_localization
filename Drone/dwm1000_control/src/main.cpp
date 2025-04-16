#include "../inc/dwm1000_ranging.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

int main() {
    dw1000_dev_instance_t device = {
        .spi_dev = "/dev/spidev0.0",
        .spi_baudrate = SLOW_SPI, //< Start mit 2MHz clock and ramp up after init
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

    controller->do_init_config();
    usleep(1000000);

    DW1000Time rx_time;
    /* after reception it will clear its state and go to idle */
    while(1) {
        controller->test_receiving_timestamp(rx_time);
        fprintf(stdout, "RX Timestamp: %llu\n", rx_time.get_timestamp());
        usleep(1000000);
    }


    //for (int i = 0; i < 10; i++) {
    //    DW1000Time test1, test2;
    //    controller->test_transmission_timestamp(test1);
    //    controller->test_transmission_timestamp(test2);

    //    DW1000Time delta = test2 - test1;
    //    fprintf(stdout, "T1 Timestamp: %llu\n", test1.get_timestamp());
    //    fprintf(stdout, "T2 Timestamp: %llu\n", test2.get_timestamp());
    //    fprintf(stdout, "Delta Timestamp: %llu\n", delta.get_timestamp());

    //    usleep(1000000);
    //}

    //for (int i = 0; i < 10; i++) {
    //    uint16_t shorta = 0;
    //    controller->set_device_short_addr(i);
    //    controller->get_device_short_addr(&shorta);
    //    fprintf(stdout, "Short Address: 0x%04X\n", shorta);
    //    usleep(1000000);
    //}

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