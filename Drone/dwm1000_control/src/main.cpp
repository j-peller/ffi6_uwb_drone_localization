#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_modes.h"
//#include "../inc/dwm1000_ranging_anchor.hpp"

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
        .irq_gpio_pin = 26,
        .rst_gpio_pin = 27
    };
    DWMController* controller = DWMController::create_instance(&device);
    if (controller == NULL) {
        fprintf(stderr, "Failed to create DWMController instance\n");
        return EXIT_FAILURE;
    }

    controller->soft_reset();
    usleep(1000000);
    controller->set_mode(THOTRO110);
    usleep(1000000);


    //DW1000Time rx_time;
    ///* after reception it will clear its state and go to idle */
    //while(1) {
    //    if (controller->test_receiving_timestamp(rx_time) == SUCCESS) {
    //        fprintf(stdout, "RX Timestamp: %lu\n", rx_time.get_timestamp());
    //    } else {
    //        fprintf(stdout, "Some Error\n");
    //    }
    //    usleep(1000000);
    //}

    //DW1000Time tx_time;
    //twr_message_t msg = {
    //        .header = {
    //            .frameCtrl = {0x41, 0x88},
    //            .seqNum = 0x00,
    //            .panID = {0xCA, 0xDE},
    //            .destAddr = { 0xFE, 0xAF },
    //            .srcAddr = { MASTER & 0xff, MASTER >> 8 }
    //        },
    //        .payload = { .report = {
    //            .type = twr_msg_type_t::TWR_MSG_TYPE_REPORT,
    //            .finalTx = {0x12, 0x34, 0x56, 0x78, 0x9A}}}
    //};

    //uint64_t ts = 0;
    //while(1) {
    //    ts++;
    //    memcpy(msg.payload.report.finalRx, &ts, 5);
    //    if (controller->test_transmission_timestamp(tx_time, (uint8_t*)&msg) == SUCCESS) {
    //        fprintf(stdout, "TX Timestamp: %lu\n", tx_time.get_timestamp());
    //    } else {
    //        fprintf(stdout, "Some Error\n");
    //    }
    //    usleep(1000000);
    //}


    /**
     * Uncomment respective for testing
     */

    DWMRanging tag = DWMRanging(controller);
    double distance = 0.0;
    tag.get_distance_to_anchor(ANCHOR_1, &distance);
    fprintf(stdout, "Got distance: %lfm\n", distance);


    //DWMRangingAnchor anchor = DWMRangingAnchor(controller);
    //while(1) {
    //    if (anchor.run_state_machine() == SUCCESS)
    //        break; 
    //}
    //fprintf(stdout, "Anchor finished\n");


    return EXIT_SUCCESS;
}