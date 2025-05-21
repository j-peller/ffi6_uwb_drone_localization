#include "../inc/dwm1000_ranging.hpp"
#include "../inc/dw1000_modes.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

/* Change depending on role state */
const dwm1000_role_t ROLE = dwm1000_role_t::DRONE;

/*  */
void run_drone(DWMController* controller);
void run_drone_calibrate(DWMController* controller);
void run_anchor(DWMController* controller);
void run_anchor_calibrate(DWMController* controller);

/**
 * 
 */
int main() {
    dw1000_dev_instance_t device = {
        .role = ROLE,
        .short_addr = (ROLE == dwm1000_role_t::DRONE) ? MASTER : ANCHOR_1,
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

    /* Reset must be performed */
    //controller->soft_reset();
    //controller->set_mode(JOPEL110);
    ///* Interrupt Mask must match Status Mask */
    //controller->setIRQMask(SYS_MASK_MRXDFR | SYS_MASK_MTXFRS);
    //usleep(1000000);


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
    //            .finalRx = {0x12, 0x34, 0x56, 0x78, 0x9A}}}
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


    switch (ROLE) {
        case dwm1000_role_t::DRONE:
            (getenv_int("DWM1000_CALIBRATE") == 1) ? run_drone_calibrate(controller) : run_drone(controller);
            break;
        case dwm1000_role_t::ANCHOR:
            (getenv_int("DWM1000_CALIBRATE") == 1) ? run_anchor_calibrate(controller) : run_anchor(controller);
            break;
    }

    return EXIT_SUCCESS;
}

void run_drone(DWMController* controller) {
    fprintf(stdout, "Running as Drone\n");
    DWMRangingDrone* tag = dynamic_cast<DWMRangingDrone*>(DWMRanging::create_instance(controller));
    if (tag == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        return;
    }

    double distance = 0.0;
    tag->get_distance_to_anchor(ANCHOR_1, &distance);
    fprintf(stdout, "Got distances: %lfm\n", distance);

    delete tag;
}

void run_drone_calibrate(DWMController* controller) {
    fprintf(stdout, "Running as Drone for calibration\n");
    DWMRangingDrone* tag = dynamic_cast<DWMRangingDrone*>(DWMRanging::create_instance(controller));
    if (tag == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        return;
    }

    double distance = 0.0;
    tag->calibrate_antenna_delay(5.0, 0.15, 50);
    tag->get_distance_to_anchor(ANCHOR_1, &distance);
    fprintf(stdout, "Got distances: %lfm\n", distance);

    delete tag;
}

void run_anchor(DWMController* controller) {
    fprintf(stdout, "Running as Anchor\n");
    DWMRangingAnchor* anchor = dynamic_cast<DWMRangingAnchor*>(DWMRanging::create_instance(controller));
    if (anchor == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        return;
    }

    while (true) {
        if (anchor->run_state_machine() == SUCCESS) {
            fprintf(stdout, "Successfull Ranging Done\n");
        }
    }

    delete anchor;
}

void run_anchor_calibrate(DWMController* controller) {
    fprintf(stdout, "Running as Anchor for calibration\n");
    DWMRangingAnchor* anchor = dynamic_cast<DWMRangingAnchor*>(DWMRanging::create_instance(controller));
    if (anchor == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        return;
    }

    anchor->calibrate_antenna_delay(50);

    delete anchor;
}