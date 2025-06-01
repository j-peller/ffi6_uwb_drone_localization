#include <gtest/gtest.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

#include "linux/spi/spidev.h"

#include "../inc/dwm1000_ranging.hpp"
#include "../inc/ws_logger.hpp"
#include "../../coords_calc/coords_calc.h"

class PositioningTest : public ::testing::Test {
protected:
    void SetUp() override {

        // Set up the DW1000 controller and drone instance        
        dw1000_dev_instance_t device = {
            .role = DRONE,
            .short_addr = MASTER,
            .spi_dev = "/dev/spidev0.0",
            .spi_baudrate = SLOW_SPI, //< Start mit 2MHz clock and ramp up after init
            .spi_bits_per_word = 8,
            .spi_mode = SPI_MODE_0,
            .gpiod_chip = "/dev/gpiochip4",
            .irq_gpio_pin = 26,
            .rst_gpio_pin = 27,
            .mode = getenv_int("DWM1000_MODE") == 1 ? JOPEL : THOTRO,
            .antenna_delay = getenv_int("DWM1000_ANTENNA_DELAY") > 0 ? getenv_int("DWM1000_ANTENNA_DELAY") : INITIAL_ANTENNA_DELAY
        };

        controller = DWMController::create_instance(&device);
        if (controller == NULL) {
            fprintf(stderr, "Failed to create DWMController instance\n");
        }

        drone = dynamic_cast<DWMRangingDrone*>(DWMRanging::create_instance(controller));
        if (drone == NULL) {
            fprintf(stderr, "Failed to create DWMRanging instance\n");
        }

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("Socket creation failed");
            exit(EXIT_FAILURE);
        }

        sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(6969); // Choose a port

        int opt = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            perror("setsockopt(SO_REUSEADDR) failed");
        }

        if (bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            perror("Bind failed");
            exit(EXIT_FAILURE);
        }
    }

    /* (0,0) Mittelpunkt */
    const pos POS_ANCHOR_1 = { .x = -0.25, .y = 0.25, .z = 0.0 };
    const pos POS_ANCHOR_2 = { .x = 0.25, .y = 0.25, .z = 0.0 };
    const pos POS_ANCHOR_3 = { .x = 0.25, .y = -0.25, .z = 0.0 };
    const pos POS_ANCHOR_4 = { .x = -0.25, .y = -0.25, .z = 0.0 };

    DWMRangingDrone* drone;
    DWMController* controller;
    sockaddr_in serv_addr;
    int sockfd;
};


// Google Test
TEST_F(PositioningTest, Positioning) {
    dwm_com_error_t ret = SUCCESS;

    distances ds = (distances) { 0.0, 0.0, 0.0, 0.0 };
    pos result = (pos) { 0.0, 0.0, 0.0 };

    listen(sockfd, 1);
    printf("Waiting for client to connect...\n");

    int client_fd = accept(sockfd, NULL, NULL);
    if (client_fd < 0) {
        perror("Accept failed");
        exit(EXIT_FAILURE);
    }
    printf("Client connected.\n");

    while (true) {
        ret = drone->get_distances_to_anchors(&ds);
        if (ret != SUCCESS) {
            fprintf(stderr, "Failed to get distances to anchors: %d\n", ret);
            continue;
        }

        result = coords_calc(
            ds.d1, ds.d2, ds.d3, ds.d4,
            POS_ANCHOR_1, 
            POS_ANCHOR_2, 
            POS_ANCHOR_3, 
            POS_ANCHOR_4
        );

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f\n", result.x, result.y, 0.0f);
        send(client_fd, buffer, strlen(buffer), 0);

        // Print the calculated position
        fprintf(stdout, "Calculated Position: x=%.2f, y=%.2f, z=%.2f\n",
               result.x, result.y, 0.0f);

        usleep(250000);
    }
}