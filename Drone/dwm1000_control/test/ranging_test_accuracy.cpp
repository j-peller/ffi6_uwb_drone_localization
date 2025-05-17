#include <gtest/gtest.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include "../inc/dwm1000_ranging.hpp"  // Include your header file
#include "linux/spi/spidev.h"

class RangingTest : public ::testing::Test {
protected:
    void SetUp() override {
        dw1000_dev_instance_t device = {
            .role = dwm1000_role_t::DRONE,
            .spi_dev = "/dev/spidev0.0",
            .spi_baudrate = SLOW_SPI, //< Start mit 2MHz clock and ramp up after init
            .spi_bits_per_word = 8,
            .spi_mode = SPI_MODE_0,
            .gpiod_chip = "/dev/gpiochip4",
            .irq_gpio_pin = 26,
            .rst_gpio_pin = 27
        };
        controller = DWMController::create_instance(&device);
        if (controller == NULL) {
            fprintf(stderr, "Failed to create DWMController instance\n");
        }

        tag = dynamic_cast<DWMRangingDrone*>(DWMRanging::create_instance(controller));
        if (tag == NULL) {
            fprintf(stderr, "Failed to create DWMRanging instance\n");
        }


        numIterations = 100;
        outputFileName = "ranging_results.txt";
        distances.reserve(numIterations);
    }

    DWMRangingDrone* tag;
    DWMController* controller;
    int numIterations;
    std::string outputFileName;
    std::vector<double> distances;
};


// Google Test
TEST_F(RangingTest, RangingStatistics) {
    distances.reserve(numIterations);

    int totalAttempts = 0;
    int failedAttempts = 0;
    int zeroDistanceCount = 0;

    // Collect distance measurements
    for (int i = 0; i < numIterations; ++i) {
        totalAttempts++;
        double dist = 0.0;
        if (tag->get_distance_to_anchor(ANCHOR_1, &dist) == dwm_com_error_t::ERROR) {
            failedAttempts++;
            continue;  // Skip this iteration if there's an error
        }

        if (dist == 0.0) {
            zeroDistanceCount++;
        }
        
        distances.push_back(dist);

    }

    // Write to output file
    std::ofstream outFile(outputFileName);
    ASSERT_TRUE(outFile.is_open()) << "Could not open output file.";

    for (const auto& val : distances) {
        outFile << val << "\n";
    }

    outFile.close();
    
    // Output basic statistics
    fprintf(stdout, "Total Attempts     : %d\n", totalAttempts);
    fprintf(stdout, "Failed Attempts    : %d\n", failedAttempts);
    fprintf(stdout, "Valid Measurements : %d\n", totalAttempts - failedAttempts);

    ASSERT_EQ(failedAttempts, 0) << "Some attempts failed.";
    ASSERT_EQ(zeroDistanceCount, 0) << "Some distances were 0.0, which may indicate invalid measurements.";
}
