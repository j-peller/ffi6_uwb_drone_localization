#include "dwm1000_ranging.hpp"

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include "INIReader.h"


void print_usage(const char* progname) {
    std::cout << "Usage: " << progname << " [-c config_file]\n"
              << "  -c <file>   Specify path to configuration file (default: "
              << DEFAULT_CONFIG_PATH << ")\n"
              << "  -h          Show this help message\n";
}

int main(int argc, char* argv[]) {

    std::string config_path = DEFAULT_CONFIG_PATH;

    int opt;
    while ((opt = getopt(argc, argv, "c:h")) != -1) {
        switch (opt) {
            case 'c':
                config_path = optarg; //< plis. do not abuse c:
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    /*  */
    INIReader reader(config_path);
    if (reader.ParseError() < 0) {
        std::cerr << "Error: Failed to load config file: " << config_path << std::endl;
        return EXIT_FAILURE;
    }

    /* Get relevant Informations for our DWMController */
    dw1000_dev_instance_t device = {
        .role = ROLE, 
        .short_addr = reader.GetInteger("anchor", "short_addr", 0xFFFF),
        .long_addr = reader.GetInteger("anchor", "long_addr", 0x0),
        .spi_dev = reader.Get("anchor", "spi_dev", "/dev/spidev0.0"),
        .spi_baudrate = SLOW_SPI, //< Start mit 2MHz clock and ramp up after init
        .spi_bits_per_word = 8,
        .spi_mode = SPI_MODE_0,
        .gpiod_chip = reader.Get("hardware", "gpiod_chip", "/dev/gpiochip4"),
        .irq_gpio_pin = reader.GetInteger("hardware", "irq_pin", 26),
        .rst_gpio_pin = reader.GetInteger("hardware", "rst_pin", 27) 
    };

    /* Create and initialize DWMController Object */
    DWMController* controller = DWMController::create_instance(&device);
    if (controller == NULL) {
        std::cerr << "Failed to create DWMController instance" << std::endl;
        return EXIT_FAILURE;
    }

    /* Create DWMRangingAnchor Object */
    DWMRangingAnchor* anchor = dynamic_cast<DWMRangingAnchor*>(DWMRangingAnchor::create_instance(controller));
    if (anchor == NULL) {
        std::cerr << "Failed to create DWMRangingAnchor instance" << std::endl;
        return EXIT_FAILURE;
    }






    switch (ROLE) {
        case dwm1000_role_t::DRONE:
            break;
        case dwm1000_role_t::ANCHOR:
            break;
    }

    return EXIT_SUCCESS;
}