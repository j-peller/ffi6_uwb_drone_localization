#include "uwb_anchor_service.hpp"
#include "dwm1000_ranging.hpp"
#include "ws_logger.hpp"

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <thread>
#include <linux/spi/spidev.h>
#include "INIReader.h"

/* Global running flag */
std::atomic<bool> running(true);

/* Global state of the anchor service */
std::atomic<anchor_state_t> current_state(RANGING);

/* Anchor State */
typedef enum {
    RANGING, 
    CALIBRATION,
    RESET
} anchor_state_t;


/* */
void print_usage(const char* progname);
void handle_signal(int signal);


int main(int argc, char* argv[]) {

    /* Register signal handlers to gracefully stop this service */
    std::signal(SIGINT, handle_signal);

    /* Location of Config file if not specified otherwise */
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

    /* Create globally available logger (separate thread) */
    WSLogger::get_instance(
        reader.Get("logging", "log_server_ip", "0.0.0.0").c_str(),
        reader.GetInteger("logging", "log_server_port", 0),
        reader.GetInteger("anchor", "short_addr", 0xFFFF)
    );

    /* Print Test Message */
    WS_LOG("WSLogger initialized for Anchor with ID: %d", reader.GetInteger("anchor", "short_addr", 0xFFFF));

    /* Get relevant Informations for our DWMController */
    dw1000_dev_instance_t device = {
        .role = ROLE, 
        .short_addr = reader.GetInteger("anchor", "short_addr", 0xFFFF),
        .spi_dev = reader.Get("anchor", "spi_dev", "/dev/spidev0.0").c_str(),
        .spi_baudrate = SLOW_SPI, //< Start mit 2MHz clock and ramp up after init
        .spi_bits_per_word = 8,
        .spi_mode = SPI_MODE_0,
        .gpiod_chip = reader.Get("hardware", "gpiod_chip", "/dev/gpiochip4").c_str(),
        .irq_gpio_pin = reader.GetInteger("hardware", "irq_pin", 26),
        .rst_gpio_pin = reader.GetInteger("hardware", "rst_pin", 27),
        .mode = static_cast<dw1000_mode_enum_t>(reader.GetInteger("anchor", "uwb_mode", 1))
    };

    /* Create and initialize DWMController Object */
    DWMController* controller = DWMController::create_instance(&device);
    if (controller == NULL) {
        std::cerr << "Failed to create DWMController instance" << std::endl;
        return EXIT_FAILURE;
    }

    /* Create DWMRangingAnchor Object */
    DWMRangingAnchor* anchor = dynamic_cast<DWMRangingAnchor*>(DWMRanging::create_instance(controller));
    if (anchor == NULL) {
        std::cerr << "Failed to create DWMRangingAnchor instance" << std::endl;
        return EXIT_FAILURE;
    }

    /* */
    usleep(1000000);


    /* TODO */
    while (running) {
        /* Run the ranging state machine */
        switch (current_state) {
            case RANGING:
                anchor->run_state_machine();
                break;

            case CALIBRATION:
                //anchor->run_calibration();
                break;

            case RESET:
                /* After Softreset, Shortaddr must be set again */
                bool performHardReset = false;
                anchor->perform_reset(performHardReset);
                break;
        }
    }

    std::cout << "Stopping Anchor..." << std::endl;
    usleep(1000000);

    /* Clean Up */
    if (anchor)
        delete anchor;

    return EXIT_SUCCESS;
}


void print_usage(const char* progname) {
    std::cout << "Usage: " << progname << " [-c config_file]\n"
              << "  -c <file>   Specify path to configuration file (default: "
              << DEFAULT_CONFIG_PATH << ")\n"
              << "  -h          Show this help message\n";
}


void handle_signal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "Received termination signal. Exiting...\n";
        running = false;
    }
}