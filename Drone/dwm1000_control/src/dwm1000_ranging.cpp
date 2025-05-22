#include "../inc/dwm1000_ranging.hpp"
#include "../../../shared/inc/helpers.hpp"
#include "../inc/dw1000_time.hpp"
#include <linux/spi/spidev.h>

DWMRanging::~DWMRanging() 
{

}

/**
 * 
 */
DWMRanging* DWMRanging::create_instance(DWMController* ctrl)
{
    DWMController*      controller = ctrl;
    DWMRanging*         instance = NULL;

    if (controller == NULL) {
        /**
         * In the Docker Compose file, the following environment variables must be set:
         * - DWM1000_SPI_DEV: SPI device path (e.g., /dev/spidev0.0)
         * - DWM1000_GPIO_DEV: GPIO device path (e.g., /dev/gpiochip0)
         * - DWM1000_IRQ_PIN: GPIO pin number for IRQ (e.g., 17)
         * - DWM1000_RST_PIN: GPIO pin number for RST (e.g., 27)
         * - DWM1000_MODE: Mode of the DW1000 (e.g., JOPEL110 or THOTRO110)
         */
        dw1000_dev_instance_t device = {
            .role               = getenv_str("DWM1000_ROLE") == "ANCHOR" ? dwm1000_role_t::ANCHOR : dwm1000_role_t::DRONE,
            .short_addr         = getenv_int("DWM1000_SHORT_ADDR"),
            .spi_dev            = getenv_str("DWM1000_SPI_DEV"),
            .spi_baudrate       = SLOW_SPI,
            .spi_bits_per_word  = 8,
            .spi_mode           = SPI_MODE_0,
            .gpiod_chip         = getenv_str("DWM1000_GPIO_DEV"),
            .irq_gpio_pin       = getenv_int("DWM1000_IRQ_PIN"),
            .rst_gpio_pin       = getenv_int("DWM1000_RST_PIN"),
            .mode               = (dw1000_mode_enum_t)getenv_int("DWM1000_MODE")
        };

        controller = DWMController::create_instance(&device);
        if (controller == NULL) {
            fprintf(stderr, "Failed to create DWMController instance\n");
            return NULL;
        }
    }

    /* Depending on our UWB Mode, configure DWM1000 accordingly */
    switch (controller->_dev_instance.mode)
    {
    case dw1000_mode_enum_t::JOPEL: // Mode 1
        fprintf(stdout, "Setting mode to JOPEL110\n");
        controller->set_mode(JOPEL110);
        break;
    case dw1000_mode_enum_t::THOTRO: // Mode 0
        fprintf(stdout, "Setting mode to THOTRO110\n");
        controller->set_mode(THOTRO110);
        break;
    default:
        fprintf(stdout, "Setting mode to JOPEL110\n");
        controller->set_mode(JOPEL110);
        break;
    }

    /* Enable Frame Filter for Data Frames */
    controller->enable_frame_filtering(SYS_CFG_FFAD);

    /* Set IRQs */
    controller->setIRQMask(SYS_MASK_MRXDFR | SYS_MASK_MTXFRS);

    /* Set configured Antenna Delay */
    controller->set_tx_antenna_delay(INITIAL_ANTENNA_DELAY);
    controller->set_rx_antenna_delay(INITIAL_ANTENNA_DELAY);
    busywait_nanoseconds(1000000);  //< Wait 1ms

    /* Depending on wether its an Anchor or TAG (Drone) return specific Ranging object */
    if (controller->get_role() == dwm1000_role_t::ANCHOR) {
        fprintf(stdout, "Its an Anchor! Congratulations!\n");
        instance = new DWMRangingAnchor(controller);

    } else if (controller->get_role() == dwm1000_role_t::DRONE) {
        fprintf(stdout, "Its a Drone! Congratulations!\n");
        instance = new DWMRangingDrone(controller);
    }
    
    if (instance == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        return NULL;
    }

    return instance;
}   


/**
 * Default Constructor to be used by ROS2 Node in combination with Docker Environment Variables
 */
DWMRanging::DWMRanging() 
    : _controller(NULL)
{
    /* */
}


/**
 * 
 */
DWMRanging::DWMRanging(DWMController* controller) 
{
    if (controller == NULL) {
        fprintf(stderr, "Failed to create DWMRanging instance\n");
        exit(EXIT_FAILURE);
    }

    _controller = controller;
}


