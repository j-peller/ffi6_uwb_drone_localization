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

    Bitrate bitrate_110k {
        .txbr = TX_FCTRL_TXBR_110k,
        .rxm110k = SYS_CFG_RXM110K,
    };

    PRF prf16 {
        .txprf = TX_FCTRL_TXPRF_16M,
        .rxfprf = CHAN_CTRL_RXFPRF_16,
        .drx_tune1a = DRX_TUNE1a_PRF16,
    };

    Channel channel5 = {
        .rf_rxctrlh = RF_RXCTRLH_NBW,
        .rf_txctrl = RF_TXCTRL_CH5,
        .tc_pgdelay = 0xB5,
        .fs_pllcfg = FS_PLLCFG_CH5,
    };

    Tuning tune_ch5 = {
        .drx_tune0b = DRX_TUNE0b_110K_STD,
        .drx_tune1a = DRX_TUNE1a_PRF16,
        .drx_tune1b = DRX_TUNE1b_110K,
        .drx_tune2 = DRX_TUNE2_PRF16_PAC64,
        .agc_tune1 = AGC_TUNE1_16M,
        .agc_tune2 = AGC_TUNE2_VAL,
        .fs_plltune = FS_PLLTUNE_CH5
    };

    Mode thotro110 {
        .channel = channel5,
        .prf = prf16,
        .bitrate = bitrate_110k,
        .preamble_code = 0x04,
        .preamble_length = TX_FCTRL_TXPSR_PE_2048,
        .sfd = SFD::STD,
        .tune = tune_ch5
    };


    controller->soft_reset();
    usleep(1000000);
    controller->set_mode(thotro110);
    usleep(1000000);


    DW1000Time rx_time;
    /* after reception it will clear its state and go to idle */
    while(1) {
        controller->test_receiving_timestamp(rx_time);
        fprintf(stdout, "RX Timestamp: %lu\n", rx_time.get_timestamp());
        usleep(1000000);
    }

    //DW1000Time tx_time;
    //uint32_t i = 0;
    //while(1) {
    //    controller->test_transmission_timestamp(tx_time, i++);
    //    fprintf(stdout, "TX Timestamp: %lu\n", tx_time.get_timestamp());
    //}


    return EXIT_SUCCESS;
}