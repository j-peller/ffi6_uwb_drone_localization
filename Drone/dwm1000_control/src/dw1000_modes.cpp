#include "../inc/dw1000_modes.h"

const dw1000_mode_t THOTRO110 = {
    .channel_num = 5,
    .channel_config = {
        .rf_rxctrlh = RF_RXCTRLH_NBW,
        .rf_txctrl = RF_TXCTRL_CH5,
        .tc_pgdelay = 0xB5,
        .fs_pllcfg = FS_PLLCFG_CH5,
    },
    .tune_config = {
        .drx_tune0b = DRX_TUNE0b_110K_STD,
        .drx_tune1a = DRX_TUNE1a_PRF16,
        .drx_tune1b = DRX_TUNE1b_110K,
        .drx_tune2 = DRX_TUNE2_PRF16_PAC64,
        .fs_plltune = FS_PLLTUNE_CH5
    },
    .fixed_config = {
        .agc_tune1 = AGC_TUNE1_16M,
        .agc_tune2 = AGC_TUNE2_VAL,
    },
    .prf_config = {
        .txprf = TX_FCTRL_TXPRF_16M,
        .rxfprf = CHAN_CTRL_RXFPRF_16,
    },
    .bitrate_config = {
        .txbr = TX_FCTRL_TXBR_110k,
        .rxm110k = SYS_CFG_RXM110K,
    },
    .preamble_code = 0x04,
    .preamble_length = TX_FCTRL_TXPSR_PE_2048,
    .sfd = SFD::STD
};