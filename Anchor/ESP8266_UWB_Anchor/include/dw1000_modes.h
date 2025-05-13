#pragma once

#ifndef DW1000_MODES_H
#define DW1000_MODES_H

#include <stdint.h>
/**
 * 
 */
typedef struct {
    uint8_t rf_rxctrlh; /* see Table 37 for 0x28:0B– RF_RXCTRLH*/
    uint32_t rf_txctrl; 
    uint8_t tc_pgdelay;
    uint32_t fs_pllcfg;
} dw1000_channel_t;

/**
 * 
 */
typedef struct {
    uint32_t txprf;
    uint32_t rxfprf;
} dw1000_prf_t;


/**
 * 
 */
typedef struct {
    uint16_t drx_tune0b;
    uint16_t drx_tune1a;
    uint16_t drx_tune1b;
    uint32_t drx_tune2;
    uint8_t fs_plltune;
} dw1000_tuning_t;


/**
 * 
 */
typedef struct {
    uint16_t agc_tune1;
    uint32_t agc_tune2;
} dw1000_fixed_config_t;


/**
 * 
 */
typedef struct {
    uint32_t txbr;      //< tx_fctrl register
    uint32_t rxm110k;   //< sys_cfg register
} dw1000_bitrate_t;


/**
 * 
 */
enum SFD {
    STD,
    DecaWave,
};


/**
 * 
 */
typedef struct {
    uint8_t channel_num;
    dw1000_channel_t        channel_config;
    dw1000_tuning_t         tune_config;
    dw1000_fixed_config_t   fixed_config;
    dw1000_prf_t            prf_config;
    dw1000_bitrate_t        bitrate_config;
    uint32_t                preamble_code;
    uint32_t                preamble_length;
    SFD                     sfd;
} dw1000_mode_t;

typedef enum {
    THOTRO,
    JOPEL,
    JOPEL2,
} dw1000_mode_enum_t;

// Deklaration der konstanten Konfiguration
extern const dw1000_mode_t THOTRO110;
extern const dw1000_mode_t JOPEL110;
extern const dw1000_mode_t JOPEL850;


#endif