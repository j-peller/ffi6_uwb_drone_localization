#pragma once

#ifndef ANCHOR_ADDRESSES_HPP
#define ANCHOR_ADDRESSES_HPP

/**
 *  Asumption: Target is orientied to the north
 *  - Top Left      = ANCHOR_1
 *  - Top Right     = ANCHOR_2
 *  - Bottom Left   = ANCHOR_3
 *  - Bottom Right  = ANCHOR_4
 */


/* Short Addresses of the DWM1000 Anchors */
#define ANCHOR_1    0xAFFE
#define ANCHOR_2    0xCAFE
#define ANCHOR_3    0xDEAD
#define ANCHOR_4    0xBEEF

#define ANCHOR_1_LONG 0xAFFEAFFEAFFEAFFE
#define ANCHOR_2_LONG 0xCAFECAFECAFECAFE
#define ANCHOR_3_LONG 0xDEADDEADDEADDEAD
#define ANCHOR_4_LONG 0xBEEFBEEFBEEFBEEF

#define BROADCAST   0xFFFF

/* Short Address of the UWB Master*/
#define MASTER      0xB00B
#define MASTER_LONG      0xB00BB00BB00BB00B

/* PAN ID */
#define DEFAULT_PAN 0xDECA

#endif