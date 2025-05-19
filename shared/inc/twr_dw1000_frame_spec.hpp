#pragma once

#ifndef TWR_DW1000_FRAME_SPEC_HPP
#define TWR_DW1000_FRAME_SPEC_HPP

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief TWR message types 
 */
typedef enum : uint8_t {
    TWR_MSG_TYPE_POLL 	= 0x01,
    TWR_MSG_TYPE_RESPONSE = 0x02,
    TWR_MSG_TYPE_FINAL    = 0x03,
    TWR_MSG_TYPE_REPORT   = 0x04,
    TWR_MSG_TYPE_RESULT   = 0x05,
} twr_msg_type_t;

#pragma pack(push, 1)

/**
 * @brief TWR message frame header
 */
typedef struct {
    uint8_t frameCtrl[2];   /* Frame control */
    uint8_t seqNum;         /* Sequence number */
    uint8_t panID[2];       /* PAN ID */
    uint8_t destAddr[2];    /* Destination address */
    uint8_t srcAddr[2];     /* Source address */
} twr_frame_header_t;


/**
 * @brief TWR init message sent by the drone to a specific anchor
 *        to initiate the TWR process. Configureable reponse delay for the
 *        anchor.
 */
typedef struct {
    uint8_t type;
    uint8_t anchorShortAddr[2];     /* Anchor short address */
    // uint8_t responseDelay[2];       /* Response delay: we dont use this at the moment */
} twr_init_message_t;


/**
 * @brief TWR response message sent by the anchor to the drone
 *        to acknowledge the poll message and to initiate the final
 *        TWR message.
 */
typedef struct {
    uint8_t type;         /* Frame type */
} twr_response_message_t;


/**
 * @brief TWR final message sent by the drone to the anchor
 *        to acknowledge the response message and to finialize
 *        the TWR process and receive the final report by the anchor.
 */
typedef struct {
    uint8_t type;         /* Frame type */
} twr_final_message_t;


/**
 * @brief TWR report message sent by the anchor to the drone
 */
typedef struct {
    uint8_t type;           /* Frame type */
    uint8_t pollRx[5];      /* Receive timestamp of initial polling message */
    uint8_t responseTx[5];  /* Response timestamp of first response */
    uint8_t finalRx[5];     /* Final timestamp of this report message to the drone */
} twr_report_message_t;


/**
 * @brief TWR result message sent by drone to the anchor during antenna calibration
 */
typedef struct {
    uint8_t type;
    uint8_t distance[2]; /* Antenna delay correction value */
} twr_result_message_t;


/**
 * @brief TWR message frame used by the Drone and Anchor
 */
typedef struct {
    twr_frame_header_t header; /* Frame header */
    union {
        twr_init_message_t      init;
        twr_response_message_t  response;
        twr_final_message_t     final;
        twr_report_message_t    report;
        twr_result_message_t    result;
    } payload; /* Frame payload */
} twr_message_t;

#pragma pack(pop)


#ifdef __cplusplus
}
#endif

#endif /* TWR_DW1000_FRAME_SPEC_HPP */