
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    CAN.h
  * @brief   This file contains all the function prototypes for
  *          the CAN.cpp file
  * @author Petar Crnjak
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Source robotics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * Based on: https://github.com/nopnop2002/Arduino-STM32-CAN
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <Arduino.h>
#include <assert.h>
#include <stdint.h>

#define DEBUG 0

// Symbolic names for bit rate of CAN message
typedef enum {
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS,
} BITRATE;

/* Real speed for bit rate of CAN message                                    */
 //uint32_t SPEED[6] = {50*1000, 100*1000, 125*1000, 250*1000, 500*1000, 1000*1000};

// Symbolic names for formats of CAN message
typedef enum {STANDARD_FORMAT, EXTENDED_FORMAT} CAN_FORMAT;

// Symbolic names for type of CAN message
typedef enum {DATA_FRAME, REMOTE_FRAME} CAN_FRAME;

typedef struct {
    uint32_t id;        // 29 bit identifier
    uint8_t data[8];    // Data field
    uint8_t len;        // Length of data field in bytes
    uint8_t ch;         // Object channel (Not used)
    uint8_t format;     // 0 - STANDARD, 1 - EXTENDED IDENTIFIER
    uint8_t type;       // 0 - DATA FRAME, 1 - REMOTE FRAME
} CAN_msg_t;

typedef struct {
    uint16_t baud_rate_prescaler;             // [1 to 1024]
    uint8_t time_segment_1;                   // [1 to 16]
    uint8_t time_segment_2;                   // [1 to 8]
    uint8_t resynchronization_jump_width;     // [1 to 4] (recommended value is 1)
} CAN_bit_timing_config_t;

#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE     1000
#define CAN_STM32_ERROR_MSR_INAK_NOT_SET         1001
#define CAN_STM32_ERROR_MSR_INAK_NOT_CLEARED     1002
#define CAN_STM32_ERROR_UNSUPPORTED_FRAME_FORMAT 1003


#define STM32_CAN_TSR_TME0              (1U << 26U) // Transmit mailbox 0 empty
#define STM32_CAN_TSR_TME1              (1U << 27U) // Transmit mailbox 1 empty
#define STM32_CAN_TSR_TME2              (1U << 28U) // Transmit mailbox 2 empty
#define STM32_CAN_TSR_TME_ANY           (STM32_CAN_TSR_TME0 | STM32_CAN_TSR_TME1 | STM32_CAN_TSR_TME2)
#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

int16_t ComputeCANTimings(const uint32_t peripheral_clock_rate,
                          const uint32_t target_bitrate,
                          CAN_bit_timing_config_t* const out_timings);

void printRegister(const char * buf, uint32_t reg);
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);
void CANSetNodeFilter(uint8_t node_id);
bool CANInit(BITRATE bitrate, int remap);
void CANReceive(CAN_msg_t* CAN_rx_msg);
/// False if no mailbox freed within 1 ms (counted in CAN_tx_dropped).
bool CANSend(CAN_msg_t* CAN_tx_msg);
/// Receive FIFO overruns and dropped transmissions since boot.
extern volatile uint32_t CAN_rx_overruns;
extern volatile uint32_t CAN_tx_dropped;
uint8_t CANMsgAvail();

#endif  // CAN_UTILS_H