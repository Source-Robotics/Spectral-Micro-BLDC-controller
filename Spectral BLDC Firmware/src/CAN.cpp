
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    CAN.cpp
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include <Arduino.h>
#include "CAN.h"


/// @brief 
/// @param peripheral_clock_rate 
/// @param target_bitrate 
/// @param out_timings 
/// @return 
int16_t ComputeCANTimings(const uint32_t peripheral_clock_rate,
                          const uint32_t target_bitrate,
                          CAN_bit_timing_config_t* const out_timings)
{
    if (target_bitrate < 1000)
    {
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;
    }

    assert(out_timings != NULL);  // NOLINT
    memset(out_timings, 0, sizeof(*out_timings));

    /*
     * Hardware configuration
     */
    static const uint8_t MaxBS1 = 16;
    static const uint8_t MaxBS2 = 8;


    const uint8_t max_quanta_per_bit = (uint8_t)((target_bitrate >= 1000000) ? 10 : 17);    // NOLINT
    assert(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

    static const uint16_t MaxSamplePointLocationPermill = 900;


    const uint32_t prescaler_bs = peripheral_clock_rate / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);    // NOLINT

    while ((prescaler_bs % (1U + bs1_bs2_sum)) != 0)
    {
        if (bs1_bs2_sum <= 2)
        {
            return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1U + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;              // No solution
    }

  
    uint8_t bs1 = (uint8_t)(((7 * bs1_bs2_sum - 1) + 4) / 8);       // Trying rounding to nearest first  // NOLINT
    uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);  // NOLINT
    assert(bs1_bs2_sum > bs1);

    {
        const uint16_t sample_point_permill = (uint16_t)(1000U * (1U + bs1) / (1U + bs1 + bs2));  // NOLINT

        if (sample_point_permill > MaxSamplePointLocationPermill)   // Strictly more!
        {
            bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) / 8);             // Nope, too far; now rounding to zero
            bs2 = (uint8_t)(bs1_bs2_sum - bs1);
        }
    }

    const bool valid = (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);


    if ((target_bitrate != (peripheral_clock_rate / (prescaler * (1U + bs1 + bs2)))) ||
        !valid)
    {
        // This actually means that the algorithm has a logic error, hence assert(0).
        assert(0);  // NOLINT
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;
    }

    out_timings->baud_rate_prescaler = (uint16_t) prescaler;
    out_timings->resynchronization_jump_width = 1;      // One is recommended by UAVCAN, CANOpen, and DeviceNet
    out_timings->time_segment_1 = bs1;
    out_timings->time_segment_2 = bs2;


    return 0;
}

/// @brief 
/// @param buf 
/// @param reg 
void printRegister(const char * buf, uint32_t reg) {
  if (DEBUG == 0) return;

}

/// @brief 
/// @param index 
/// @param scale 
/// @param mode 
/// @param fifo 
/// @param bank1 
/// @param bank2 
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 27) return;

  CAN1->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN1->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN1->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
    if (mode == 0) {
    CAN1->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN1->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN1->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN1->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN1->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN1->FA1R |= (0x1UL<<index);                // Activate filter

}

/// Accept only frames addressed to this node (node id = top 4 bits of the 11-bit id).
void CANSetNodeFilter(uint8_t node_id) {
  uint32_t id   = ((uint32_t)(node_id & 0xFU) << 7U) << 21U;
  uint32_t mask = ((uint32_t)0xFU << 7U) << 21U;
  CAN1->FMR |= 0x1UL;                          // Filter initialization mode
  CANSetFilter(0, 1, 0, 0, id, mask);
  CAN1->FMR &= ~(0x1UL);
}

/// @brief 
/// @param bitrate 
/// @param remap 
/// @return 
bool CANInit(BITRATE bitrate, int remap)
{

   uint32_t SPEED[6] = {50*1000, 100*1000, 125*1000, 250*1000, 500*1000, 1000*1000};
  // Reference manual
  // https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf

  RCC->APB1ENR |= 0x2000000UL;       // Enable CAN clock 
  RCC->APB2ENR |= 0x1UL;             // Enable AFIO clock
  AFIO->MAPR   &= 0xFFFF9FFF;        // reset CAN remap
                                     // CAN_RX mapped to PA11, CAN_TX mapped to PA12

  if (remap == 0) {
    RCC->APB2ENR |= 0x4UL;           // Enable GPIOA clock
    GPIOA->CRH   &= ~(0xFF000UL);    // Configure PA12(0b0000) and PA11(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOA->CRH   |= 0xB8FFFUL;       // Configure PA12(0b1011) and PA11(0b1000)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     
    //GPIOA->ODR |= 0x1UL << 12;       // PA12 Upll-up
    GPIOA->ODR |= 0x1UL << 11;       // PA11 Upll-up
    
  }
                                
  if (remap == 2) {
    AFIO->MAPR   |= 0x00004000;      // set CAN remap
                                     // CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)

    RCC->APB2ENR |= 0x8UL;           // Enable GPIOB clock
    GPIOB->CRH   &= ~(0xFFUL);       // Configure PB9(0b0000) and PB8(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOB->CRH   |= 0xB8UL;          // Configure PB9(0b1011) and PB8(0b1000)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     
    GPIOB->ODR |= 0x1UL << 8;        // PB8 Upll-up
  }
  
  if (remap == 3) {
    AFIO->MAPR   |= 0x00005000;      // set CAN remap
                                     // CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)

    RCC->APB2ENR |= 0x20UL;          // Enable GPIOD clock
    GPIOD->CRL   &= ~(0xFFUL);       // Configure PD1(0b0000) and PD0(0b0000)
                                     // 0b0000
                                     //   MODE=00(Input mode)
                                     //   CNF=00(Analog mode)

    GPIOD->CRH   |= 0xB8UL;          // Configure PD1(0b1011) and PD0(0b1000)
                                     // 0b1000
                                     //   MODE=00(Input mode)
                                     //   CNF=10(Input with pull-up / pull-down)
                                     // 0b1011
                                     //   MODE=11(Output mode, max speed 50 MHz) 
                                     //   CNF=10(Alternate function output Push-pull
                                     
    GPIOD->ODR |= 0x1UL << 0;        // PD0 Upll-up
  }

  CAN1->MCR |= 0x1UL;                   // Require CAN1 to Initialization mode 
  while (!(CAN1->MSR & 0x1UL));         // Wait for Initialization mode

  //CAN1->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
  // TXFP: transmit mailboxes go out in request order, so replies are not reordered.
  CAN1->MCR = 0x45UL;
   
  // Set bit timing register 
  CAN_bit_timing_config_t timings;
  uint32_t target_bitrate = SPEED[bitrate];
  int result = ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), target_bitrate, &timings);
  if (result) while(true);  
  CAN1->BTR = (((timings.resynchronization_jump_width - 1U) &    3U) << 24U) |
              (((timings.time_segment_1 - 1U)               &   15U) << 16U) |
              (((timings.time_segment_2 - 1U)               &    7U) << 20U) |
              ((timings.baud_rate_prescaler - 1U)           & 1023U);

  // Configure Filters to default values
  CAN1->FMR |= 0x1UL;                   // Set to filter initialization mode
  CAN1->FMR &= 0xFFFFC0FF;              // Clear CAN2 start bank
  // bxCAN has 28 filters.
  // These filters are shared by both CAN1 and CAN2.
  // STM32F103 has only CAN1, so all 28 are used for CAN1
  CAN1->FMR |= 0x1C << 8;               // Assign all filters to CAN1

  // Set fileter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL); 
  
  CAN1->FMR &= ~(0x1UL);                // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;
  bool can1 = false;
  CAN1->MCR &= ~(0x1UL);                // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN1->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //Serial.print("can1=");
  //Serial.println(can1);
  if (can1) {
    //Serial.println("CAN1 initialize ok");
  } else {
    //Serial.println("CAN1 initialize fail!!");
    return false;
  }
  return true; 
}

/// @brief 
/// @param CAN_rx_msg 
void CANReceive(CAN_msg_t* CAN_rx_msg)
{
  uint32_t id = CAN1->sFIFOMailBox[0].RIR;
  if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
      CAN_rx_msg->format = STANDARD_FORMAT;;
      CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
  } 
  else {                               // Extended frame format
      CAN_rx_msg->format = EXTENDED_FORMAT;;
      CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
  }

  if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
      CAN_rx_msg->type = DATA_FRAME;
  }
  else {                               // Remote frame
      CAN_rx_msg->type = REMOTE_FRAME;
  }

  
  CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
  
  CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
  CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
  CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  // Count an overrun before the release below clears its flag.
  if (CAN1->RF0R & 0x10UL) {
    CAN_rx_overruns = CAN_rx_overruns + 1;
    CAN1->RF0R |= 0x10UL;
  }
  // Release FIFO 0 output mailbox.
  // Make the next incoming message available.
  CAN1->RF0R |= 0x20UL;
}

/// @brief 
/// @param CAN_tx_msg 
volatile uint32_t CAN_rx_overruns = 0;
volatile uint32_t CAN_tx_dropped = 0;

bool CANSend(CAN_msg_t* CAN_tx_msg)
{
  uint32_t out = 0;
  if (CAN_tx_msg->format == EXTENDED_FORMAT) { // Extended frame format
      out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  }
  else {                                       // Standard frame format
      out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
  }

  // Remote frame
  if (CAN_tx_msg->type == REMOTE_FRAME) {
      out |= STM32_CAN_TIR_RTR;
  }

  // Wait at most 1 ms for a free mailbox; drop the frame rather than stall loop().
  uint32_t started = micros();
  while (!(CAN1->TSR & STM32_CAN_TSR_TME_ANY)) {
    if (micros() - started > 1000U) {
      CAN_tx_dropped = CAN_tx_dropped + 1;
      return false;
    }
  }
  uint8_t mb = (CAN1->TSR >> 24U) & 0x3U;      // CODE: the mailbox to use next

  CAN1->sTxMailBox[mb].TDTR &= ~(0xF);
  CAN1->sTxMailBox[mb].TDTR |= CAN_tx_msg->len & 0xFUL;

  CAN1->sTxMailBox[mb].TDLR = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                               ((uint32_t) CAN_tx_msg->data[2] << 16) |
                               ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[0]      ));
  CAN1->sTxMailBox[mb].TDHR = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                               ((uint32_t) CAN_tx_msg->data[6] << 16) |
                               ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[4]      ));

  // Send Go
  CAN1->sTxMailBox[mb].TIR = out | STM32_CAN_TIR_TXRQ;
  return true;
}

/// @brief 
/// @param  
/// @return 
uint8_t CANMsgAvail(void)
{
  // Check for pending FIFO 0 messages
  return CAN1->RF0R & 0x3UL;
}