/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "adc_init.h"

// hadc1 is defined once in main.cpp; adc_init.h declares it extern for use here.

/// @brief ADC1 init function
void MX_ADC1_Init(void)
{

  /* Enable clock of ADCx peripheral */
  __HAL_RCC_ADC1_CLK_ENABLE();

  /* Configure ADCx clock prescaler */
  /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
  /*          datasheet).                                                     */
  /*          Therefore, ADC clock prescaler must be configured in function   */
  /*          of ADC clock source frequency to remain below this maximum      */
  /*          frequency.  

                                                 */
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3; //3
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 3;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 
  HAL_ADC_Init(&hadc1);
  

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4; // sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;

  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; ///ADC_SAMPLETIME_7CYCLES_5
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  
}


/*
HAL_ADC_Stop(&hadc1); is SLOW!
The function typically disables the ADC hardware, stops any ongoing conversions, 
and performs any necessary cleanup operations.
*/

/// @brief 
/// @param  
/// https://community.st.com/s/question/0D50X00009fFZDOSA4/stm32f072-can-adc-conversion-polling-be-done-for-multiple-channel
/// https://controllerstech.com/stm32-adc-multi-channel-without-dma/
/// This is not optimal 
/// Optimal is to start ADC and read all channels then stop
/// optimal solutions:
// https://community.st.com/s/question/0D50X00009XkeQ9SAJ/multichannel-adc-reading
void ADC_CHANNEL_4_SELECT_SENSE1(void){
    // Select ADC channel
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; ///ADC_SAMPLETIME_7CYCLES_5
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/// @brief 
/// @param  
/// @return 
int ADC_CHANNEL_4_READ_SENSE1(void){

  ADC_CHANNEL_4_SELECT_SENSE1();
  // Read selected ADC channel
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  int AD_RES = HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  return AD_RES;

} 

/// @brief 
/// @param  
void ADC_CHANNEL_3_SELECT_SENSE2(void){
    // Select ADC channel
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; ///ADC_SAMPLETIME_7CYCLES_5
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/// @brief 
/// @param  
/// @return 
int ADC_CHANNEL_3_READ_SENSE2(void){

  ADC_CHANNEL_3_SELECT_SENSE2();
  // Read selected ADC channel
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  int AD_RES = HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  return AD_RES;

} 

/// @brief 
/// @param  
void ADC_CHANNEL_6_SELECT_VBUS(void){
    // Select ADC channel
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; ///ADC_SAMPLETIME_7CYCLES_5
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/// @brief 
/// @param  
/// @return 
int ADC_CHANNEL_6_READ_VBUS(void){

  ADC_CHANNEL_6_SELECT_VBUS();
  // Read selected ADC channel
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  int AD_RES = HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  return AD_RES;

} 

/// @brief 
/// @param  
void ADC_CHANNEL_5_SELECT_TEMP(void){
    // Select ADC channel
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; ///ADC_SAMPLETIME_7CYCLES_5
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/// @brief 
/// @param  
/// @return 
int ADC_CHANNEL_5_READ_TEMP(void){

  ADC_CHANNEL_5_SELECT_TEMP();
  // Read selected ADC channel
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  int AD_RES = HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  return AD_RES;

} 

/// @brief 
/// @param  
/// @return 
int READ_ALL_ADC_CHANNELS(void){

  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result & Map It To PWM DutyCycle
  int AD_RES = HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  return AD_RES;

}
