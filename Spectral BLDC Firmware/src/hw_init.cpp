/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hw_init.cpp
  * @brief   This file provides code for hardware initialization
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

#include "hw_init.h"

// TODO HAL sensor init
/// @brief Init all digital input pins
/// @param  
void Init_Digital_Inputs(void)
{


	// Fault indication pin. Pulled logic-low with fault condition;
	// open-drain output requires an external pullup.
	// Fault conditions are:
	// VM undervoltage - Operation will resume when VM rises above the UVLO threshold. FAULT pin will be released after operation has resumed.
	// Thermal shutdown - Once the die temperature has fallen to a safe level operation will automatically resume. FAULT pin will be released after operation has resumed.
	// Overcurrent protection - The driver remains off until either assertion of nRESET or the cycling of VM power.
	// 
	// Pulling nRESET low clears all errors, but disables outputs
	// If after RESET there can still be over temperature and undervoltage error 
	pinMode(FAULT, INPUT);
	attachInterrupt(digitalPinToInterrupt(FAULT),DRV_ERROR_INTERRUPT,FALLING);
	// TODO HAL sensor init
	
}

/// @brief  Driver Fault interrupt
void DRV_ERROR_INTERRUPT(){
	controller.Driver_error = 1;

}

/// @brief Init all digital output pins
void Init_Digital_Outputs(void)
{

	pinMode(EN1, OUTPUT);
	pinMode(EN2, OUTPUT);
	pinMode(EN3, OUTPUT);

	pinMode(SLEEP, OUTPUT);

	pinMode(RESET, OUTPUT);

	pinMode(PWM_CH1, OUTPUT);
	pinMode(PWM_CH2, OUTPUT);
	pinMode(PWM_CH3, OUTPUT);

	pinMode(LED, OUTPUT);
	//pinMode(ADDITIONAL, OUTPUT);

}

/// @brief System clock config
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	// No HAL_RCC_MCOConfig() here. It was in this function up to V107 and turned PA8 into the 8 MHz clock output
	// (MCO). PA8 is EN2, the phase-B enable of the gate driver: phase B then ran on a clock instead of a level, and
	// every digitalWriteFast(EN2, ...) was ignored (found 2026-11-10; standards note 2026-11-spectral-109-fault-analysis).

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/// @brief  Init system clock
/// @param multiplier clock multiplier
/// Default clock multiplier is 16 for 128Mhz clock speed
void TM_SystemClock_Config(int multiplier)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	// revert to HSE source
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE; // <- HSE
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	// Setup PLL with new multiplier
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	// set clock multiplier
	if (multiplier == 9)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	}
	else if (multiplier == 10)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
	}
	else if (multiplier == 11)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL11;
	}
	else if (multiplier == 12)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	}
	else if (multiplier == 13)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL13;
	}
	else if (multiplier == 14)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL14;
	}
	else if (multiplier == 15)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
	}
	else if (multiplier == 16)
	{
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	}
	else
	{ // if wrong number use stock speed
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	}

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	// Go back to PLL source
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // <- PLL welcome back
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
