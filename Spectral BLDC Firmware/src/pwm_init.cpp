/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pwm_init.cpp
  * @brief   This file provides code for initialization of PWM peripheral
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


#include "pwm_init.h"

/** 
 * Init HIGH side pin for our PWM 
 * Inputs are PIN that needs to be part of TIMx CHANNEL and PWM frequency
 * https://www.st.com/resource/en/datasheet/cd00161566.pdf PAGE 30
 * @param[in] ulPin
 * @param[in] PWM_freq
*/
HardwareTimer* pwm_high(int ulPin, uint32_t PWM_freq)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  
  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
  }
  HardwareTimer *TMR = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  TMR->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  TMR->setOverflow(PWM_freq, HERTZ_FORMAT);
  TMR->pause();
  TMR->refresh();
  return TMR;
}

/** 
 * Init HIGH side pin for our PWM AND create interrupt that triggers 
 * TWICE during PWM period
 * Inputs are ulPin that needs to be part of TIMx CHANNEL and PWM frequency
 * int_callback is function we will be calling twice every pwm period
 * https://www.st.com/resource/en/datasheet/cd00161566.pdf PAGE 30
 * @param[in] ulPin
 * @param[in] PWM_freq
 * @param[in] int_callback
*/
HardwareTimer* pwm_freq(int ulPin, uint32_t PWM_freq, void (*int_callback)())
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  
  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
  }
  HardwareTimer *TMR = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  TMR->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  TMR->setOverflow(PWM_freq, HERTZ_FORMAT);
  TMR->attachInterrupt(*int_callback);
  TMR->pause();
  TMR->refresh();
  return TMR;
}





/** 
 * Init HIGH side pin for our PWM AND create interrupt that triggers 
 * TWICE during PWM period
 * Inputs are ulPin that needs to be part of TIMx CHANNEL and PWM frequency
 * int_callback is function we will be calling twice every pwm period
 * https://www.st.com/resource/en/datasheet/cd00161566.pdf PAGE 30
 * @param[in] ulPin
 * @param[in] PWM_freq
 * @param[in] int_callback
*/
HardwareTimer* pwm_freq_2(int ulPin, uint32_t PWM_freq, void (*int_callback)())
{

  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  
  uint32_t index = get_timer_index(Instance);
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
  }
  HardwareTimer *TMR = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  TMR->setMode(channel, TIMER_DISABLED);
  TMR->setOverflow(PWM_freq, HERTZ_FORMAT);
  // Check if callback should be called this time
  
  TMR->attachInterrupt(*int_callback);

  TMR->pause();
  TMR->refresh();
 
  return TMR;
}




/** 
 * Init LOW side pin for our PWM 
 * Inputs are PIN that needs to be part of TIMx CHANNEL and PWM frequency
 * https://www.st.com/resource/en/datasheet/cd00161566.pdf PAGE 30
 * @param[in] ulPin
 * @param[in] PWM_freq
*/
HardwareTimer* pwm_low(int ulPin, uint32_t PWM_freq)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
    
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    HAL_TIM_PWM_ConfigChannel(&(HardwareTimer_Handle[index]->handle), &sConfigOC, channel);
  }
  HardwareTimer *TMR = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  TMR->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, pin);
  TMR->setOverflow(PWM_freq, HERTZ_FORMAT);
  TMR->pause();
  TMR->refresh();
  return TMR;
}

/// @brief Per-pin resolved timer+channel, so pwm_set() doesn't repeat the pin-map lookup
/// chain (digitalPinToPinName -> pinmap_peripheral -> get_timer_index -> pinmap_function)
/// on every call. The mapping from a pin to its timer/channel is fixed at compile time,
/// so it only needs to be resolved once per pin, the first time that pin is used.
struct PwmPinCache
{
  int pin = -1;
  HardwareTimer *tmr = nullptr;
  uint32_t channel = 0;
};

static PwmPinCache pwm_cache[4];

static void pwm_resolve(int ulPin, HardwareTimer **tmr_out, uint32_t *channel_out)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
  *tmr_out = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  *channel_out = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
}

/**
 * Set value to PWM pin to desired duty cycle
 * ulPin is pin we want to set, value is value we set (duty)
 * resolution sets maximum value we can give to duty
 * @param[in] ulPin
 * @param[in] value
 * @param[in] resolution
*/
void pwm_set(int ulPin, uint32_t value, int resolution)
{
  // Fast path: pin already resolved (the normal case -- called every control-loop tick
  // with one of a small fixed set of pins).
  for (int i = 0; i < 4; i++)
  {
    if (pwm_cache[i].pin == ulPin)
    {
      pwm_cache[i].tmr->setCaptureCompare(pwm_cache[i].channel, value, (TimerCompareFormat_t)resolution);
      return;
    }
  }

  // Slow path: first call for this pin. Resolve once, cache it, and use it immediately.
  for (int i = 0; i < 4; i++)
  {
    if (pwm_cache[i].pin == -1)
    {
      HardwareTimer *tmr;
      uint32_t channel;
      pwm_resolve(ulPin, &tmr, &channel);

      pwm_cache[i].pin = ulPin;
      pwm_cache[i].tmr = tmr;
      pwm_cache[i].channel = channel;

      tmr->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
      return;
    }
  }

  // Cache full (more than 4 distinct PWM pins in use) -- fall back to resolving every call.
  HardwareTimer *tmr;
  uint32_t channel;
  pwm_resolve(ulPin, &tmr, &channel);
  tmr->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}


/// @brief Align pwms
/// @param TMR Hardware timer we use#
void pwm_align(HardwareTimer *TMR)
{

  TMR->pause();
  TMR->refresh();
  TMR->resume();

}
