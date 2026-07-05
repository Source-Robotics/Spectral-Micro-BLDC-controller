/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   This file provides code for all the function prototypes for motor_control.cpp
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

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTOR_CONT_
#define MOTOR_CONT_

#include <Arduino.h>
#include <stdio.h>
#include "hw_init.h"
#include "pwm_init.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "adc_init.h"
#include "iodefs.h"
#include "qfplib-m3.h"
#include "SerialPacketParser.h"
#include "constants.h"
#include "motor_control.h"
#include "common.h"
#include "temperature_table.h"


const float CURRENT_SENSE_CONSTANT = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN;
const float CURRENT_SENSE_CONSTANT_mV = ((ADC_MIDPOINT / ADC_MIDPOINT_BIT) / SENSE_RESISTOR) / CURRENT_AMP_GAIN * 1000;
//const int VOLTAGE_SENSE_CONSTANT = 

bool parityCheck(uint16_t data);
void Collect_data();
float Get_current(int adc_value);
int Get_voltage_mA(int adc_value);
int Get_current_mA(int adc_value);
int Get_ADC_Value(int current_mA);
void Position_mode();
void Velocity_mode();
void Torque_mode();
void Update_IT_callback_calib();
void IT_callback(void);
void Enable_drive(void);
void Calib_report(Stream &Serialport);
void PD_mode();
void Open_loop_speed(float speed, int voltage);
void LED_status(uint32_t ms);
void Phase_order();
void Collect_data2();
void Gripper_mode();
void Calibrate_gripper();
void Get_first_encoder();
void Voltage_Torque_mode();
int Calibrate_Angle_Offset2();
int Calibrate_Angle_Offset_Align();

#endif
